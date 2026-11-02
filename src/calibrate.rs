//! `fw-fanctrl calibrate`: measure fan and thermal behaviour of this machine and
//! derive a quiet strategy from it.
//!
//! Phases:
//! 1. EC sensor names and thermal thresholds (read-only EC queries)
//! 2. fan sweep: duty → RPM, and the duty at which the fan starts from standstill
//! 3. idle baseline per sensor
//! 4. load step: which sensors react slowly (board) and which spike (CPU package)
//! 5. sustained load at increasing fixed duties until the CPU no longer
//!    throttles thermally; that duty and the board temperature it settles at are
//!    the anchor of the curve
//!
//! The fan is always handed back to the EC afterwards (drop guard, signals, panic hook).

use std::cell::Cell;
use std::fs;
use std::hint::black_box;
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Child, Command};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serde_json::{Value, json};

use crate::config::Config;
use crate::daemon;
use crate::ec::{CrosEc, EcCommand, Fan, Thresholds};
use crate::error::{Error, Result, bail};
use crate::signals;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Prefer {
    Quiet,
    Balanced,
    Cool,
}

impl Prefer {
    pub fn parse(s: &str) -> Option<Prefer> {
        match s {
            "quiet" => Some(Prefer::Quiet),
            "balanced" => Some(Prefer::Balanced),
            "cool" => Some(Prefer::Cool),
            _ => None,
        }
    }
    fn name(self) -> &'static str {
        match self {
            Prefer::Quiet => "quiet",
            Prefer::Balanced => "balanced",
            Prefer::Cool => "cool",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Options {
    pub prefer: Prefer,
    pub load_cmd: Option<String>,
    pub apply: Option<String>,
    pub config: PathBuf,
    /// Keep the board sensors at about this temperature in idle / light use
    /// (cooler surface), measured in an extra phase.
    pub comfort_temp: Option<f64>,
}

// ---------------------------------------------------------------- analysis (pure)

#[derive(Debug, Clone)]
pub struct SensorInfo {
    pub index: usize,
    pub name: String,
    pub thresholds: Option<Thresholds>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SensorClass {
    /// Slow board sensor, drives the curve.
    Board,
    /// Reacts within seconds (CPU package); used only to judge throttling.
    Spiky,
    /// The EC does not use it for fan control (fan_off >= 90 °C or disabled).
    IgnoredByEc,
    /// Barely reacts to load (e.g. battery).
    Unresponsive,
}

fn mean(v: &[f64]) -> f64 {
    v.iter().sum::<f64>() / v.len().max(1) as f64
}

fn stddev(v: &[f64]) -> f64 {
    let m = mean(v);
    (v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / v.len().max(1) as f64).sqrt()
}

/// Classify one sensor from its idle samples and the samples after a load step
/// (1 Hz, step starts at index 0).
pub fn classify(info: &SensorInfo, idle: &[f64], step: &[f64]) -> SensorClass {
    let ec_ignored = match info.thresholds {
        Some(t) => t.fan_off.is_none_or(|off| off >= 90),
        None => false, // unknown: judge by behaviour only
    };
    if ec_ignored {
        return SensorClass::IgnoredByEc;
    }
    if idle.is_empty() || step.len() < 10 {
        return SensorClass::Unresponsive;
    }
    let base = mean(idle);
    let total = mean(&step[step.len() - 5..]) - base;
    let fast = mean(&step[2..5]) - base;
    if stddev(idle) > 1.5 || (total >= 3.0 && fast / total > 0.5) {
        SensorClass::Spiky
    } else if total < 2.0 {
        SensorClass::Unresponsive
    } else {
        SensorClass::Board
    }
}

/// A temperature series has settled if the last 20 s are within 0.5 °C of the
/// 20 s window that ended 40 s earlier.
pub fn settled(series: &[f64]) -> bool {
    if series.len() < 60 {
        return false;
    }
    let n = series.len();
    (mean(&series[n - 20..]) - mean(&series[n - 60..n - 40])).abs() < 0.5
}

#[derive(Debug, Clone, PartialEq)]
pub struct Equilibrium {
    pub duty: u8,
    pub rpm: u16,
    /// Max of the board sensors, mean of the last 20 s.
    pub board_temp: f64,
    /// Index of the hottest board sensor.
    pub dominant: usize,
    pub spiky_max: Option<f64>,
    /// Time spent thermally throttled in the last 60 s
    /// (`package_throttle_total_time_ms`; None = not available).
    pub throttle_ms: Option<u64>,
    /// New throttle events in the last 60 s. Weak signal: the counter only
    /// increments on entering throttling, not while staying throttled.
    pub throttle_events: Option<u64>,
    /// Package power over the last 20 s and the long-term limit (RAPL PL1).
    pub package_w: Option<f64>,
    pub pl1_w: Option<f64>,
    /// Average CPU frequency over the last 20 s.
    pub mhz: Option<f64>,
    /// Frequency of the reference run at 100 % fan under the same load.
    pub ref_mhz: Option<f64>,
    /// A sensor came within 2 °C of its EC "high" threshold.
    pub guard_hit: bool,
}

impl Equilibrium {
    /// The CPU is limited by its power budget, not by temperature.
    pub fn power_limited(&self) -> bool {
        matches!((self.package_w, self.pl1_w), (Some(p), Some(l)) if l > 0.0 && p >= 0.9 * l)
    }

    /// Frequency relative to the reference run, if both are known.
    pub fn relative_mhz(&self) -> Option<f64> {
        match (self.mhz, self.ref_mhz) {
            (Some(m), Some(r)) if r > 0.0 => Some(m / r),
            _ => None,
        }
    }

    /// The CPU is not thermally limited at this duty.
    ///
    /// The decisive signal is performance: thermal limiting costs frequency, so the
    /// CPU must reach at least 95 % of the frequency it achieves at 100 % fan under
    /// the same load. Kernel throttle counters veto on top (they only advance at
    /// episode boundaries, so they cannot prove the absence of throttling). Without
    /// a frequency reading, a package near TjMax below PL1 counts as limited.
    pub fn ok(&self) -> bool {
        if self.guard_hit {
            return false;
        }
        if self.throttle_ms.is_some_and(|ms| ms > 600)
            || self.throttle_events.is_some_and(|n| n > 0)
        {
            return false;
        }
        match self.relative_mhz() {
            Some(rel) => rel >= 0.95,
            None => {
                let hot = self.spiky_max.is_some_and(|t| t >= 90.0);
                !(hot && !self.power_limited())
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct CurveInput {
    pub idle_temp: f64,
    /// Lowest duty the fan starts at from 0 % and keeps running at (→ `minSpeed`).
    pub min_speed: u8,
    pub anchor: Equilibrium,
    /// EC thresholds of the dominant sensor.
    pub dominant_thresholds: Option<Thresholds>,
    pub sensors: Vec<usize>,
    /// (temperature, duty that holds the board there in idle)
    pub comfort: Option<(f64, u8)>,
}

/// Build the strategy JSON. Shape: off until shortly above idle, gentle rise to the
/// anchor (quietest throttle-free duty at its settled board temperature), then up to
/// 100 % at the EC's own fan_max for the dominant sensor.
pub fn build_strategy(c: &CurveInput, prefer: Prefer) -> Value {
    let (on_offset, factor, up, down, delay) = match prefer {
        Prefer::Quiet => (4.0, 1.0, 0.5, 0.2, 30),
        Prefer::Balanced => (3.0, 1.3, 1.0, 0.3, 20),
        Prefer::Cool => (2.0, 1.7, 2.0, 0.5, 10),
    };
    let floor = c.min_speed.clamp(5, 40);
    let anchor_duty = ((c.anchor.duty as f64 * factor).round() as u8).clamp(floor + 1, 100);
    let anchor_t = c.anchor.board_temp.floor();
    let idle = c.idle_temp.round();

    let fan_max_t = c
        .dominant_thresholds
        .and_then(|t| t.fan_max)
        .map(|t| t as f64)
        .unwrap_or(anchor_t + 12.0)
        .max(anchor_t + 6.0)
        .min(100.0);
    let critical = {
        let mut t = fan_max_t + 2.0;
        if let Some(high) = c.dominant_thresholds.and_then(|t| t.high) {
            t = t.min(high as f64 - 3.0);
        }
        t.max(fan_max_t).min(100.0)
    };

    let mid_t = ((anchor_t + fan_max_t) / 2.0).round();
    let mid_speed = anchor_duty + (100 - anchor_duty) / 3;

    // Fan-on: either just above idle, or around the comfort temperature, where the
    // curve gives the duty that was measured to hold the board there.
    let (off_t, on_t, on_speed) = match c.comfort {
        Some((t, duty)) if t.round() <= anchor_t - 3.0 => {
            let t = t.round();
            (t - 2.0, t, duty.max(floor).min(anchor_duty))
        }
        _ => {
            let off_t = (idle + on_offset).min(anchor_t - 4.0).max(1.0);
            (off_t, (off_t + 2.0).min(anchor_t - 1.0), floor)
        }
    };

    let candidates = [
        (0.0, 0),
        (off_t.max(1.0), 0),
        (on_t, on_speed),
        (anchor_t, anchor_duty),
        (mid_t, mid_speed),
        (fan_max_t, 100),
    ];
    // keep temps strictly ascending and speeds non-decreasing
    let mut curve: Vec<(f64, u8)> = Vec::new();
    for (t, s) in candidates {
        if curve.last().is_none_or(|&(lt, ls)| t > lt && s >= ls) {
            curve.push((t, s));
        }
    }

    json!({
        "fanSpeedUpdateFrequency": 2,
        "movingAverageInterval": 20,
        "sensors": c.sensors,
        "speedCurve": curve.iter().map(|(t, s)| json!({"temp": t, "speed": s})).collect::<Vec<_>>(),
        "rampUpRate": up,
        "rampDownRate": down,
        "rampDownDelay": delay,
        "criticalTemp": critical,
        "minSpeed": floor,
    })
}

/// Insert `strategy` as `name` into the config JSON and validate the result.
pub fn merge_strategy(config_raw: &str, name: &str, strategy: &Value) -> Result<String> {
    let mut data: Value = serde_json::from_str(config_raw)?;
    let strategies = data
        .get_mut("strategies")
        .and_then(Value::as_object_mut)
        .ok_or_else(|| Error("configuration has no 'strategies' object".into()))?;
    strategies.insert(name.to_string(), strategy.clone());
    let out = serde_json::to_string_pretty(&data)?;
    Config::parse(&out)?; // validate before anything touches the disk
    Ok(out)
}

// ------------------------------------------------------------- measurement

/// Hands control back to the EC and stops the load however we leave.
struct Guard<'a> {
    ec: &'a CrosEc,
    load: Option<Load>,
}

impl Drop for Guard<'_> {
    fn drop(&mut self) {
        if let Some(load) = self.load.take() {
            load.stop();
        }
        match self.ec.send(EcCommand::AutoFanCtrl) {
            Ok(()) => eprintln!("EC automatic fan control restored"),
            Err(e) => eprintln!("[Error] > restoring EC fan control failed: {e}"),
        }
    }
}

struct Load {
    stop: Arc<AtomicBool>,
    threads: Vec<JoinHandle<()>>,
    child: Option<Child>,
}

impl Load {
    fn start(cmd: Option<&str>) -> Result<Load> {
        let stop = Arc::new(AtomicBool::new(false));
        let n = std::thread::available_parallelism().map_or(4, |n| n.get());
        let threads = (0..n)
            .map(|i| {
                let stop = Arc::clone(&stop);
                std::thread::spawn(move || burn(&stop, i as u64))
            })
            .collect();
        let child = match cmd {
            Some(c) => {
                let argv = split_load_cmd(c)?;
                let mut command = Command::new(&argv[0]);
                command.args(&argv[1..]).process_group(0);
                // SAFETY: prctl is async-signal-safe; the child gets SIGTERM if we die
                // (e.g. SIGKILL), so a load cannot outlive the calibration.
                unsafe {
                    command.pre_exec(|| {
                        libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGTERM);
                        Ok(())
                    });
                }
                Some(
                    command
                        .spawn()
                        .map_err(|e| Error(format!("cannot start --load-cmd: {e}")))?,
                )
            }
            None => None,
        };
        Ok(Load {
            stop,
            threads,
            child,
        })
    }

    fn stop(mut self) {
        self.stop.store(true, Ordering::Relaxed);
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
        if let Some(mut child) = self.child.take() {
            let pgid = child.id() as libc::pid_t;
            // SAFETY: plain kill(2) on the process group we created.
            unsafe { libc::kill(-pgid, libc::SIGTERM) };
            let deadline = Instant::now() + Duration::from_secs(3);
            while Instant::now() < deadline {
                if matches!(child.try_wait(), Ok(Some(_))) {
                    return;
                }
                std::thread::sleep(Duration::from_millis(100));
            }
            unsafe { libc::kill(-pgid, libc::SIGKILL) };
            let _ = child.wait();
        }
    }
}

/// Busy loop for one CPU. Uses 256-bit FMA when available, which draws
/// considerably more power than integer code (closer to stress-ng, games and
/// compilers), so the calibration errs on the warm side.
fn burn(stop: &AtomicBool, seed: u64) {
    #[cfg(target_arch = "x86_64")]
    if std::arch::is_x86_feature_detected!("avx2") && std::arch::is_x86_feature_detected!("fma") {
        // SAFETY: the required CPU features were detected at runtime.
        unsafe { burn_avx2_fma(stop) };
        return;
    }
    burn_scalar(stop, seed);
}

#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2,fma")]
fn burn_avx2_fma(stop: &AtomicBool) {
    use std::arch::x86_64::{__m256d, _mm256_fmadd_pd, _mm256_set1_pd};
    let a = _mm256_set1_pd(0.999_999);
    let b = _mm256_set1_pd(1e-7);
    // 10 independent chains keep both FMA ports busy despite the 4-cycle latency
    let mut acc: [__m256d; 10] = [_mm256_set1_pd(1.0); 10];
    while !stop.load(Ordering::Relaxed) {
        for _ in 0..20_000 {
            for v in acc.iter_mut() {
                *v = _mm256_fmadd_pd(*v, a, b);
            }
        }
        acc = black_box(acc);
    }
}

fn burn_scalar(stop: &AtomicBool, seed: u64) {
    let mut x = seed + 1;
    let mut f = [1.0f64; 8];
    while !stop.load(Ordering::Relaxed) {
        for _ in 0..20_000 {
            x = x
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            for v in f.iter_mut() {
                *v = v.mul_add(0.999_999, 1e-7);
            }
        }
        black_box((x, f));
    }
}

fn read_u64(path: &str) -> Option<u64> {
    fs::read_to_string(path).ok()?.trim().parse().ok()
}

const THROTTLE_DIR: &str = "/sys/devices/system/cpu/cpu0/thermal_throttle";
const RAPL_DIR: &str = "/sys/class/powercap/intel-rapl:0";

#[derive(Debug, Clone, Copy)]
struct CpuSample {
    at: Instant,
    throttle_ms: Option<u64>,
    throttle_count: Option<u64>,
    energy_uj: Option<u64>,
    mhz: Option<f64>,
}

impl CpuSample {
    fn now() -> CpuSample {
        CpuSample {
            at: Instant::now(),
            throttle_ms: read_u64(&format!("{THROTTLE_DIR}/package_throttle_total_time_ms")),
            throttle_count: read_u64(&format!("{THROTTLE_DIR}/package_throttle_count")),
            energy_uj: read_u64(&format!("{RAPL_DIR}/energy_uj")),
            mhz: avg_mhz(),
        }
    }
}

fn delta(a: Option<u64>, b: Option<u64>) -> Option<u64> {
    Some(b?.saturating_sub(a?))
}

/// Average package power between two samples (None on counter wrap or missing RAPL).
fn watts(a: &CpuSample, b: &CpuSample) -> Option<f64> {
    let (ea, eb) = (a.energy_uj?, b.energy_uj?);
    let secs = b.at.duration_since(a.at).as_secs_f64();
    (eb >= ea && secs > 0.0).then(|| (eb - ea) as f64 / 1e6 / secs)
}

fn pl1_watts() -> Option<f64> {
    read_u64(&format!("{RAPL_DIR}/constraint_0_power_limit_uw")).map(|uw| uw as f64 / 1e6)
}

fn avg_mhz() -> Option<f64> {
    let mut sum = 0.0;
    let mut n = 0;
    for entry in fs::read_dir("/sys/devices/system/cpu").ok()?.flatten() {
        let p = entry.path().join("cpufreq/scaling_cur_freq");
        if let Some(khz) = fs::read_to_string(p)
            .ok()
            .and_then(|s| s.trim().parse::<f64>().ok())
        {
            sum += khz / 1000.0;
            n += 1;
        }
    }
    (n > 0).then(|| sum / n as f64)
}

fn rpm(ec: &CrosEc) -> u16 {
    match ec.fans().ok().and_then(|f| f.first().copied()) {
        Some(Fan::Rpm(r)) => r,
        _ => 0,
    }
}

/// `--load-cmd` is split like a shell would (quotes), but not run through a shell,
/// so the death signal reaches the program itself. Use `sh -c '...'` explicitly
/// for pipelines.
fn split_load_cmd(cmd: &str) -> Result<Vec<String>> {
    match shlex::split(cmd) {
        Some(v) if !v.is_empty() => Ok(v),
        _ => bail!("--load-cmd: cannot parse '{cmd}'"),
    }
}

fn interrupted() -> Result<()> {
    if signals::take().is_some() {
        bail!("calibration interrupted");
    }
    Ok(())
}

struct Sampler<'a> {
    ec: &'a CrosEc,
    infos: &'a [SensorInfo],
    /// Last lease renewal, if the service is paused for us.
    lease: Option<Cell<Instant>>,
}

/// The service is paused with a lease; if this process dies, the service takes
/// over again after at most this long.
const LEASE_SECS: u64 = 30;
const LEASE_RENEW: Duration = Duration::from_secs(10);

fn pause_service() -> Result<String> {
    daemon::send_command(&["pause".into(), "--lease".into(), LEASE_SECS.to_string()])
}

impl Sampler<'_> {
    /// One reading per second. Returns the readings and whether a sensor is within
    /// 2 °C of its EC "high" threshold.
    fn sample(&self) -> Result<(Vec<Option<f64>>, bool)> {
        std::thread::sleep(Duration::from_secs(1));
        interrupted()?;
        if let Some(last) = &self.lease
            && last.get().elapsed() >= LEASE_RENEW
        {
            if let Err(e) = pause_service() {
                eprintln!("[Warning] > renewing the service pause failed: {e}");
            }
            last.set(Instant::now());
        }
        let temps: Vec<Option<f64>> = self
            .ec
            .temperatures()?
            .into_iter()
            .map(|t| t.map(f64::from))
            .collect();
        let guard = self.infos.iter().any(|i| {
            let high = i.thresholds.and_then(|t| t.high);
            matches!((temps.get(i.index).copied().flatten(), high), (Some(t), Some(h)) if t >= h as f64 - 2.0)
        });
        Ok((temps, guard))
    }
}

fn column(rows: &[Vec<Option<f64>>], idx: usize) -> Vec<f64> {
    rows.iter()
        .filter_map(|r| r.get(idx).copied().flatten())
        .collect()
}

fn max_of(row: &[Option<f64>], idx: &[usize]) -> Option<(f64, usize)> {
    idx.iter()
        .filter_map(|&i| row.get(i).copied().flatten().map(|t| (t, i)))
        .max_by(|a, b| a.0.total_cmp(&b.0))
}

/// Wait (up to `secs`) until the RPM satisfies `done`; returns the last RPM.
fn wait_rpm(ec: &CrosEc, sampler: &Sampler, done: impl Fn(u16) -> bool, secs: u32) -> Result<u16> {
    let mut r = rpm(ec);
    for _ in 0..secs {
        if done(r) {
            break;
        }
        sampler.sample()?;
        r = rpm(ec);
    }
    Ok(r)
}

/// 0 % and wait until the fan has stopped (up to 30 s).
fn stop_fan(ec: &CrosEc, sampler: &Sampler) -> Result<()> {
    ec.send(EcCommand::SetFanDuty(0))?;
    wait_rpm(ec, sampler, |r| r == 0, 30)?;
    Ok(())
}

/// RPM once it is stable (two readings within 2 %, at most 15 s).
fn stable_rpm(ec: &CrosEc, sampler: &Sampler) -> Result<u16> {
    let (mut prev, mut stable, mut r) = (0u16, 0, 0);
    for i in 0..15 {
        sampler.sample()?;
        r = rpm(ec);
        let tol = (prev as f64 * 0.02).max(30.0);
        stable = if i >= 2 && (r as f64 - prev as f64).abs() <= tol {
            stable + 1
        } else {
            0
        };
        prev = r;
        if stable >= 2 {
            break;
        }
    }
    Ok(r)
}

struct Groups<'a> {
    board: &'a [usize],
    spiky: &'a [usize],
}

/// Run at a fixed duty under the current load until the board temperature settles
/// (90 s .. 4 min) and summarize the last part.
fn hold(
    ec: &CrosEc,
    sampler: &Sampler,
    g: &Groups,
    duty: u8,
    ref_mhz: Option<f64>,
) -> Result<Equilibrium> {
    ec.send(EcCommand::SetFanDuty(duty))?;
    let mut board_series = Vec::new();
    let mut rows = Vec::new();
    let mut cpu = Vec::new();
    let mut guard_hit = false;
    for s in 0..240 {
        let (row, hit) = sampler.sample()?;
        cpu.push(CpuSample::now());
        if let Some((t, _)) = max_of(&row, g.board) {
            board_series.push(t);
        }
        rows.push(row);
        if hit {
            guard_hit = true;
            break;
        }
        if s >= 90 && settled(&board_series) {
            break;
        }
    }
    let last = &rows[rows.len().saturating_sub(20)..];
    let (board_temp, dominant) = g
        .board
        .iter()
        .map(|&i| (mean(&column(last, i)), i))
        .max_by(|a, b| a.0.total_cmp(&b.0))
        .unwrap_or((0.0, g.board[0]));
    let spiky_max = g
        .spiky
        .iter()
        .filter_map(|&i| column(last, i).into_iter().reduce(f64::max))
        .reduce(f64::max);
    let (w60, w20) = (
        &cpu[cpu.len().saturating_sub(60)..],
        &cpu[cpu.len().saturating_sub(20)..],
    );
    let (first, last_s) = (w60[0], w60[w60.len() - 1]);
    let mhz: Vec<f64> = w20.iter().filter_map(|c| c.mhz).collect();
    Ok(Equilibrium {
        duty,
        rpm: rpm(ec),
        board_temp,
        dominant,
        spiky_max,
        throttle_ms: delta(first.throttle_ms, last_s.throttle_ms),
        throttle_events: delta(first.throttle_count, last_s.throttle_count),
        package_w: watts(&w20[0], &w20[w20.len() - 1]),
        pl1_w: pl1_watts(),
        mhz: (!mhz.is_empty()).then(|| mean(&mhz)),
        ref_mhz,
        guard_hit,
    })
}

fn report(label: &str, eq: &Equilibrium) {
    let na = || "n/a".to_string();
    let verdict = if label.is_empty() {
        if eq.ok() { "ok" } else { "thermally limited" }
    } else {
        label
    };
    eprintln!(
        "  {:>3}% ({} RPM): board {:.1}°C (sensor {}), package {}, {} ({}), throttled {} ms/60 s ({} events), {} of PL1 {} → {}",
        eq.duty,
        eq.rpm,
        eq.board_temp,
        eq.dominant,
        eq.spiky_max.map_or("-".into(), |t| format!("{t:.0}°C")),
        eq.mhz.map_or("-".into(), |m| format!("{m:.0} MHz")),
        eq.relative_mhz()
            .map_or("-".into(), |r| format!("{:.0} % of reference", r * 100.0)),
        eq.throttle_ms.map_or_else(na, |n| n.to_string()),
        eq.throttle_events.map_or_else(na, |n| n.to_string()),
        eq.package_w.map_or_else(na, |w| format!("{w:.1} W")),
        eq.pl1_w.map_or_else(na, |w| format!("{w:.0} W")),
        verdict
    );
    if label == "reference" && !eq.ok() {
        eprintln!("  note: the CPU is thermally limited even at full fan speed");
    }
}

/// Idle (no load): cool down to `target` at full fan, then let a slow PI loop hold the
/// board there for 4 minutes. The average duty of the last 2 minutes is what the
/// curve needs at `target`. `None` if the target is not reachable in idle.
fn comfort_hold(
    ec: &CrosEc,
    sampler: &Sampler,
    board: &[usize],
    target: f64,
) -> Result<Option<(f64, u8)>> {
    ec.send(EcCommand::SetFanDuty(100))?;
    let mut reached = false;
    for _ in 0..300 {
        let (row, _) = sampler.sample()?;
        if max_of(&row, board).is_some_and(|(t, _)| t <= target + 0.5) {
            reached = true;
            break;
        }
    }
    if !reached {
        eprintln!(
            "  {target}°C is not reachable in idle even at full fan; ignoring --comfort-temp"
        );
        return Ok(None);
    }
    let (kp, ki) = (8.0, 0.1); // %/°C, %/(°C·s): slow, the board sensors lag
    let mut integral = 20.0;
    let mut current = 100u8;
    let (mut duties, mut temps) = (Vec::new(), Vec::new());
    for _ in 0..240 {
        let (row, _) = sampler.sample()?;
        let Some((t, _)) = max_of(&row, board) else {
            continue;
        };
        let err = t - target;
        integral = (integral + ki * err).clamp(0.0, 100.0);
        let duty = (kp * err + integral).clamp(0.0, 100.0).round() as u8;
        if duty != current {
            ec.send(EcCommand::SetFanDuty(duty))?;
            current = duty;
        }
        duties.push(duty as f64);
        temps.push(t);
    }
    if duties.len() < 120 {
        bail!("comfort phase: too few board readings");
    }
    let n = duties.len();
    let duty = mean(&duties[n - 120..]).ceil() as u8;
    eprintln!(
        "  holding {:.1}°C needs about {duty}% ({} RPM)",
        mean(&temps[n - 120..]),
        rpm(ec)
    );
    Ok(Some((target, duty)))
}

pub fn run(opts: &Options) -> Result<()> {
    // Everything that can be checked up front is checked before 10-20 minutes of load.
    if let Some(name) = &opts.apply {
        if name.is_empty()
            || !name
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
        {
            bail!("invalid strategy name '{name}' (allowed: a-z, A-Z, 0-9, _ and -)");
        }
        let cfg = Config::load(&opts.config)?;
        if cfg.strategy(name).is_some() {
            eprintln!(
                "[Warning] > strategy '{name}' exists in {} and will be replaced (a backup is kept)",
                opts.config.display()
            );
        }
    }
    if let Some(c) = &opts.load_cmd {
        split_load_cmd(c)?;
    }

    let ec = CrosEc::open()?;
    signals::install();
    let service_paused = match pause_service() {
        Ok(resp) if resp.starts_with("[Error] > ") => {
            let _ = daemon::send_command(&["resume".to_string()]);
            bail!("the running fw-fanctrl service could not hand control to the EC: {resp}");
        }
        Ok(_) => true,
        Err(_) => false, // no service running
    };
    if service_paused {
        eprintln!("fw-fanctrl service paused for the calibration (lease {LEASE_SECS} s, renewed)");
    }
    let result = measure(&ec, opts, service_paused);
    if service_paused {
        if let (Ok(_), Some(_)) = (&result, &opts.apply) {
            let _ = daemon::send_command(&["reload".to_string()]);
        }
        let _ = daemon::send_command(&["resume".to_string()]);
        eprintln!("fw-fanctrl service resumed");
    }
    result
}

fn measure(ec: &CrosEc, opts: &Options, service_paused: bool) -> Result<()> {
    let mut guard = Guard { ec, load: None };
    let started = Instant::now();
    let total = if opts.comfort_temp.is_some() { 6 } else { 5 };
    let step = |n: u8, what: &str| eprintln!("\n[{n}/{total}] {what}");

    // 1. sensors
    step(1, "reading EC sensor information");
    let temps = ec.temperatures()?;
    let infos: Vec<SensorInfo> = temps
        .iter()
        .enumerate()
        .filter(|(_, t)| t.is_some())
        .map(|(i, _)| SensorInfo {
            index: i,
            name: ec
                .sensor_name(i as u8)
                .unwrap_or_else(|_| format!("sensor {i}")),
            thresholds: ec.thresholds(i as u8).ok(),
        })
        .collect();
    if infos.is_empty() {
        bail!("no temperature sensors found");
    }
    for i in &infos {
        let t = i.thresholds.unwrap_or_default();
        let f = |v: Option<i32>| v.map_or("-".to_string(), |v| v.to_string());
        eprintln!(
            "  {:>2} {:<20} high {:>3}  fan_off {:>3}  fan_max {:>3}",
            i.index,
            i.name,
            f(t.high),
            f(t.fan_off),
            f(t.fan_max)
        );
    }
    let sampler = Sampler {
        ec,
        infos: &infos,
        lease: service_paused.then(|| Cell::new(Instant::now())),
    };

    // 2. fan: start from standstill, lowest running duty, duty → RPM
    step(
        2,
        "fan: start from standstill, lowest running duty, duty → RPM",
    );
    // The EC only kick-starts the fan when leaving 0 %, so every start attempt
    // begins at 0 % with the fan at rest.
    let mut start = None;
    for d in [5u8, 7, 10, 12, 15, 20, 25, 30] {
        stop_fan(ec, &sampler)?;
        ec.send(EcCommand::SetFanDuty(d))?;
        let r = wait_rpm(ec, &sampler, |r| r > 0, 8)?;
        eprintln!(
            "  0 → {d:>2}%: {}",
            if r > 0 {
                format!("starts ({r} RPM)")
            } else {
                "stays off".into()
            }
        );
        if r > 0 {
            start = Some(d);
            break;
        }
    }
    let start = start.unwrap_or(30);
    // Once running, it may keep going at lower duties.
    ec.send(EcCommand::SetFanDuty(20))?;
    stable_rpm(ec, &sampler)?;
    let mut lowest_running = 20;
    for d in [15u8, 12, 10, 8, 6, 5, 4, 3] {
        ec.send(EcCommand::SetFanDuty(d))?;
        let r = stable_rpm(ec, &sampler)?;
        eprintln!("  running at {d:>2}%: {r} RPM");
        if r == 0 {
            break;
        }
        lowest_running = d;
    }
    let min_speed = start.max(lowest_running);
    eprintln!(
        "  minSpeed {min_speed}% (starts from 0 % at {start}%, keeps running down to {lowest_running}%)"
    );
    stop_fan(ec, &sampler)?;
    let mut map_duties = vec![min_speed, 15, 20, 25, 30, 40, 50, 60, 80, 100];
    map_duties.retain(|d| *d >= min_speed);
    map_duties.sort_unstable();
    map_duties.dedup();
    for duty in map_duties {
        ec.send(EcCommand::SetFanDuty(duty))?;
        let r = stable_rpm(ec, &sampler)?;
        eprintln!("  {duty:>3}% → {r} RPM");
    }

    // 3. idle baseline
    let test_duty = min_speed.max(20);
    step(3, &format!("idle baseline at {test_duty}% (40 s)"));
    ec.send(EcCommand::SetFanDuty(test_duty))?;
    let mut idle_rows = Vec::new();
    for i in 0..40 {
        let (row, _) = sampler.sample()?;
        if i >= 20 {
            idle_rows.push(row);
        }
    }

    // 4. load step
    step(4, "load step on all CPUs (45 s)");
    if let Some(c) = &opts.load_cmd {
        eprintln!("  additional load: {c}");
    }
    guard.load = Some(Load::start(opts.load_cmd.as_deref())?);
    let mut step_rows = Vec::new();
    for _ in 0..45 {
        let (row, hit) = sampler.sample()?;
        step_rows.push(row);
        if hit {
            eprintln!("  a sensor is near its EC 'high' threshold, cutting the step short");
            break;
        }
    }
    let classes: Vec<(usize, SensorClass)> = infos
        .iter()
        .map(|i| {
            (
                i.index,
                classify(
                    i,
                    &column(&idle_rows, i.index),
                    &column(&step_rows, i.index),
                ),
            )
        })
        .collect();
    for (idx, class) in &classes {
        eprintln!("  sensor {idx}: {class:?}");
    }
    let board: Vec<usize> = classes
        .iter()
        .filter(|(_, c)| *c == SensorClass::Board)
        .map(|(i, _)| *i)
        .collect();
    let spiky: Vec<usize> = classes
        .iter()
        .filter(|(_, c)| matches!(c, SensorClass::Spiky | SensorClass::IgnoredByEc))
        .map(|(i, _)| *i)
        .collect();
    let board = if board.is_empty() {
        eprintln!("  no slow board sensor found, falling back to all sensors");
        infos.iter().map(|i| i.index).collect()
    } else {
        board
    };

    // 5. reference at full fan, then increasing duties under the same load
    step(
        5,
        "sustained load: reference at 100 % fan, then increasing fan speeds (up to 4 min each)",
    );
    let groups = Groups {
        board: &board,
        spiky: &spiky,
    };
    let mut reference = hold(ec, &sampler, &groups, 100, None)?;
    reference.ref_mhz = reference.mhz;
    report("reference", &reference);
    let mut duties: Vec<u8> = vec![min_speed, 20, 30, 40, 55, 75];
    duties.retain(|d| *d >= min_speed && *d < 100);
    duties.sort_unstable();
    duties.dedup();
    let mut anchor = None;
    for duty in duties {
        let eq = hold(ec, &sampler, &groups, duty, reference.mhz)?;
        report("", &eq);
        if eq.ok() {
            anchor = Some(eq);
            break;
        }
    }
    let anchor = match anchor {
        Some(a) => a,
        None => {
            eprintln!(
                "  note: only full fan speed avoids thermal limiting; the curve tops out there"
            );
            reference
        }
    };
    if let Some(load) = guard.load.take() {
        load.stop();
    }

    // 6. optional: idle comfort
    let comfort = match opts.comfort_temp {
        Some(t) => {
            step(
                6,
                &format!("idle comfort: which fan speed holds the board at {t}°C (up to 9 min)"),
            );
            comfort_hold(ec, &sampler, &board, t)?
        }
        None => None,
    };
    drop(guard); // EC auto

    let idle_temp = board
        .iter()
        .map(|&i| mean(&column(&idle_rows, i)))
        .reduce(f64::max)
        .unwrap_or(40.0);
    let input = CurveInput {
        idle_temp,
        min_speed,
        dominant_thresholds: infos
            .iter()
            .find(|i| i.index == anchor.dominant)
            .and_then(|i| i.thresholds),
        anchor,
        sensors: board,
        comfort,
    };
    let strategy = build_strategy(&input, opts.prefer);

    eprintln!(
        "\ncalibration finished in {:.0} min (prefer: {})",
        started.elapsed().as_secs_f64() / 60.0,
        opts.prefer.name()
    );
    println!("{}", serde_json::to_string_pretty(&strategy)?);

    if let Some(name) = &opts.apply {
        apply(&opts.config, name, &strategy)?;
    }
    Ok(())
}

fn apply(path: &Path, name: &str, strategy: &Value) -> Result<()> {
    let raw = fs::read_to_string(path)?;
    let merged = merge_strategy(&raw, name, strategy)?;
    let stamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let backup = path.with_extension(format!("json.bak-{stamp}"));
    fs::copy(path, &backup)?;
    Config::parse(&merged)?.save(path)?;
    eprintln!(
        "saved as strategy '{name}' in {} (backup: {})\nactivate with: fw-fanctrl use {name}   (or set \"defaultStrategy\": \"{name}\")",
        path.display(),
        backup.display()
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn info(fan_off: Option<i32>, fan_max: Option<i32>) -> SensorInfo {
        SensorInfo {
            index: 0,
            name: "s".into(),
            thresholds: Some(Thresholds {
                high: Some(88),
                fan_off,
                fan_max,
                ..Default::default()
            }),
        }
    }

    #[test]
    fn classifies_measured_sensors() {
        // shapes from a Framework 13 Pro (Intel) run, 1 Hz
        let idle = vec![40.0; 20];
        let board: Vec<f64> = (0..45).map(|i| 40.0 + i as f64 * 0.25).collect();
        let peci: Vec<f64> = (0..45)
            .map(|i| if i < 2 { 51.0 } else { 70.0 + i as f64 * 0.2 })
            .collect();
        let battery = vec![31.0; 45];
        assert_eq!(
            classify(&info(Some(40), Some(78)), &idle, &board),
            SensorClass::Board
        );
        assert_eq!(
            classify(&info(Some(40), Some(78)), &[51.0; 20], &peci),
            SensorClass::Spiky
        );
        assert_eq!(
            classify(&info(Some(103), Some(105)), &[51.0; 20], &peci),
            SensorClass::IgnoredByEc
        );
        assert_eq!(
            classify(&info(Some(40), Some(50)), &[31.0; 20], &battery),
            SensorClass::Unresponsive
        );
        let noisy: Vec<f64> = (0..20)
            .map(|i| if i % 2 == 0 { 40.0 } else { 45.0 })
            .collect();
        assert_eq!(
            classify(&info(Some(40), Some(78)), &noisy, &board),
            SensorClass::Spiky
        );
    }

    #[test]
    fn settled_detection() {
        let rising: Vec<f64> = (0..100).map(|i| 40.0 + i as f64 * 0.1).collect();
        assert!(!settled(&rising));
        let flat: Vec<f64> = (0..100).map(|i| 60.0 + (i % 3) as f64 * 0.1).collect();
        assert!(settled(&flat));
        assert!(!settled(&flat[..50]));
    }

    fn eq(duty: u8, temp: f64) -> Equilibrium {
        Equilibrium {
            duty,
            rpm: 0,
            board_temp: temp,
            dominant: 1,
            spiky_max: Some(96.0),
            throttle_ms: Some(0),
            throttle_events: Some(0),
            package_w: Some(20.0),
            pl1_w: Some(28.0),
            mhz: None,
            ref_mhz: None,
            guard_hit: false,
        }
    }

    fn input() -> CurveInput {
        CurveInput {
            idle_temp: 40.2,
            min_speed: 8,
            anchor: eq(30, 66.7),
            dominant_thresholds: Some(Thresholds {
                high: Some(88),
                fan_off: Some(40),
                fan_max: Some(78),
                ..Default::default()
            }),
            sensors: vec![0, 1, 3],
            comfort: None,
        }
    }

    #[test]
    fn quiet_curve_from_measurements() {
        let s = build_strategy(&input(), Prefer::Quiet);
        assert_eq!(
            s["speedCurve"],
            json!([
                {"temp": 0.0, "speed": 0}, {"temp": 44.0, "speed": 0}, {"temp": 46.0, "speed": 8},
                {"temp": 66.0, "speed": 30}, {"temp": 72.0, "speed": 53}, {"temp": 78.0, "speed": 100}
            ])
        );
        assert_eq!(s["criticalTemp"], 80.0);
        assert_eq!(s["sensors"], json!([0, 1, 3]));
        // the generated strategy must be a valid config strategy
        let cfg = json!({"defaultStrategy": "x", "strategies": {"x": s}});
        Config::parse(&cfg.to_string()).unwrap();
    }

    #[test]
    fn prefer_shifts_curve_up() {
        let q = build_strategy(&input(), Prefer::Quiet);
        let c = build_strategy(&input(), Prefer::Cool);
        let anchor = |v: &Value| v["speedCurve"][3]["speed"].as_u64().unwrap();
        assert!(anchor(&c) > anchor(&build_strategy(&input(), Prefer::Balanced)));
        assert!(anchor(&build_strategy(&input(), Prefer::Balanced)) > anchor(&q));
        assert!(c["rampUpRate"].as_f64() > q["rampUpRate"].as_f64());
        Config::parse(&json!({"defaultStrategy": "x", "strategies": {"x": c}}).to_string())
            .unwrap();
    }

    #[test]
    fn degenerate_inputs_still_give_valid_curves() {
        let mut i = input();
        i.idle_temp = 64.0; // idle close to the anchor
        i.dominant_thresholds = None;
        i.anchor = eq(100, 66.0);
        let s = build_strategy(&i, Prefer::Cool);
        Config::parse(&json!({"defaultStrategy": "x", "strategies": {"x": s}}).to_string())
            .unwrap();
    }

    #[test]
    fn throttle_judgement_combines_all_signals() {
        // cool package, no counters moving
        let mut e = eq(30, 60.0);
        e.spiky_max = Some(85.0);
        assert!(e.ok());
        // each signal on its own marks the duty as thermally limited
        let mut t = e.clone();
        t.throttle_ms = Some(601);
        assert!(!t.ok());
        let mut t = e.clone();
        t.throttle_events = Some(1);
        assert!(!t.ok());
        // the case from the Framework 13 Pro: counters static, package at 96 °C,
        // power below PL1 → held back by temperature
        let mut t = e.clone();
        t.spiky_max = Some(96.0);
        assert!(!t.ok());
        // same temperature at PL1: power-limited, more fan would not help
        t.package_w = Some(27.0);
        assert!(t.ok());
        // no RAPL, no counters: near-TjMax counts as throttling
        let mut a = eq(30, 60.0);
        (a.throttle_ms, a.throttle_events, a.package_w, a.pl1_w) = (None, None, None, None);
        assert!(!a.ok());
        a.spiky_max = Some(89.0);
        assert!(a.ok());
        a.guard_hit = true;
        assert!(!a.ok());
    }

    #[test]
    fn frequency_decides_when_available() {
        // first calibration run on the Framework 13 Pro: same clocks at every duty, the
        // old "hot and below PL1" rule wrongly rejected 20 % and 30 %
        let run = |duty, package, w, mhz| Equilibrium {
            duty,
            spiky_max: Some(package),
            package_w: Some(w),
            pl1_w: Some(35.0),
            mhz: Some(mhz),
            ref_mhz: Some(3090.0),
            ..eq(duty, 61.0)
        };
        assert!(run(20, 92.0, 25.6, 3005.0).ok()); // 97 % of reference (noise is ~±1.5 %)
        assert!(run(30, 90.0, 25.3, 2997.0).ok());
        assert!(!run(20, 99.0, 20.0, 2700.0).ok()); // lost 13 % → thermally limited
        let mut t = run(20, 92.0, 25.6, 3005.0);
        t.throttle_events = Some(2); // counters still veto
        assert!(!t.ok());
    }

    #[test]
    fn comfort_moves_fan_on_point() {
        let mut i = input();
        i.comfort = Some((40.0, 22));
        let s = build_strategy(&i, Prefer::Quiet);
        let curve = &s["speedCurve"];
        assert_eq!(curve[1], json!({"temp": 38.0, "speed": 0}));
        assert_eq!(curve[2], json!({"temp": 40.0, "speed": 22}));
        Config::parse(&json!({"defaultStrategy": "x", "strategies": {"x": s}}).to_string())
            .unwrap();
        // a duty below what starts the fan is raised to the floor
        i.comfort = Some((40.0, 3));
        assert_eq!(
            build_strategy(&i, Prefer::Quiet)["speedCurve"][2]["speed"],
            8
        );
        // comfort too close to the anchor is ignored
        i.comfort = Some((65.0, 25));
        assert_eq!(
            build_strategy(&i, Prefer::Quiet)["speedCurve"][1]["temp"],
            44.0
        );
    }

    #[test]
    fn burn_runs_and_stops() {
        let stop = Arc::new(AtomicBool::new(false));
        let s2 = Arc::clone(&stop);
        let h = std::thread::spawn(move || burn(&s2, 1));
        std::thread::sleep(Duration::from_millis(50));
        stop.store(true, Ordering::Relaxed);
        h.join().unwrap();
    }

    #[test]
    fn merge_validates_and_keeps_existing() {
        let raw = include_str!("../tests/upstream_config.json");
        let s = build_strategy(&input(), Prefer::Quiet);
        let merged = merge_strategy(raw, "calibrated", &s).unwrap();
        let c = Config::parse(&merged).unwrap();
        assert!(c.strategy("lazy").is_some());
        assert_eq!(
            c.strategy("calibrated").unwrap().sensors,
            Some(vec![0, 1, 3])
        );
        assert!(merge_strategy(raw, "bad", &json!({"speedCurve": []})).is_err());
    }
}
