//! `print all --watch`: repeat `print all` like watch(1) and report CPU throttling.
//!
//! This runs in the client, so the service needs no extra wakeups and does not expose
//! the root-only RAPL energy counter through its world-accessible socket. Without root,
//! the power reading is simply missing.

use std::fs;
use std::io::{IsTerminal, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use serde_json::{Value, json};

use crate::calibrate::{CpuSample, POWER_LIMIT_SHARE, delta, read_u64, watts};
use crate::daemon;
use crate::error::Result;
use crate::output::{OutputFormat, Reply, to_py_json};

const CLEAR_SCREEN: &str = "\x1b[H\x1b[2J";
/// Time before the first frame, so that it can already show the package power.
const FIRST_SAMPLE_DELAY: Duration = Duration::from_millis(250);
/// Hysteresis: a power limit episode starts at `POWER_LIMIT_SHARE` of PL1 and only
/// ends below this share, so power hovering around the threshold is one episode.
const POWER_LIMIT_EXIT_SHARE: f64 = 0.8;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Limit {
    Power,
    Thermal,
}

impl Limit {
    fn name(self) -> &'static str {
        match self {
            Limit::Power => "power limit",
            Limit::Thermal => "thermal",
        }
    }
    fn key(self) -> &'static str {
        match self {
            Limit::Power => "power",
            Limit::Thermal => "thermal",
        }
    }
}

/// The CPU package temperature and the limit the CPU regulates to, in °C.
#[derive(Debug, Clone, Copy, PartialEq)]
struct PackageTemp {
    temp: f64,
    limit: Option<f64>,
}

/// coretemp's package sensor (Intel only).
struct PackageSensor {
    input: PathBuf,
    limit: Option<f64>,
}

impl PackageSensor {
    fn find() -> Option<PackageSensor> {
        let is_coretemp = |dir: &PathBuf| {
            fs::read_to_string(dir.join("name")).is_ok_and(|n| n.trim() == "coretemp")
        };
        let dir = fs::read_dir("/sys/class/hwmon")
            .ok()?
            .flatten()
            .map(|e| e.path())
            .find(is_coretemp)?;
        let base = fs::read_dir(&dir).ok()?.flatten().find_map(|e| {
            let name = e.file_name().into_string().ok()?;
            let base = name.strip_suffix("_label")?.to_string();
            let label = fs::read_to_string(e.path()).ok()?;
            (label.trim() == "Package id 0").then_some(base)
        })?;
        let millidegrees = |suffix: &str| {
            let path = dir.join(format!("{base}_{suffix}"));
            read_u64(path.to_str()?)
                .filter(|v| *v > 0)
                .map(|v| v as f64 / 1000.0)
        };
        Some(PackageSensor {
            input: dir.join(format!("{base}_input")),
            // `max` honours a TCC offset, `crit` is TjMax itself
            limit: millidegrees("max").or_else(|| millidegrees("crit")),
        })
    }

    fn read(&self) -> Option<PackageTemp> {
        let raw = read_u64(self.input.to_str()?)?;
        Some(PackageTemp {
            temp: raw as f64 / 1000.0,
            limit: self.limit,
        })
    }
}

#[derive(Debug, Clone, PartialEq)]
struct Report {
    limit: Option<Limit>,
    /// The latest limit of an earlier interval and how long ago it ended.
    last: Option<(Limit, Duration)>,
    watch_ms: Option<u64>,
    watch_events: Option<u64>,
    boot_ms: Option<u64>,
    boot_events: Option<u64>,
    /// Time and episodes at the power limit during this watch (`None` = not measurable).
    power_limited: Option<PowerLimited>,
    package: Option<PackageTemp>,
    mhz: Option<f64>,
    power_w: Option<f64>,
    pl1_w: Option<f64>,
    /// Highest PL1 seen during this watch, if the current one is lower.
    pl1_lowered_from: Option<f64>,
}

/// Counted by the watch itself, one whole interval at a time: the kernel has no such
/// counter by default (`package_power_limit_count` needs the `int_pln_enable` boot option).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct PowerLimited {
    ms: u64,
    events: u64,
}

/// Compares consecutive samples.
struct Tracker {
    start: CpuSample,
    prev: CpuSample,
    highest_pl1: Option<f64>,
    last: Option<(Limit, Instant)>,
    power_limited: Option<PowerLimited>,
    was_at_pl1: bool,
}

impl Tracker {
    fn new(start: CpuSample) -> Tracker {
        Tracker {
            start,
            prev: start,
            highest_pl1: start.pl1_w,
            last: None,
            power_limited: None,
            was_at_pl1: false,
        }
    }

    fn update(&mut self, cur: CpuSample, package: Option<PackageTemp>) -> Report {
        let advanced = |a: Option<u64>, b: Option<u64>| delta(a, b).is_some_and(|d| d > 0);
        let counters = advanced(self.prev.throttle_ms, cur.throttle_ms)
            || advanced(self.prev.throttle_count, cur.throttle_count);
        // The counters only advance when an episode starts or ends, so a package held
        // at its limit is the only sign of an episode that is still going on.
        let at_limit = package.is_some_and(|p| p.limit.is_some_and(|l| p.temp >= l));
        let power_w = watts(&self.prev, &cur);
        let at_pl1 = match (power_w, cur.pl1_w) {
            (Some(p), Some(l)) if l > 0.0 => {
                let share = if self.was_at_pl1 {
                    POWER_LIMIT_EXIT_SHARE
                } else {
                    POWER_LIMIT_SHARE
                };
                Some(p >= share * l)
            }
            _ => None,
        };
        let limit = if counters || at_limit {
            Some(Limit::Thermal)
        } else if at_pl1 == Some(true) {
            Some(Limit::Power)
        } else {
            None
        };
        // Independent of `limit`: both limits can apply in the same interval.
        if let Some(limited) = at_pl1 {
            let total = self.power_limited.get_or_insert_default();
            if limited {
                total.ms += cur.at.duration_since(self.prev.at).as_millis() as u64;
                total.events += u64::from(!self.was_at_pl1);
            }
            self.was_at_pl1 = limited;
        }

        self.highest_pl1 = match (self.highest_pl1, cur.pl1_w) {
            (Some(a), Some(b)) => Some(a.max(b)),
            (a, b) => a.or(b),
        };
        let report = Report {
            limit,
            last: self
                .last
                .filter(|_| limit.is_none())
                .map(|(l, at)| (l, cur.at.duration_since(at))),
            watch_ms: delta(self.start.throttle_ms, cur.throttle_ms),
            watch_events: delta(self.start.throttle_count, cur.throttle_count),
            boot_ms: cur.throttle_ms,
            boot_events: cur.throttle_count,
            power_limited: self.power_limited,
            package,
            mhz: cur.mhz,
            power_w,
            pl1_w: cur.pl1_w,
            pl1_lowered_from: self
                .highest_pl1
                .filter(|h| cur.pl1_w.is_some_and(|p| p < *h)),
        };
        if let Some(l) = limit {
            self.last = Some((l, cur.at));
        }
        self.prev = cur;
        report
    }
}

fn clock(d: Duration) -> String {
    let s = d.as_secs();
    if s >= 3600 {
        format!("{:02}:{:02}:{:02}", s / 3600, s / 60 % 60, s % 60)
    } else {
        format!("{:02}:{:02}", s / 60, s % 60)
    }
}

fn counters(ms: Option<u64>, events: Option<u64>, label: &str) -> Option<String> {
    Some(format!("{} ms, {} events {label}", ms?, events?))
}

impl Report {
    fn natural(&self) -> String {
        let throttled = match (self.limit, self.last) {
            (Some(l), _) => format!("YES, {}", l.name()),
            (None, Some((l, ago))) => format!("no (last: {}, {} ago)", l.name(), clock(ago)),
            (None, None) => "no".to_string(),
        };
        let thermal = [
            counters(self.watch_ms, self.watch_events, "in this watch"),
            counters(self.boot_ms, self.boot_events, "since boot"),
        ]
        .into_iter()
        .flatten()
        .collect::<Vec<_>>()
        .join("; ");
        let thermal = if thermal.is_empty() {
            "n/a".to_string()
        } else {
            thermal
        };
        let power_limited = self.power_limited.map_or("n/a".to_string(), |p| {
            format!("{} ms, {} events in this watch", p.ms, p.events)
        });
        let package = self.package.map_or("n/a".to_string(), |p| match p.limit {
            Some(l) => format!("{:.0}°C (limit {l:.0}°C)", p.temp),
            None => format!("{:.0}°C", p.temp),
        });
        let mhz = self
            .mhz
            .map_or("n/a".to_string(), |m| format!("{m:.0} MHz"));
        let power = self
            .power_w
            .map_or("n/a (RAPL not readable, try sudo)".to_string(), |p| {
                format!("{p:.1} W")
            });
        let pl1 = self
            .pl1_w
            .map_or("n/a".to_string(), |l| format!("{l:.1} W"));
        let lowered = self
            .pl1_lowered_from
            .map_or(String::new(), |h| format!(" (lowered from {h:.1} W)"));
        format!(
            "Throttled: {throttled}\nThermalThrottle: {thermal}\n\
             PowerLimitThrottle: {power_limited}\nCpuPackage: {package}\n\
             CpuFrequency: {mhz}\nPackagePower: {power}\nPL1: {pl1}{lowered}\n"
        )
    }

    fn json(&self) -> Value {
        json!({
            "state": self.limit.map_or("none", Limit::key),
            "last": self.last.map(|(l, ago)| json!({
                "state": l.key(),
                "secondsAgo": ago.as_secs(),
            })),
            "thermalWatchMs": self.watch_ms,
            "thermalWatchEvents": self.watch_events,
            "thermalBootMs": self.boot_ms,
            "thermalBootEvents": self.boot_events,
            "powerLimitWatchMs": self.power_limited.map(|p| p.ms),
            "powerLimitWatchEvents": self.power_limited.map(|p| p.events),
            "packageTemperature": self.package.map(|p| p.temp),
            "packageTemperatureLimit": self.package.and_then(|p| p.limit),
            "averageMhz": self.mhz,
            "packageWatts": self.power_w,
            "pl1Watts": self.pl1_w,
            "pl1LoweredFromWatts": self.pl1_lowered_from,
        })
    }
}

fn request(fmt: OutputFormat) -> std::result::Result<String, String> {
    let argv: &[&str] = match fmt {
        OutputFormat::Natural => &["print", "all"],
        OutputFormat::Json => &["--output-format", "JSON", "print", "all"],
    };
    let argv: Vec<String> = argv.iter().map(|s| s.to_string()).collect();
    daemon::send_command(&argv).map_err(|e| e.0)
}

/// One JSON document per line: the `print all` reply plus a `throttling` object.
fn json_frame(reply: std::result::Result<String, String>, report: &Report) -> String {
    let mut doc = match reply {
        Ok(text) => serde_json::from_str(&text).unwrap_or_else(|e| {
            Reply::Error(format!("invalid reply from the service: {e}")).json()
        }),
        Err(e) => Reply::Error(e).json(),
    };
    doc["throttling"] = report.json();
    to_py_json(&doc) + "\n"
}

fn natural_frame(
    reply: std::result::Result<String, String>,
    report: &Report,
    interval: u64,
    running: Duration,
) -> String {
    // The service may be restarting: keep watching and show why there is no data.
    let body = reply.unwrap_or_else(|e| Reply::Error(e).natural());
    format!(
        "Every {interval}s, running for {}. Ctrl-C to stop.\n\n{}\n\n{}",
        clock(running),
        body.trim_end(),
        report.natural()
    )
}

/// Runs until interrupted or until nothing reads the output any more.
pub fn run(interval: u64, fmt: OutputFormat) -> Result<()> {
    let interval_d = Duration::from_secs(interval);
    let sensor = PackageSensor::find();
    let started = Instant::now();
    let mut tracker = Tracker::new(CpuSample::now());
    std::thread::sleep(FIRST_SAMPLE_DELAY);

    let mut stdout = std::io::stdout();
    let terminal = stdout.is_terminal();
    let mut first = true;
    loop {
        let frame_start = Instant::now();
        let package = sensor.as_ref().and_then(PackageSensor::read);
        let report = tracker.update(CpuSample::now(), package);
        let reply = request(fmt);
        let frame = match fmt {
            OutputFormat::Json => json_frame(reply, &report),
            OutputFormat::Natural => {
                let frame = natural_frame(reply, &report, interval, started.elapsed());
                if terminal {
                    format!("{CLEAR_SCREEN}{frame}")
                } else if first {
                    frame
                } else {
                    format!("\n{frame}")
                }
            }
        };
        first = false;
        if stdout
            .write_all(frame.as_bytes())
            .and_then(|()| stdout.flush())
            .is_err()
        {
            return Ok(()); // e.g. `... | head`
        }
        std::thread::sleep(interval_d.saturating_sub(frame_start.elapsed()));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(at: Instant, ms: u64, events: u64, energy_j: u64, pl1: f64) -> CpuSample {
        CpuSample {
            at,
            throttle_ms: Some(ms),
            throttle_count: Some(events),
            energy_uj: Some(energy_j * 1_000_000),
            mhz: Some(3000.0),
            pl1_w: Some(pl1),
        }
    }

    fn cool() -> Option<PackageTemp> {
        Some(PackageTemp {
            temp: 88.0,
            limit: Some(100.0),
        })
    }

    #[test]
    fn below_the_limits_is_not_throttled() {
        let t0 = Instant::now();
        let mut t = Tracker::new(sample(t0, 500, 2, 0, 33.0));
        let r = t.update(
            sample(t0 + Duration::from_secs(3), 500, 2, 30, 33.0),
            cool(),
        );
        assert_eq!(r.limit, None);
        assert_eq!(r.power_w, Some(10.0));
        assert_eq!((r.watch_ms, r.watch_events), (Some(0), Some(0)));
        assert_eq!((r.boot_ms, r.boot_events), (Some(500), Some(2)));
        assert!(r.natural().starts_with("Throttled: no\n"));
    }

    #[test]
    fn thermal_counters_and_package_at_limit_count_as_thermal() {
        let t0 = Instant::now();
        let s = Duration::from_secs(3);
        let mut t = Tracker::new(sample(t0, 500, 2, 0, 33.0));
        let r = t.update(sample(t0 + s, 900, 3, 30, 33.0), cool());
        assert_eq!(r.limit, Some(Limit::Thermal));
        assert_eq!((r.watch_ms, r.watch_events), (Some(400), Some(1)));

        // counters silent during an ongoing episode, package at TjMax
        let at_limit = Some(PackageTemp {
            temp: 100.0,
            limit: Some(100.0),
        });
        let r = t.update(sample(t0 + 2 * s, 900, 3, 60, 33.0), at_limit);
        assert_eq!(r.limit, Some(Limit::Thermal));
    }

    #[test]
    fn power_at_pl1_is_power_limited_and_thermal_wins() {
        let t0 = Instant::now();
        let s = Duration::from_secs(1);
        let mut t = Tracker::new(sample(t0, 0, 0, 0, 30.0));
        // 27 W = 90 % of PL1
        let r = t.update(sample(t0 + s, 0, 0, 27, 30.0), cool());
        assert_eq!(r.limit, Some(Limit::Power));
        assert_eq!(
            r.power_limited,
            Some(PowerLimited {
                ms: 1000,
                events: 1
            })
        );
        // still the same episode, although thermal wins the verdict
        let r = t.update(sample(t0 + 2 * s, 100, 1, 54, 30.0), cool());
        assert_eq!(r.limit, Some(Limit::Thermal));
        assert_eq!(
            r.power_limited,
            Some(PowerLimited {
                ms: 2000,
                events: 1
            })
        );
        // below the limit: nothing added
        let r = t.update(sample(t0 + 3 * s, 100, 1, 64, 30.0), cool());
        assert_eq!(
            r.power_limited,
            Some(PowerLimited {
                ms: 2000,
                events: 1
            })
        );
        // a new episode
        let r = t.update(sample(t0 + 4 * s, 100, 1, 94, 30.0), cool());
        assert_eq!(
            r.power_limited,
            Some(PowerLimited {
                ms: 3000,
                events: 2
            })
        );
        assert!(
            r.natural()
                .contains("PowerLimitThrottle: 3000 ms, 2 events in this watch\n")
        );
    }

    #[test]
    fn power_limit_episode_has_hysteresis() {
        let t0 = Instant::now();
        let s = Duration::from_secs(1);
        let mut t = Tracker::new(sample(t0, 0, 0, 0, 30.0));
        let mut energy = 0;
        let mut step = |i: u32, watts: u64| {
            energy += watts;
            t.update(sample(t0 + i * s, 0, 0, energy, 30.0), cool())
        };
        // 26 W = 87 %: not enough to start an episode
        let r = step(1, 26);
        assert_eq!(r.limit, None);
        assert_eq!(r.power_limited, Some(PowerLimited::default()));
        // 27 W = 90 % starts it, 25 W = 83 % keeps it
        step(2, 27);
        let r = step(3, 25);
        assert_eq!(r.limit, Some(Limit::Power));
        assert_eq!(
            r.power_limited,
            Some(PowerLimited {
                ms: 2000,
                events: 1
            })
        );
        // 23 W = 77 % ends it, 26 W does not start a new one, 27 W does
        let r = step(4, 23);
        assert_eq!(r.limit, None);
        let r = step(5, 26);
        assert_eq!(r.limit, None);
        assert_eq!(
            r.power_limited,
            Some(PowerLimited {
                ms: 2000,
                events: 1
            })
        );
        let r = step(6, 27);
        assert_eq!(
            r.power_limited,
            Some(PowerLimited {
                ms: 3000,
                events: 2
            })
        );
    }

    #[test]
    fn missing_counters_and_power_are_not_throttling() {
        let t0 = Instant::now();
        let empty = |at| CpuSample {
            at,
            throttle_ms: None,
            throttle_count: None,
            energy_uj: None,
            mhz: None,
            pl1_w: None,
        };
        let mut t = Tracker::new(empty(t0));
        let r = t.update(empty(t0 + Duration::from_secs(3)), None);
        assert_eq!(r.limit, None);
        assert_eq!(
            r.natural(),
            "Throttled: no\nThermalThrottle: n/a\nPowerLimitThrottle: n/a\n\
             CpuPackage: n/a\nCpuFrequency: n/a\n\
             PackagePower: n/a (RAPL not readable, try sudo)\nPL1: n/a\n"
        );
    }

    #[test]
    fn remembers_the_last_limit_and_a_lowered_pl1() {
        let t0 = Instant::now();
        let s = Duration::from_secs(3);
        let mut t = Tracker::new(sample(t0, 0, 0, 0, 33.0));
        t.update(sample(t0 + s, 100, 1, 30, 33.0), cool());
        let r = t.update(sample(t0 + 2 * s, 100, 1, 60, 30.0), cool());
        assert_eq!(r.limit, None);
        assert_eq!(r.last, Some((Limit::Thermal, s)));
        assert_eq!(r.pl1_lowered_from, Some(33.0));
        let text = r.natural();
        assert!(
            text.contains("Throttled: no (last: thermal, 00:03 ago)"),
            "{text}"
        );
        assert!(text.contains("PL1: 30.0 W (lowered from 33.0 W)"), "{text}");
    }

    #[test]
    fn json_frame_adds_throttling_and_survives_errors() {
        let t0 = Instant::now();
        let mut t = Tracker::new(sample(t0, 0, 0, 0, 33.0));
        let r = t.update(sample(t0 + Duration::from_secs(1), 0, 0, 10, 33.0), cool());

        let ok = json_frame(Ok(r#"{"status": "success", "speed": 42}"#.into()), &r);
        let doc: Value = serde_json::from_str(&ok).unwrap();
        assert_eq!(doc["speed"], 42);
        assert_eq!(doc["throttling"]["state"], "none");
        assert_eq!(doc["throttling"]["packageTemperature"], 88.0);

        let err = json_frame(Err("no service".into()), &r);
        let doc: Value = serde_json::from_str(&err).unwrap();
        assert_eq!(doc["status"], "error");
        assert_eq!(doc["throttling"]["state"], "none");
    }

    #[test]
    fn frame_shows_service_errors_and_formats_time() {
        let t0 = Instant::now();
        let mut t = Tracker::new(sample(t0, 0, 0, 0, 33.0));
        let r = t.update(sample(t0 + Duration::from_secs(1), 0, 0, 10, 33.0), cool());
        let f = natural_frame(Err("no service".into()), &r, 3, Duration::from_secs(75));
        assert!(f.starts_with("Every 3s, running for 01:15."), "{f}");
        assert!(f.contains("[Error] > An error occurred: no service"), "{f}");
        assert_eq!(clock(Duration::from_secs(3725)), "01:02:05");
    }
}
