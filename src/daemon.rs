//! The service: control loop, fail-safe handling and the command socket.

use std::fs;
use std::io::{ErrorKind, Read, Write};
use std::os::fd::AsRawFd;
use std::os::unix::fs::PermissionsExt;
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use crate::cli::{self, Command, PrintSelection};
use crate::config::{Config, Strategy};
use crate::control::{Controller, Tick};
use crate::ec::Fan;
use crate::error::Result;
use crate::hw::{self, Hardware};
use crate::output::{OutputFormat, Reply, Status};
use crate::signals::{self, Signal};
use crate::systemd;

pub const SOCKET_PATH: &str = "/run/fw-fanctrl/.fw-fanctrl.commands.sock";
/// Startup error when another instance owns the socket; main maps it to exit code 3
/// (the unit has `RestartPreventExitStatus=3`, so systemd does not keep retrying).
pub const ALREADY_RUNNING: &str = "another fw-fanctrl service is already running";

/// The command socket; `FW_FANCTRL_SOCKET` overrides it (testing without root).
pub fn socket_path() -> PathBuf {
    std::env::var_os("FW_FANCTRL_SOCKET").map_or_else(|| PathBuf::from(SOCKET_PATH), PathBuf::from)
}

/// Consecutive hardware errors before handing control back to the EC.
const MAX_FAILURES: u32 = 3;
/// Re-send an unchanged duty this often, in case something reset the EC to auto mode.
const REFRESH_INTERVAL: Duration = Duration::from_secs(30);
const TICK: Duration = Duration::from_secs(1);
/// Emergency margin below a sensor's EC "high" threshold.
const EMERGENCY_MARGIN: f64 = 3.0;

pub struct Service {
    config: Config,
    config_path: PathBuf,
    overwritten: Option<String>,
    active: bool,
    controller: Controller,
    hw: Box<dyn Hardware>,
    on_ac: bool,
    last: Tick,
    fans: Vec<Fan>,
    sensor_temps: Vec<Option<f64>>,
    failures: u32,
    failsafe: bool,
    last_write: Option<(u8, Instant)>,
    force_write: bool,
    /// Handing control back to the EC failed; retried every tick until it works.
    need_auto: bool,
    /// Pause lease: resume automatically at this point unless renewed.
    pause_until: Option<Instant>,
    /// EC "high" threshold per sensor index, read once the sensors are known.
    limits: Option<Vec<Option<f64>>>,
}

impl Service {
    pub fn new(
        config: Config,
        config_path: PathBuf,
        hw: Box<dyn Hardware>,
        strategy: Option<&str>,
    ) -> Service {
        let mut svc = Service {
            config,
            config_path,
            overwritten: None,
            active: true,
            controller: Controller::new(),
            hw,
            on_ac: true,
            last: Tick::default(),
            fans: Vec::new(),
            sensor_temps: Vec::new(),
            failures: 0,
            failsafe: false,
            last_write: None,
            force_write: true,
            need_auto: false,
            pause_until: None,
            limits: None,
        };
        if let Ok(ac) = svc.hw.on_ac() {
            svc.on_ac = ac;
        }
        match strategy {
            Some(s) if !s.is_empty() && svc.config.strategy(s).is_some() => {
                svc.overwritten = Some(s.into())
            }
            Some(s) if !s.is_empty() => {
                eprintln!(
                    "[Warning] > unknown strategy '{s}' in {}, using the default strategy '{}' (available: {})",
                    svc.config_path.display(),
                    svc.config.default_strategy,
                    svc.config.names().join(", ")
                )
            }
            _ => {}
        }
        svc
    }

    pub fn current_strategy(&self) -> &Strategy {
        if let Some(s) = self
            .overwritten
            .as_deref()
            .and_then(|n| self.config.strategy(n))
        {
            return s;
        }
        if self.on_ac {
            self.config.default()
        } else {
            self.config.discharging()
        }
    }

    fn is_default(&self) -> bool {
        self.overwritten.is_none()
    }

    /// Hand control to the EC; on failure remember to retry every tick.
    fn hand_to_ec(&mut self) -> Result<()> {
        match self.hw.auto_fan() {
            Ok(()) => {
                self.need_auto = false;
                Ok(())
            }
            Err(e) => {
                eprintln!("[Error] > restoring EC fan control failed, will retry: {e}");
                self.need_auto = true;
                Err(e)
            }
        }
    }

    fn fail(&mut self, what: String) {
        self.failures += 1;
        eprintln!("[Error] > {what} ({}/{MAX_FAILURES})", self.failures);
        if self.failures >= MAX_FAILURES && !self.failsafe {
            eprintln!("[Error] > entering fail-safe: handing fan control back to the EC");
            let _ = self.hand_to_ec();
            self.failsafe = true;
            self.force_write = true;
            // the readings that come back after recovery may be very different
            self.controller.reset();
        }
    }

    fn pause(&mut self, lease: Option<u64>) -> Result<()> {
        self.pause_until = lease.map(|s| Instant::now() + Duration::from_secs(s));
        if !self.active && !self.need_auto {
            // already paused and the EC is in charge: only the lease was renewed.
            // Re-sending auto mode would disturb e.g. a running calibration.
            return Ok(());
        }
        self.active = false;
        self.hand_to_ec()
    }

    fn resume(&mut self) {
        self.active = true;
        self.pause_until = None;
        self.need_auto = false;
        self.force_write = true;
        // start from the curve target for the current readings, not from the
        // state before the pause (the machine may have heated up meanwhile)
        self.controller.reset();
    }

    /// One control step (once per second).
    pub fn tick(&mut self) {
        if self.need_auto {
            let _ = self.hand_to_ec();
            if !self.need_auto {
                eprintln!("EC automatic fan control restored");
            }
        }
        if !self.active {
            if self.pause_until.is_some_and(|t| Instant::now() >= t) {
                eprintln!("pause lease expired, resuming fan control");
                self.resume();
            } else {
                return;
            }
        }
        // AC state first: it decides the strategy, which decides the sensors.
        if let Ok(ac) = self.hw.on_ac() {
            self.on_ac = ac;
        }
        let strategy = self.current_strategy().clone();
        let temps = match self.hw.temperatures() {
            Ok(t) => t,
            Err(e) => return self.fail(format!("reading temperatures failed: {e}")),
        };
        self.sensor_temps = temps.clone();
        let temp = match hw::select_max(&temps, strategy.sensors.as_deref()) {
            Ok(t) => t,
            Err(e) => return self.fail(format!("reading temperatures failed: {e}")),
        };
        self.fans = self.hw.fans().unwrap_or_default();

        // Global guard, independent of the strategy's sensor selection.
        let n = temps.len();
        let limits = self
            .limits
            .get_or_insert_with(|| self.hw.high_thresholds(n));
        let emergency = near_limit(&temps, limits);

        let tick = self
            .controller
            .tick(&strategy, temp, TICK.as_secs_f64(), emergency);
        if tick.emergency && !self.last.emergency {
            eprintln!("[Warning] > a sensor is near its EC 'high' threshold: fan to 100 %");
        }
        self.last = tick;

        let due = self.force_write
            || self
                .last_write
                .is_none_or(|(v, at)| v != tick.output || at.elapsed() >= REFRESH_INTERVAL);
        if due {
            if let Err(e) = self.hw.set_speed(tick.output) {
                return self.fail(format!("setting fan speed failed: {e}"));
            }
            self.last_write = Some((tick.output, Instant::now()));
            self.force_write = false;
        }
        if self.failsafe {
            eprintln!("hardware access recovered, leaving fail-safe");
            self.failsafe = false;
        }
        self.failures = 0;
    }

    pub fn status(&self) -> Status {
        Status {
            strategy: self.current_strategy().name.clone(),
            default: self.is_default(),
            speed: self.controller.output().unwrap_or(0),
            target_speed: self.last.target,
            temperature: self.last.temperature,
            moving_average_temperature: self.last.moving_average,
            effective_temperature: self.last.effective,
            active: self.active,
            failsafe: self.failsafe,
            fan_rpm: self
                .fans
                .iter()
                .map(|f| match f {
                    Fan::Rpm(r) => Some(*r),
                    Fan::Stalled => None,
                })
                .collect(),
            sensors: self.sensor_temps.clone(),
            config_path: self.config_path.display().to_string(),
            config: self.config.data.clone(),
        }
    }

    fn reload(&mut self) -> Result<()> {
        self.config = Config::load(&self.config_path)?;
        if self
            .overwritten
            .as_deref()
            .is_some_and(|n| self.config.strategy(n).is_none())
        {
            self.overwritten = None;
        }
        self.controller.reset_timecount();
        Ok(())
    }

    pub fn handle(&mut self, cmd: &Command) -> Reply {
        let current = |s: &Self| (s.current_strategy().name.clone(), s.is_default());
        match cmd {
            Command::Reset => {
                self.overwritten = None;
                self.controller.reset_timecount();
                let (strategy, default) = current(self);
                Reply::StrategyReset { strategy, default }
            }
            Command::Use(name) if name == "defaultStrategy" => self.handle(&Command::Reset),
            Command::Use(name) => {
                if self.config.strategy(name).is_none() {
                    return Reply::Error(format!("The specified strategy is invalid: {name}"));
                }
                self.overwritten = Some(name.clone());
                self.controller.reset_timecount();
                let (strategy, default) = current(self);
                Reply::StrategyChange { strategy, default }
            }
            Command::Reload => match self.reload() {
                Ok(()) => {
                    let (strategy, default) = current(self);
                    Reply::Reload { strategy, default }
                }
                Err(e) => Reply::Error(e.0),
            },
            Command::Pause { lease } => match self.pause(*lease) {
                Ok(()) => Reply::Pause,
                Err(e) => Reply::Error(format!(
                    "restoring EC fan control failed (retrying every second): {e}"
                )),
            },
            Command::Resume => {
                self.resume();
                let (strategy, default) = current(self);
                Reply::Resume { strategy, default }
            }
            Command::Print(sel) => match sel {
                PrintSelection::All => Reply::Status(Box::new(self.status())),
                PrintSelection::Active => Reply::Active(self.active),
                PrintSelection::Current => {
                    let (strategy, default) = current(self);
                    Reply::Current { strategy, default }
                }
                PrintSelection::List => Reply::List(self.config.names()),
                PrintSelection::Speed => Reply::Speed(self.controller.output().unwrap_or(0)),
            },
            Command::SetConfig(raw) => {
                let config = match Config::parse(raw) {
                    Ok(c) => c,
                    Err(e) => return Reply::Error(e.0),
                };
                if let Err(e) = config.save(&self.config_path) {
                    return Reply::Error(format!("saving configuration failed: {e}"));
                }
                self.config = config;
                if self
                    .overwritten
                    .as_deref()
                    .is_some_and(|n| self.config.strategy(n).is_none())
                {
                    self.overwritten = None;
                }
                self.controller.reset_timecount();
                let (strategy, default) = current(self);
                Reply::SetConfig {
                    strategy,
                    default,
                    path: self.config_path.display().to_string(),
                    config: self.config.data.clone(),
                }
            }
            other => Reply::Error(format!("command not supported by the service: {other:?}")),
        }
    }

    pub fn shutdown(&mut self) {
        for attempt in 1..=5 {
            match self.hw.auto_fan() {
                Ok(()) => return,
                Err(e) => eprintln!("[Error] > restoring EC fan control failed ({attempt}/5): {e}"),
            }
            std::thread::sleep(Duration::from_millis(200));
        }
        // ExecStopPost=fw-fanctrl restore-auto is the next line of defence
    }
}

/// Any sensor within `EMERGENCY_MARGIN` of its EC "high" threshold.
fn near_limit(temps: &[Option<f64>], limits: &[Option<f64>]) -> bool {
    temps
        .iter()
        .zip(limits)
        .any(|(t, l)| matches!((t, l), (Some(t), Some(l)) if *t >= l - EMERGENCY_MARGIN))
}

fn lock(svc: &Mutex<Service>) -> MutexGuard<'_, Service> {
    svc.lock().unwrap_or_else(|p| p.into_inner())
}

// --- socket

fn bind_socket(path: &Path) -> Result<UnixListener> {
    if let Some(dir) = path.parent() {
        fs::create_dir_all(dir)?;
    }
    if path.exists() {
        if UnixStream::connect(path).is_ok() {
            return Err(ALREADY_RUNNING.into());
        }
        fs::remove_file(path)?;
    }
    let listener = UnixListener::bind(path)?;
    // upstream makes the socket world-accessible so unprivileged users can switch strategies
    fs::set_permissions(path, fs::Permissions::from_mode(0o777))?;
    Ok(listener)
}

/// Read one request. Our client half-closes after sending; upstream's Python
/// client does not, so we also stop after a short idle period.
fn read_request(stream: &mut UnixStream) -> std::io::Result<String> {
    const LIMIT: usize = 1 << 20;
    let mut data = Vec::new();
    let mut buf = [0u8; 8192];
    stream.set_read_timeout(Some(Duration::from_secs(2)))?;
    loop {
        match stream.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => {
                data.extend_from_slice(&buf[..n]);
                if data.len() > LIMIT {
                    return Err(std::io::Error::other("request too large"));
                }
                stream.set_read_timeout(Some(Duration::from_millis(100)))?;
            }
            Err(e)
                if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut)
                    && !data.is_empty() =>
            {
                break;
            }
            Err(e) => return Err(e),
        }
    }
    Ok(String::from_utf8_lossy(&data).into_owned())
}

/// Uid of the connected client (SO_PEERCRED).
fn peer_uid(stream: &UnixStream) -> Option<u32> {
    let mut cred = libc::ucred {
        pid: 0,
        uid: 0,
        gid: 0,
    };
    let mut len = size_of::<libc::ucred>() as libc::socklen_t;
    // SAFETY: getsockopt writes at most `len` bytes into `cred`.
    let ret = unsafe {
        libc::getsockopt(
            stream.as_raw_fd(),
            libc::SOL_SOCKET,
            libc::SO_PEERCRED,
            (&mut cred as *mut libc::ucred).cast(),
            &mut len,
        )
    };
    (ret == 0).then_some(cred.uid)
}

/// `uid`: the client; the socket is world-accessible (like upstream), but only root
/// may rewrite the configuration file.
pub fn answer(svc: &Mutex<Service>, request: &str, uid: Option<u32>) -> String {
    let parsed = shlex::split(request)
        .ok_or_else(|| "invalid quoting".into())
        .and_then(|argv| cli::parse(&argv, true));
    match parsed {
        Ok(args) if args.command == Command::Help => cli::USAGE.to_string(),
        Ok(args) if matches!(args.command, Command::SetConfig(_)) && uid != Some(0) => {
            Reply::Error("set_config requires root".into()).render(args.output_format)
        }
        Ok(args) => lock(svc).handle(&args.command).render(args.output_format),
        Err(e) => {
            let fmt = if request.contains("JSON") {
                OutputFormat::Json
            } else {
                OutputFormat::Natural
            };
            Reply::Error(format!(
                "An error occurred while treating a socket command: {e}"
            ))
            .render(fmt)
        }
    }
}

fn serve(listener: UnixListener, svc: Arc<Mutex<Service>>) {
    for stream in listener.incoming() {
        let Ok(mut stream) = stream else { continue };
        let uid = peer_uid(&stream);
        let response = match read_request(&mut stream) {
            Ok(req) => answer(&svc, &req, uid),
            Err(e) => Reply::Error(format!("reading command failed: {e}")).natural(),
        };
        let _ = stream.write_all(response.as_bytes());
        let _ = stream.shutdown(std::net::Shutdown::Both);
    }
}

pub fn send_command(argv: &[String]) -> Result<String> {
    let path = socket_path();
    let mut stream = UnixStream::connect(&path).map_err(|e| {
        format!(
            "cannot connect to {}: {e} (is the fw-fanctrl service running?)",
            path.display()
        )
    })?;
    let request = shlex::try_join(argv.iter().map(String::as_str))
        .map_err(|e| format!("cannot encode command: {e}"))?;
    // don't block e.g. the suspend hook forever if the service hangs
    stream.set_read_timeout(Some(Duration::from_secs(10)))?;
    stream.write_all(request.as_bytes())?;
    stream.shutdown(std::net::Shutdown::Write)?;
    let mut response = String::new();
    stream.read_to_string(&mut response)?;
    Ok(response)
}

// --- main loop

pub fn run(svc: Service, output_format: OutputFormat, silent: bool) -> Result<()> {
    let svc = Arc::new(Mutex::new(svc));
    let sock_path = socket_path();
    let listener = bind_socket(&sock_path)?;
    {
        let svc = Arc::clone(&svc);
        std::thread::Builder::new()
            .name("socket".into())
            .spawn(move || serve(listener, svc))?;
    }
    signals::install();
    systemd::notify("READY=1");

    let mut next = Instant::now();
    loop {
        match signals::take() {
            Some(Signal::Stop) => break,
            Some(Signal::Reload) => {
                let mut s = lock(&svc);
                match s.reload() {
                    Ok(()) => eprintln!("configuration reloaded"),
                    Err(e) => {
                        eprintln!("[Error] > reload failed, keeping previous configuration: {e}")
                    }
                }
            }
            None => {}
        }

        let now = Instant::now();
        if now >= next {
            let mut s = lock(&svc);
            s.tick();
            let status = s.status();
            drop(s);
            if !silent && status.active {
                println!(
                    "{}",
                    Reply::Status(Box::new(status.clone())).render(output_format)
                );
            }
            systemd::notify(&format!(
                "WATCHDOG=1\nSTATUS={}: {:.1}°C → {}% (target {}%){}",
                status.strategy,
                status.temperature,
                status.speed,
                status.target_speed,
                if !status.active {
                    " [paused]"
                } else if status.failsafe {
                    " [fail-safe]"
                } else {
                    ""
                }
            ));
            next += TICK;
            if next < now {
                next = now + TICK; // we fell behind (e.g. after suspend), don't burst
            }
        }
        // Sleep until the next tick: one wakeup per second (laptop power). Signals are
        // handled at the latest one tick later.
        std::thread::sleep(next.saturating_duration_since(Instant::now()));
    }

    systemd::notify("STOPPING=1");
    lock(&svc).shutdown();
    let _ = fs::remove_file(&sock_path);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::Error;

    /// Scripted hardware for tests.
    struct Mock {
        temps: Arc<Mutex<Vec<Result<f64>>>>,
        log: Arc<Mutex<Vec<String>>>,
        fail_set: bool,
        /// number of upcoming auto_fan() calls that fail
        fail_auto: Arc<Mutex<u32>>,
        high: Option<f64>,
    }

    impl Hardware for Mock {
        fn temperatures(&mut self) -> Result<Vec<Option<f64>>> {
            let mut t = self.temps.lock().unwrap();
            let next = if t.len() > 1 {
                t.remove(0)
            } else {
                t[0].as_ref().map(|v| *v).map_err(|e| Error(e.0.clone()))
            };
            next.map(|v| vec![Some(v)])
        }
        fn set_speed(&mut self, p: u8) -> Result<()> {
            self.log.lock().unwrap().push(format!("set {p}"));
            if self.fail_set {
                Err("boom".into())
            } else {
                Ok(())
            }
        }
        fn auto_fan(&mut self) -> Result<()> {
            let mut f = self.fail_auto.lock().unwrap();
            if *f > 0 {
                *f -= 1;
                self.log.lock().unwrap().push("auto failed".into());
                return Err("busy".into());
            }
            self.log.lock().unwrap().push("auto".into());
            Ok(())
        }
        fn on_ac(&mut self) -> Result<bool> {
            Ok(true)
        }
        fn fans(&mut self) -> Result<Vec<Fan>> {
            Ok(vec![])
        }
        fn is_live(&self) -> bool {
            false
        }
        fn high_thresholds(&mut self, n: usize) -> Vec<Option<f64>> {
            vec![self.high; n]
        }
    }

    struct Fixture {
        svc: Service,
        log: Arc<Mutex<Vec<String>>>,
        temps: Arc<Mutex<Vec<Result<f64>>>>,
        fail_auto: Arc<Mutex<u32>>,
    }

    fn fixture(temps: Vec<Result<f64>>, fail_set: bool, high: Option<f64>) -> Fixture {
        let log = Arc::new(Mutex::new(vec![]));
        let temps = Arc::new(Mutex::new(temps));
        let fail_auto = Arc::new(Mutex::new(0));
        let hw = Mock {
            temps: Arc::clone(&temps),
            log: Arc::clone(&log),
            fail_set,
            fail_auto: Arc::clone(&fail_auto),
            high,
        };
        let config = Config::parse(include_str!("../tests/upstream_config.json")).unwrap();
        let svc = Service::new(
            config,
            "/nonexistent/config.json".into(),
            Box::new(hw),
            None,
        );
        Fixture {
            svc,
            log,
            temps,
            fail_auto,
        }
    }

    fn service(temps: Vec<Result<f64>>, fail_set: bool) -> (Service, Arc<Mutex<Vec<String>>>) {
        let f = fixture(temps, fail_set, None);
        (f.svc, f.log)
    }

    #[test]
    fn writes_only_on_change() {
        let (mut s, log) = service(vec![Ok(40.0)], false);
        for _ in 0..5 {
            s.tick();
        }
        assert_eq!(*log.lock().unwrap(), ["set 15"]);
    }

    #[test]
    fn failsafe_after_three_read_errors_and_recovery() {
        let err = || Err(Error("sensor".into()));
        let (mut s, log) = service(vec![Ok(40.0), err(), err(), err(), err(), Ok(40.0)], false);
        for _ in 0..6 {
            s.tick();
        }
        assert!(!s.failsafe);
        // one "auto" despite four errors, and the duty is re-applied on recovery
        assert_eq!(*log.lock().unwrap(), ["set 15", "auto", "set 15"]);
    }

    #[test]
    fn failsafe_on_write_errors() {
        let (mut s, log) = service(vec![Ok(40.0)], true);
        for _ in 0..4 {
            s.tick();
        }
        assert!(s.failsafe);
        assert_eq!(
            *log.lock().unwrap(),
            ["set 15", "set 15", "set 15", "auto", "set 15"]
        );
    }

    #[test]
    fn failed_handover_is_retried_while_paused() {
        let mut f = fixture(vec![Ok(40.0)], false, None);
        f.svc.tick();
        *f.fail_auto.lock().unwrap() = 2;
        assert!(
            f.svc
                .handle(&Command::Pause { lease: None })
                .natural()
                .starts_with("[Error] >")
        );
        f.svc.tick(); // still failing
        f.svc.tick(); // succeeds
        f.svc.tick(); // nothing more to do
        assert_eq!(
            *f.log.lock().unwrap(),
            ["set 15", "auto failed", "auto failed", "auto"]
        );
        assert!(!f.svc.active);
    }

    #[test]
    fn failed_handover_in_failsafe_is_retried() {
        let err = || Err(Error("sensor".into()));
        let mut f = fixture(
            vec![Ok(40.0), err(), err(), err(), err(), err()],
            false,
            None,
        );
        *f.fail_auto.lock().unwrap() = 1;
        for _ in 0..5 {
            f.svc.tick();
        }
        assert_eq!(*f.log.lock().unwrap(), ["set 15", "auto failed", "auto"]);
    }

    #[test]
    fn resume_starts_from_current_readings() {
        // lazy: update every 5 s, 30 s average. Idle at 40 °C, then paused while it heats up.
        let mut f = fixture(vec![Ok(40.0)], false, None);
        for _ in 0..30 {
            f.svc.tick();
        }
        f.svc.handle(&Command::Pause { lease: None });
        *f.temps.lock().unwrap() = vec![Ok(85.0)];
        f.svc.handle(&Command::Resume);
        f.svc.tick();
        // with the stale 40 °C history this would be min(avg 40, 85) → 15 %
        assert_eq!(f.log.lock().unwrap().last().unwrap(), "set 100");
    }

    #[test]
    fn pause_lease_expires_and_renewal_does_not_resend_auto() {
        let mut f = fixture(vec![Ok(40.0)], false, None);
        f.svc.tick();
        f.svc.handle(&Command::Pause { lease: Some(30) });
        f.svc.handle(&Command::Pause { lease: Some(30) }); // renewal
        f.svc.tick();
        assert!(!f.svc.active);
        f.svc.pause_until = Some(Instant::now()); // lease runs out
        f.svc.tick();
        assert!(f.svc.active);
        assert_eq!(*f.log.lock().unwrap(), ["set 15", "auto", "set 15"]);
    }

    #[test]
    fn emergency_on_any_sensor_near_ec_limit() {
        let mut f = fixture(vec![Ok(60.0)], false, Some(62.0));
        f.svc.tick();
        assert!(f.svc.last.emergency);
        assert_eq!(*f.log.lock().unwrap(), ["set 100"]);
        let mut g = fixture(vec![Ok(60.0)], false, Some(70.0));
        g.svc.tick();
        assert!(!g.svc.last.emergency);
        assert_eq!(*g.log.lock().unwrap(), ["set 21"]);
    }

    #[test]
    fn set_config_requires_root() {
        let (s, _) = service(vec![Ok(40.0)], false);
        let s = Mutex::new(s);
        let cfg = r#"set_config '{"defaultStrategy":"a","strategies":{"a":{"speedCurve":[{"temp":0,"speed":0}]}}}'"#;
        assert_eq!(
            answer(&s, cfg, Some(1000)),
            "[Error] > An error occurred: set_config requires root"
        );
        assert_eq!(
            answer(&s, cfg, None),
            "[Error] > An error occurred: set_config requires root"
        );
        // root passes the permission check (and here fails on the unwritable test path)
        assert!(answer(&s, cfg, Some(0)).contains("saving configuration failed"));
    }

    #[test]
    fn commands() {
        let (s, log) = service(vec![Ok(40.0)], false);
        let s = Mutex::new(s);
        let user = Some(1000);
        assert_eq!(
            answer(&s, "use agile", user),
            "Strategy in use: 'agile'\nDefault: False"
        );
        assert_eq!(
            answer(&s, "--output-format JSON print current", user),
            r#"{"status": "success", "strategy": "agile", "default": false}"#
        );
        assert!(
            answer(&s, "use nope", user)
                .starts_with("[Error] > An error occurred: The specified strategy is invalid")
        );
        assert_eq!(
            answer(&s, "reset", user),
            "Strategy reset to default! Strategy in use: 'lazy'\nDefault: True"
        );
        assert_eq!(
            answer(&s, "pause", user),
            "Service paused! The hardware fan control will take over"
        );
        lock(&s).tick(); // paused: no hardware access
        assert_eq!(*log.lock().unwrap(), ["auto"]);
        assert_eq!(answer(&s, "print active", user), "Active: False");
        assert!(answer(&s, "resume", user).starts_with("Service resumed!"));
        lock(&s).tick();
        assert_eq!(*log.lock().unwrap(), ["auto", "set 15"]);
        assert!(answer(&s, "run", user).starts_with("[Error] >"));
        assert!(answer(&s, "set_config '{broken'", Some(0)).starts_with("[Error] >"));
    }

    #[test]
    fn near_limit_checks_each_sensor_against_its_own_threshold() {
        let temps = [Some(85.0), Some(96.0), None];
        assert!(near_limit(&temps, &[Some(88.0), Some(120.0), Some(50.0)]));
        assert!(!near_limit(&temps, &[Some(89.0), Some(120.0), Some(10.0)]));
        assert!(!near_limit(&temps, &[]));
    }
}
