//! The service: control loop, fail-safe handling and the command socket.

use std::fs;
use std::io::{ErrorKind, Read, Write};
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
use crate::hw::Hardware;
use crate::output::{OutputFormat, Reply, Status};
use crate::signals::{self, Signal};
use crate::systemd;

pub const SOCKET_PATH: &str = "/run/fw-fanctrl/.fw-fanctrl.commands.sock";

/// The command socket; `FW_FANCTRL_SOCKET` overrides it (testing without root).
pub fn socket_path() -> PathBuf {
    std::env::var_os("FW_FANCTRL_SOCKET").map_or_else(|| PathBuf::from(SOCKET_PATH), PathBuf::from)
}

/// Consecutive hardware errors before handing control back to the EC.
const MAX_FAILURES: u32 = 3;
/// Re-send an unchanged duty this often, in case something reset the EC to auto mode.
const REFRESH_INTERVAL: Duration = Duration::from_secs(30);
const TICK: Duration = Duration::from_secs(1);

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
    failures: u32,
    failsafe: bool,
    last_write: Option<(u8, Instant)>,
    force_write: bool,
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
            failures: 0,
            failsafe: false,
            last_write: None,
            force_write: true,
        };
        if let Ok(ac) = svc.hw.on_ac() {
            svc.on_ac = ac;
        }
        match strategy {
            Some(s) if !s.is_empty() && svc.config.strategy(s).is_some() => {
                svc.overwritten = Some(s.into())
            }
            Some(s) if !s.is_empty() => {
                eprintln!("unknown strategy '{s}', using the default strategy")
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

    fn fail(&mut self, what: String) {
        self.failures += 1;
        eprintln!("[Error] > {what} ({}/{MAX_FAILURES})", self.failures);
        if self.failures >= MAX_FAILURES && !self.failsafe {
            eprintln!("[Error] > entering fail-safe: handing fan control back to the EC");
            if let Err(e) = self.hw.auto_fan() {
                eprintln!("[Error] > restoring EC fan control failed: {e}");
            }
            self.failsafe = true;
            self.force_write = true;
        }
    }

    /// One control step (once per second).
    pub fn tick(&mut self) {
        if !self.active {
            return;
        }
        let temp = match self.hw.temperature() {
            Ok(t) => t,
            Err(e) => return self.fail(format!("reading temperature failed: {e}")),
        };
        if let Ok(ac) = self.hw.on_ac() {
            self.on_ac = ac;
        }
        self.fans = self.hw.fans().unwrap_or_default();

        let strategy = self.current_strategy().clone();
        let tick = self.controller.tick(&strategy, temp, TICK.as_secs_f64());
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
            Command::Pause => {
                self.active = false;
                if let Err(e) = self.hw.auto_fan() {
                    return Reply::Error(format!("restoring EC fan control failed: {e}"));
                }
                Reply::Pause
            }
            Command::Resume => {
                self.active = true;
                self.force_write = true;
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
        if let Err(e) = self.hw.auto_fan() {
            eprintln!("[Error] > restoring EC fan control failed: {e}");
        }
    }
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
            return Err("another fw-fanctrl service is already running".into());
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

pub fn answer(svc: &Mutex<Service>, request: &str) -> String {
    let parsed = shlex::split(request)
        .ok_or_else(|| "invalid quoting".into())
        .and_then(|argv| cli::parse(&argv, true));
    match parsed {
        Ok(args) if args.command == Command::Help => cli::USAGE.to_string(),
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
        let response = match read_request(&mut stream) {
            Ok(req) => answer(&svc, &req),
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
    }

    impl Hardware for Mock {
        fn temperature(&mut self) -> Result<f64> {
            let mut t = self.temps.lock().unwrap();
            if t.len() > 1 {
                t.remove(0)
            } else {
                t[0].as_ref().map(|v| *v).map_err(|e| Error(e.0.clone()))
            }
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
    }

    fn service(temps: Vec<Result<f64>>, fail_set: bool) -> (Service, Arc<Mutex<Vec<String>>>) {
        let log = Arc::new(Mutex::new(vec![]));
        let hw = Mock {
            temps: Arc::new(Mutex::new(temps)),
            log: Arc::clone(&log),
            fail_set,
        };
        let config = Config::parse(include_str!("../tests/upstream_config.json")).unwrap();
        (
            Service::new(
                config,
                "/nonexistent/config.json".into(),
                Box::new(hw),
                None,
            ),
            log,
        )
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
    fn commands() {
        let (s, log) = service(vec![Ok(40.0)], false);
        let s = Mutex::new(s);
        assert_eq!(
            answer(&s, "use agile"),
            "Strategy in use: 'agile'\nDefault: False"
        );
        assert_eq!(
            answer(&s, "--output-format JSON print current"),
            r#"{"status": "success", "strategy": "agile", "default": false}"#
        );
        assert!(
            answer(&s, "use nope")
                .starts_with("[Error] > An error occurred: The specified strategy is invalid")
        );
        assert_eq!(
            answer(&s, "reset"),
            "Strategy reset to default! Strategy in use: 'lazy'\nDefault: True"
        );
        assert_eq!(
            answer(&s, "pause"),
            "Service paused! The hardware fan control will take over"
        );
        lock(&s).tick(); // paused: no hardware access
        assert_eq!(*log.lock().unwrap(), ["auto"]);
        assert_eq!(answer(&s, "print active"), "Active: False");
        assert!(answer(&s, "resume").starts_with("Service resumed!"));
        lock(&s).tick();
        assert_eq!(*log.lock().unwrap(), ["auto", "set 15"]);
        assert!(answer(&s, "run").starts_with("[Error] >"));
        assert!(answer(&s, "set_config '{broken'").starts_with("[Error] >"));
    }
}
