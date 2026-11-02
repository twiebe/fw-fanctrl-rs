//! Command line parsing, compatible with upstream's argparse interface.
//! The same parser handles commands received over the socket (`remote`).

use std::path::PathBuf;

use crate::error::{Result, bail};
use crate::output::OutputFormat;

pub use crate::config::DEFAULT_CONFIG_PATH;

#[derive(Debug, Clone, PartialEq)]
pub enum Backend {
    Ec,
    DryRun,
    Simulated(PathBuf),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PrintSelection {
    All,
    Active,
    Current,
    List,
    Speed,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    Run {
        strategy: Option<String>,
        config: PathBuf,
        silent: bool,
        backend: Backend,
    },
    Use(String),
    Reset,
    Reload,
    /// `lease`: resume automatically after this many seconds unless renewed
    /// (used by `calibrate`, so a killed calibration cannot leave the fan unmanaged).
    Pause {
        lease: Option<u64>,
    },
    Resume,
    Print(PrintSelection),
    SetConfig(String),
    /// Set `defaultStrategy` (or `strategyOnDischarging`) in the configuration file.
    /// `strategy: None` (`--clear`) unsets `strategyOnDischarging`.
    SetDefault {
        strategy: Option<String>,
        discharging: bool,
    },
    // local-only extras
    Probe,
    TestFan {
        percent: u8,
        seconds: u64,
    },
    CalibrateMeasure(crate::calibrate::MeasureOptions),
    CalibrateApply(crate::calibrate::ApplyOptions),
    RestoreAuto,
    Help,
}

impl Command {
    /// Commands executed by the CLI process itself instead of the service.
    pub fn is_local(&self) -> bool {
        matches!(
            self,
            Command::Run { .. }
                | Command::CalibrateMeasure(_)
                | Command::CalibrateApply(_)
                | Command::Probe
                | Command::TestFan { .. }
                | Command::RestoreAuto
                | Command::Help
        )
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Args {
    pub output_format: OutputFormat,
    pub command: Command,
}

pub const USAGE: &str = "\
usage: fw-fanctrl [--socket-controller unix] [--output-format NATURAL|JSON] <command> [...]

control Framework's laptop fan(s) with a speed curve (smooth ramping supported)

commands:
  run [strategy] [--config PATH] [--silent] [--dry-run] [--simulate TEMP_FILE]
                      run the service (--dry-run: read sensors, never write to the EC)
  use <strategy>      change the current strategy
  reset               reset to the default strategy
  reload              reload the configuration file
  pause [--lease N]   pause the service (EC automatic control takes over);
                      with --lease it resumes by itself after N seconds unless renewed
  resume              resume the service
  print [all|active|current|list|speed]
                      print the selected information (default: all)
  set_config <json>   replace the service configuration with the provided one
  set-default <strategy> [--discharging]
                      make a strategy the default (or the one used on battery) and save
                      it in the configuration file (root only)
  set-default --discharging --clear
                      use the default strategy on battery as well (root only)

local commands (need root, do not talk to the service):
  probe               print EC temperatures, fan speeds and AC state
  test-fan <percent> [--seconds N]
                      set the fan duty for N seconds (default 5), then restore EC control
  restore-auto        give fan control back to the EC
  calibrate measure [--load-cmd CMD] [--output PATH]
                      measure fan and thermals once (~30 min, puts the CPU under
                      load) and save the data (default /etc/fw-fanctrl/calibration.json)
  calibrate apply [quiet|balanced|cool] [--comfort-temp C] [--name NAME] [--default]
            [--dry-run] [--config PATH] [--calibration PATH]
                      create a strategy from the saved data (default: balanced, named
                      calibrated-<preference>); --comfort-temp keeps the board (surface)
                      at about C °C in idle; --default also makes it the default strategy;
                      --dry-run only prints it
";

struct Tokens<'a> {
    items: &'a [String],
    pos: usize,
}

impl<'a> Tokens<'a> {
    fn next(&mut self) -> Option<&'a str> {
        let t = self.items.get(self.pos)?;
        self.pos += 1;
        Some(t)
    }
    fn peek(&self) -> Option<&'a str> {
        self.items.get(self.pos).map(|s| s.as_str())
    }
    /// Value for an option given either as `--opt value` or `--opt=value`.
    fn value(&mut self, opt: &str, inline: Option<&'a str>) -> Result<&'a str> {
        match inline.or_else(|| self.next()) {
            Some(v) => Ok(v),
            None => bail!("argument {opt}: expected one argument"),
        }
    }
}

fn split_opt(tok: &str) -> (&str, Option<&str>) {
    match tok.split_once('=') {
        Some((k, v)) if k.starts_with("--") => (k, Some(v)),
        _ => (tok, None),
    }
}

pub fn parse(argv: &[String], remote: bool) -> Result<Args> {
    let mut t = Tokens {
        items: argv,
        pos: 0,
    };
    let mut output_format = OutputFormat::Natural;

    // global options
    let cmd = loop {
        let Some(tok) = t.next() else {
            bail!("the following arguments are required: command");
        };
        let (opt, inline) = split_opt(tok);
        match opt {
            "--socket-controller" | "--sc" => {
                let v = t.value(opt, inline)?;
                if v != "unix" {
                    bail!(
                        "argument --socket-controller: invalid choice: '{v}' (choose from 'unix')"
                    );
                }
            }
            "--output-format" => {
                output_format = match t.value(opt, inline)? {
                    "NATURAL" => OutputFormat::Natural,
                    "JSON" => OutputFormat::Json,
                    v => bail!(
                        "argument --output-format: invalid choice: '{v}' (choose from 'NATURAL', 'JSON')"
                    ),
                };
            }
            "-h" | "--help" => {
                return Ok(Args {
                    output_format,
                    command: Command::Help,
                });
            }
            _ if opt.starts_with('-') => bail!("unrecognized arguments: {tok}"),
            _ => break tok,
        }
    };

    let command = match cmd {
        "run" if !remote => parse_run(&mut t)?,
        "use" => match t.next() {
            Some(s) => Command::Use(s.to_string()),
            None => bail!("the following arguments are required: strategy"),
        },
        "reset" => Command::Reset,
        "reload" => Command::Reload,
        "pause" => {
            let mut lease = None;
            if let Some(tok) = t.peek()
                && tok.starts_with("--lease")
            {
                t.next();
                let (opt, inline) = split_opt(tok);
                lease = match t.value(opt, inline)?.parse::<u64>() {
                    Ok(v) if (1..=3600).contains(&v) => Some(v),
                    _ => bail!("--lease must be between 1 and 3600 seconds"),
                };
            }
            Command::Pause { lease }
        }
        "resume" => Command::Resume,
        "print" => Command::Print(match t.next().unwrap_or("all") {
            "all" => PrintSelection::All,
            "active" => PrintSelection::Active,
            "current" => PrintSelection::Current,
            "list" => PrintSelection::List,
            "speed" => PrintSelection::Speed,
            v => bail!(
                "argument print_selection: invalid choice: '{v}' (choose from 'all', 'active', 'current', 'list', 'speed')"
            ),
        }),
        "set_config" => match t.next() {
            Some(s) => Command::SetConfig(s.to_string()),
            None => bail!("the following arguments are required: provided_config"),
        },
        "set-default" => {
            let mut strategy = None;
            let mut discharging = false;
            let mut clear = false;
            while let Some(tok) = t.peek() {
                if tok == "-h" || tok == "--help" {
                    break;
                }
                t.next();
                match tok {
                    "--discharging" => discharging = true,
                    "--clear" => clear = true,
                    _ if tok.starts_with('-') => bail!("unrecognized arguments: {tok}"),
                    _ if strategy.is_none() => strategy = Some(tok.to_string()),
                    _ => bail!("unrecognized arguments: {tok}"),
                }
            }
            match (strategy, clear) {
                (Some(_), true) => bail!("set-default: give either a strategy or --clear"),
                (None, true) if !discharging => {
                    bail!(
                        "set-default: --clear only works with --discharging (a default strategy is required)"
                    )
                }
                (None, false) => bail!("the following arguments are required: strategy"),
                (strategy, _) => Command::SetDefault {
                    strategy,
                    discharging,
                },
            }
        }
        "probe" if !remote => Command::Probe,
        "restore-auto" if !remote => Command::RestoreAuto,
        "test-fan" if !remote => parse_test_fan(&mut t)?,
        "calibrate" if !remote => parse_calibrate(&mut t)?,
        other => bail!("invalid command: '{other}' (see 'fw-fanctrl --help')"),
    };

    if let Some(extra) = t.next() {
        if extra == "-h" || extra == "--help" {
            return Ok(Args {
                output_format,
                command: Command::Help,
            });
        }
        bail!("unrecognized arguments: {extra}");
    }
    Ok(Args {
        output_format,
        command,
    })
}

fn parse_run(t: &mut Tokens) -> Result<Command> {
    let mut strategy = None;
    let mut config = PathBuf::from(DEFAULT_CONFIG_PATH);
    let mut silent = false;
    let mut backend = Backend::Ec;
    while let Some(tok) = t.peek() {
        if tok == "-h" || tok == "--help" {
            break;
        }
        t.next();
        let (opt, inline) = split_opt(tok);
        match opt {
            "--config" | "-c" => config = PathBuf::from(t.value(opt, inline)?),
            "--silent" | "-s" => silent = true,
            // upstream's only choice is framework_tool; we accept it (and ectool) as
            // aliases for the native EC backend so existing unit files keep working
            "--hardware-controller" | "--hc" => match t.value(opt, inline)? {
                "framework_tool" | "ectool" | "cros_ec" => {}
                v => bail!("argument --hardware-controller: invalid choice: '{v}'"),
            },
            "--dry-run" => backend = Backend::DryRun,
            "--simulate" => backend = Backend::Simulated(PathBuf::from(t.value(opt, inline)?)),
            _ if opt.starts_with('-') => bail!("unrecognized arguments: {tok}"),
            _ if strategy.is_none() => strategy = Some(tok.to_string()),
            _ => bail!("unrecognized arguments: {tok}"),
        }
    }
    Ok(Command::Run {
        strategy,
        config,
        silent,
        backend,
    })
}

fn parse_calibrate(t: &mut Tokens) -> Result<Command> {
    use crate::calibrate::{ApplyOptions, CALIBRATION_PATH, MeasureOptions, Prefer};
    match t.next() {
        Some("measure") => {
            let mut opts = MeasureOptions {
                load_cmd: None,
                output: PathBuf::from(CALIBRATION_PATH),
            };
            while let Some(tok) = t.peek() {
                if tok == "-h" || tok == "--help" {
                    break;
                }
                t.next();
                let (opt, inline) = split_opt(tok);
                match opt {
                    "--load-cmd" => opts.load_cmd = Some(t.value(opt, inline)?.to_string()),
                    "--output" | "-o" => opts.output = PathBuf::from(t.value(opt, inline)?),
                    _ => bail!("unrecognized arguments: {tok}"),
                }
            }
            Ok(Command::CalibrateMeasure(opts))
        }
        Some("apply") => {
            let mut opts = ApplyOptions {
                prefer: Prefer::Balanced,
                comfort_temp: None,
                name: None,
                dry_run: false,
                set_default: false,
                config: PathBuf::from(DEFAULT_CONFIG_PATH),
                calibration: PathBuf::from(CALIBRATION_PATH),
            };
            let mut prefer_seen = false;
            while let Some(tok) = t.peek() {
                if tok == "-h" || tok == "--help" {
                    break;
                }
                t.next();
                let (opt, inline) = split_opt(tok);
                match opt {
                    "--comfort-temp" => {
                        opts.comfort_temp = match t.value(opt, inline)?.parse::<f64>() {
                            Ok(v) if (25.0..=80.0).contains(&v) => Some(v),
                            _ => bail!("--comfort-temp must be between 25 and 80 °C"),
                        };
                    }
                    "--name" => opts.name = Some(t.value(opt, inline)?.to_string()),
                    "--dry-run" => opts.dry_run = true,
                    "--default" => opts.set_default = true,
                    "--config" | "-c" => opts.config = PathBuf::from(t.value(opt, inline)?),
                    "--calibration" => opts.calibration = PathBuf::from(t.value(opt, inline)?),
                    _ if opt.starts_with('-') => bail!("unrecognized arguments: {tok}"),
                    _ if !prefer_seen => {
                        opts.prefer = match Prefer::parse(tok) {
                            Some(p) => p,
                            None => bail!(
                                "invalid preference: '{tok}' (choose from 'quiet', 'balanced', 'cool')"
                            ),
                        };
                        prefer_seen = true;
                    }
                    _ => bail!("unrecognized arguments: {tok}"),
                }
            }
            Ok(Command::CalibrateApply(opts))
        }
        Some(other) => {
            bail!("invalid calibrate command: '{other}' (choose from 'measure', 'apply')")
        }
        None => bail!("calibrate needs a command: 'measure' or 'apply'"),
    }
}

fn parse_test_fan(t: &mut Tokens) -> Result<Command> {
    let Some(p) = t.next() else {
        bail!("the following arguments are required: percent");
    };
    let percent = match p.parse::<u8>() {
        Ok(v) if v <= 100 => v,
        _ => bail!("percent must be an integer between 0 and 100"),
    };
    let mut seconds = 5;
    while let Some(tok) = t.peek() {
        if !tok.starts_with("--seconds") {
            break;
        }
        t.next();
        let (opt, inline) = split_opt(tok);
        seconds = match t.value(opt, inline)?.parse::<u64>() {
            Ok(v) if (1..=60).contains(&v) => v,
            _ => bail!("--seconds must be between 1 and 60"),
        };
    }
    Ok(Command::TestFan { percent, seconds })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(s: &str, remote: bool) -> Result<Args> {
        parse(&shlex::split(s).unwrap(), remote)
    }

    #[test]
    fn upstream_service_command_line() {
        let a = p(
            r#"--output-format "JSON" run --config "/etc/fw-fanctrl/config.json" --silent"#,
            false,
        )
        .unwrap();
        assert_eq!(a.output_format, OutputFormat::Json);
        assert_eq!(
            a.command,
            Command::Run {
                strategy: None,
                config: "/etc/fw-fanctrl/config.json".into(),
                silent: true,
                backend: Backend::Ec
            }
        );
        let a = p("run lazy --hc framework_tool -c /tmp/x.json", false).unwrap();
        assert!(matches!(a.command, Command::Run { strategy: Some(ref s), .. } if s == "lazy"));
    }

    #[test]
    fn client_commands() {
        assert_eq!(
            p("use agile", true).unwrap().command,
            Command::Use("agile".into())
        );
        assert_eq!(
            p("print", true).unwrap().command,
            Command::Print(PrintSelection::All)
        );
        assert_eq!(
            p("--sc unix print list", true).unwrap().command,
            Command::Print(PrintSelection::List)
        );
        assert_eq!(
            p("pause --lease 30", true).unwrap().command,
            Command::Pause { lease: Some(30) }
        );
        assert!(p("pause --lease 0", true).is_err());
        assert_eq!(
            p("--output-format=JSON pause", true).unwrap().output_format,
            OutputFormat::Json
        );
        assert_eq!(
            p("set-default calibrated-quiet --discharging", true)
                .unwrap()
                .command,
            Command::SetDefault {
                strategy: Some("calibrated-quiet".into()),
                discharging: true
            }
        );
        assert_eq!(
            p("set-default --discharging --clear", true)
                .unwrap()
                .command,
            Command::SetDefault {
                strategy: None,
                discharging: true
            }
        );
        for bad in [
            "set-default",
            "set-default a b",
            "set-default --clear",
            "set-default a --discharging --clear",
        ] {
            assert!(p(bad, true).is_err(), "{bad}");
        }
        assert_eq!(
            p(r#"set_config '{"a": 1}'"#, true).unwrap().command,
            Command::SetConfig(r#"{"a": 1}"#.into())
        );
    }

    #[test]
    fn remote_cannot_run_local_commands() {
        for c in [
            "run",
            "probe",
            "restore-auto",
            "test-fan 50",
            "calibrate measure",
            "calibrate apply",
        ] {
            assert!(p(c, true).is_err(), "{c}");
        }
    }

    #[test]
    fn calibrate_commands() {
        use crate::calibrate::Prefer;
        let Command::CalibrateMeasure(m) =
            p(r#"calibrate measure --load-cmd "vkcube --c 1""#, false)
                .unwrap()
                .command
        else {
            panic!()
        };
        assert_eq!(m.load_cmd.as_deref(), Some("vkcube --c 1"));
        assert_eq!(m.output, PathBuf::from("/etc/fw-fanctrl/calibration.json"));

        let Command::CalibrateApply(a) = p("calibrate apply", false).unwrap().command else {
            panic!()
        };
        assert_eq!(a.prefer, Prefer::Balanced);
        assert!(a.name.is_none() && !a.dry_run && a.comfort_temp.is_none());

        let Command::CalibrateApply(a) = p(
            "calibrate apply quiet --comfort-temp 43 --name mac --dry-run --default",
            false,
        )
        .unwrap()
        .command
        else {
            panic!()
        };
        assert_eq!(a.prefer, Prefer::Quiet);
        assert_eq!(a.comfort_temp, Some(43.0));
        assert_eq!(a.name.as_deref(), Some("mac"));
        assert!(a.dry_run && a.set_default);

        for bad in [
            "calibrate",
            "calibrate run",
            "calibrate apply loud",
            "calibrate apply quiet cool",
            "calibrate apply --comfort-temp 10",
            "calibrate measure --prefer quiet",
        ] {
            assert!(p(bad, false).is_err(), "{bad}");
        }
    }

    #[test]
    fn errors() {
        assert!(p("", false).is_err());
        assert!(p("print foo", true).is_err());
        assert!(p("use", true).is_err());
        assert!(p("--output-format XML print", true).is_err());
        assert!(p("reset extra", true).is_err());
        assert!(p("test-fan 101", false).is_err());
        assert_eq!(
            p("test-fan 30 --seconds 10", false).unwrap().command,
            Command::TestFan {
                percent: 30,
                seconds: 10
            }
        );
    }
}
