mod cli;
mod config;
mod control;
mod daemon;
mod ec;
mod error;
mod hw;
mod output;
mod signals;
mod systemd;

use std::process::ExitCode;
use std::time::{Duration, Instant};

use cli::{Args, Backend, Command};
use config::Config;
use ec::{CrosEc, EcCommand, Fan};
use error::Result;
use output::{OutputFormat, Reply};

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let args = match cli::parse(&argv, false) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{}\n\n{}", Reply::Error(e.0).natural(), cli::USAGE);
            return ExitCode::FAILURE;
        }
    };

    if !args.command.is_local() {
        return match daemon::send_command(&argv) {
            Ok(resp) if resp.starts_with("[Error] > ") => {
                eprintln!("{resp}");
                ExitCode::FAILURE
            }
            Ok(resp) => {
                println!("{resp}");
                ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("{}", Reply::Error(e.0).render(args.output_format));
                ExitCode::FAILURE
            }
        };
    }

    match run_local(args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("{}", Reply::Error(e.0).natural());
            ExitCode::FAILURE
        }
    }
}

/// If we crash while the fan is in manual mode, give control back to the EC.
fn install_panic_failsafe() {
    let default_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        default_hook(info);
        match ec::restore_auto() {
            Ok(()) => eprintln!("panic: EC automatic fan control restored"),
            Err(e) => eprintln!("panic: restoring EC fan control failed: {e}"),
        }
    }));
}

fn run_local(args: Args) -> Result<()> {
    match args.command {
        Command::Help => {
            print!("{}", cli::USAGE);
            Ok(())
        }
        Command::Run {
            strategy,
            config,
            silent,
            backend,
        } => {
            let cfg = Config::load(&config)?;
            let hw: Box<dyn hw::Hardware> = match backend {
                Backend::Ec => Box::new(hw::Ec(CrosEc::open()?)),
                Backend::DryRun => Box::new(hw::DryRun(CrosEc::open()?)),
                Backend::Simulated(temp_file) => Box::new(hw::Simulated {
                    temp_file,
                    speed: 0,
                }),
            };
            if hw.is_live() {
                install_panic_failsafe();
            }
            let svc = daemon::Service::new(cfg, config, hw, strategy.as_deref());
            daemon::run(svc, args.output_format, silent)
        }
        Command::Probe => probe(args.output_format),
        Command::TestFan { percent, seconds } => test_fan(percent, seconds),
        Command::RestoreAuto => {
            ec::restore_auto()?;
            println!("EC automatic fan control restored");
            Ok(())
        }
        _ => unreachable!("remote commands are sent to the service"),
    }
}

fn fans_str(fans: &[Fan]) -> String {
    let v: Vec<String> = fans
        .iter()
        .map(|f| match f {
            Fan::Rpm(r) => format!("{r} RPM"),
            Fan::Stalled => "stalled".into(),
        })
        .collect();
    v.join(", ")
}

fn probe(fmt: OutputFormat) -> Result<()> {
    let ec = CrosEc::open()?;
    let temps = ec.temperatures()?;
    let fans = ec.fans()?;
    let ac = ec.on_ac()?;
    if fmt == OutputFormat::Json {
        let fans: Vec<Option<u16>> = fans
            .iter()
            .map(|f| match f {
                Fan::Rpm(r) => Some(*r),
                Fan::Stalled => None,
            })
            .collect();
        let v = serde_json::json!({"temperatures": temps, "max": temps.iter().max(), "fanRpm": fans, "ac": ac});
        println!("{}", output::to_py_json(&v));
    } else {
        let t: Vec<String> = temps.iter().map(|t| format!("{t}°C")).collect();
        println!("Temperatures: {}", t.join(", "));
        println!(
            "Max: {}",
            temps
                .iter()
                .max()
                .map_or("n/a".into(), |t| format!("{t}°C"))
        );
        println!("Fans: {}", fans_str(&fans));
        println!("AC: {}", if ac { "connected" } else { "disconnected" });
    }
    Ok(())
}

fn test_fan(percent: u8, seconds: u64) -> Result<()> {
    let ec = CrosEc::open()?;
    install_panic_failsafe();
    signals::install();
    println!("setting fan duty to {percent}% for {seconds}s (Ctrl-C restores EC control early)");
    let result = (|| -> Result<()> {
        ec.send(EcCommand::SetFanDuty(percent))?;
        let end = Instant::now() + Duration::from_secs(seconds);
        let mut next_print = Instant::now();
        while Instant::now() < end {
            if signals::take().is_some() {
                break;
            }
            if Instant::now() >= next_print {
                let temps = ec.temperatures().unwrap_or_default();
                println!(
                    "  fans: {:<20} max temp: {}",
                    fans_str(&ec.fans().unwrap_or_default()),
                    temps
                        .iter()
                        .max()
                        .map_or("n/a".into(), |t| format!("{t}°C"))
                );
                next_print += Duration::from_secs(1);
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        Ok(())
    })();
    // always restore, even if setting the duty failed
    let restored = ec.send(EcCommand::AutoFanCtrl);
    result?;
    restored?;
    println!("EC automatic fan control restored");
    Ok(())
}
