//! Command results, rendered exactly like upstream's NATURAL and JSON formats.

use std::io;

use serde_json::{Value, json};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputFormat {
    Natural,
    Json,
}

#[derive(Debug, Clone)]
pub struct Status {
    pub strategy: String,
    pub default: bool,
    pub speed: u8,
    pub target_speed: u8,
    pub temperature: f64,
    pub moving_average_temperature: f64,
    pub effective_temperature: f64,
    pub active: bool,
    pub failsafe: bool,
    pub fan_rpm: Vec<Option<u16>>,
    /// Raw reading per sensor index.
    pub sensors: Vec<Option<f64>>,
    pub config_path: String,
    pub config: Value,
}

#[derive(Debug, Clone)]
pub enum Reply {
    Error(String),
    StrategyReset {
        strategy: String,
        default: bool,
    },
    StrategyChange {
        strategy: String,
        default: bool,
    },
    Reload {
        strategy: String,
        default: bool,
    },
    Pause,
    Resume {
        strategy: String,
        default: bool,
    },
    Current {
        strategy: String,
        default: bool,
    },
    List(Vec<String>),
    Speed(u8),
    Active(bool),
    SetConfig {
        strategy: String,
        default: bool,
        path: String,
        config: Value,
    },
    Status(Box<Status>),
}

/// Python `str(float)`: always at least one decimal.
pub fn py_float(v: f64) -> String {
    if v.fract() == 0.0 && v.abs() < 1e16 {
        format!("{v:.1}")
    } else {
        format!("{v}")
    }
}

fn py_bool(b: bool) -> &'static str {
    if b { "True" } else { "False" }
}

impl Reply {
    pub fn render(&self, fmt: OutputFormat) -> String {
        match fmt {
            OutputFormat::Natural => self.natural(),
            OutputFormat::Json => to_py_json(&self.json()),
        }
    }

    pub fn natural(&self) -> String {
        let nl = "\n";
        match self {
            Reply::Error(r) => format!("[Error] > An error occurred: {r}"),
            Reply::StrategyReset { strategy, default } => format!(
                "Strategy reset to default! Strategy in use: '{strategy}'{nl}Default: {}",
                py_bool(*default)
            ),
            Reply::StrategyChange { strategy, default } | Reply::Current { strategy, default } => {
                format!(
                    "Strategy in use: '{strategy}'{nl}Default: {}",
                    py_bool(*default)
                )
            }
            Reply::Reload { strategy, default } => format!(
                "Reloaded with success! Strategy in use: '{strategy}'{nl}Default: {}",
                py_bool(*default)
            ),
            Reply::Pause => "Service paused! The hardware fan control will take over".into(),
            Reply::Resume { strategy, default } => format!(
                "Service resumed!{nl}Strategy in use: '{strategy}'{nl}Default: {}",
                py_bool(*default)
            ),
            Reply::List(l) => format!("Strategy list: {nl}- {}", l.join(&format!("{nl}- "))),
            Reply::Speed(s) => format!("Current fan speed: '{s}%'"),
            Reply::Active(a) => format!("Active: {}", py_bool(*a)),
            Reply::SetConfig {
                strategy,
                default,
                path,
                config,
            } => format!(
                "Configuration updated with success: {}.{nl}Strategy in use: {strategy}{nl}Default: {}",
                to_py_json(&json!({"path": path, "data": config})),
                py_bool(*default)
            ),
            Reply::Status(s) => {
                let mut out = format!(
                    "Strategy: '{}'{nl}Default: {}{nl}Speed: {}%{nl}TargetSpeed: {}%{nl}\
                     Temp: {}°C{nl}MovingAverageTemp: {}°C{nl}EffectiveTemp: {}°C{nl}Active: {}{nl}",
                    s.strategy,
                    py_bool(s.default),
                    s.speed,
                    s.target_speed,
                    py_float(s.temperature),
                    py_float(s.moving_average_temperature),
                    py_float(s.effective_temperature),
                    py_bool(s.active),
                );
                if s.failsafe {
                    out += &format!("Failsafe: True (EC automatic control){nl}");
                }
                let rpm: Vec<String> = s
                    .fan_rpm
                    .iter()
                    .map(|r| r.map_or("stalled".into(), |v| v.to_string()))
                    .collect();
                if !rpm.is_empty() {
                    out += &format!("FanRPM: {}{nl}", rpm.join(", "));
                }
                let sensors: Vec<String> = s
                    .sensors
                    .iter()
                    .enumerate()
                    .filter_map(|(i, t)| t.map(|t| format!("{i}={t}")))
                    .collect();
                if !sensors.is_empty() {
                    out += &format!("Sensors: {}{nl}", sensors.join(" "));
                }
                out += &format!(
                    "DefaultStrategy: '{}'{nl}DischargingStrategy: '{}'{nl}",
                    s.config["defaultStrategy"].as_str().unwrap_or(""),
                    s.config["strategyOnDischarging"].as_str().unwrap_or("")
                );
                out
            }
        }
    }

    pub fn json(&self) -> Value {
        let ok = "success";
        match self {
            Reply::Error(r) => json!({"status": "error", "reason": r}),
            Reply::StrategyReset { strategy, default }
            | Reply::StrategyChange { strategy, default }
            | Reply::Reload { strategy, default }
            | Reply::Resume { strategy, default }
            | Reply::Current { strategy, default } => {
                json!({"status": ok, "strategy": strategy, "default": default})
            }
            Reply::Pause => json!({"status": ok}),
            Reply::List(l) => json!({"status": ok, "strategies": l}),
            // upstream sends the speed as a string here
            Reply::Speed(s) => json!({"status": ok, "speed": s.to_string()}),
            Reply::Active(a) => json!({"status": ok, "active": a}),
            Reply::SetConfig {
                strategy,
                default,
                path,
                config,
            } => json!({
                "status": ok, "strategy": strategy,
                "configuration": {"path": path, "data": config}, "default": default
            }),
            Reply::Status(s) => json!({
                "status": ok,
                "strategy": s.strategy,
                "default": s.default,
                "speed": s.speed,
                "temperature": s.temperature,
                "movingAverageTemperature": s.moving_average_temperature,
                "effectiveTemperature": s.effective_temperature,
                "active": s.active,
                "configuration": {"path": s.config_path, "data": s.config},
                "targetSpeed": s.target_speed,
                "failsafe": s.failsafe,
                "fanRpm": s.fan_rpm,
                "sensorTemperatures": s.sensors,
            }),
        }
    }
}

/// Python `json.dumps` default separators: `", "` and `": "`.
struct PyFormatter;

impl serde_json::ser::Formatter for PyFormatter {
    fn begin_array_value<W: ?Sized + io::Write>(
        &mut self,
        w: &mut W,
        first: bool,
    ) -> io::Result<()> {
        if first { Ok(()) } else { w.write_all(b", ") }
    }
    fn begin_object_key<W: ?Sized + io::Write>(
        &mut self,
        w: &mut W,
        first: bool,
    ) -> io::Result<()> {
        if first { Ok(()) } else { w.write_all(b", ") }
    }
    fn begin_object_value<W: ?Sized + io::Write>(&mut self, w: &mut W) -> io::Result<()> {
        w.write_all(b": ")
    }
}

pub fn to_py_json(v: &Value) -> String {
    let mut buf = Vec::new();
    let mut ser = serde_json::Serializer::with_formatter(&mut buf, PyFormatter);
    serde::Serialize::serialize(v, &mut ser).expect("serializing a Value cannot fail");
    String::from_utf8(buf).expect("serde_json emits UTF-8")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn python_like_json() {
        let v = json!({"status": "success", "strategies": ["a", "b"], "t": 45.0});
        assert_eq!(
            to_py_json(&v),
            r#"{"status": "success", "strategies": ["a", "b"], "t": 45.0}"#
        );
    }

    #[test]
    fn natural_matches_upstream() {
        let r = Reply::StrategyChange {
            strategy: "lazy".into(),
            default: false,
        };
        assert_eq!(r.natural(), "Strategy in use: 'lazy'\nDefault: False");
        assert_eq!(
            Reply::List(vec!["a".into(), "b".into()]).natural(),
            "Strategy list: \n- a\n- b"
        );
        assert_eq!(Reply::Speed(35).natural(), "Current fan speed: '35%'");
        assert_eq!(
            Reply::Error("x".into()).natural(),
            "[Error] > An error occurred: x"
        );
        assert_eq!(
            to_py_json(&Reply::Speed(35).json()),
            r#"{"status": "success", "speed": "35"}"#
        );
    }

    #[test]
    fn floats_like_python() {
        assert_eq!(py_float(45.0), "45.0");
        assert_eq!(py_float(45.25), "45.25");
        assert_eq!(py_float(51.33), "51.33");
    }
}
