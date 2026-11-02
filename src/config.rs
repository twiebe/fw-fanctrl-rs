//! Configuration compatible with upstream fw-fanctrl's `config.json`,
//! extended by optional per-strategy ramp ("stagger") settings.

use std::fs;
use std::path::Path;

use serde::Deserialize;
use serde_json::{Map, Value};

use crate::error::{Error, Result, bail};

pub const DEFAULT_CONFIG: &str = include_str!("../resources/config.json");
pub const DEFAULT_CONFIG_PATH: &str = "/etc/fw-fanctrl/config.json";
pub const SCHEMA_REF: &str = "./config.schema.json";

#[derive(Debug, Clone, PartialEq)]
pub struct CurvePoint {
    pub temp: f64,
    pub speed: u8,
}

/// Smoothing behaviour of a strategy. All fields are optional in the file;
/// without them a strategy behaves exactly like upstream (instant changes).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Ramp {
    /// Max increase in percent per second. `None` = instant.
    pub up_rate: Option<f64>,
    /// Max decrease in percent per second. `None` = instant.
    pub down_rate: Option<f64>,
    /// Seconds the target must stay below the current speed before slowing down.
    pub down_delay: f64,
    /// Raw temperature at which ramp-up limiting and averaging are bypassed.
    pub critical_temp: Option<f64>,
    /// Lowest duty the fan reliably runs at. Outputs between 0 and this are never
    /// sent: starting jumps straight to it (the EC only kick-starts the fan when
    /// leaving 0 %), and a running fan stays at it until the target is 0.
    pub min_speed: Option<u8>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Strategy {
    pub name: String,
    pub fan_speed_update_frequency: u32,
    pub moving_average_interval: u32,
    pub speed_curve: Vec<CurvePoint>,
    pub ramp: Ramp,
    /// Sensor indices whose maximum drives the curve; `None` = all (upstream).
    pub sensors: Option<Vec<u8>>,
}

impl Strategy {
    /// Curve interpolation, identical to upstream `adapt_speed`.
    pub fn target_speed(&self, temp: f64) -> u8 {
        let curve = &self.speed_curve;
        let mut lo = 0;
        let mut hi = curve.len() - 1;
        for (i, p) in curve.iter().enumerate() {
            if temp > p.temp {
                lo = i;
            } else {
                hi = i;
                break;
            }
        }
        if lo == hi {
            return curve[lo].speed;
        }
        let (a, b) = (&curve[lo], &curve[hi]);
        if b.temp == a.temp {
            return b.speed; // step (upstream's loop never gets here, guard anyway)
        }
        let slope = (b.speed as f64 - a.speed as f64) / (b.temp - a.temp);
        // Python's int() truncates towards zero, so does `as`.
        let speed = (a.speed as f64 + (temp - a.temp) * slope) as i64;
        speed.clamp(0, 100) as u8
    }
}

#[derive(Debug, Clone)]
pub struct Config {
    /// The configuration as JSON (key order preserved), used for printing and saving.
    pub data: Value,
    pub default_strategy: String,
    /// Empty string = use the default strategy on battery as well.
    pub strategy_on_discharging: String,
    strategies: Vec<Strategy>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
struct RawConfig {
    #[serde(rename = "$schema")]
    _schema: Option<String>,
    default_strategy: String,
    #[serde(default)]
    strategy_on_discharging: String,
    strategies: Map<String, Value>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
struct RawStrategy {
    fan_speed_update_frequency: Option<u32>,
    moving_average_interval: Option<u32>,
    speed_curve: Vec<RawPoint>,
    ramp_up_rate: Option<f64>,
    ramp_down_rate: Option<f64>,
    ramp_down_delay: Option<f64>,
    critical_temp: Option<f64>,
    sensors: Option<Vec<u8>>,
    min_speed: Option<u8>,
}

/// EC memory map: 16 sensors in bank A plus 8 in bank B.
pub const MAX_SENSORS: u8 = 24;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawPoint {
    temp: f64,
    speed: u8,
}

fn valid_name(name: &str) -> bool {
    !name.is_empty()
        && name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
}

fn check_range(strategy: &str, field: &str, v: f64, min: f64, max: f64) -> Result<()> {
    if !(v.is_finite() && v >= min && v <= max) {
        bail!("strategy '{strategy}': {field} must be between {min} and {max} (got {v})");
    }
    Ok(())
}

fn parse_strategy(name: &str, value: &Value) -> Result<Strategy> {
    if !valid_name(name) {
        bail!("invalid strategy name '{name}' (allowed: a-z, A-Z, 0-9, _ and -)");
    }
    let raw: RawStrategy = serde_json::from_value(value.clone())
        .map_err(|e| Error(format!("strategy '{name}': {e}")))?;

    let freq = raw.fan_speed_update_frequency.unwrap_or(5);
    check_range(name, "fanSpeedUpdateFrequency", freq as f64, 1.0, 10.0)?;
    let mai = raw.moving_average_interval.unwrap_or(20);
    check_range(name, "movingAverageInterval", mai as f64, 1.0, 100.0)?;

    if raw.speed_curve.is_empty() {
        bail!("strategy '{name}': speedCurve must contain at least one point");
    }
    let mut curve = Vec::with_capacity(raw.speed_curve.len());
    for p in &raw.speed_curve {
        check_range(name, "speedCurve temp", p.temp, 0.0, 100.0)?;
        check_range(name, "speedCurve speed", p.speed as f64, 0.0, 100.0)?;
        // Equal temps are allowed (upstream accepts them; they form a step).
        if let Some(prev) = curve.last().map(|c: &CurvePoint| c.temp)
            && p.temp < prev
        {
            bail!("strategy '{name}': speedCurve temps must be ascending");
        }
        curve.push(CurvePoint {
            temp: p.temp,
            speed: p.speed,
        });
    }

    for (field, rate) in [
        ("rampUpRate", raw.ramp_up_rate),
        ("rampDownRate", raw.ramp_down_rate),
    ] {
        if let Some(r) = rate {
            check_range(name, field, r, 0.01, 100.0)?;
        }
    }
    let down_delay = raw.ramp_down_delay.unwrap_or(0.0);
    check_range(name, "rampDownDelay", down_delay, 0.0, 600.0)?;
    if let Some(t) = raw.critical_temp {
        check_range(name, "criticalTemp", t, 0.0, 110.0)?;
    }
    if let Some(m) = raw.min_speed {
        check_range(name, "minSpeed", m as f64, 1.0, 50.0)?;
    }
    if let Some(sensors) = &raw.sensors {
        if sensors.is_empty() {
            bail!("strategy '{name}': sensors must not be empty (omit it to use all sensors)");
        }
        for (i, &idx) in sensors.iter().enumerate() {
            if idx >= MAX_SENSORS {
                bail!(
                    "strategy '{name}': sensor index {idx} out of range (0-{})",
                    MAX_SENSORS - 1
                );
            }
            if sensors[..i].contains(&idx) {
                bail!("strategy '{name}': sensor index {idx} listed twice");
            }
        }
    }

    Ok(Strategy {
        name: name.to_string(),
        fan_speed_update_frequency: freq,
        moving_average_interval: mai,
        speed_curve: curve,
        ramp: Ramp {
            up_rate: raw.ramp_up_rate,
            down_rate: raw.ramp_down_rate,
            down_delay,
            critical_temp: raw.critical_temp,
            min_speed: raw.min_speed,
        },
        sensors: raw.sensors,
    })
}

impl Config {
    pub fn parse(raw: &str) -> Result<Config> {
        let mut data: Value = serde_json::from_str(raw)
            .map_err(|e| Error(format!("Error parsing configuration: {e}")))?;
        let parsed: RawConfig = serde_json::from_value(data.clone())
            .map_err(|e| Error(format!("Invalid configuration: {e}")))?;

        if parsed.strategies.is_empty() {
            bail!("Invalid configuration: at least one strategy is required");
        }
        let strategies = parsed
            .strategies
            .iter()
            .map(|(name, v)| parse_strategy(name, v))
            .collect::<Result<Vec<_>>>()?;

        let exists = |n: &str| strategies.iter().any(|s| s.name == n);
        if !exists(&parsed.default_strategy) {
            bail!(
                "Default strategy '{}' is not a valid strategy.",
                parsed.default_strategy
            );
        }
        if !parsed.strategy_on_discharging.is_empty() && !exists(&parsed.strategy_on_discharging) {
            bail!(
                "Discharging strategy '{}' is not a valid strategy.",
                parsed.strategy_on_discharging
            );
        }

        let obj = data.as_object_mut().expect("validated as object above");
        if !obj.contains_key("$schema") {
            obj.insert("$schema".into(), Value::String(SCHEMA_REF.into()));
        }
        if !obj.contains_key("strategyOnDischarging") {
            obj.insert("strategyOnDischarging".into(), Value::String(String::new()));
        }

        Ok(Config {
            data,
            default_strategy: parsed.default_strategy,
            strategy_on_discharging: parsed.strategy_on_discharging,
            strategies,
        })
    }

    /// Load the configuration. Like upstream, a missing file at the default path is
    /// created from the built-in default. Any other missing path is an error, so a
    /// mistyped `--config` cannot silently run the default strategies.
    pub fn load(path: &Path) -> Result<Config> {
        if !path.exists() {
            if path != Path::new(DEFAULT_CONFIG_PATH) {
                bail!("configuration file {} does not exist", path.display());
            }
            if let Some(dir) = path.parent() {
                fs::create_dir_all(dir)?;
            }
            fs::write(path, DEFAULT_CONFIG)
                .map_err(|e| Error(format!("cannot create {}: {e}", path.display())))?;
        }
        let raw = fs::read_to_string(path)
            .map_err(|e| Error(format!("cannot read {}: {e}", path.display())))?;
        Config::parse(&raw)
    }

    /// Save atomically (write temp file, then rename), 4-space indent like upstream.
    pub fn save(&self, path: &Path) -> Result<()> {
        let mut buf = Vec::new();
        let fmt = serde_json::ser::PrettyFormatter::with_indent(b"    ");
        let mut ser = serde_json::Serializer::with_formatter(&mut buf, fmt);
        serde::Serialize::serialize(&self.data, &mut ser)?;
        buf.push(b'\n');
        let tmp = path.with_extension("json.tmp");
        let write = || -> std::io::Result<()> {
            let mut f = fs::File::create(&tmp)?;
            std::io::Write::write_all(&mut f, &buf)?;
            f.sync_all()?;
            fs::rename(&tmp, path)
        };
        if let Err(e) = write() {
            let _ = fs::remove_file(&tmp);
            return Err(e.into());
        }
        Ok(())
    }

    pub fn strategy(&self, name: &str) -> Option<&Strategy> {
        self.strategies.iter().find(|s| s.name == name)
    }

    pub fn names(&self) -> Vec<String> {
        self.strategies.iter().map(|s| s.name.clone()).collect()
    }

    pub fn default(&self) -> &Strategy {
        self.strategy(&self.default_strategy)
            .expect("validated in parse")
    }

    pub fn discharging(&self) -> &Strategy {
        if self.strategy_on_discharging.is_empty() {
            self.default()
        } else {
            self.strategy(&self.strategy_on_discharging)
                .expect("validated in parse")
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn upstream_config() -> &'static str {
        include_str!("../tests/upstream_config.json")
    }

    #[test]
    fn parses_upstream_default_config() {
        let c = Config::parse(upstream_config()).unwrap();
        assert_eq!(c.default_strategy, "lazy");
        assert_eq!(c.strategy_on_discharging, "");
        assert_eq!(
            c.names(),
            [
                "laziest",
                "lazy",
                "medium",
                "agile",
                "very-agile",
                "deaf",
                "aeolus"
            ]
        );
        let lazy = c.strategy("lazy").unwrap();
        assert_eq!(lazy.fan_speed_update_frequency, 5);
        assert_eq!(lazy.moving_average_interval, 30);
        assert_eq!(lazy.ramp, Ramp::default());
        assert_eq!(c.discharging().name, "lazy");
    }

    #[test]
    fn parses_builtin_default_with_ramp() {
        let c = Config::parse(DEFAULT_CONFIG).unwrap();
        let s = c.strategy("smooth").unwrap();
        assert_eq!(s.ramp.up_rate, Some(2.0));
        assert_eq!(s.ramp.down_rate, Some(0.5));
        assert_eq!(s.ramp.down_delay, 30.0);
        assert_eq!(s.ramp.critical_temp, Some(90.0));
    }

    #[test]
    fn interpolation_matches_upstream() {
        let c = Config::parse(upstream_config()).unwrap();
        let lazy = c.strategy("lazy").unwrap();
        // curve: 0→15, 50→15, 65→25, 70→35, 75→50, 85→100
        assert_eq!(lazy.target_speed(0.0), 15);
        assert_eq!(lazy.target_speed(-5.0), 15);
        assert_eq!(lazy.target_speed(50.0), 15);
        assert_eq!(lazy.target_speed(57.5), 20); // 15 + 7.5 * 10/15 = 20.0
        assert_eq!(lazy.target_speed(60.0), 21); // 15 + 10 * 0.666 = 21.66 -> 21
        assert_eq!(lazy.target_speed(72.0), 41); // 35 + 2 * 3 = 41
        assert_eq!(lazy.target_speed(85.0), 100);
        assert_eq!(lazy.target_speed(99.0), 100);
    }

    #[test]
    fn rejects_bad_configs() {
        let cases = [
            (
                r#"{"defaultStrategy":"x","strategyOnDischarging":"","strategies":{"a":{"speedCurve":[{"temp":0,"speed":1}]}}}"#,
                "Default strategy",
            ),
            (
                r#"{"defaultStrategy":"a","strategyOnDischarging":"b","strategies":{"a":{"speedCurve":[{"temp":0,"speed":1}]}}}"#,
                "Discharging strategy",
            ),
            (
                r#"{"defaultStrategy":"a","strategyOnDischarging":"","strategies":{"a":{"speedCurve":[]}}}"#,
                "at least one point",
            ),
            (
                r#"{"defaultStrategy":"a","strategyOnDischarging":"","strategies":{"a":{"speedCurve":[{"temp":50,"speed":1},{"temp":40,"speed":2}]}}}"#,
                "ascending",
            ),
            (
                r#"{"defaultStrategy":"a","strategyOnDischarging":"","strategies":{"a":{"speedCurve":[{"temp":0,"speed":101}]}}}"#,
                "speed",
            ),
            (
                r#"{"defaultStrategy":"a","strategyOnDischarging":"","strategies":{"a":{"speedCurve":[{"temp":0,"speed":1}],"rampUpRtae":1}}}"#,
                "unknown field",
            ),
            (
                r#"{"defaultStrategy":"a","strategyOnDischarging":"","strategies":{"a":{"speedCurve":[{"temp":0,"speed":1}],"rampUpRate":0}}}"#,
                "rampUpRate",
            ),
            (
                r#"{"defaultStrategy":"a","strategyOnDischarging":"","strategies":{"a":{"speedCurve":[{"temp":0,"speed":1}],"fanSpeedUpdateFrequency":0}}}"#,
                "fanSpeedUpdateFrequency",
            ),
            (
                r#"{"defaultStrategy":"a","strategyOnDischarging":"","strategies":{"a b":{"speedCurve":[{"temp":0,"speed":1}]}}}"#,
                "invalid strategy name",
            ),
            (
                r#"{"defaultStrategy":"a","strategyOnDischarging":"","strategies":{"a":{"speedCurve":[{"temp":0,"speed":1}],"sensors":[]}}}"#,
                "must not be empty",
            ),
            (
                r#"{"defaultStrategy":"a","strategyOnDischarging":"","strategies":{"a":{"speedCurve":[{"temp":0,"speed":1}],"sensors":[1,24]}}}"#,
                "out of range",
            ),
            (
                r#"{"defaultStrategy":"a","strategyOnDischarging":"","strategies":{"a":{"speedCurve":[{"temp":0,"speed":1}],"sensors":[1,1]}}}"#,
                "twice",
            ),
            (
                r#"{"defaultStrategy":"a","foo":1,"strategies":{"a":{"speedCurve":[{"temp":0,"speed":1}]}}}"#,
                "unknown field",
            ),
        ];
        for (raw, needle) in cases {
            let err = Config::parse(raw).unwrap_err().0;
            assert!(err.contains(needle), "expected '{needle}' in '{err}'");
        }
    }

    #[test]
    fn fills_in_schema_and_keeps_key_order() {
        let c = Config::parse(r#"{"defaultStrategy":"a","strategies":{"b":{"speedCurve":[{"temp":0,"speed":1}]},"a":{"speedCurve":[{"temp":0,"speed":1}]}}}"#).unwrap();
        assert_eq!(c.data["$schema"], SCHEMA_REF);
        assert_eq!(c.names(), ["b", "a"]);
    }

    #[test]
    fn step_curves_like_upstream() {
        let c = Config::parse(r#"{"defaultStrategy":"a","strategies":{"a":{"speedCurve":[
            {"temp":0,"speed":10},{"temp":50,"speed":15},{"temp":50,"speed":30},{"temp":80,"speed":100}]}}}"#)
        .unwrap();
        let s = c.strategy("a").unwrap();
        // upstream adapt_speed: <= 50 interpolates towards the first 50 point, > 50 from the second
        assert_eq!(s.target_speed(25.0), 12);
        assert_eq!(s.target_speed(50.0), 15);
        assert_eq!(s.target_speed(50.5), 31);
        assert_eq!(s.target_speed(80.0), 100);
        let only = Config::parse(
            r#"{"defaultStrategy":"a","strategies":{"a":{"speedCurve":[
            {"temp":50,"speed":15},{"temp":50,"speed":30}]}}}"#,
        )
        .unwrap();
        assert_eq!(only.strategy("a").unwrap().target_speed(40.0), 15);
        assert_eq!(only.strategy("a").unwrap().target_speed(60.0), 30);
    }

    #[test]
    fn save_roundtrip() {
        let dir = std::env::temp_dir().join(format!("fwfan-test-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let path = dir.join("config.json");
        let err = Config::load(&path).unwrap_err().0; // only the default path is auto-created
        assert!(err.contains("does not exist"), "{err}");
        assert!(!path.exists());
        let c = Config::parse(DEFAULT_CONFIG).unwrap();
        c.save(&path).unwrap();
        let again = Config::load(&path).unwrap();
        assert_eq!(c.data, again.data);
        fs::remove_dir_all(&dir).unwrap();
    }
}
