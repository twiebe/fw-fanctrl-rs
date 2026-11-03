//! Pure control logic: temperature history, curve target and ramp limiting.
//! No I/O here, so everything is unit-testable.

use std::collections::VecDeque;

use crate::config::Strategy;

const HISTORY_LEN: usize = 100;
/// With `minSpeed`: once off, stay off at least this long. The EC only kick-starts
/// the fan if it has seen 0 % for a moment; a quick off → on would leave it stalled.
pub const MIN_OFF_SECS: f64 = 3.0;
/// Comfort controller: integral gain in %/(°C·s). Slow on purpose: the board
/// sensors lag by minutes, and a hunting fan is worse than a steady one.
pub const COMFORT_KI: f64 = 0.05;
/// Backing off is twice as fast: too much fan is audible, too little only means
/// the board warms up slowly and the controller comes back.
pub const COMFORT_KI_DOWN: f64 = 0.1;
/// The comfort controller only adds fan up to this far above its target; beyond
/// that the curve alone is in charge (load).
pub const COMFORT_ZONE: f64 = 4.0;
/// No correction while the temperature is within this of the comfort target
/// (sensors report whole degrees).
pub const COMFORT_DEADBAND: f64 = 0.5;
pub const COMFORT_MAX_SPEED: u8 = 30;

pub fn round2(v: f64) -> f64 {
    (v * 100.0).round() / 100.0
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Tick {
    pub temperature: f64,
    pub moving_average: f64,
    pub effective: f64,
    /// Curve speed the controller is heading to.
    pub target: u8,
    /// Speed to apply to the fan now.
    pub output: u8,
    pub critical: bool,
    /// Output of the comfort controller, if the strategy has `comfortTemp`.
    pub comfort: Option<u8>,
    /// A sensor reached its EC "high" threshold (minus margin): fan at 100 %.
    pub emergency: bool,
}

#[derive(Debug, Clone)]
pub struct Controller {
    history: VecDeque<f64>,
    timecount: u32,
    target: f64,
    /// Curve part of `target` (re-evaluated every `fanSpeedUpdateFrequency` ticks).
    curve_target: f64,
    /// Comfort controller state (percent).
    comfort: f64,
    output: Option<f64>,
    /// Seconds the target has been continuously below the output.
    below_for: f64,
    /// Seconds the output has been 0.
    off_for: f64,
}

impl Default for Controller {
    fn default() -> Self {
        Self::new()
    }
}

impl Controller {
    pub fn new() -> Self {
        Controller {
            history: VecDeque::from(vec![0.0; HISTORY_LEN]),
            timecount: 0,
            target: 0.0,
            curve_target: 0.0,
            comfort: 0.0,
            output: None,
            below_for: 0.0,
            off_for: MIN_OFF_SECS, // unknown history: allow an immediate start
        }
    }

    /// Forget history and output, e.g. after a pause or fail-safe: the conditions
    /// may have changed completely, so the next tick starts at the curve target
    /// instead of ramping from a stale value.
    pub fn reset(&mut self) {
        *self = Controller::new();
    }

    /// Upstream: force a curve re-evaluation on the next tick (strategy change).
    /// The ramp continues from the current output, so there is no jump.
    pub fn reset_timecount(&mut self) {
        self.timecount = 0;
    }

    pub fn output(&self) -> Option<u8> {
        self.output.map(|o| o.round() as u8)
    }

    /// The temperature dropped by more than 0.5 °C within 80 s (mean of the last
    /// 20 readings vs. 80..100 s ago): about 0.4 °C/min or faster.
    fn falling(&self) -> bool {
        let valid: Vec<f64> = self.history.iter().copied().filter(|t| *t > 0.0).collect();
        let n = valid.len();
        if n < 100 {
            return false;
        }
        let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;
        mean(&valid[n - 20..]) - mean(&valid[n - 100..n - 80]) < -0.5
    }

    /// Mean of the last `interval` valid (> 0) readings, upstream semantics.
    pub fn moving_average(&self, interval: u32, current: f64) -> f64 {
        let valid: Vec<f64> = self.history.iter().copied().filter(|t| *t > 0.0).collect();
        let n = (interval as usize).min(valid.len());
        if n == 0 {
            return current;
        }
        let slice = &valid[valid.len() - n..];
        round2(slice.iter().sum::<f64>() / n as f64)
    }

    /// One control step. Upstream runs this once per second (`dt` = 1.0).
    /// `emergency` (any sensor near its EC limit, independent of the strategy's
    /// sensor selection) forces 100 % without ramping.
    pub fn tick(&mut self, s: &Strategy, temp: f64, dt: f64, emergency: bool) -> Tick {
        let avg = self.moving_average(s.moving_average_interval, temp);
        // Upstream: effective temp = min(moving average, current).
        let effective = round2(avg.min(temp));
        let critical = emergency || s.ramp.critical_temp.is_some_and(|c| temp >= c);

        if emergency {
            self.curve_target = 100.0;
            self.target = 100.0;
        } else if critical {
            // Emergency: follow the raw temperature, re-evaluated every tick.
            self.curve_target = s.target_speed(temp) as f64;
            self.target = self.curve_target;
        } else {
            if self.timecount.is_multiple_of(s.fan_speed_update_frequency) {
                self.curve_target = s.target_speed(effective) as f64;
                self.timecount = 0;
            }
            self.target = self.curve_target;
            if let Some(ct) = s.ramp.comfort_temp {
                let max = s.ramp.comfort_max_speed.unwrap_or(COMFORT_MAX_SPEED) as f64;
                let falling = self.falling();
                self.comfort =
                    comfort_step(self.comfort, avg, ct, max, self.curve_target, falling, dt);
                self.target = self.target.max(self.comfort.round());
            }
        }

        let target = self.target;
        let prev = self.output;
        let out = match self.output {
            None => target, // first tick: start at the target, nothing to ramp from
            Some(cur) if target > cur => {
                self.below_for = 0.0;
                match s.ramp.up_rate {
                    _ if critical => target,
                    Some(rate) => (cur + rate * dt).min(target),
                    None => target,
                }
            }
            Some(cur) if target < cur => {
                self.below_for += dt;
                if self.below_for + 1e-9 < s.ramp.down_delay {
                    cur
                } else {
                    match s.ramp.down_rate {
                        Some(rate) => (cur - rate * dt).max(target),
                        None => target,
                    }
                }
            }
            Some(cur) => {
                self.below_for = 0.0;
                cur
            }
        };
        let out = match s.ramp.min_speed {
            Some(min) => {
                let out = apply_min_speed(out, target, prev, min as f64);
                let just_stopped = prev.is_some_and(|p| p <= 0.0) && self.off_for < MIN_OFF_SECS;
                if out > 0.0 && just_stopped && !critical {
                    0.0
                } else {
                    out
                }
            }
            None => out,
        };
        if out <= 0.0 {
            self.off_for += dt;
        } else {
            self.off_for = 0.0;
        }
        self.output = Some(out);

        self.history.pop_front();
        self.history.push_back(temp);
        self.timecount += 1;

        Tick {
            temperature: temp,
            moving_average: avg,
            effective,
            target: target as u8,
            output: out.round() as u8,
            critical,
            comfort: s.ramp.comfort_temp.map(|_| self.comfort.round() as u8),
            emergency,
        }
    }
}

/// One step of the comfort integral controller. Outside the deadband it moves by
/// `COMFORT_KI` (up) or `COMFORT_KI_DOWN` (down) per degree of error and second.
/// It never adds fan while the curve already asks for more, above the comfort
/// zone (load: the curve's job), or while the temperature is falling anyway
/// (after load), so it does not wind up and is at its idle value right after.
/// While falling to or below the target it backs off even inside the deadband:
/// what light load made necessary is not needed any more.
fn comfort_step(
    u: f64,
    temp: f64,
    target: f64,
    max: f64,
    curve: f64,
    falling: bool,
    dt: f64,
) -> f64 {
    let err = temp - target;
    if falling && err <= COMFORT_DEADBAND {
        // cooling down to the target or below: the extra fan was for a heat load
        // that is gone, back off as if 1 °C too cool (also inside the deadband)
        return (u - COMFORT_KI_DOWN * dt).max(0.0);
    }
    if err.abs() <= COMFORT_DEADBAND {
        return u;
    }
    let e = err - COMFORT_DEADBAND.copysign(err);
    if e > 0.0 {
        if curve > u || falling || err > COMFORT_ZONE {
            return u;
        }
        (u + COMFORT_KI * e * dt).min(max)
    } else {
        (u + COMFORT_KI_DOWN * e * dt).max(0.0)
    }
}

/// Keep the output out of the band (0, min): from standstill only start once the
/// target reaches `min` (and then jump to it); while running never go below `min`
/// unless the target is 0.
fn apply_min_speed(out: f64, target: f64, prev: Option<f64>, min: f64) -> f64 {
    let was_off = prev.is_none_or(|p| p <= 0.0);
    if out <= 0.0 {
        0.0
    } else if was_off {
        if target >= min { out.max(min) } else { 0.0 }
    } else if out < min {
        if target > 0.0 { min } else { 0.0 }
    } else {
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{CurvePoint, Ramp};

    fn strategy(freq: u32, mai: u32, ramp: Ramp) -> Strategy {
        Strategy {
            name: "t".into(),
            fan_speed_update_frequency: freq,
            moving_average_interval: mai,
            // linear: 40°C → 0 %, 90°C → 100 %  (2 % per °C)
            speed_curve: vec![
                CurvePoint {
                    temp: 40.0,
                    speed: 0,
                },
                CurvePoint {
                    temp: 90.0,
                    speed: 100,
                },
            ],
            ramp,
            sensors: None,
        }
    }

    fn run(c: &mut Controller, s: &Strategy, temps: &[f64]) -> Vec<u8> {
        temps
            .iter()
            .map(|t| c.tick(s, *t, 1.0, false).output)
            .collect()
    }

    #[test]
    fn without_ramp_behaves_like_upstream() {
        let s = strategy(1, 1, Ramp::default());
        let mut c = Controller::new();
        // tick 1: no history → avg = current
        assert_eq!(c.tick(&s, 60.0, 1.0, false).output, 40);
        // tick 2: avg over last 1 reading = 60, current 80 → effective 60
        assert_eq!(c.tick(&s, 80.0, 1.0, false).output, 40);
        // tick 3: avg 80, current 80
        assert_eq!(c.tick(&s, 80.0, 1.0, false).output, 80);
        // drop: min(avg, current) reacts immediately downwards
        assert_eq!(c.tick(&s, 50.0, 1.0, false).output, 20);
    }

    #[test]
    fn update_frequency_holds_target() {
        let s = strategy(3, 1, Ramp::default());
        let mut c = Controller::new();
        let out = run(&mut c, &s, &[60.0, 90.0, 90.0, 90.0, 90.0]);
        // re-evaluated only on ticks 1 and 4
        assert_eq!(out, [40, 40, 40, 100, 100]);
    }

    #[test]
    fn ramps_up_and_down_with_rates() {
        let ramp = Ramp {
            up_rate: Some(5.0),
            down_rate: Some(2.0),
            ..Default::default()
        };
        let s = strategy(1, 1, ramp);
        let mut c = Controller::new();
        run(&mut c, &s, &[40.0, 40.0]); // settle at 0 %
        let up = run(&mut c, &s, &[90.0; 25]);
        // first 90°C tick still sees avg 40 → target 0; then +5 %/s up to 100
        assert_eq!(&up[..5], &[0, 5, 10, 15, 20]);
        assert_eq!(*up.last().unwrap(), 100);
        let down = run(&mut c, &s, &[40.0; 5]);
        assert_eq!(down, [98, 96, 94, 92, 90]);
    }

    #[test]
    fn down_delay_holds_speed() {
        let ramp = Ramp {
            down_rate: Some(10.0),
            down_delay: 3.0,
            ..Default::default()
        };
        let s = strategy(1, 1, ramp);
        let mut c = Controller::new();
        run(&mut c, &s, &[90.0, 90.0]);
        let out = run(&mut c, &s, &[40.0; 6]);
        assert_eq!(out, [100, 100, 90, 80, 70, 60]);
    }

    #[test]
    fn rising_target_resets_down_delay() {
        let ramp = Ramp {
            down_rate: Some(10.0),
            down_delay: 3.0,
            ..Default::default()
        };
        let s = strategy(1, 1, ramp);
        let mut c = Controller::new();
        run(&mut c, &s, &[90.0, 90.0]);
        // The first hot tick still sees the old average (upstream min(avg, current)),
        // so the delay expires once; the rising target then resets the timer.
        let out = run(
            &mut c,
            &s,
            &[40.0, 40.0, 40.0, 90.0, 90.0, 40.0, 40.0, 40.0, 40.0],
        );
        assert_eq!(out, [100, 100, 90, 80, 100, 100, 100, 90, 80]);
    }

    #[test]
    fn critical_temp_bypasses_ramp_and_average() {
        let ramp = Ramp {
            up_rate: Some(1.0),
            critical_temp: Some(85.0),
            ..Default::default()
        };
        let s = strategy(5, 30, ramp);
        let mut c = Controller::new();
        run(&mut c, &s, &[50.0; 10]); // 20 %
        let t = c.tick(&s, 88.0, 1.0, false);
        assert!(t.critical);
        assert_eq!(t.output, 96); // raw 88°C on the curve, no ramp, no averaging
        // just below critical: averaging + ramp apply again, output holds
        let t = c.tick(&s, 84.0, 1.0, false);
        assert!(!t.critical);
        assert_eq!(t.output, 96);
    }

    #[test]
    fn strategy_change_does_not_jump() {
        let ramp = Ramp {
            up_rate: Some(2.0),
            down_rate: Some(1.0),
            ..Default::default()
        };
        let slow = strategy(1, 1, ramp);
        let mut c = Controller::new();
        run(&mut c, &slow, &[50.0, 50.0]); // 20 %
        let mut steep = slow.clone();
        steep.speed_curve[0].temp = 0.0; // shift curve upwards
        c.reset_timecount();
        let out = run(&mut c, &steep, &[50.0; 3]);
        assert_eq!(out, [22, 24, 26]);
    }

    #[test]
    fn emergency_forces_full_speed_and_reset_forgets_state() {
        let ramp = Ramp {
            up_rate: Some(0.5),
            down_rate: Some(0.2),
            down_delay: 30.0,
            ..Default::default()
        };
        let s = strategy(1, 20, ramp);
        let mut c = Controller::new();
        run(&mut c, &s, &[45.0; 10]); // 10 %
        let t = c.tick(&s, 45.0, 1.0, true);
        assert!(t.emergency && t.critical);
        assert_eq!(t.output, 100);
        // stale state after a pause: reset starts at the curve target of the new reading
        c.reset();
        assert_eq!(c.tick(&s, 85.0, 1.0, false).output, 90);
    }

    #[test]
    fn min_speed_skips_the_stall_band() {
        let ramp = Ramp {
            up_rate: Some(1.0),
            down_rate: Some(1.0),
            min_speed: Some(10),
            ..Default::default()
        };
        let s = strategy(1, 1, ramp);
        let mut c = Controller::new();
        // 40 °C → target 0: off. 43 °C → target 6 (< min): stays off
        assert_eq!(run(&mut c, &s, &[40.0, 43.0, 43.0]), [0, 0, 0]);
        // 46 °C → target 12 (one tick later, upstream's min(avg, current)): start,
        // jumping straight to 10 instead of ramping 1, 2, 3 …
        assert_eq!(run(&mut c, &s, &[46.0, 46.0, 46.0, 46.0]), [0, 10, 11, 12]);
        // back to a target of 6: ramp down, then hold 10 while the target is > 0
        assert_eq!(run(&mut c, &s, &[43.0, 43.0, 43.0, 43.0]), [11, 10, 10, 10]);
        // target 0: off without passing through the stall band
        assert_eq!(run(&mut c, &s, &[40.0]), [0]);
        // restarting right away is held off for MIN_OFF_SECS (EC needs to see 0 %)
        assert_eq!(run(&mut c, &s, &[50.0, 50.0, 50.0, 50.0]), [0, 0, 10, 11]);
        // without minSpeed the ramp walks through the band
        let plain = strategy(
            1,
            1,
            Ramp {
                up_rate: Some(1.0),
                ..Default::default()
            },
        );
        let mut c = Controller::new();
        assert_eq!(run(&mut c, &plain, &[40.0, 46.0, 46.0]), [0, 0, 1]);
    }

    fn comfort_strategy(off_until: f64, min_speed: Option<u8>) -> Strategy {
        let mut s = strategy(
            1,
            1,
            Ramp {
                min_speed,
                comfort_temp: Some(41.0),
                ..Default::default()
            },
        );
        s.speed_curve = vec![
            CurvePoint {
                temp: off_until,
                speed: 0,
            },
            CurvePoint {
                temp: 90.0,
                speed: 100,
            },
        ];
        s
    }

    #[test]
    fn comfort_controller_adds_fan_slowly_and_backs_off() {
        let s = comfort_strategy(50.0, Some(10));
        let mut c = Controller::new();
        // 43 °C: 1.5 °C beyond the deadband → +0.075 %/s; off until minSpeed
        let out = run(&mut c, &s, &[43.0; 140]);
        assert_eq!(out[100], 0);
        assert_eq!(c.tick(&s, 43.0, 1.0, false).comfort, Some(11));
        assert_eq!(*out.last().unwrap(), 10);
        // slow approach (no "falling"): keeps adding while beyond the deadband
        let approach: Vec<f64> = (0..300).map(|i| 43.0 - 1.6 * i as f64 / 300.0).collect();
        run(&mut c, &s, &approach);
        // within the deadband nothing moves
        let held = run(&mut c, &s, &[41.4; 60]);
        assert!(held.windows(2).all(|w| w[0] == w[1]), "{held:?}");
        assert!(held[0] > 15, "{}", held[0]);
        // a sudden drop counts as falling: backs off faster at first, then by
        // 0.05 %/s, and holds minSpeed until it reaches 0
        let down = run(&mut c, &s, &[40.0; 500]);
        assert_eq!(down[150], 10);
        assert_eq!(*down.last().unwrap(), 0);
        assert_eq!(c.tick(&s, 40.0, 1.0, false).comfort, Some(0));
    }

    #[test]
    fn comfort_controller_does_not_wind_up_under_load() {
        // a curve that rises slowly above the comfort zone, like a calibrated one
        let mut s = comfort_strategy(45.0, Some(10));
        s.speed_curve.insert(
            1,
            CurvePoint {
                temp: 65.0,
                speed: 50,
            },
        );
        s.ramp.up_rate = Some(1.0);
        s.ramp.down_rate = Some(0.3);
        let mut c = Controller::new();
        run(&mut c, &s, &[43.0; 150]); // idle: ~11 %
        let idle = c.tick(&s, 43.0, 1.0, false).comfort.unwrap();
        assert!((10..=12).contains(&idle), "{idle}");
        // load: up to 65 °C within a minute, 5 minutes there, 10 minutes back to 41
        let mut temps: Vec<f64> = (0..60).map(|i| 43.0 + 22.0 * i as f64 / 60.0).collect();
        temps.extend([65.0; 300]);
        temps.extend((0..600).map(|i| 65.0 - 24.0 * i as f64 / 600.0));
        let mut peak = 0;
        for t in temps {
            peak = peak.max(c.tick(&s, t, 1.0, false).comfort.unwrap());
        }
        assert!(peak <= idle + 1, "comfort wound up to {peak} (idle {idle})");
        let after = run(&mut c, &s, &[41.0; 120]);
        assert!(*after.last().unwrap() <= idle + 1);
    }

    #[test]
    fn comfort_controller_backs_off_after_light_load() {
        // light load holds 44 °C (inside the zone, the curve is off): the
        // controller rightly goes to its cap; then the load ends
        let s = comfort_strategy(50.0, Some(10));
        let mut c = Controller::new();
        run(&mut c, &s, &[44.0; 1000]);
        assert_eq!(c.tick(&s, 44.0, 1.0, false).comfort, Some(30));
        let fall: Vec<f64> = (0..300).map(|i| 44.0 - 3.0 * i as f64 / 300.0).collect();
        run(&mut c, &s, &fall);
        // backed off a good part already while falling to the target
        let at_target = c.tick(&s, 41.0, 1.0, false).comfort.unwrap();
        assert!(at_target <= 25, "still at {at_target}% at the target");
        // the remaining surplus keeps cooling the board below the target
        run(&mut c, &s, &[41.0; 60]);
        run(&mut c, &s, &[40.0; 300]);
        let after = c.tick(&s, 40.0, 1.0, false).comfort.unwrap();
        assert!(after <= 10, "still at {after}%");
    }

    #[test]
    fn comfort_controller_is_capped() {
        let s = comfort_strategy(95.0, None);
        let mut c = Controller::new();
        let out = run(&mut c, &s, &[44.0; 2000]);
        assert_eq!(*out.last().unwrap(), COMFORT_MAX_SPEED);
    }

    #[test]
    fn moving_average_ignores_initial_zeros() {
        let c = Controller::new();
        assert_eq!(c.moving_average(30, 55.0), 55.0);
        let mut c = Controller::new();
        let s = strategy(1, 3, Ramp::default());
        run(&mut c, &s, &[50.0, 60.0]);
        assert_eq!(c.moving_average(3, 0.0), 55.0);
    }
}
