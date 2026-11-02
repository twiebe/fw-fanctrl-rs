//! Pure control logic: temperature history, curve target and ramp limiting.
//! No I/O here, so everything is unit-testable.

use std::collections::VecDeque;

use crate::config::Strategy;

const HISTORY_LEN: usize = 100;

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
}

#[derive(Debug, Clone)]
pub struct Controller {
    history: VecDeque<f64>,
    timecount: u32,
    target: f64,
    output: Option<f64>,
    /// Seconds the target has been continuously below the output.
    below_for: f64,
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
            output: None,
            below_for: 0.0,
        }
    }

    /// Upstream: force a curve re-evaluation on the next tick (strategy change).
    /// The ramp continues from the current output, so there is no jump.
    pub fn reset_timecount(&mut self) {
        self.timecount = 0;
    }

    pub fn output(&self) -> Option<u8> {
        self.output.map(|o| o.round() as u8)
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
    pub fn tick(&mut self, s: &Strategy, temp: f64, dt: f64) -> Tick {
        let avg = self.moving_average(s.moving_average_interval, temp);
        // Upstream: effective temp = min(moving average, current).
        let effective = round2(avg.min(temp));
        let critical = s.ramp.critical_temp.is_some_and(|c| temp >= c);

        if critical {
            // Emergency: follow the raw temperature, re-evaluated every tick.
            self.target = s.target_speed(temp) as f64;
        } else if self.timecount.is_multiple_of(s.fan_speed_update_frequency) {
            self.target = s.target_speed(effective) as f64;
            self.timecount = 0;
        }

        let target = self.target;
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
        }
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
        temps.iter().map(|t| c.tick(s, *t, 1.0).output).collect()
    }

    #[test]
    fn without_ramp_behaves_like_upstream() {
        let s = strategy(1, 1, Ramp::default());
        let mut c = Controller::new();
        // tick 1: no history → avg = current
        assert_eq!(c.tick(&s, 60.0, 1.0).output, 40);
        // tick 2: avg over last 1 reading = 60, current 80 → effective 60
        assert_eq!(c.tick(&s, 80.0, 1.0).output, 40);
        // tick 3: avg 80, current 80
        assert_eq!(c.tick(&s, 80.0, 1.0).output, 80);
        // drop: min(avg, current) reacts immediately downwards
        assert_eq!(c.tick(&s, 50.0, 1.0).output, 20);
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
        let t = c.tick(&s, 88.0, 1.0);
        assert!(t.critical);
        assert_eq!(t.output, 96); // raw 88°C on the curve, no ramp, no averaging
        // just below critical: averaging + ramp apply again, output holds
        let t = c.tick(&s, 84.0, 1.0);
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
    fn moving_average_ignores_initial_zeros() {
        let c = Controller::new();
        assert_eq!(c.moving_average(30, 55.0), 55.0);
        let mut c = Controller::new();
        let s = strategy(1, 3, Ramp::default());
        run(&mut c, &s, &[50.0, 60.0]);
        assert_eq!(c.moving_average(3, 0.0), 55.0);
    }
}
