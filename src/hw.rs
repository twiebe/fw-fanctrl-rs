//! Hardware abstraction: the real EC, a dry-run wrapper and a simulator.

use std::fs;
use std::path::PathBuf;

use crate::ec::{CrosEc, EcCommand, Fan};
use crate::error::{Result, bail};

pub trait Hardware: Send {
    /// Temperature per sensor index in °C (`None` = not present / invalid).
    fn temperatures(&mut self) -> Result<Vec<Option<f64>>>;
    fn set_speed(&mut self, percent: u8) -> Result<()>;
    /// Hand fan control back to the EC.
    fn auto_fan(&mut self) -> Result<()>;
    fn on_ac(&mut self) -> Result<bool>;
    fn fans(&mut self) -> Result<Vec<Fan>>;
    /// True if this backend touches real hardware with writes.
    fn is_live(&self) -> bool;
    /// EC "high" threshold in °C per sensor index for the first `n` sensors
    /// (`None` = unknown or disabled).
    fn high_thresholds(&mut self, _n: usize) -> Vec<Option<f64>> {
        Vec::new()
    }
}

fn ec_high_thresholds(ec: &CrosEc, n: usize) -> Vec<Option<f64>> {
    (0..n.min(255))
        .map(|i| {
            ec.thresholds(i as u8)
                .ok()
                .and_then(|t| t.high)
                .map(f64::from)
        })
        .collect()
}

fn to_f64(temps: Vec<Option<i32>>) -> Vec<Option<f64>> {
    temps.into_iter().map(|t| t.map(f64::from)).collect()
}

/// Highest valid reading among `sensors` (all sensors if `None`).
pub fn select_max(temps: &[Option<f64>], sensors: Option<&[u8]>) -> Result<f64> {
    let valid = |i: usize| temps.get(i).copied().flatten().filter(|t| *t > 0.0);
    let max = match sensors {
        Some(idx) => idx
            .iter()
            .filter_map(|&i| valid(i as usize))
            .reduce(f64::max),
        None => (0..temps.len()).filter_map(valid).reduce(f64::max),
    };
    match (max, sensors) {
        (Some(t), _) => Ok(t),
        (None, Some(idx)) => bail!("no valid temperature reading from sensors {idx:?}"),
        (None, None) => bail!("no valid temperature sensor reading"),
    }
}

pub struct Ec(pub CrosEc);

impl Hardware for Ec {
    fn temperatures(&mut self) -> Result<Vec<Option<f64>>> {
        Ok(to_f64(self.0.temperatures()?))
    }
    fn set_speed(&mut self, percent: u8) -> Result<()> {
        self.0.send(EcCommand::SetFanDuty(percent))
    }
    fn auto_fan(&mut self) -> Result<()> {
        self.0.send(EcCommand::AutoFanCtrl)
    }
    fn on_ac(&mut self) -> Result<bool> {
        self.0.on_ac()
    }
    fn fans(&mut self) -> Result<Vec<Fan>> {
        self.0.fans()
    }
    fn is_live(&self) -> bool {
        true
    }
    fn high_thresholds(&mut self, n: usize) -> Vec<Option<f64>> {
        ec_high_thresholds(&self.0, n)
    }
}

/// Reads from the real EC, but only logs writes.
pub struct DryRun(pub CrosEc);

impl Hardware for DryRun {
    fn temperatures(&mut self) -> Result<Vec<Option<f64>>> {
        Ok(to_f64(self.0.temperatures()?))
    }
    fn set_speed(&mut self, percent: u8) -> Result<()> {
        eprintln!("[dry-run] would set fan duty to {percent}%");
        Ok(())
    }
    fn auto_fan(&mut self) -> Result<()> {
        eprintln!("[dry-run] would restore EC automatic fan control");
        Ok(())
    }
    fn on_ac(&mut self) -> Result<bool> {
        self.0.on_ac()
    }
    fn fans(&mut self) -> Result<Vec<Fan>> {
        self.0.fans()
    }
    fn is_live(&self) -> bool {
        false
    }
    fn high_thresholds(&mut self, n: usize) -> Vec<Option<f64>> {
        ec_high_thresholds(&self.0, n)
    }
}

/// No hardware at all. Temperatures are read from a file: whitespace-separated
/// values in °C, one per sensor index (`-` = no reading), e.g. `38 45 - 36 72`.
/// Lets the control loop run on any machine.
pub struct Simulated {
    pub temp_file: PathBuf,
    pub speed: u8,
}

impl Hardware for Simulated {
    fn temperatures(&mut self) -> Result<Vec<Option<f64>>> {
        let raw = fs::read_to_string(&self.temp_file)?;
        Ok(raw
            .split_whitespace()
            .map(|v| v.parse::<f64>().ok())
            .collect())
    }
    fn set_speed(&mut self, percent: u8) -> Result<()> {
        self.speed = percent;
        Ok(())
    }
    fn auto_fan(&mut self) -> Result<()> {
        eprintln!("[simulated] restore EC automatic fan control");
        Ok(())
    }
    fn on_ac(&mut self) -> Result<bool> {
        Ok(true)
    }
    fn fans(&mut self) -> Result<Vec<Fan>> {
        Ok(vec![Fan::Rpm(self.speed as u16 * 60)])
    }
    fn is_live(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensor_selection() {
        let t = [Some(38.0), Some(45.0), None, Some(36.0), Some(72.0)];
        assert_eq!(select_max(&t, None).unwrap(), 72.0);
        assert_eq!(select_max(&t, Some(&[0, 1, 3])).unwrap(), 45.0);
        assert_eq!(select_max(&t, Some(&[0, 2, 9])).unwrap(), 38.0);
        assert!(select_max(&t, Some(&[2, 9])).is_err());
        assert!(select_max(&[None, Some(0.0)], None).is_err());
    }
}
