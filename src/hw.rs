//! Hardware abstraction: the real EC, a dry-run wrapper and a simulator.

use std::fs;
use std::path::PathBuf;

use crate::ec::{CrosEc, EcCommand, Fan};
use crate::error::{Result, bail};

pub trait Hardware: Send {
    /// Highest valid sensor temperature in °C.
    fn temperature(&mut self) -> Result<f64>;
    fn set_speed(&mut self, percent: u8) -> Result<()>;
    /// Hand fan control back to the EC.
    fn auto_fan(&mut self) -> Result<()>;
    fn on_ac(&mut self) -> Result<bool>;
    fn fans(&mut self) -> Result<Vec<Fan>>;
    /// True if this backend touches real hardware with writes.
    fn is_live(&self) -> bool;
}

fn max_temp(temps: &[i32]) -> Result<f64> {
    match temps.iter().copied().filter(|t| *t > 0).max() {
        Some(t) => Ok(t as f64),
        None => bail!("no valid temperature sensor reading"),
    }
}

pub struct Ec(pub CrosEc);

impl Hardware for Ec {
    fn temperature(&mut self) -> Result<f64> {
        max_temp(&self.0.temperatures()?)
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
}

/// Reads from the real EC, but only logs writes.
pub struct DryRun(pub CrosEc);

impl Hardware for DryRun {
    fn temperature(&mut self) -> Result<f64> {
        max_temp(&self.0.temperatures()?)
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
}

/// No hardware at all. Temperature is read from a file (one number in °C),
/// so the control loop can be exercised on any machine.
pub struct Simulated {
    pub temp_file: PathBuf,
    pub speed: u8,
}

impl Hardware for Simulated {
    fn temperature(&mut self) -> Result<f64> {
        let raw = fs::read_to_string(&self.temp_file)?;
        match raw.trim().parse::<f64>() {
            Ok(t) if t > 0.0 => Ok(t),
            _ => bail!("simulated temperature file contains no valid value"),
        }
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
