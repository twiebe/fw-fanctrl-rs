//! Minimal access to the Framework EC through the kernel's `/dev/cros_ec`.
//!
//! Safety design: this module can only send the host commands listed in
//! [`EcCommand`]: two writes (fan duty, automatic fan control) and two
//! read-only queries (sensor info, thermal thresholds). There is no generic
//! pass-through, so nothing in this program can ever reach flash, reboot,
//! battery or threshold-setting commands.
//!
//! Layouts follow the ChromiumOS EC headers (`ec_commands.h`), the kernel's
//! `cros_ec_chardev.h` and FrameworkComputer/framework-system.

use std::fs::File;
use std::os::fd::AsRawFd;

use crate::error::{Error, Result, bail};

pub const DEV_PATH: &str = "/dev/cros_ec";

// --- kernel ioctl interface (include/linux/platform_data/cros_ec_chardev.h)

const EC_MEMMAP_SIZE: usize = 255;

#[repr(C)]
struct CmdHeader {
    version: u32,
    command: u32,
    outsize: u32,
    insize: u32,
    result: u32,
    // followed by `data[]`
}

#[repr(C)]
struct CmdBuf {
    hdr: CmdHeader,
    data: [u8; CMD_DATA_SIZE],
}

/// Largest request is 4 bytes, largest response 33 bytes.
const CMD_DATA_SIZE: usize = 64;

#[repr(C)]
struct ReadMem {
    offset: u32,
    bytes: u32,
    buffer: [u8; EC_MEMMAP_SIZE],
}

const fn iowr(ty: u32, nr: u32, size: usize) -> u32 {
    (3 << 30) | ((size as u32) << 16) | (ty << 8) | nr
}
const CROS_EC_DEV_IOCXCMD: u32 = iowr(0xEC, 0, size_of::<CmdHeader>());
const CROS_EC_DEV_IOCRDMEM: u32 = iowr(0xEC, 1, size_of::<ReadMem>());

// --- EC memory map (ec_commands.h)

const EC_MEMMAP_TEMP_SENSOR: u32 = 0x00; // 16 sensors
const EC_MEMMAP_FAN: u32 = 0x10; // 4 x u16 RPM
const EC_MEMMAP_TEMP_SENSOR_B: u32 = 0x18; // 8 more sensors (thermal version >= 2)
const EC_MEMMAP_ID: u32 = 0x20; // "EC"
const EC_MEMMAP_THERMAL_VERSION: u32 = 0x23;
const EC_MEMMAP_BATT_FLAG: u32 = 0x4c;
const EC_BATT_FLAG_AC_PRESENT: u8 = 0x01;

/// Raw temperature values are Kelvin - 200 (EC_TEMP_SENSOR_OFFSET), so °C = raw - 73.
const TEMP_RAW_TO_CELSIUS: i32 = 200 - 273;
// 0xFC..=0xFF: not calibrated / not powered / error / not present
const TEMP_FIRST_INVALID: u8 = 0xFC;

const FAN_NOT_PRESENT: u16 = 0xFFFF;
const FAN_STALLED: u16 = 0xFFFE;

// --- the whitelist

/// The only host commands this program can send.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EcCommand {
    /// EC_CMD_PWM_SET_FAN_DUTY (0x0024) v0: all fans to `percent` (manual mode).
    SetFanDuty(u8),
    /// EC_CMD_THERMAL_AUTO_FAN_CTRL (0x0052) v0: hand control back to the EC.
    AutoFanCtrl,
    /// EC_CMD_TEMP_SENSOR_GET_INFO (0x0070) v0, read-only: sensor name.
    TempSensorInfo(u8),
    /// EC_CMD_THERMAL_GET_THRESHOLD (0x0051) v1, read-only. (0x0050 would *set*
    /// thresholds and is deliberately not available.)
    ThermalThreshold(u8),
}

impl EcCommand {
    /// (command id, version, request payload, expected response size)
    fn encode(self) -> Result<(u32, u32, Vec<u8>, usize)> {
        Ok(match self {
            EcCommand::SetFanDuty(p) => {
                if p > 100 {
                    bail!("refusing fan duty {p}% (> 100)");
                }
                (0x0024, 0, (p as u32).to_le_bytes().to_vec(), 0)
            }
            EcCommand::AutoFanCtrl => (0x0052, 0, Vec::new(), 0),
            EcCommand::TempSensorInfo(id) => (0x0070, 0, vec![id], 33),
            EcCommand::ThermalThreshold(id) => (0x0051, 1, (id as u32).to_le_bytes().to_vec(), 32),
        })
    }
}

/// EC thermal configuration of one sensor, in °C (`None` = disabled).
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct Thresholds {
    pub warn: Option<i32>,
    pub high: Option<i32>,
    pub halt: Option<i32>,
    pub fan_off: Option<i32>,
    pub fan_max: Option<i32>,
}

fn decode_thresholds(raw: &[u8]) -> Result<Thresholds> {
    if raw.len() < 32 {
        bail!("short thermal threshold response ({} bytes)", raw.len());
    }
    let k = |i: usize| u32::from_le_bytes(raw[i * 4..i * 4 + 4].try_into().unwrap());
    let c = |kelvin: u32| (kelvin != 0).then(|| kelvin as i32 - 273);
    // layout: temp_host[3], temp_host_release[3], temp_fan_off, temp_fan_max
    Ok(Thresholds {
        warn: c(k(0)),
        high: c(k(1)),
        halt: c(k(2)),
        fan_off: c(k(6)),
        fan_max: c(k(7)),
    })
}

fn decode_name(raw: &[u8]) -> String {
    let name = &raw[..raw.len().min(32)];
    let end = name.iter().position(|&b| b == 0).unwrap_or(name.len());
    String::from_utf8_lossy(&name[..end]).into_owned()
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Fan {
    Rpm(u16),
    Stalled,
}

pub struct CrosEc {
    file: File,
}

impl CrosEc {
    pub fn open() -> Result<CrosEc> {
        let file = File::open(DEV_PATH).map_err(|e| {
            Error(format!(
                "cannot open {DEV_PATH}: {e} (is the cros_ec_lpcs module loaded and are you root?)"
            ))
        })?;
        let ec = CrosEc { file };
        // Sanity check: the memory map must start its ID block with "EC".
        let id = ec.read_mem(EC_MEMMAP_ID, 2)?;
        if id != b"EC" {
            bail!("unexpected EC memory map id {id:02x?}, refusing to continue");
        }
        Ok(ec)
    }

    pub fn send(&self, cmd: EcCommand) -> Result<()> {
        self.xfer(cmd).map(|_| ())
    }

    pub fn sensor_name(&self, id: u8) -> Result<String> {
        Ok(decode_name(&self.xfer(EcCommand::TempSensorInfo(id))?))
    }

    pub fn thresholds(&self, id: u8) -> Result<Thresholds> {
        decode_thresholds(&self.xfer(EcCommand::ThermalThreshold(id))?)
    }

    fn xfer(&self, cmd: EcCommand) -> Result<Vec<u8>> {
        let (command, version, payload, insize) = cmd.encode()?;
        let mut buf = CmdBuf {
            hdr: CmdHeader {
                version,
                command,
                outsize: payload.len() as u32,
                insize: insize as u32,
                result: 0xFF,
            },
            data: [0; CMD_DATA_SIZE],
        };
        buf.data[..payload.len()].copy_from_slice(&payload);
        // SAFETY: `buf` is a properly laid out cros_ec_command followed by a data
        // buffer larger than both outsize and insize; the kernel bounds its copies
        // by those sizes.
        let ret = unsafe { libc::ioctl(self.file.as_raw_fd(), CROS_EC_DEV_IOCXCMD as _, &mut buf) };
        if ret < 0 {
            bail!(
                "EC command {cmd:?} failed: {}",
                std::io::Error::last_os_error()
            );
        }
        if buf.hdr.result != 0 {
            bail!("EC rejected {cmd:?} with status {}", buf.hdr.result);
        }
        // the ioctl returns the number of response bytes
        Ok(buf.data[..(ret as usize).min(insize)].to_vec())
    }

    /// Read-only access to the EC memory map.
    fn read_mem(&self, offset: u32, len: u32) -> Result<Vec<u8>> {
        if (offset + len) as usize > EC_MEMMAP_SIZE {
            bail!("memmap read out of range");
        }
        let mut rm = ReadMem {
            offset,
            bytes: len,
            buffer: [0; EC_MEMMAP_SIZE],
        };
        // SAFETY: `rm` matches struct cros_ec_readmem; the kernel writes at most
        // `bytes` (<= EC_MEMMAP_SIZE) into `buffer`.
        let ret = unsafe { libc::ioctl(self.file.as_raw_fd(), CROS_EC_DEV_IOCRDMEM as _, &mut rm) };
        if ret < 0 {
            bail!("EC memmap read failed: {}", std::io::Error::last_os_error());
        }
        if (ret as u32) < len {
            bail!("EC memmap read returned {ret} of {len} bytes");
        }
        Ok(rm.buffer[..len as usize].to_vec())
    }

    /// Temperature per sensor index in °C (`None` = not present / invalid).
    /// Indices match `framework_tool --thermal` / `--thermalget`.
    pub fn temperatures(&self) -> Result<Vec<Option<i32>>> {
        let mut raw = self.read_mem(EC_MEMMAP_TEMP_SENSOR, 16)?;
        if self.read_mem(EC_MEMMAP_THERMAL_VERSION, 1)?[0] >= 2 {
            raw.extend(self.read_mem(EC_MEMMAP_TEMP_SENSOR_B, 8)?);
        }
        Ok(decode_temps(&raw))
    }

    pub fn fans(&self) -> Result<Vec<Fan>> {
        Ok(decode_fans(&self.read_mem(EC_MEMMAP_FAN, 8)?))
    }

    pub fn on_ac(&self) -> Result<bool> {
        Ok(self.read_mem(EC_MEMMAP_BATT_FLAG, 1)?[0] & EC_BATT_FLAG_AC_PRESENT != 0)
    }
}

fn decode_temps(raw: &[u8]) -> Vec<Option<i32>> {
    raw.iter()
        .map(|&b| (b < TEMP_FIRST_INVALID).then_some(b as i32 + TEMP_RAW_TO_CELSIUS))
        .collect()
}

fn decode_fans(raw: &[u8]) -> Vec<Fan> {
    raw.as_chunks::<2>()
        .0
        .iter()
        .map(|c| u16::from_le_bytes(*c))
        .filter(|&v| v != FAN_NOT_PRESENT)
        .map(|v| {
            if v == FAN_STALLED {
                Fan::Stalled
            } else {
                Fan::Rpm(v)
            }
        })
        .collect()
}

/// Best effort, used from the panic hook and `restore-auto`: open a fresh
/// handle and give fan control back to the EC.
pub fn restore_auto() -> Result<()> {
    let file = File::open(DEV_PATH)?;
    CrosEc { file }.send(EcCommand::AutoFanCtrl)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ioctl_numbers_match_kernel() {
        // _IOWR(0xEC, 0, struct cros_ec_command) / _IOWR(0xEC, 1, struct cros_ec_readmem)
        assert_eq!(size_of::<CmdHeader>(), 20);
        assert_eq!(size_of::<ReadMem>(), 264);
        assert_eq!(CROS_EC_DEV_IOCXCMD, 0xC014_EC00);
        assert_eq!(CROS_EC_DEV_IOCRDMEM, 0xC108_EC01);
    }

    #[test]
    fn whitelist_encoding() {
        assert_eq!(
            EcCommand::SetFanDuty(42).encode().unwrap(),
            (0x24, 0, vec![42, 0, 0, 0], 0)
        );
        assert_eq!(
            EcCommand::AutoFanCtrl.encode().unwrap(),
            (0x52, 0, vec![], 0)
        );
        assert_eq!(
            EcCommand::TempSensorInfo(3).encode().unwrap(),
            (0x70, 0, vec![3], 33)
        );
        assert_eq!(
            EcCommand::ThermalThreshold(1).encode().unwrap(),
            (0x51, 1, vec![1, 0, 0, 0], 32)
        );
        assert!(EcCommand::SetFanDuty(101).encode().is_err());
    }

    #[test]
    fn threshold_and_name_decoding() {
        // cpu_f75303@4d from a Framework 13 Pro: - 88 98 / fan 40..78
        let kelvin = [0u32, 361, 371, 0, 0, 0, 313, 351];
        let raw: Vec<u8> = kelvin.iter().flat_map(|k| k.to_le_bytes()).collect();
        assert_eq!(
            decode_thresholds(&raw).unwrap(),
            Thresholds {
                warn: None,
                high: Some(88),
                halt: Some(98),
                fan_off: Some(40),
                fan_max: Some(78)
            }
        );
        let mut name = b"cpu_f75303@4d".to_vec();
        name.resize(33, 0);
        assert_eq!(decode_name(&name), "cpu_f75303@4d");
    }

    #[test]
    fn temperature_decoding() {
        // 0x7f = 127 → 54 °C; 0x89 → 64 °C; invalid markers keep their index as None
        assert_eq!(
            decode_temps(&[0x7f, 0xff, 0x89, 0xfe, 0xfd, 0xfc]),
            [Some(54), None, Some(64), None, None, None]
        );
    }

    #[test]
    fn fan_decoding() {
        assert_eq!(
            decode_fans(&[0x10, 0x0e, 0xff, 0xff, 0xfe, 0xff, 0, 0]),
            [Fan::Rpm(3600), Fan::Stalled, Fan::Rpm(0)]
        );
    }
}
