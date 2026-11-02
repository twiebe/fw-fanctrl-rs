# fw-fanctrl-rs

A drop-in Rust reimplementation of [fw-fanctrl](https://github.com/TamtamHero/fw-fanctrl)
for Framework laptops, with **smooth (staggered) fan ramping**: the fan speeds up
gradually and slows down even more gradually, similar to a MacBook, instead of
jumping between curve points.

- Same configuration file (`/etc/fw-fanctrl/config.json`), same CLI, same command
  socket and output formats (NATURAL / JSON) as upstream. Existing configs work unchanged.
- New optional per-strategy keys for ramping (see below).
- Talks to the EC directly through the kernel's `/dev/cros_ec` with a hard-coded
  whitelist of three operations. It never spawns `framework_tool` or `ectool`.
- A single ~500 KB binary that needs only glibc, uses about 2 MB of RAM and does one
  tick per second.

## Configuration

Everything from upstream applies (`defaultStrategy`, `strategyOnDischarging`,
`strategies.*.fanSpeedUpdateFrequency`, `movingAverageInterval`, `speedCurve`).
Each strategy may additionally define:

| Key             | Unit | Default    | Meaning |
|-----------------|------|------------|---------|
| `rampUpRate`    | %/s  | instant    | Maximum increase of the fan duty per second. |
| `rampDownRate`  | %/s  | instant    | Maximum decrease of the fan duty per second. |
| `rampDownDelay` | s    | `0`        | How long the target has to stay below the current speed before the fan starts slowing down. Short load drops are ignored. |
| `criticalTemp`  | °C   | none       | At or above this *raw* temperature, averaging and ramp-up limiting are bypassed and the fan goes straight to the curve value. |

A strategy without these keys behaves exactly like upstream. Example (also shipped as
the `smooth` strategy in the default config):

```json
"smooth": {
    "fanSpeedUpdateFrequency": 2,
    "movingAverageInterval": 20,
    "speedCurve": [
        { "temp": 0,  "speed": 0 },
        { "temp": 50, "speed": 0 },
        { "temp": 60, "speed": 20 },
        { "temp": 70, "speed": 35 },
        { "temp": 78, "speed": 55 },
        { "temp": 85, "speed": 100 }
    ],
    "rampUpRate": 2.0,
    "rampDownRate": 0.5,
    "rampDownDelay": 30,
    "criticalTemp": 90
}
```

Rates are used instead of a fixed ramp duration on purpose. With a fixed duration,
a change from 20 to 25 % takes as long as one from 20 to 100 %. With a rate, small
corrections are quick and big jumps are spread out.

Differences in validation compared to upstream:

- Unknown keys are rejected, so a typo like `rampUpRtae` fails loudly instead of being ignored.
- `speedCurve` temperatures must be strictly ascending.
- `strategyOnDischarging` may be omitted (it is treated as `""`).

`resources/config.schema.json` is the upstream schema extended by the keys above.

## Commands

All upstream commands work as before:

```
fw-fanctrl [--output-format NATURAL|JSON] use <strategy> | reset | reload | pause | resume
fw-fanctrl print [all|active|current|list|speed]
fw-fanctrl set_config '<json>'
fw-fanctrl run [strategy] [--config PATH] [--silent]
```

`print all` additionally reports `TargetSpeed`, the fan RPM and whether the
fail-safe is active. `kill -HUP` (or `systemctl reload fw-fanctrl`) reloads the config.

The following extras run locally as root and do not talk to the service:

| Command | Purpose |
|---------|---------|
| `run --dry-run` | Read the real sensors and run the full control loop, but never write to the EC. |
| `run --simulate FILE` | No hardware at all. The temperature is read from `FILE`, for testing the loop anywhere. |
| `probe` | Print EC temperatures, fan RPM and AC state. |
| `test-fan <percent> [--seconds N]` | Set one duty for N seconds (default 5), then hand control back to the EC. |
| `restore-auto` | Hand fan control back to the EC. |

`FW_FANCTRL_SOCKET` overrides the socket path (default
`/run/fw-fanctrl/.fw-fanctrl.commands.sock`), which is useful for tests without root.

## Safety

- **Whitelist:** Only `EC_CMD_PWM_SET_FAN_DUTY` (0x24, clamped to 0–100) and
  `EC_CMD_THERMAL_AUTO_FAN_CTRL` (0x52) can be sent, plus read-only memory-map reads.
  The EC command interface is never exposed generically.
- **Sanity check at startup:** The EC memory map must identify itself as `EC`, or the
  service refuses to run.
- **Fail-safe means EC automatic control, not 100 %:**
  - after 3 consecutive read or write errors (control resumes when the hardware recovers),
  - on `pause`, on suspend (via `fw-fanctrl-suspend.service`) and on SIGTERM/SIGINT,
  - on panic (from a panic hook),
  - after any stop, including crashes and watchdog kills (`ExecStopPost=fw-fanctrl restore-auto`).
- **Watchdog:** The service runs as `Type=notify` with `WatchdogSec=15`, so a hung loop is restarted.
- **Periodic refresh:** An unchanged duty is re-sent every 30 s, in case the EC was reset to
  auto mode behind our back.
- **CPU protection is independent of this program:** The CPU throttles itself, and the
  firmware shuts the system down at critical temperatures, whatever the fan does.

## First test on the device

```sh
cargo build --release
B=target/release/fw-fanctrl

sudo $B probe                  # compare with: sudo framework_tool --thermal
sudo $B test-fan 30 --seconds 5   # fan audibly changes, then returns to EC control
sudo systemctl stop fw-fanctrl # stop the preinstalled service first
sudo $B run smooth --dry-run   # full loop, prints what it would set, writes nothing
```

## Installation (Bazzite / ostree)

```sh
cargo build --release
sudo ./install.sh              # or: sudo ./install.sh /path/to/fw-fanctrl
```

`install.sh` puts the binary in `/usr/local/bin`, which is writable on ostree and
comes before `/usr/bin` in `PATH`. It installs `fw-fanctrl.service` and
`fw-fanctrl-suspend.service` into `/etc/systemd/system`, where they override vendor
units of the same name. An existing `/etc/fw-fanctrl/config.json` is kept. Check the
vendor unit names with `systemctl list-unit-files 'fw-fanctrl*'`. If they differ,
disable those units manually. `sudo ./install.sh --uninstall` reverts to the vendor
service.

To make the smooth behaviour the default, set `"defaultStrategy": "smooth"` (or add the
ramp keys to your own strategy), then run `fw-fanctrl reload`.

## Development

```sh
cargo test
cargo clippy --all-targets
```

The control logic (`src/control.rs`) is pure and unit-tested. The EC layer (`src/ec.rs`)
follows the ChromiumOS EC headers and FrameworkComputer/framework-system.

## License

BSD-3-Clause, like upstream fw-fanctrl (the default strategies and output formats are
derived from it).
