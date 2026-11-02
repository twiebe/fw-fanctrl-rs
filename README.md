# fw-fanctrl-rs

A drop-in Rust reimplementation of [fw-fanctrl](https://github.com/TamtamHero/fw-fanctrl)
for Framework laptops, with **smooth (staggered) fan ramping**: the fan speeds up
gradually and slows down even more gradually, similar to a MacBook, instead of
jumping between curve points.

- Same configuration file (`/etc/fw-fanctrl/config.json`), same CLI, same command
  socket and output formats (NATURAL / JSON) as upstream. Existing configs work unchanged.
- New optional per-strategy keys for ramping (see below).
- Talks to the EC directly through the kernel's `/dev/cros_ec` with a hard-coded
  whitelist of four commands (two writes, two read-only queries). It never spawns
  `framework_tool` or `ectool`.
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
| `criticalTemp`  | °C   | none       | At or above this *raw* temperature (of the selected sensors), averaging and ramp-up limiting are bypassed and the fan goes straight to the curve value. |
| `minSpeed`      | %    | none       | Lowest duty the fan reliably runs at. Duties between 0 and this are never sent. From standstill the fan starts only once the target reaches `minSpeed`, and then jumps straight to it. A running fan stays at `minSpeed` until the target is 0, and once off it stays off for at least 3 s. Background: the EC only kick-starts the fan when leaving 0 %, so a ramp through 1–9 % leaves it stalled until a much higher duty. |
| `comfortTemp`   | °C   | none       | Idle / light-use target temperature (of the selected sensors). A slow integral controller adds as much fan as is needed to hold it, on top of the curve and never below it. See [Comfort temperature](#comfort-temperature). |
| `comfortMaxSpeed` | %  | `30`       | Upper limit of the comfort controller. Needs `comfortTemp`. |
| `sensors`       | list | all        | EC sensor indices whose maximum drives the curve, e.g. `[0, 1, 3]`. Indices are those shown by `fw-fanctrl probe` and `framework_tool --thermalget`. |

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

### Comfort temperature

A fixed curve cannot know which fan speed holds a warm-but-idle machine at a pleasant
surface temperature: that depends on the room, on whether it sits on a desk or a lap, on
a dock or a closed lid. With `comfortTemp` this part is regulated at runtime instead:

- Outside a ±0.5 °C deadband (the sensors report whole degrees), the comfort speed moves by
  0.05 % per second and degree of error. It is slow on purpose: the board sensors react
  with minutes of delay, and a hunting fan is more annoying than a steady one.
- The fan target is the higher of the curve and the comfort speed. Under load the curve
  takes over; while the curve asks for more, the comfort speed does not wind up, so it is
  ready again as soon as the load ends.
- It is limited to `comfortMaxSpeed` (default 30 %). `minSpeed`, the ramps, `criticalTemp`
  and the emergency guard apply as usual.
- `fw-fanctrl print` shows its current value as `ComfortSpeed`.

Upstream fw-fanctrl does not know the key; the same strategy would run there as a plain
curve.

### Choosing sensors

Upstream uses the hottest of all sensors. On the Intel Framework 13 Pro that is
`peci-temp`, the CPU package temperature, which jumps by 20 °C within a second under
load and makes the fan nervous. The EC's own automatic control effectively ignores PECI
(its `fan_off`/`fan_max` thresholds are 103/105 °C) and follows the board sensors, which
react slowly. Selecting those board sensors gives naturally smooth behaviour, and the CPU
still protects itself by throttling. Compare `sudo framework_tool --thermalget` with
`sudo fw-fanctrl probe` to find the indices on your machine.
[`examples/framework13-pro-intel-core-ultra-3.json`](examples/framework13-pro-intel-core-ultra-3.json)
is a quiet, MacBook-like strategy tuned on a Framework 13 Pro with Intel Core Ultra
Series 3 that uses local, CPU and DDR board sensors. The AMD variant has a different
sensor layout, so check its indices before reusing the example. On the Intel machine 10 % duty ≈ 900 RPM, 30 % ≈ 2800, 50 % ≈ 4150 and 100 % ≈ 6640 RPM.

Rates are used instead of a fixed ramp duration on purpose. With a fixed duration,
a change from 20 to 25 % takes as long as one from 20 to 100 %. With a rate, small
corrections are quick and big jumps are spread out.

Differences in validation compared to upstream:

- Unknown keys are rejected, so a typo like `rampUpRtae` fails loudly instead of being ignored.
- `speedCurve` temperatures must be ascending. Equal temperatures are allowed and form a
  step, as in upstream.
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

Added by fw-fanctrl-rs:

```
fw-fanctrl set-default <strategy> [--discharging]
fw-fanctrl set-default --discharging --clear
```

`set-default` makes a strategy the `defaultStrategy` (with `--discharging`: the
`strategyOnDischarging`). `--discharging --clear` sets `strategyOnDischarging` back to `""`,
so the default strategy is used on battery as well. The configuration file is saved and the
service switches at once. Like `set_config`, it is only accepted from root.

`print all` additionally reports `TargetSpeed`, the fan RPM, the reading of every
sensor and whether the fail-safe is active. `kill -HUP` (or `systemctl reload fw-fanctrl`) reloads the config.

The following extras run locally as root and do not talk to the service:

| Command | Purpose |
|---------|---------|
| `run --dry-run` | Read the real sensors and run the full control loop, but never write to the EC. |
| `run --simulate FILE` | No hardware at all. Temperatures are read from `FILE` (one value per sensor index, `-` for none, e.g. `38 45 - 36 72`), for testing the loop anywhere. |
| `probe` | Print EC temperatures per sensor index, fan RPM and AC state. |
| `test-fan <percent> [--seconds N]` | Set one duty for N seconds (default 5), then hand control back to the EC. |
| `restore-auto` | Hand fan control back to the EC. |
| `calibrate measure` / `calibrate apply` | Measure this machine once and create strategies from it, see below. |

`FW_FANCTRL_SOCKET` overrides the socket path (default
`/run/fw-fanctrl/.fw-fanctrl.commands.sock`), which is useful for tests without root.

## Calibration

Calibration has two steps. **Measure once**, which takes a while and needs the hardware.
Then **apply** the result as often as you like, which is instant.

```sh
sudo fw-fanctrl calibrate measure [--load-cmd CMD] [--output PATH]
sudo fw-fanctrl calibrate apply [quiet|balanced|cool] [--comfort-temp C] [--name NAME] [--default] [--dry-run]
fw-fanctrl use calibrated-balanced
```

### `calibrate measure`

It takes about 30 minutes and puts all CPU cores under full load. The raw data is saved
to `/etc/fw-fanctrl/calibration.json`, and a summary at the end shows the result for each
preference. The built-in
load uses 256-bit FMA when the CPU supports it, which draws more power than integer code
and comes closer to stress-ng, compilers and games. The calibration therefore errs on the
warm side. A running service is paused for the duration and resumed afterwards. The phases:

1. **Sensors:** Reads the EC sensor names and thermal thresholds.
2. **Fan:** Measures the duty that starts the fan, jumping from 0 % with the fan at rest
   each time (the EC only kick-starts it when leaving 0 %). The fan has to *keep* turning
   after the start-up kick is over, not just spin briefly. It also measures the lowest duty
   a running fan keeps turning at. The higher of the two becomes `minSpeed`. Then it maps duty to RPM.
3. **Idle baseline:** Records every sensor while the machine is idle.
4. **Load step:** Classifies the sensors. *Board* sensors react slowly and drive the curve.
   *Spiky* sensors (the CPU package) and sensors the EC itself ignores for fan control are
   only used to judge throttling. Sensors that barely react (battery) are dropped.
5. **Sustained load:** First a reference run at 100 % fan. It measures the CPU frequency
   without any thermal limit. Then increasing fixed fan speeds, starting at the start duty,
   each until the board temperature settles. It stops once even `cool` is satisfied, so every
   preference has an answer. `apply` later picks the quietest step that is good enough for
   the chosen preference (the anchor of the curve):
   - **Performance decides:** The average frequency must reach a share of the reference
     that the preference sets (see the table below). Performance can be lost in two ways. The
     CPU throttles itself, or the platform lowers the power limit on a warm machine (e.g.
     thermald, which lowered PL1 from 33 to 30 W on the Framework 13 Pro). A hot package on
     its own proves nothing: a CPU at 92 °C running the same clocks as at full fan is not
     held back.
   - **Kernel counters can veto:** More than 1 % throttle time in the last 60 s
     (`package_throttle_total_time_ms`) or new `package_throttle_count` events mark the
     speed as limited. They cannot prove the *absence* of throttling, because both only
     advance at the start or end of a throttle episode.
   - **End reference:** If no step satisfies `cool`, full fan is measured once more after
     the same soak time, and the steps are also judged against it. Some power budgets are
     time-dependent, and then an early reference is unfairly fast. The quietest step that
     passes against the end reference becomes the anchor.
   - **Settled or capped:** Each step shows whether the board temperature settled or the
     step stopped at its 4-minute cap.
   - **PL1:** The effective PL1 is the lower of the MSR and the MMIO RAPL limit, read every
     second. Firmware, DPTF and thermald often lower only the MMIO one. A step shows
     "lowered from … W" when the limit dropped below the highest value seen.
   - **Fallback without a frequency reading:** A package at 90 °C or more while drawing less
     than 90 % of RAPL PL1 counts as limited.

Idle comfort is not measured. It is regulated at runtime (`--comfort-temp`, see below),
which also follows changes in ambient temperature and placement.

### `calibrate apply`

This turns the saved data into a strategy and adds it to the configuration as
`calibrated-<preference>`, or as `--name`. The config is validated first, the previous file is
kept as `config.json.bak-<timestamp>`, and a running service reloads it. `--dry-run` only
prints the strategy, and `--default` also makes it the `defaultStrategy`. The output also says
how old the measurement is.

The generated strategy is off until a few degrees above idle, rises gently to the anchor and
reaches 100 % at the EC's own `fan_max` for the dominant sensor. It also gets `minSpeed`.

`--comfort-temp C` sets `comfortTemp` (see [Comfort temperature](#comfort-temperature)):
the board is held at about C °C in idle and light use with as little fan as possible. The
curve then stays off up to C+2 °C and leaves that range to the comfort controller. C must
be at least 6 °C below the anchor temperature.

The preference (default `balanced`) sets the trade-off between noise and performance. The anchor is the quietest fan
speed that reaches this share of the full-fan performance, and real CPU throttling is never
accepted. `quiet` is MacBook-like: it accepts that the platform lowers the power limit on a
warm machine instead of spinning the fan up.

| Preference | Required performance | Fan starts at (without `--comfort-temp`) | Ramps (up / down / delay) | Framework 13 Pro example |
|------------|----------------------|---------------|---------------------------|-----|
| `quiet` | 85 % | idle + 4 °C | 0.5 / 0.2 %/s, 30 s | 40 % (3700 RPM) |
| `balanced` (default) | 92 % | idle + 3 °C | 1.0 / 0.3 %/s, 20 s | 55 % (4700 RPM) |
| `cool` | 97 % | idle + 2 °C | 2.0 / 0.5 %/s, 10 s | 100 % |

For example, all three side by side, with the quiet one on battery:

```sh
for p in quiet balanced cool; do sudo fw-fanctrl calibrate apply $p --comfort-temp 43; done
sudo fw-fanctrl set-default calibrated-balanced
sudo fw-fanctrl set-default calibrated-quiet --discharging
```

`--load-cmd` (for `measure`) runs an additional load next to the CPU load, e.g. a GPU benchmark
(`--load-cmd "glmark2 --run-forever"`). The command is split like a shell would split it,
but it is not run through a shell, so it receives SIGTERM if the calibration dies. Use
`sh -c '…'` explicitly for pipelines. It runs as root and is stopped at the end. Graphical
programs may need your session environment as root. Alternatively, start the extra load
yourself before calibrating, because the measurement includes whatever is running.

While calibrating, the running service is paused with a 30 s lease that is renewed every
10 s. If the calibration is killed (even with `kill -9`), the service takes over the fan
again within 30 s.

Results depend on ambient temperature and on what is plugged in. Measure again if the
conditions change a lot (summer, docked with the lid closed, and so on).

## Safety

- **Whitelist:** Only four EC host commands can be sent:
  - `EC_CMD_PWM_SET_FAN_DUTY` (0x24, clamped to 0–100)
  - `EC_CMD_THERMAL_AUTO_FAN_CTRL` (0x52)
  - `EC_CMD_TEMP_SENSOR_GET_INFO` (0x70, read-only)
  - `EC_CMD_THERMAL_GET_THRESHOLD` (0x51, read-only)

  Memory-map reads are also allowed and are read-only. Setting thresholds (0x50), flash,
  reboot and battery commands are unreachable, because the EC command interface is never
  exposed generically.
- **Calibration guard:** If any sensor gets within 2 °C of its EC `high` threshold, that fan
  speed is counted as failed. The fan is handed back to the EC when the calibration ends, is
  interrupted (Ctrl-C) or fails.
- **Sanity check at startup:** The EC memory map must identify itself as `EC`, or the
  service refuses to run.
- **Fail-safe means EC automatic control, not 100 %:**
  - after 3 consecutive read or write errors (control resumes when the hardware recovers),
  - on `pause`, on suspend (via `fw-fanctrl-suspend.service`) and on SIGTERM/SIGINT,
  - on panic (from a panic hook),
  - after any stop, including crashes and watchdog kills (`ExecStopPost=fw-fanctrl restore-auto`).

  If handing control back fails (EC busy), it is retried every second until it works.
- **Fresh start after pause and fail-safe:** On `resume` and on recovery the controller
  forgets its history and starts at the curve value for the current readings. A machine that
  heated up meanwhile does not get a slow ramp from the old value.
- **Stall detection:** If the fan reports 0 RPM for 5 s while it is driven at `minSpeed`
  or more (15 % without `minSpeed`), the service holds 0 % for 3 s and starts it again, so
  the EC gives it a start-up kick. After 3 failed restarts in a row the EC takes over for
  10 minutes before the service tries again.
- **Global emergency guard:** If *any* sensor, selected in `sensors` or not, comes within
  3 °C of its EC `high` threshold, the fan goes to 100 % at once.
- **Second instance:** If another fw-fanctrl owns the socket, the service exits with code 3
  and systemd does not restart it (`RestartPreventExitStatus=3`). `/run/fw-fanctrl` is kept
  across restarts, so the other instance's socket is never deleted.
- **Socket permissions:** The socket is world-accessible like upstream, so desktop tools can
  run `use`, `print`, `pause` and so on. `set_config`, which rewrites the config file, is only
  accepted from root (checked with `SO_PEERCRED`).
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
