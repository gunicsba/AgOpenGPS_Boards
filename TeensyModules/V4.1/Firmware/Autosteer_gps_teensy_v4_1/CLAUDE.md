# Unified Autosteer Firmware (Teensy 4.1 V4.1)

This sketch is being merged from four previously-separate codebases into one configurable
firmware image, on branch `unified_autosteer_firmware` (base: `hydliftv2_TM171`). If you're
picking this up fresh, read this whole file before changing anything — a lot of the design
here exists because of a specific bug or hardware constraint discovered the hard way.

## Where this came from

- **`hydliftv2_TM171`** (this repo, branch) — base skeleton. Hydraulic/DC valve steering,
  hydraulic lift relays (`MachineHydraulicLift.ino`), TM171 IMU option, GNSS-port auto-detect.
- **`Keya_TM171`** (this repo, branch) — same skeleton with Keya CAN motor steering instead of
  PWM/valve. Its `isKeya` was a hard-coded compile-time bool; Phase 1 below made it runtime.
- **`AIO_Keya_WasKeyaFiltre`** (`D:\AgOpenGPS\AIO_Keya_WasKeyaFiltre`, standalone project, not a
  branch of this repo) — the web config server and the "wasless" auto-zero engine (Keya's own
  CAN encoder used as a virtual WAS instead of the ADS1115 pot).
- **`AOG_CAN_Teensy4.1/Autosteer_AOGv5_Teensy4.1UDP_SteerReadyCAN`** (separate repo/folder) —
  dual-path engage model (physical button + tablet button both live at once). **Not yet
  ported** — still a planned phase.

Full design rationale and diagrams: ask about the "Unified Autosteer Firmware" RFC artifact if
you need the original reviewed proposal; the phase history in `git log` on this branch is the
source of truth for what's actually landed.

## Build / flash — read this before touching Teensyduino

- Toolchain: `teensy:avr` core, **pinned at 1.57.3**. A newer Teensyduino causes a boot-loop
  on this hardware for reasons not yet root-caused. Do not run a core upgrade for this board.
- Bundled `arduino-cli` (no separate install needed) lives at:
  `C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe`
- FQBN: `teensy:avr:teensy41`
- Compile: `arduino-cli compile --fqbn teensy:avr:teensy41 <path to this folder>`
- Compile + flash a connected board: add `--upload -p <port>` (find the port with
  `arduino-cli board list` — the Teensy shows up with protocol `teensy`, something like
  `usb:0/140000/0/2`, distinct from its COM-port identity used for the serial monitor).
- The Teensy's USB serial (for boot logs) is a *different* port than the one arduino-cli
  uploads through — find it via `Get-CimInstance Win32_PnPEntity` matching `VID_16C0&PID_0483`
  (PJRC's Teensy Serial VID/PID), not by process of elimination.
- **Known gotcha**: this file has two overloaded `calcChecksum()` functions (one for RELPOSNED,
  one for UBX auto-baud). Arduino's ctags-based auto-prototype generator is a text heuristic,
  not a real parser, and has silently failed to generate a prototype for one overload before
  (triggered by unrelated code changes elsewhere shifting line numbers). Explicit forward
  declarations for both overloads are already in place right after `struct ubxPacket` in the
  main `.ino` — if you see `'ubxPacket' was not declared` or `too many arguments to function
  'calcChecksum'`, that's this class of bug; don't just add more code and hope, check those
  declarations are still there.

## Architecture

### Steering driver (`Setup.SteerDriverType`)
`STEER_DRIVER_HYDRAULIC` (0, default) or `STEER_DRIVER_KEYA` (1). Replaces what used to be a
hard-coded `bool isKeya` in the `Keya_TM171` branch. `motorDrive()` in `AutosteerPID.ino`
dispatches on this. **Important**: the Cytron-enable line (`PWM2_RPWM`/`DIR1_RL_ENABLE`
toggling in `Autosteer.ino`'s `autosteerLoop()`) runs for *every* driver type, not just
hydraulic — on this board that same line also drives the steer-button LED backlight via a
lock circuit, unrelated to motor control. Don't re-gate it behind `SteerDriverType` (this was
tried and reverted — see git history "Fix: don't gate the Cytron-enable line off for Keya").

### Wasless mode (Keya-encoder-as-WAS)
Active when `SteerDriverType == STEER_DRIVER_KEYA && steerConfig.IsDanfoss`. Reuses AOG's
"Danfoss valve" checkbox bit — safe because a real Danfoss valve and a Keya motor are never on
the same board. When active, `steerAngleActual` comes from Keya's own CAN encoder
(`keyaEncoderRaw`, tracked in `KeyaCANBUS.ino` from the heartbeat's cumulative angle bytes)
instead of the ADS1115 WAS, continuously re-zeroed by the auto-zero engine in `Autosteer.ino`
(ported from `AIO_Keya_WasKeyaFiltre`, tunable via the web page or the serial `z` menu in
`zAutoZeroMenu.ino`). ADC failure is not fatal in this mode.

The auto-zero engine's "is the tractor going straight" check reads gyro yaw rate from
whichever IMU is actually active (`currentYawDeg()` in `Autosteer.ino` — BNO's `yaw` global,
converted from its native degrees-x10 scale, or TM171's `tm171YawDeg`). If you add a third
heading source, feed it through that same function rather than reading a raw IMU global
directly from inside the auto-zero block — it was a real bug once that the block only read
BNO's global, silently making the "gyro says straight" check always true on TM171-only boards.

### Kickout / pressure sensor (`Setup.PressureSensorType`)
Only matters when `steerConfig.PressureSensor == 1` (AOG's own setting). Replaces what used to
be a compile-time `#define JOHNDEERE` flag:
- `PRESSURE_SENSOR_GENERIC` (0) — plain analog read.
- `PRESSURE_SENSOR_JOHNDEERE` (1) — PWM duty-cycle capture (JD's factory sensor).
- `PRESSURE_SENSOR_DANFOSS` (2) — pulse frequency mapped to pressure %, calibrated by
  `pressureSensorMaxHz` (the frequency that reads as 100%).
Pin/ISR setup for whichever type is active happens in `pressureSensorInit()`, called from
`autosteerSetup()` *after* `steerConfig` loads from EEPROM — this used to run before the load
completed (reading stale/default config), same class of bug as the ADC-fatality check below.

### Serial port assignment (`PortConfig`, in the main `.ino`)
Per-*port*, not per-module: the Board Setup page has one dropdown per physical port
(`Serial2`/`Serial5`/`Serial7`), each picking what's expected there — `Auto-detect` (default),
`GPS 1`, `GPS 2`, `TM171 IMU`, or `Unused`. This maps more directly onto "what's wired where"
than the first version of this feature did (a dropdown per module, each picking a port), and
means GPS2 simply exists once some port claims that role — no separate GPS-count field needed.
`portPinnedTo(role)` finds the (first) port explicitly assigned a role; `portIsFree(port)`
tells auto-detect whether a port is fair game (role is still `Auto`) or reserved for something
else it must leave alone. A manual TM171 assignment is tried regardless of whether a CMPS/BNO
was already found — the auto-detect path, by contrast, is nested inside
`if (!useCMPS && !useBNO08x)`, so a false-positive CMPS ACK on the I2C bus (or a real CMPS/BNO
also present) silently skips TM171 detection entirely in auto mode. This was a real field
complaint (TM171 physically wired, "not detected") before the manual override existed — though
note that on at least one board, TM171 still wasn't detected under *any* firmware version
tested, which points at wiring/power on that specific unit rather than a firmware bug; the
manual port pin doesn't help if nothing is actually coming out of the sensor's TX pin.
TM171's factory-default wiring is `Serial7` (per `TM171.ino`'s own `SerialImu` default).
Changing anything in `PortConfig` or the driver/pressure-sensor type from the web page
triggers an automatic reboot (`SCB_AIRCR` reset) since these are only read once in `setup()`
— don't try to apply them live.

## EEPROM address map

Do not place a new field at an address without checking this table — a silent overlap between
two structs means they corrupt each other's data on every save, not just once. This bit the
project once already: `AutoZeroParams` from the source project used offset 90, which overlaps
`hydConfig` at 100 the moment both hydraulic-lift and wasless features exist in the same image.

| Offset | Contents | Owner |
|---|---|---|
| 0 | `EEP_Ident` (int16) | version guard — bump when any struct below changes shape |
| 10 | `Storage steerSettings` | Kp, PWM limits, wasOffset, steerSensorCounts |
| 40 | `Setup steerConfig` | driver type, pressure sensor type, all AOG PGN 251 bits |
| 60 | `ConfigIP networkAddress` | 3 IP octets |
| 100 | `Config hydConfig` (`MachineHydraulicLift.ino`) | lift timing, relay pins, 8 bytes |
| 114 | `keyaTicksPerDeg` (float) | Keya encoder ticks-per-degree calibration |
| 120 | `AutoZeroParams azParams` (~40 bytes, ends ~159) | wasless auto-zero tuning |
| 170/174/178/182 | EMA yaw/roll/pitch/stop (float each) | BNO anti-jitter filters |
| 190 | `pressureSensorMaxHz` (float) | Danfoss pulse-frequency calibration |
| 200 | `PortConfig` | GPS/TM171 serial port assignment |

When adding a new persisted value: pick an address with a comfortable gap from its neighbors
(structs' actual compiled size can differ from a naive field count due to alignment padding),
and add a row to this table in the same commit.

## Things that were deliberately decided, not overlooked

- **AgOpenGPS itself is out of scope.** No new PGN 251 bits, no changes to the desktop app.
  Every board-specific setting lives on this board's own web page. Don't propose "just add a
  bit to PGN 251" as a shortcut.
- **Settings loss on the `EEP_Ident` bump to 2500 was accepted**, not a bug to fix — hydraulic
  lift and steer settings reset to firmware defaults on that one upgrade; nothing here tries
  to migrate the old layout byte-for-byte.
- **Engage logic is still the old branch-on-switch-type model** (`SteerSwitch`/`SteerButton`/
  neither). The planned replacement (always-on physical button *and* tablet button, ported
  from the `SteerReadyCAN` folder) has been discussed but not implemented — this fleet uses
  momentary buttons only (no toggle switches), and buttons are a known field failure point, so
  not gating the tablet engage behind physical-button health is the intended direction once
  that phase lands.
- **OTA firmware updates are not implemented.** Planned approach is FlasherX (writes to the
  Teensy 4.1's onboard QSPI flash) plus a `POST /update` route on the existing web server —
  needs its own safety spike (rollback on a bad image) before it's trusted in the field.

## Debugging on the bench

The web config server (`zWebConfig.ino`, default IP `192.168.5.126`) is two pages, on purpose:
`GET /` is Status (live WAS angle, speed, filtered GPS heading, wasless zero status, IMU
detected) and auto-refreshes every 4s via a plain `<meta http-equiv="refresh">` — safe to
reload constantly since it has no inputs to lose. `GET /setup` is every actual setting (Board
Setup, auto-zero tuning, Keya calibration, EMA filters) as one form, and *never* reloads
itself — there used to be a single page with a JS "pause the reload while the user is editing"
guard, but it only watched `<input>` elements, not `<select>`, so the Board Setup dropdowns
never paused it and the page could reload out from under someone mid-selection. Don't
recombine these into one auto-refreshing page without solving that properly. `POST /save`
handles both; it redirects back to `/setup`.

`curl` works fine for smoke-testing `POST /save` — see git history around the Phase 2 commit
for example payloads. Auto-zero debug logging goes to `Serial` at 115200 baud, prefixed
`[AZ-PRECISE]`/`[AZ-FAST]`/`[AZ]`, and the `z` serial command opens a live tuning menu.

**A remote script cannot reliably capture the very start of a boot log.** The Teensy's USB
serial re-enumerates across any reset or reflash, and `Serial.print` over USB silently drops
bytes if nothing has the port open yet — a script that opens the port only after triggering a
reset/upload will consistently miss everything from early `setup()` (this was tried multiple
times expecting it to just be a timing/race issue; it wasn't). If you need to see setup-time
detection output (CMPS/BNO/TM171 probing, etc.), either keep a serial monitor continuously
open through the reset, or — preferred — surface the thing you need to check as a value on the
Status page instead of trying to catch it in a boot log.
