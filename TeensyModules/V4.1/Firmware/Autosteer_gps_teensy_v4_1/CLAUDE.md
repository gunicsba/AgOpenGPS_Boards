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
  dual-path engage model (physical button + tablet button both live at once). Ported.

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

## OTA firmware updates

Built on [FlasherX](https://github.com/joepasquariello/FlasherX)'s flash primitives
(`FlashTxx.h`/`FlashTxx.c`, vendored in this folder). `FlashTxx.c` is untouched — it's a plain
`.c` file on purpose, compiles as C, and `zOTA.ino` wraps its header include in
`extern "C" { ... }` when pulling those declarations into C++; don't hand-edit it, pull a fresh
copy from upstream if a real change there is ever needed. `FlashTxx.h` has **one intentional
deviation** from upstream, in `FLASH_RESERVE` for `ARDUINO_TEENSY41` — `0x40*FLASH_SECTOR_SIZE`
(256KB), not upstream's `4*FLASH_SECTOR_SIZE` (16KB). Root cause and the two dead ends tried
first, for anyone touching this again:

- Live-testing OTA first hit `"image too large for the staging buffer"`. `firmware_buffer_init()`
  (`FlashTxx.c`) finds staging space by scanning *downward* from the top of flash for the first
  non-erased byte. This sketch uses `EEPROM.h`, whose emulation on Teensy 4.x reserves the real
  top **256KB** of flash for wear-leveling — with `FLASH_RESERVE` only excluding 16KB, the scan
  ran straight into that EEPROM region's genuinely non-erased data (this project writes to
  EEPROM constantly — `portConfig`, `azParams`, etc.) and stopped almost immediately, reporting
  a tiny/invalid buffer. Confirmed against a community report of this exact failure and fix
  ([PJRC forum, "OTA through Ethernet with Teensy 4.1"](https://forum.pjrc.com/index.php?threads/ota-through-ethernet-with-teensy-4-1.72233/)).
  **Fix: size `FLASH_RESERVE` to actually cover the EEPROM emulation region.** Don't shrink it
  back down while `EEPROM.h` is in use.
- Before finding that, RAM-based staging (`RAM_BUFFER_SIZE > 0`, the *other* mode FlasherX
  supports) was tried as a way to sidestep the flash-scan heuristic entirely. It's a dead end on
  this specific build: real free heap at runtime measured only **~3.3KB** via `mallinfo()`
  (added as a diagnostic on the `/ota` page — `mi.fordblks`), identical immediately after a
  fresh reboot as after normal use, ruling out a runtime leak — something during `setup()`
  itself (most likely `NativeEthernet`'s own FNET stack/socket buffers, not confirmed further)
  consumes nearly all of RAM2, and the compiler's static "free for malloc/new" estimate
  (~500KB) doesn't account for it at all. [A reference project](https://github.com/ssaenger/FlasherX-Ethernet_Support)
  and the PJRC thread above both got RAM staging (200-256KB) working on Teensy 4.1, but both
  use **QNEthernet**, not `NativeEthernet` — if RAM staging is ever worth revisiting, that
  library difference is the first thing to look at, not the buffer size.

Deliberately **not** using FlasherX's own `update_firmware()` (in its `FXUtil.cpp`, not
vendored here) — that one is interactive, prompting for confirmation over the same `Stream`
it's reading the hex file from, which has no equivalent over a one-shot HTTP POST. Instead
`zOTA.ino` implements an explicit two-step flow:

- `GET /ota` — upload form (the 4th web nav tab).
- `POST /ota/stage` — streams the uploaded `.hex` directly off the TCP connection into a
  flash-based staging buffer above the running code, parsing and flash-writing one Intel HEX
  line at a time. Never buffers the whole upload in RAM — this matters, since a real `.hex`
  for this sketch is several hundred KB of ASCII. Validates the `FLASH_ID` marker (below)
  before allowing a flash. Uploading alone never flashes anything.
- `POST /ota/confirm` — copies the staged image into place and reboots into it
  (`flash_move()`). Point of no return for that session: recoverable via USB + the physical
  PROGRAM button if something goes wrong, same as any other bad flash, but the running
  firmware cannot undo it once `flash_move()` starts (it doesn't return — it reboots
  partway through).
- `POST /ota/cancel` — frees the staged buffer, discards it.

**The `FLASH_ID` marker is load-bearing, not just a log line.** `otaSetup()` in `zOTA.ino`
does `Serial.println(FLASH_ID)` at boot — that's what forces the compiler to embed the
literal string `"fw_teensy41"` somewhere in this build's own flash content. `check_flash_id()`
scans a *newly uploaded* image for that same string before allowing a flash, to confirm it
was actually built for this exact board (not a Teensy 4.0 build, not an unrelated project's
hex file). If a future firmware ever drops that print line, OTA uploads will still stage
successfully but will always get rejected at the `FLASH_ID` check — keep it.

Why `webConfigLoop()`'s normal request handling doesn't apply to `/ota/stage`: every other
POST handler buffers the whole body into a `String` before processing. For a firmware upload
that's the wrong shape (RAM pressure, and no reason to hold the whole file at once) — so
`webConfigLoop()` special-cases `/ota/stage` to leave the body untouched on the still-open
`EthernetClient` and hand it directly to `otaStageFromClient()`, which reads it as a `Stream`
(same interface FlasherX itself expects — `EthernetClient` already implements `Stream`).

`OtaStageResult` and the functions that take/return it needed their own explicit forward
declarations positioned right after the struct/typedef definitions in `zOTA.ino`, same
pattern and same reason as the `calcChecksum()` fix in the main `.ino` — Arduino's
auto-prototype generator hoists a prototype for every function to the top of the
concatenated sketch, before a type defined further down in the same file would exist yet.

**Not yet live-tested.** Compiles clean and the design has been reasoned through carefully,
but the actual upload → stage → confirm → `flash_move()` → reboot cycle has never been
exercised against real hardware. Test deliberately, with the physical USB/PROGRAM-button
recovery path confirmed working first (it already is, on this project's bench unit) — this
is the one piece of this whole firmware where a bug has a real chance of requiring that
recovery path, not just an inconvenience.

## Things that were deliberately decided, not overlooked

- **AgOpenGPS itself is out of scope.** No new PGN 251 bits, no changes to the desktop app.
  Every board-specific setting lives on this board's own web page. Don't propose "just add a
  bit to PGN 251" as a shortcut.
- **Settings loss on the `EEP_Ident` bump to 2500 was accepted**, not a bug to fix — hydraulic
  lift and steer settings reset to firmware defaults on that one upgrade; nothing here tries
  to migrate the old layout byte-for-byte.
- **Engage is always-on dual path, not the old branch-on-switch-type model.** The physical
  button (`STEERSW_PIN`) and the tablet's onscreen button (`guidanceStatus` bit 0) are both
  read every cycle in `autosteerLoop()`, regardless of AOG's `SteerSwitch`/`SteerButton`
  setting — deliberately, ported from `SteerReadyCAN`. This fleet wires momentary buttons only
  (no toggle switches), and buttons are a known field failure point, so not gating the tablet
  engage behind physical-button health is the point, not an oversight. The two inputs are
  handled differently because they're shaped differently: the physical button is a momentary
  press (toggles `currentState`/`steerSwitch` on each press-edge), the tablet button is a
  level (`guidanceStatus` bit 0 directly reflects AOG's desired state, so it's mirrored onto
  the same latch on each change rather than toggled). `steerConfig.SteerSwitch`/`SteerButton`
  are still parsed from AOG's PGN 251 (can't change AOG's own UI) but no longer consulted for
  engage logic — this is what the RFC's "Keya can't engage under None" open question expected
  to resolve as a side effect, since the branch it lived in doesn't exist anymore; not
  independently re-verified on hardware after the rewrite, just structurally true.
- **OTA firmware updates are implemented but not yet live-tested.** See the "OTA firmware
  updates" section above for the full design (FlasherX-based, explicit stage/confirm/cancel
  over the web UI). There's no automatic rollback on a bad image — the safety net is the
  `FLASH_ID` check at stage time plus the explicit confirm step, not an undo after the fact.
- **The web server has no authentication, on any page.** Anyone on the same network can
  change the steering driver type, reassign serial ports, upload and flash new firmware, or
  send arbitrary bytes to a GPS/TM171 port via the terminal — all with nothing but the
  board's IP. Accepted as reasonable for a board that only ever sits on an isolated farm LAN;
  not something to assume is fine in a different network context without revisiting this.

## Debugging on the bench

The web config server (`zWebConfig.ino` + `zOTA.ino` + `zWebTerminal.ino`, default IP
`192.168.5.126`) is five pages, on purpose:
- `GET /` — Status: live WAS angle, speed, filtered GPS heading, wasless zero status, IMU
  detected, a "Wasless: ACTIVE/inactive" indicator, a Raw Inputs block (WAS pot ADC counts,
  steer/work/remote switch pin states, kickout/current sensor reading), a Motor Output block
  (PWM value, direction, the PWM2_RPWM lock/enable line, the DIR1_RL_ENABLE pin — labeled with
  a note that pin *roles* differ by driver type: Cytron uses PWM1_LPWM as PWM+direction and
  DIR1_RL_ENABLE as the actual direction pin while PWM2_RPWM is repurposed as enable/lock;
  IBT2 uses PWM1_LPWM/PWM2_RPWM as separate forward/reverse channels and DIR1_RL_ENABLE just
  enables both halves; Keya uses none of them, values shown are the CAN command instead), and
  the Keya auto-zero tracking cards. Polls its own `/status/data` JSON endpoint every 400ms via `fetch()` and
  patches values in place (element ids `s_*` in `sendStatusPage()`/`sendStatusData()`,
  `zWebConfig.ino`) — no full-page reload, safe to poll constantly since it has no inputs to
  lose. Used to be a `<meta http-equiv="refresh">` every 4s; switched to JS polling because a
  quick button press could sit unreflected on screen for most of that 4s window even though
  the firmware itself reacted immediately. Keep `/status/data`'s response small (see the
  socket-buffer note in the terminal bugs section below) — it's plain numbers/short strings,
  nowhere near the limit, but don't grow it into something that dumps large text.
- `GET /board` — driver type, kickout sensor type + Danfoss Hz calibration, per-port serial
  role assignment, IMU anti-jitter EMA filters. `POST /saveboard` handles it, redirects back
  to `/board`, and may reboot the board (see the architecture section above).
- `GET /wasless` — the auto-zero engine's own tuning (heading sources, stability
  conditions/durations, Keya encoder calibration). `POST /savewasless` handles it, redirects
  back to `/wasless`, never reboots. Shows a prominent banner for whether wasless mode is
  actually active right now (`waslessActive()`, same condition as the SteerDriverType/
  IsDanfoss check elsewhere) — these settings silently do nothing when it isn't, so the
  banner exists specifically so that's never a surprise.
- `GET /ota` — firmware update, see the OTA section above.
- `GET /terminal` — remote serial terminal, see below.

None of these do a full-page reload. Status and Terminal both poll their own JSON data endpoint
via `fetch()` and patch the DOM in place (400ms and 2.5s respectively); Board/Wasless/OTA are
plain forms with no live polling at all. There used to be a single combined settings page with
a JS "pause the reload while the user is editing"
guard, but it only watched `<input>` elements, not `<select>`, so dropdowns never paused it
and the page could reload out from under someone mid-selection — don't reintroduce an
auto-refreshing settings page without solving that class of bug properly (or just don't
auto-refresh a page with a form on it).

### Ethernet PHY needs a real power cycle after a soft reset — HTTP-specific

Reproduced repeatedly during this session: after any soft reset of the board (a fresh
`arduino-cli upload`, or `teensy_reboot`/the physical PROGRAM button), the web server (TCP,
port 80) stays completely unreachable — not even ARP resolves — while UDP keeps working fine
(AgOpenGPS traffic, `ReceiveUdp()`) and the board is otherwise alive (status LED blinking,
`loop()` running). The only thing that recovers it is pulling power and reconnecting it, not
just resetting the MCU.

This points at the Teensy 4.1's on-board Ethernet PHY (the official add-on's chip, on the
official RJ45/PHY circuit — not an external W5500/SPI chip) not getting properly
re-initialized by a software-only reset, rather than a bug in this project's code: `NativeEthernet`'s
`EthernetServer`/TCP path apparently depends on PHY link-state that only a real power-on reset
clears reliably, while raw UDP transmission/reception doesn't hit whatever gets left wedged.
This was NOT introduced by any change this session — it reproduced against known-good, previously
committed code too, before any terminal/OTA work landed.

**Practical consequence**: after every reflash, power-cycle the board before judging whether an
HTTP-facing change (web config server, OTA, terminal) actually worked or not — a soft
reset alone will look like a total regression even when the new code is fine.

**Update — this PHY theory is now only a partial explanation, and "every reflash" turned out to
be an overstatement.** While chasing the terminal view showing zero data despite confirmed real
traffic, two concrete, unrelated-to-PHY bugs were found and fixed (both below): a real deadlock
in the vendored `NativeEthernet` library, and a response bigger than the library's own
per-socket buffer. Given the deadlock bug's shape — a blocking call inside the single-threaded
request handler that only recovers via a full power-on reset — it's plausible some *other*
blocking call already in this codebase (not necessarily `flush()`) causes some or all of the
original "needs a power cycle" symptom too, rather than it purely being PHY link state.

Later in the same session, several further reflashes (removing the UART bridge feature below,
then adding the TM171 settings page) came back up on their own — **no power cycle needed** —
even though nothing in those changes touched Ethernet/PHY init at all. The best available
explanation: the power-cycle requirement was likely never an inherent property of *every*
reflash, but was triggered by the heavy, rapid `curl` testing done earlier in the session
(dozens of back-to-back connections while chasing the terminal bugs) — exactly the kind of
connection churn this stack is fragile under. Once that stopped, later reflashes stayed
healthy without it. This is a plausible explanation, not a confirmed one — don't state it as
fact to a user without saying so. Practical takeaway: don't assume a power cycle is required
after every reflash going forward, but don't be surprised if one occasionally still is,
especially after a stretch of heavy HTTP testing against the board.

### Two real bugs behind "terminal shows zero data despite confirmed live traffic"

Both found by testing against `Serial2`, which the diagnostic line on `/terminal` (see below)
confirmed was actually carrying the GPS's real 460800-baud traffic (`SerialGPS -> Serial2`,
`totalBytes` counter climbing) — proving the tap/buffer plumbing itself was correct — while the
page's live view stayed completely empty with no error, and even a single isolated `curl` to
`/terminal/data` failed (`Empty reply from server` / `CONN_RESET`) after a long idle period.

1. **`EthernetClient::flush()` in the vendored library never terminates under realistic
   conditions.** `NativeEthernetClient.cpp:281`:
   ```cpp
   void EthernetClient::flush()
   {
       while (sockindex < Ethernet.socket_num) {
           uint8_t stat = Ethernet.socketStatus(sockindex);
           if (stat != SnSR::ESTABLISHED && stat != SnSR::CLOSE_WAIT) return;
           if (Ethernet.socketSendAvailable(sockindex) >= Ethernet.socket_size) return;
       }
   }
   ```
   `sockindex` is never mutated in the loop body — the only way out is one of the two `return`s.
   If the socket is `ESTABLISHED` and its send buffer isn't fully drained (exactly the case
   right after writing a multi-KB response), this spins forever: draining that buffer requires
   processing an incoming ACK, which requires the very `loop()` this call is currently blocking
   to keep running. It was added here (briefly, then reverted — see `zWebConfig.ino`, end of
   `webConfigLoop()`) to try to fix truncated responses and made things *worse* (consistent
   `Failed to fetch` / timeouts) — that's what exposed the bug. **Do not call `.flush()` on an
   `EthernetClient` in this codebase.** `client.stop()` alone is sufficient and doesn't have
   this problem.
2. **`/terminal/data`'s response could exceed the library's own per-socket buffer.**
   `FNET_SOCKET_DEFAULT_SIZE` (`NativeEthernet.h`) is only **2048 bytes**. The old
   `MAX_PER_POLL = 1024` produced up to 2048 hex characters alone, before the JSON wrapper —
   enough to fill or exceed the entire TX buffer in one response. Once full, further writes
   either stall (nothing will free the space — see bug 1, same underlying cause: freeing it
   needs an ACK, which needs `loop()` to keep running) or get silently dropped, truncating the
   JSON body with **no error visible on either side** — `fetch()` just gets a `200 OK` with a
   broken body. Fixed by capping `MAX_PER_POLL` at 384 bytes in `zWebTerminal.ino` — comfortably
   under the 2048-byte buffer with headroom for the JSON wrapper and other traffic. If you ever
   need to raise this, keep it well under 2048 total (hex chars + wrapper), not just under 2048
   hex chars.

### Remote serial terminal (`zWebTerminal.ino`)

View and send raw bytes on `Serial2`/`Serial3`(RTK)/`Serial5`/`Serial7` from the web UI,
without physical USB access. This exists specifically because the TM171-not-detected
investigation had no way to check "is anything even coming out of this wire" short of
physically re-wiring a logic analyzer or a second USB-serial adapter.

It does **not** own these ports — it taps the bytes at the points the firmware already reads
them (GPS/GPS2 ingestion and the RTK passthrough in the main `.ino`, `TM171process()` in
`TM171.ino`), one extra `termTapPort()`/`termTapSerial3()` call at each site, into a 2KB
per-port ring buffer. The real parsing logic is untouched. `GET /terminal/data?port=N&since=N`
returns only the bytes newer than `since` as hex, polled by the page every ~500ms and appended
client-side — this is why the page itself never needs to reload. `POST /terminal/send` writes
URL-decoded text to a port (also useful for driving the existing serial-menu commands —
`zAutoZeroMenu.ino`'s `z` menu, `zHandlers.ino`'s `EY`/`ER`/`EP`/`ES` — remotely instead of
only from a USB serial monitor). `POST /terminal/baud` re-`begin()`s a port at a different
rate for diagnostics — this **will** disrupt normal use of that port (GPS reception, TM171
parsing, whatever currently owns it) until it's changed back or the board reboots; the page
says so, it's not hidden.

Named `zWebTerminal.ino` specifically so it sorts after `zWebConfig.ino` in the concatenated
sketch, since it reuses `sendOK`/`sendHead`/`sendNav`/`extractFloat` from there — avoids
relying on cross-file auto-prototyping being order-independent (it is, for well-known types;
this just removes the need to reason about it). `TermRingBuf` needed the same explicit-
prototype-after-the-type fix as `OtaStageResult`/`ota_hex_info_t` — see those comments for
the general pattern if you add another struct used as a function parameter anywhere.

`curl` works fine for smoke-testing `POST /saveboard` and `POST /savewasless` — see git
history around the Phase 2/3 commits for example payloads. Auto-zero debug logging goes to
`Serial` at 115200 baud, prefixed `[AZ-PRECISE]`/`[AZ-FAST]`/`[AZ]`, and the `z` serial
command opens a live tuning menu.

### TM171 (SYD Dynamics) parameter read/write (`zWebImu.ino`)

The `/imu` page reads and writes the TM171's own settings (UART1 baud rate, output/inhibit
rate, accelerometer/magnetometer sensor-fusion gains) directly over its existing UART
connection, using the IMU's native "EasyProtocol" configuration protocol — this replaced the
raw TCP<->UART bridge idea below once it became clear the actual parameter set could just be
sent over the wire ourselves, with no external tool or Windows virtual COM port needed at all.

- **Protocol reference**: SYD Dynamics *TransducerM TM3xx User Guide v1.35-R1* (from
  [syd-dynamics.com/download-center/](https://www.syd-dynamics.com/download-center/) — "For
  newest TM171/TM151/TM210, please refer to this document"). Packet framing (`0xAA 0x55
  [Package Length] [4-byte header: cmd:7,res:3,fromId:11,toId:11, little-endian, first-
  declared-field=LSB] [payload] [CRC16 lo,hi]`) is identical to what `TM171.ino` already
  parses on receive — `tm171SendObject()` in `zWebImu.ino` reuses `MODBUS_CRC16_v3()` from
  `TM171.ino` unmodified, with the exact same buffer/count convention `GoodCRC()` already
  uses. Verified against the manual's own worked example (`aa55080c08000016000000e0ed` =
  broadcast Request for the Status object, id 22).
- Two un-timestamped object types (unlike the RPY/Status/Euler/Raw/Gravity telemetry objects
  `TM171.ino` parses, which all have a leading 4-byte timestamp before their named fields —
  Setting and Request do not): **Request** (id 12, 4-byte payload: byte 0 = id of the object
  being requested) and **Setting** (id 21, 20-byte payload: `switches`(u32) /
  `reserved`(u16, must be 1152) / `uart1Baud`(u16, unit 100bps) / `canBaud`(u16) /
  `gainAcc`(u16, unit 0.01) / `gainMag`(u16, unit 0.01) / `inhibitTime`(u16, ms) /
  `silentTime`(u32, seconds)).
- **The `switches` 32-bit bitfield's layout was reconstructed, not copied verbatim** — the
  manual's own C struct listing got its comments reflowed/shifted by PDF text extraction, so
  several field-to-comment pairings had to be inferred from *meaning* rather than trusted
  literally. Cross-checked by confirming every reconstructed field's declared bit-width sums
  to exactly 32 (only one specific reordering produces that). Confidence is high but not
  absolute. Because of this, `tm171WriteSettings()` in `zWebImu.ino` **never synthesizes the
  full `switches` word from scratch** — it only ever flips `TM171_SW_SAVE_PERMANENT` and
  clears `TM171_SW_REQUEST_ACK` on top of a value most recently read from the real device
  (`tm171Settings.switchesRaw`), and refuses to run at all until a real Setting-object
  response has been received at least once. This is also literally what the manual itself
  recommends: "firstly request Setting Object, make the modifications and then send back.
  This ensures only setting of interest gets changed." If you ever need to touch one of the
  other bits (sensor/output enables, boot mode, etc.), re-derive its position the same way
  (cross-check against the 32-bit total) before trusting it — don't assume the existing
  `TM171_SW_*` defines cover bits they don't already define.
- If the UART1 baud rate is actually changed, `tm171WriteSettings()` re-`begin()`s
  `SerialImu` at the new rate immediately after sending — otherwise the board would lose
  contact with the IMU the instant it applied the change (`TM171setup()` always re-begins at
  a fixed 115200 bps, which would then mismatch).
- The page never auto-refreshes and issues a Request/reload manually (`POST /imu/read`) rather
  than faking a synchronous read across the UART round-trip — same reasoning as the Terminal
  page: an honest async fetch-then-reload beats a fragile attempt to block the HTTP response on
  a serial reply that might not come back in time.

### Raw TCP<->UART bridge — tried, scrapped

A raw TCP<->UART passthrough (so a vendor IMU config tool needing a real COM port could reach
Serial7/TM171 via a Windows virtual COM port bridged over TCP) was implemented and then
deliberately removed in favor of driving the IMU's config protocol directly from this
firmware instead (see the wasless/TM171 architecture notes above) — no point maintaining a
whole bridge subsystem if the actual parameter set can just be sent over the wire ourselves.
Worth knowing if this idea comes up again, so it isn't rediscovered the hard way:

- The board only has **8 total sockets** (`MAX_SOCK_NUM` in `NativeEthernet.h` for Teensy
  4.1 — TCP and UDP share the same pool). 4 are already permanently committed at boot
  (`Eth_udpPAOGI`, `Eth_udpNtrip`, `Eth_udpAutoSteer`, the web server's listener), leaving only
  4 free — not enough headroom to run one permanently-open listener per UART given how much
  socket-pressure trouble this stack has already caused this session at far lighter load.
- This vendored `NativeEthernetServer` has no public `end()`/`stop()`/close — once `begin()`
  claims a socket for LISTEN, there's no documented way to release it from application code.
  Any future "pick which port to bridge" UI would need a reboot to switch, same as `/saveboard`.
- On the Windows tooling side (separate from the above, but also worth remembering): com0com's
  signed driver still hit Code 52 (signature verification failure) on this machine, and Secure
  Boot blocked the standard `bcdedit /set testsigning on` fix. com2tcp itself only ships as
  source on SourceForge (compiles fine with MSVC's `cl.exe`, nothing exotic) — a third-party
  site was found selling a precompiled installer for this GPL tool that turned out to be an
  empty, non-functional repackage. A separate "Windows 11 signature patch" advertised on that
  same third-party site was not recognized or endorsed by com0com's actual upstream maintainer
  and had unexplained signing provenance — don't install it.

**A remote script cannot reliably capture the very start of a boot log.** The Teensy's USB
serial re-enumerates across any reset or reflash, and `Serial.print` over USB silently drops
bytes if nothing has the port open yet — a script that opens the port only after triggering a
reset/upload will consistently miss everything from early `setup()` (this was tried multiple
times expecting it to just be a timing/race issue; it wasn't). If you need to see setup-time
detection output (CMPS/BNO/TM171 probing, etc.), either keep a serial monitor continuously
open through the reset, or — preferred — surface the thing you need to check as a value on the
Status page instead of trying to catch it in a boot log.
