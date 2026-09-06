# Repo notes for Claude

This is a hardware + firmware monorepo for AgOpenGPS boards (see `README.md` for the general
layout: `ArduinoModules`, `Esp32Modules`, `TeensyModules`, `Misc`).

## Active work: unified autosteer firmware

Branch `unified_autosteer_firmware` is merging four previously-separate Teensy 4.1 autosteer
codebases (hydraulic/DC steering, Keya CAN steering, a web config server + wasless auto-zero
engine, and a planned dual-path engage model) into one configurable firmware image.

**If you're working anywhere under `TeensyModules/V4.1/Firmware/Autosteer_gps_teensy_v4_1/`,
read that folder's `CLAUDE.md` first** — it has the build/flash instructions (including a
Teensyduino version pin that matters — a newer core boot-loops this hardware), the full
EEPROM address map, and several "this was deliberately decided, don't redo it" notes from
bugs already found and fixed. Skipping it risks repeating debugging that's already been done.
