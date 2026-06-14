# 10-inch FPV quad — ArduPilot config & tooling

GEPRC Mark 4 V2 10" frame with a **Matek H743** flight controller running
**ArduCopter 4.6**. This repo stores the FC config, Lua scripts, peripheral
drivers, and ground-station tools for the build.

## Hardware summary

| Part | Notes |
|------|-------|
| Frame | GEPRC Mark 4 V2 10" |
| FC | Matek H743 (ArduPilot) |
| VTX | IRC Tramp-compatible (TBS Unify Pro or similar) |
| Rangefinder | JC02-1 laser (UART, 9600 baud) |
| RC link | ELRS → TX16S |
| Telemetry | LTE bridge — see [LTE_Telemetry](../LTE_Telemetry/README.md) |

## Repo layout

```
arduconfig.param          Full ArduCopter parameter dump (load via Mission Planner)
VTX_Tramp.lua            ArduPilot Lua script — IRC Tramp VTX power control
jc02/
  jc02_rangefinder.lua   ArduPilot Lua driver for JC02-1 laser rangefinder
  jc02_monitor.py        PC-side serial monitor / protocol capture tool (COM23, 9600)
  capture_cont.py        Continuous measurement capture script for bench testing
MP/
  joystickaxisArduCopter2.xml    Mission Planner joystick axis mapping (TX16S USB)
  joystickbuttonsArduCopter2.xml Mission Planner joystick button mapping (TX16S USB)
tx16/
  MODELS/                EdgeTX model backup for this quad
BTFL_cli_backup_*.txt   Legacy Betaflight CLI dumps (OMNIBUSF4SD, pre-ArduPilot)
```

## ArduPilot parameters

`arduconfig.param` is a full parameter dump — load it in Mission Planner via
**CONFIG → Full Parameter List → Load from file**.

Key non-default settings:

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `SERIAL2_PROTOCOL` | 2 | MAVLink2 on UART2 (LTE telemetry RPi) |
| `SERIAL2_BAUD` | 460 | 460800 baud to match mavlink-router |
| `RC_OVERRIDE_TIME` | 3 | Accept RC_CHANNELS_OVERRIDE for 3 s after last packet |
| `RC_OPTIONS` | 32 | Allow RC override from GCS |
| `FS_GCS_ENABLE` | 0 | GCS failsafe — review before flying LTE-only |
| `FS_THR_ENABLE` | 1 | RC throttle failsafe → RTL |
| `SCR_ENABLE` | 1 | Lua scripting enabled |
| `RNGFND2_TYPE` | 36 | Lua rangefinder (JC02-1 via jc02_rangefinder.lua) |

## Lua scripts

### `VTX_Tramp.lua`

Controls VTX power level over IRC Tramp protocol via a free UART.

**ArduPilot setup:**
- `SERIALx_PROTOCOL = 28` (Scripting) on the UART wired to VTX Tramp input
- `SCR_USER1` = startup power level index
- `SCR_USER2` = scripting serial port index

### `jc02/jc02_rangefinder.lua`

Driver for the JC02-1 laser rangefinder, fed to ArduPilot as `RNGFND2`.

**Wiring:**

| Wire | Connect to |
|------|-----------|
| White | GND |
| LtBlue | VCC (3.3 V) |
| Yellow | FC UART Rx |
| Black | FC UART Tx |
| Red | 3.3 V (power enable, keep HIGH) |

**ArduPilot setup:**
- `SERIALn_PROTOCOL = 28`, `SERIALn_BAUD = 9` (9600 bps)
- `RNGFND2_TYPE = 36` (Lua)
- `RNGFND1_TYPE = 10` (MAVLink, existing short-range sensor — do not change)

## Mission Planner joystick (TX16S via USB)

`MP/joystickaxisArduCopter2.xml` and `MP/joystickbuttonsArduCopter2.xml` are
Mission Planner joystick config files for the TX16S in USB HID mode.

To restore: copy both files to `%LOCALAPPDATA%\MissionPlanner\` (or wherever
Mission Planner stores `joystick.xml` on your system) and reload MP.

## TX16S model backup

`tx16/MODELS/` contains the EdgeTX model file for this quad. Restore via
EdgeTX Companion or by copying to the `MODELS/` folder on the TX16S SD card.

## Related

- [LTE_Telemetry](../LTE_Telemetry/README.md) — RPi 4G telemetry bridge used with this quad
