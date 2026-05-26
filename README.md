# Tab5 Scan Tool

A motorsport-grade automotive scantool and CAN sniffer for the **M5Stack Tab5**
(ESP32-P4, 1280×720 IPS touchscreen) driving an **OBDLink EX** over the Tab5's
USB-A host port. Built on **ESP-IDF 5.4** and **LVGL 9**. Tuned for the
**2013 Ford Explorer Sport 3.5 EcoBoost** test vehicle but kept multi-car:
standard SAE PIDs are universal; enhanced/module access uses a Ford profile.

---

## Table of contents

- [Feature summary](#feature-summary)
- [Screens & options](#screens--options)
  - [Live Dash — gauge channels & layouts](#live-dash)
  - [Settings — every adjustable option](#settings)
- [Audible alerts](#audible-alerts)
- [GPS / GNSS (SAM-M10Q)](#gps--gnss)
- [SD card layout](#sd-card-layout)
- [Custom / manufacturer PIDs](#custom--manufacturer-pids)
- [DTC description database](#dtc-description-database)
- [MAP / boost sensor scaling](#map--boost-sensor-scaling)
- [Build-time configuration (`app_config.h`)](#build-time-configuration)
- [Architecture](#architecture)
- [Build & flash](#build--flash)
- [Hardware notes](#hardware-notes)

---

## Feature summary

- **Dual-mode OBD engine**
  - *Polling*: round-robin OBD requests for standard SAE PIDs (RPM, MAP/boost,
    AFR, ignition timing, coolant, IAT, load, baro, fuel trims, speed, MAF, …)
    plus data-driven manufacturer PIDs (knock retard, charge-air temp, turbo
    boost) from the custom-PID loader. Polling is gated by a **supported-PID
    scan** so only PIDs the ECU advertises are requested.
  - *Raw sniffer*: STN monitor (`STMA` / filtered) streaming raw CAN frames to
    the UI and SD card with a hex pass-filter keypad.
- **Strict task separation**: LVGL on core 1, USB/OBD I/O on core 0, with a
  PSRAM-backed lock-free frame ring so high-rate CAN traffic never blocks the UI.
- **Auto-reconnecting USB host** (FTDI / CP210x / CH34x via `usb_host_vcp`).
- **GPS / GNSS** (optional u-blox SAM-M10Q on Port A / UART): live position,
  speed, heading, altitude and satellite count; GPS-clocked 0-60 / ¼-mile;
  GPX track recording to SD; GPS columns added to the telemetry log.
- **Thirteen LVGL screens** (see below).
- **Configurable dash**: 20 selectable gauge channels, switchable 6/4/2-gauge
  layouts, per-gauge trend sparkline, and a global top status bar (battery + MIL)
  on every screen. Assignments + layout persist in NVS. USA units (mph, °F, psi).
- **Fuel economy + trip computer**: instantaneous and trip-average MPG from
  MAF + AFR, with trip distance/fuel/time (logged to CSV).
- **Freeze-frame (Mode 02)**, **I/M readiness**, **VIN + offline VIN decode**,
  **multi-ECU module scan (UDS)** with DTC descriptions, **read/clear all DTC
  types (03/07/0A)**.
- **Audible alerts**: spoken WAV clips (or fallback beeps) for knock, overboost,
  coolant/oil over-temp, and MIL, with hysteresis + cooldown.
- **Auto-identify on connect**: VIN and I/M readiness read automatically; Home
  Check-Engine/MIL badge and Vehicle/Readiness screens populate hands-free.
- **Link watchdog**: a hung adapter/bus auto-re-inits to recover mid-drive.
- **High-speed CSV logging** to microSD (decoded telemetry or raw CAN), plus a
  persistent `diag.log` mirroring the console across boots.
- **Tab5 battery**: internal charging enabled; live battery % in the top bar.

---

## Screens & options

The UI is a left-rail tabview. Twelve screens, in nav order:

| # | Tab | Screen | What it shows / does |
|---|-----|--------|----------------------|
| 1 | HOME  | Home launcher | Connection state, quick links, Check-Engine/MIL badge |
| 2 | DASH  | Live Dash | Configurable gauge grid (see below) |
| 3 | PERF  | Performance | 0–60 mph & ¼-mile timers, session peaks, trip computer, RESET SESSION |
| 4 | DTC   | Trouble Codes | Read/clear stored, pending & permanent DTCs (Modes 03/07/0A) with descriptions |
| 5 | I/M   | Readiness | I/M monitor readiness (catalyst, EVAP, O2, misfire, …) + MIL status |
| 6 | VEH   | Vehicle Info | VIN, decoded make/year, CALID, ECU name (Mode 09) |
| 7 | LIVE  | Live Data | Full scrolling PID table with per-parameter MIN/MAX recording |
| 8 | MOD   | Module Scan | Enumerate ECUs (Ford 7E0–7E7 or generic), read & clear per-module DTCs |
| 9 | LOG   | Data Logging | Logging on/off + status; writes telemetry/CAN CSV to SD |
| 10 | CAN   | Sniffer | Raw CAN frame stream with hex pass-filter keypad |
| 11 | GRAPH | Trend Graph | Full-screen live line chart; pick any of 10 channels from a dropdown |
| 12 | GPS   | GPS / Track | Live fix (position, speed, heading, altitude, sats, HDOP) + GPX track RECORD toggle |
| 13 | SET   | Settings | Adapter/UI configuration (see below) |

A persistent **top status bar** (44 px) sits above all screens: battery % +
charge state on the right, MIL / stored-code summary on the left.

### Live Dash

Each gauge slot is an arc with a numeric readout, colored warn/danger zones, and
a 48-point scrolling trend **sparkline** at the bottom. Tap a gauge's dropdown to
reassign its channel; the choice persists in NVS per slot.

**Layout switcher** — a top menu-bar button cycles the gauge grid:

| Layout | Grid | Gauge height |
|--------|------|--------------|
| 6 GAUGES | 3 × 2 | 160 px |
| 4 GAUGES | 2 × 2 | 210 px |
| 2 GAUGES | 1 × 2 | 250 px |

**Selectable gauge channels (20):**

| # | Channel | Unit | Range | Warn | Danger | Notes |
|---|---------|------|-------|------|--------|-------|
| 0 | RPM | rpm | 0–8000 | 6000 | 6600 | redline zones |
| 1 | Boost | psi | −15…ceiling | 80% | 93% | from enhanced Turbo Boost PID if loaded, else MAP-derived |
| 2 | AFR | — | 8–20 | — | — | context-dependent |
| 3 | Ign Timing | deg | −10…50 | — | — | |
| 4 | Coolant | °F | 0–260 | 220 | 240 | hot |
| 5 | IAT1 (Intake) | °F | 0–250 | 150 | 180 | heat soak |
| 6 | Speed | mph | 0–140 | — | — | |
| 7 | MAP | psi | 0…3-bar | — | — | saturates at 255 kPa single-byte |
| 8 | Throttle | % | 0–100 | — | — | |
| 9 | Eng Load | % | 0–100 | — | — | |
| 10 | Battery | V | 8–16 | 12.0 | 11.5 | low = bad |
| 11 | Barometric | psi | 10–16 | — | — | |
| 12 | Mass Airflow | g/s | 0–300 | — | — | |
| 13 | MPG | mpg | 0–60 | — | — | instantaneous |
| 14 | Short Fuel Tr | % | −25…25 | — | — | STFT |
| 15 | Long Fuel Tr | % | −25…25 | — | — | LTFT |
| 16 | Fuel Level | % | 0–100 | 15 | 5 | low = bad |
| 17 | Oil Temp | °F | 0–300 | 250 | 270 | hot |
| 18 | Ambient | °F | −20…120 | — | — | |
| 19 | IAT2 (Charge) | °F | 0–300 | 160 | 185 | post-intercooler charge-air temp |

### Settings

| Option | Control | Values / behavior | Persisted |
|--------|---------|-------------------|-----------|
| USB Baud Rate | dropdown | 115200 / 230400 / 500000 / 1000000 / 2000000 — renegotiated at runtime via `STBR` | NVS `baud` |
| CAN Network | switch | off = HS-CAN, on = MS-CAN | NVS `bus` |
| Screen Brightness | slider | 10–100 % (BSP backlight PWM, applied live) | NVS `bright` |
| Audible Alerts | switch + **TEST** | mute toggle; TEST plays the "knock" clip | NVS `alerts` |
| Adapter Link | label + **RECONNECT** | live link state; force re-init | — |
| SD Data Files | label | DTC-DB count (SD vs built-in), custom-PID count + source, ECU-supported PID count | — |

The adapter link state shown: `NO ADAPTER`, `INITIALIZING…`, `ENUMERATING…`,
`ONLINE`, or `ERROR`.

---

## Audible alerts

Each alert plays `/sdcard/sounds/<name>.wav` if present (drop in your own voice
clip — e.g. a "knock knock knock" recording), otherwise a synthesized fallback
beep through the Tab5 speaker. An alert re-arms only after the value drops back
below `threshold − hysteresis`, and won't repeat within the cooldown.

| Alert | WAV name | Trigger | Default threshold |
|-------|----------|---------|-------------------|
| Knock | `knock.wav` | active knock retard | `ALERT_KNOCK_DEG` = 3.0° |
| Overboost | `boost.wav` | boost over limit | `ALERT_BOOST_PSI` = 25 psi |
| Coolant | `coolant.wav` | coolant over-temp | `ALERT_COOLANT_F` = 240 °F |
| Oil | `oil.wav` | oil over-temp | `ALERT_OIL_F` = 270 °F |
| MIL | `mil.wav` | check-engine on | — |

Global knobs (in `app_config.h`): `ALERT_VOLUME_PCT` = 70, `ALERT_COOLDOWN_MS`
= 8000. Mute the whole engine from the Settings screen.

---

## GPS / GNSS

An optional **u-blox SAM-M10Q** GNSS module wired to **Port A** (GPIO53/54) is
read as a **UART** NMEA-0183 device on UART1: the module's TX (NMEA out) goes to
`GPS_UART_RX_PIN` (default G53), and `GPS_UART_TX_PIN` (G54) -> module RX is only
needed to push config. The reader validates the NMEA checksum and parses `RMC`
(position, ground speed, heading, date/time, fix validity) and `GGA` (altitude,
satellites, HDOP, fix quality). A bare u-blox module defaults to 9600 baud /
1 Hz; at that rate GPS-clocked 0-60 timing resolves to ~1 s steps (raise the
module's update rate and baud for finer launch timing).

What it enables:

- **GPS-clocked performance runs** — the 0-60 mph and ¼-mile timers prefer GPS
  ground speed whenever a fix is present (more accurate trap speed + distance
  than integrating OBD VSS). Set `PERF_USE_GPS` to 0 to always use OBD VSS.
- **Live GPS readout** on the Performance screen and the dedicated GPS screen
  (position, speed, heading, altitude, satellites, HDOP, fix state).
- **GPX track recording** — the GPS screen's RECORD button writes
  `/sdcard/track-*.gpx` (standard GPX 1.1 trackpoints with timestamps), openable
  in Google Earth, Strava, etc. Recording is handled on the I/O core.
- **GPS columns in the telemetry CSV** (`gps_lat`, `gps_lon`, `gps_speed_kph`,
  `gps_course`, `gps_alt_m`, `gps_sats`).

Configuration (`app_config.h`): `GPS_ENABLED`, `GPS_UART_NUM` (1),
`GPS_UART_RX_PIN` (53), `GPS_UART_TX_PIN` (54), `GPS_BAUD` (9600),
`PERF_USE_GPS`. If the module isn't wired up, leave `GPS_ENABLED` at 1 — the
GPS screen just shows "acquiring"/"no module" and nothing else is affected. If
no fix ever appears, the RX/TX pins are likely swapped.

---

## SD card layout

All SD files are optional; the tool runs without a card (logging auto-disables).

```
/sdcard/
├── custom_pids.csv      # manufacturer/enhanced PID definitions (overrides built-in)
├── dtc_db.csv           # DTC code -> description database (overrides built-in)
├── sounds/
│   ├── knock.wav        # voice/sound clips for each alert (16-bit PCM WAV)
│   ├── boost.wav
│   ├── coolant.wav
│   ├── oil.wav
│   └── mil.wav
├── telemetry-*.csv      # decoded telemetry log (polling mode) — written by the tool
├── canlog-*.csv         # raw CAN frame log (sniffer mode) — written by the tool
├── track-*.gpx          # GPX track log (GPS RECORD) — written by the tool
└── diag.log             # persistent console mirror across boots
```

---

## Custom / manufacturer PIDs

Standard SAE J1979 PIDs are built in and decode correctly as-is. Manufacturer
parameters (knock retard, charge-air temp, turbo boost, …) are **data-driven**:

- If `/sdcard/custom_pids.csv` is present it is loaded (any make), one PID per
  line: `name,request,bytes,signed,scale,offset,unit`.
- Otherwise a small built-in **Ford EcoBoost** default set is used (also used as
  the fallback when the VIN's make can't be determined).

See `custom_pids.csv` in the repo root for an annotated template.

> The built-in Ford scalings are community FORScan figures and **should be
> verified against your vehicle** — edit the CSV to correct them.

---

## DTC description database

DTC code-to-meaning lookup is **hybrid**:

- A small built-in generic SAE table (~90 common P-codes) is always available.
- If `/sdcard/dtc_db.csv` is present it is loaded into PSRAM and searched first
  (binary search). The repo ships a full ~9.2k-row Ford `dtc_db.csv`; see
  `dtc_db.example.csv` for the format.

The Settings → SD Data Files row reports which source is active and how many
descriptions are loaded.

---

## MAP / boost sensor scaling

Boost is derived from manifold absolute pressure. The MAP-sensor full scale is
set in `main/app_config.h`:

```c
#define MAP_SENSOR_BAR  3.0f   // 3-bar sensor: 0–300 kPa absolute
```

This scales the **Boost** and **MAP** dash gauges automatically — a 3-bar
sensor gives a ~43.5 psi MAP range and a ~28.8 psi gauge-boost ceiling, with
the overboost warn/danger zones tracking that ceiling.

> **Single-byte limit:** standard OBD PID `0x0B` (MAP) returns one byte, so it
> saturates at **255 kPa (~22.3 psi boost)** regardless of the physical sensor.
> To read the full 3-bar range above that, source boost from the enhanced
> 2-byte **Turbo Boost** PID via the custom-PID loader.

---

## Build-time configuration

All app-level tunables live in `main/app_config.h`:

| Group | Macro | Default | Purpose |
|-------|-------|---------|---------|
| Core affinity | `APP_CORE_UI` / `APP_CORE_IO` | 1 / 0 | LVGL on core 1, USB/OBD on core 0 |
| Priorities | `PRIO_USB_RX` / `PRIO_OBD_TASK` / `PRIO_UI_TASK` / `PRIO_SD_LOGGER` | — | FreeRTOS task priorities |
| Stacks | `STACK_USB_RX` / `STACK_OBD_TASK` / `STACK_UI_TASK` / `STACK_SD_LOGGER` | 6/8/12/6 KB | per-task stack sizes |
| Buffers | `USB_RX_STREAM_BYTES` | 32 KB | RX byte stream depth |
| Buffers | `FRAME_RING_SLOTS` | 4096 | parsed-CAN ring (~160 KB PSRAM) |
| Buffers | `OBD_CMD_QUEUE_DEPTH` | 16 | UI→OBD command queue |
| Adapter | `OBD_DEFAULT_BAUD` / `OBD_FALLBACK_BAUD` | 115200 / 38400 | open rate / pre-v2 fallback |
| Adapter | `OBD_LINK_TIMEOUT_MS` | 2500 | watchdog dead-link threshold |
| Adapter | `OBD_CMD_TIMEOUT_MS` | 1000 | per-command prompt timeout |
| Boost | `MAP_SENSOR_BAR` | 3.0 | MAP sensor full-scale (bar) |
| Logging | `SD_MOUNT_POINT` / `LOG_FLUSH_EVERY_N_ROWS` | `/sdcard` / 64 | mount point / fsync batch |
| UI | `UI_GAUGE_REFRESH_MS` | 33 | ~30 fps gauge/sparkline refresh |
| UI | `UI_SNIFFER_DRAIN_MS` / `UI_SNIFFER_MAX_ROWS` | 50 / 200 | sniffer cadence / scrollback |
| Alerts | `ALERT_VOLUME_PCT` / `ALERT_COOLDOWN_MS` | 70 / 8000 | speaker volume / repeat lockout |
| Alerts | `ALERT_KNOCK_DEG` / `ALERT_BOOST_PSI` / `ALERT_COOLANT_F` / `ALERT_OIL_F` | 3.0 / 25 / 240 / 270 | alert thresholds |
| GPS | `GPS_ENABLED` / `GPS_UART_NUM` | 1 / 1 | GNSS on Port A (UART NMEA) |
| GPS | `GPS_UART_RX_PIN` / `GPS_UART_TX_PIN` / `GPS_BAUD` | 53 / 54 / 9600 | Port A pins + baud |
| GPS | `PERF_USE_GPS` | 1 | prefer GPS speed for perf timers when a fix is up |

---

## Architecture

```
                 core 0 (I/O)                         core 1 (UI)
  USB Host ISR ─► RX StreamBuffer ─► OBD task ─┬─► TelemetryState ─► LVGL timers ─► widgets
                                               ├─► sniffer FrameRing ─► Sniffer screen
                                               └─► logger FrameRing ─► SD logger task
        UI commands (mode, filter, DTC, baud) ◄── EventBus command queue ◄── screens
                       audible alerts task ◄── TelemetryState (knock/boost/temp/MIL)
  GPS (UART G53/54) ─► GPS task ─► GpsFix (EventBus) ─► perf timers / GPS screen / CSV
                                └─► GPX track file (SD)
```

| Layer        | Files                                                       |
|--------------|-------------------------------------------------------------|
| Entry        | `main/main.cpp`, `main/app_config.h`                        |
| Core         | `main/core/` (ring buffer, event bus, shared state, power)  |
| USB transport| `main/usb/` (`usb_host_cdc`, `obd_link`)                    |
| OBD protocol | `main/obd/` (STN commands, parser, PID defs, DTC/VIN, task) |
| GPS          | `main/gps/gps.*` (UART NMEA reader + GPX track logger)      |
| Audio        | `main/audio/alerts.*`                                       |
| Logging      | `main/logging/sd_logger.*`                                  |
| UI           | `main/ui/` (theme, tab shell, 13 screens)                   |

---

## Build & flash

ESP32-P4 must be built with native **ESP-IDF 5.4** (`idf.py`); the M5Stack Tab5
BSP requires ≥ 5.4:

```bash
idf.py set-target esp32p4
idf.py build flash monitor
```

Managed components (LVGL, esp_lvgl_port, M5Stack Tab5 BSP, USB VCP/FTDI drivers,
esp_codec_dev) are resolved automatically from `main/idf_component.yml`.

> **Note:** the PlatformIO / `pioarduino` path is currently broken for a pure
> ESP-IDF P4 build (linker `sram_seg` vs `sram_low` section mismatch). Use
> `idf.py`. When flashing, use `--after hard_reset` so the adapter's DTR/RTS
> lines don't drop the board into download mode.

---

## Hardware notes

- Connect the OBDLink EX to the Tab5 **USB-A host** port. The P4 supplies the
  device; no powered hub is required for the EX.
- The adapter link opens at **115200 baud** (the EX/STN power-on rate); it can
  be renegotiated higher at runtime on the Settings screen (`STBR`). CAN-bus
  selection (HS/MS) and brightness also live there.
- microSD logging writes `telemetry-*.csv` (polling) or `canlog-*.csv`
  (sniffing) depending on the active mode. The telemetry CSV appends a column
  per loaded custom PID and writes one row per fresh poll.
