# Tab5 Scan Tool

A motorsport-grade automotive scantool and CAN sniffer for the **M5Stack Tab5**
(ESP32-P4, 1280×720 IPS touchscreen) driving an **OBDLink EX** over the Tab5's
USB-A host port. Built on **ESP-IDF 5.5** and **LVGL 9**. Tuned for the
**2013 Ford Explorer Sport 3.5 EcoBoost** test vehicle but kept multi-car:
standard SAE PIDs are universal; enhanced/module access uses a Ford profile.

## Features

- **Dual-mode OBD engine**
  - *Mode A — Polling*: round-robin OBD requests for the standard SAE PIDs
    (RPM, MAP/boost, AFR, ignition timing, coolant, IAT, load, baro, …) plus
    data-driven manufacturer PIDs (knock retard, charge-air temp, turbo boost)
    from the custom-PID loader.
  - *Mode B — Raw Sniffer*: STN monitor (`STMA` / filtered) streaming raw CAN
    frames to the UI and SD card with a hex pass-filter keypad.
- **Strict task separation**: LVGL on core 1, USB/OBD I/O on core 0, with a
  PSRAM-backed lock-free frame ring so high-rate CAN traffic never blocks the UI.
- **Auto-reconnecting USB host** (FTDI / CP210x / CH34x via `usb_host_vcp`).
- **Eleven LVGL screens**: Home launcher, Live Dash (configurable gauge grid),
  Live Data, Performance (0-60 / ¼-mile + session peaks), Trouble Codes
  (read/clear, all DTC types), Module Scan (multi-ECU read + clear), Vehicle
  Info (VIN/CALID/ECU), I/M Readiness, Data Logging, CAN Sniffer, Settings.
- **Configurable dash**: each gauge slot picks its parameter from a dropdown
  (incl. boost, MAF, MPG); assignments persist in NVS. USA units (mph, °F, psi).
- **Fuel economy + trip computer**: instantaneous and trip-average MPG derived
  from MAF + AFR, with trip distance/fuel/time (shown on Performance & Live
  Data, logged to CSV).
- **Freeze-frame (Mode 02)**: the sensor snapshot the ECU latched when a DTC
  was set, shown from the Diagnostics screen.
- **Auto-identify on connect**: VIN and I/M readiness are read automatically, so
  Vehicle Info, Readiness, and the Home Check-Engine/MIL badge populate without
  manual taps. Live Data records per-parameter MIN/MAX.
- **Link watchdog**: a hung adapter/bus (no data for 8 s) triggers an automatic
  re-init to recover mid-drive.
- **High-speed CSV logging** to microSD (decoded telemetry or raw CAN), on by
  default and auto-disabled when no card is present. A persistent `diag.log`
  mirrors the console across boots for post-drive review.
- **Tab5 battery**: internal charging enabled; battery % shown on Home whether
  plugged in or running off the pack.

## Architecture

```
                 core 0 (I/O)                         core 1 (UI)
  USB Host ISR ─► RX StreamBuffer ─► OBD task ─┬─► TelemetryState ─► LVGL timers ─► widgets
                                               ├─► sniffer FrameRing ─► Sniffer screen
                                               └─► logger FrameRing ─► SD logger task
        UI commands (mode, filter, DTC, baud) ◄── EventBus command queue ◄── screens
```

| Layer        | Files                                              |
|--------------|----------------------------------------------------|
| Entry        | `main/main.cpp`, `main/app_config.h`               |
| Core         | `main/core/` (ring buffer, event bus, shared state)|
| USB transport| `main/usb/` (`usb_host_cdc`, `obd_link`)           |
| OBD protocol | `main/obd/` (STN commands, parser, PID defs, task) |
| Logging      | `main/logging/sd_logger.*`                         |
| UI           | `main/ui/` (theme, tab shell, 6 screens)           |

## Build

ESP32-P4 must be built with native **ESP-IDF 5.5** (`idf.py`):

```bash
idf.py set-target esp32p4
idf.py build flash monitor
```

Managed components (LVGL, esp_lvgl_port, M5Stack Tab5 BSP, USB VCP/FTDI
drivers) are resolved automatically from `main/idf_component.yml`.

> **Note:** the PlatformIO / `pioarduino` path is currently broken for a pure
> ESP-IDF P4 build (linker `sram_seg` vs `sram_low` section mismatch). Use
> `idf.py`. When flashing, use `--after hard_reset` so the adapter's DTR/RTS
> lines don't drop the board into download mode.

## MAP / boost sensor

Boost is derived from manifold absolute pressure. The MAP-sensor full scale is
set in `main/app_config.h`:

```c
#define MAP_SENSOR_BAR  3.0f   // 3-bar sensor: 0–300 kPa absolute
```

This scales the **Boost** and **MAP** dash gauges automatically — a 3-bar
sensor gives a ~43.5 psi MAP range and a ~28.8 psi gauge-boost ceiling, with
the overboost warn/danger zones tracking that ceiling.

> **Single-byte limit:** standard OBD PID `0x0B` (MAP) returns one byte, so it
> saturates at **255 kPa (~22.3 psi boost)** no matter the physical sensor. To
> read the full 3-bar range above that, source boost from the enhanced 2-byte
> **Turbo Boost** PID via the custom-PID loader (below).

## Hardware notes

- Connect the OBDLink EX to the Tab5 **USB-A host** port. The P4 supplies the
  device; no powered hub is required for the EX.
- The adapter link opens at **115200 baud** (the EX/STN power-on rate); it can
  be renegotiated higher at runtime on the Settings screen (`STBR`).
  Brightness and HS-CAN/MS-CAN selection also live there.
- microSD logging writes `telemetry-*.csv` (polling) or `canlog-*.csv`
  (sniffing) depending on the active mode. The telemetry CSV appends a column
  per loaded custom PID and writes one row per fresh poll.

## Custom / manufacturer PIDs

Standard SAE J1979 PIDs are built in and decode correctly as-is. Manufacturer
parameters (knock retard, charge-air temp, turbo boost, …) are **data-driven**:

- If `/sdcard/custom_pids.csv` is present it is loaded (any make), one PID per
  line: `name,request,bytes,signed,scale,offset,unit`.
- Otherwise a small built-in **Ford EcoBoost** default set is used.

> The built-in Ford scalings are community FORScan figures and **should be
> verified against your vehicle** — edit the CSV to correct them.
