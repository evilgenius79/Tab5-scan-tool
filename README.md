# Tab5 Scan Tool

A motorsport-grade automotive scantool and CAN sniffer for the **M5Stack Tab5**
(ESP32-P4, 1280×720 IPS touchscreen) driving an **OBDLink EX** over the Tab5's
USB-A host port. Built on **ESP-IDF 5.3+** and **LVGL 9**.

## Features

- **Dual-mode OBD engine**
  - *Mode A — Polling*: ultra-fast STN `STPX` PID requests (RPM, MAP, knock
    retard, charge-air temp, HPFP, AFR, …).
  - *Mode B — Raw Sniffer*: STN monitor (`STMA` / filtered `STM`) streaming raw
    CAN frames to the UI and SD card.
- **Strict task separation**: LVGL on core 1, USB/OBD I/O on core 0, with a
  PSRAM-backed lock-free frame ring so high-rate CAN traffic never blocks the UI.
- **Auto-reconnecting USB host** (FTDI / CP210x / CH34x via `usb_host_vcp`).
- **Six LVGL screens**: Live Dash, Performance (0-60 / ¼-mile), Diagnostics
  (DTC read/clear), Data Logging, CAN Sniffer + hex filter keypad, Settings.
- **High-speed CSV logging** to microSD (decoded telemetry or raw CAN).

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

ESP32-P4 support is most reliable with a native ESP-IDF build:

```bash
idf.py set-target esp32p4
idf.py build flash monitor
```

Managed components (LVGL, esp_lvgl_port, M5Stack Tab5 BSP, USB VCP/FTDI
drivers) are resolved automatically from `main/idf_component.yml`.

A `platformio.ini` is also provided (use the `pioarduino` platform fork for P4
support), but `idf.py` is the recommended path.

## Hardware notes

- Connect the OBDLink EX to the Tab5 **USB-A host** port. The P4 supplies the
  device; no powered hub is required for the EX.
- The default adapter link runs at **2 Mbps**; change it on the Settings screen
  (`STBR`). Brightness and HS-CAN/MS-CAN selection also live there.
- microSD logging writes `telemetry-*.csv` (polling) or `canlog-*.csv`
  (sniffing) depending on the active mode when logging is toggled on.

## Calibration warning

The **enhanced PIDs** (knock retard, charge-air temp, HPFP) in
`main/obd/pid_definitions.h` use placeholder headers and scaling modeled on a
common turbo platform. **These are manufacturer/ECU-specific and must be
retuned for your vehicle.** Standard SAE J1979 PIDs decode correctly as-is.
