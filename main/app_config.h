// =============================================================================
//  app_config.h - Build-time configuration for the Tab5 Scan Tool.
// -----------------------------------------------------------------------------
//  Centralizes all tunables: core affinity, task priorities, stack sizes,
//  ring-buffer geometry, and adapter defaults. Hardware pin muxing for the
//  DSI panel / touch / SD card is owned by the M5Stack Tab5 BSP, so only
//  app-level knobs live here.
// =============================================================================
#pragma once

#include "freertos/FreeRTOS.h"

// Firmware version string, shown in the top status bar.
#define APP_FW_VERSION   "v1.0"

// -----------------------------------------------------------------------------
//  Core affinity
// -----------------------------------------------------------------------------
//  The ESP32-P4 is dual-core. We pin LVGL to core 1 so it gets a predictable,
//  uninterrupted slice for rendering, and keep all USB/OBD I/O on core 0 where
//  the USB Host ISR also lands. This avoids cross-core cache thrash on the
//  LVGL draw buffers.
#define APP_CORE_UI          1
#define APP_CORE_IO          0

// -----------------------------------------------------------------------------
//  Task priorities (higher = more urgent). configMAX_PRIORITIES is 25 by default.
// -----------------------------------------------------------------------------
//  The USB polling task must out-prioritize everything else so we never drop
//  inbound CAN frames at high bus load. The UI runs comfortably mid-stack; the
//  SD logger is lowest because bursty flash writes can tolerate latency thanks
//  to the ring buffer in front of it.
#define PRIO_USB_RX          (configMAX_PRIORITIES - 2)   // just below timer svc
#define PRIO_OBD_TASK        (configMAX_PRIORITIES - 4)
#define PRIO_UI_TASK         (tskIDLE_PRIORITY + 4)
#define PRIO_SD_LOGGER       (tskIDLE_PRIORITY + 2)

// -----------------------------------------------------------------------------
//  Stack sizes (bytes)
// -----------------------------------------------------------------------------
#define STACK_USB_RX         (6 * 1024)
#define STACK_OBD_TASK       (8 * 1024)
#define STACK_UI_TASK        (12 * 1024)   // LVGL + screen builders are stack-hungry
#define STACK_SD_LOGGER      (6 * 1024)

// -----------------------------------------------------------------------------
//  Ring / stream buffer geometry
// -----------------------------------------------------------------------------
//  RX byte stream: raw bytes from the USB host callback into the OBD parser.
//  Sized for a worst-case burst of STMA monitor output between parser wakeups.
#define USB_RX_STREAM_BYTES      (32 * 1024)

//  Parsed CAN frame ring (sniffer + logger consumers). Each slot is a
//  can_frame_t (~40 B); 4096 slots ~= 160 KB, allocated from PSRAM.
#define FRAME_RING_SLOTS         4096

//  Outbound command queue depth (UI -> OBD task requests).
#define OBD_CMD_QUEUE_DEPTH      16

// -----------------------------------------------------------------------------
//  OBDLink EX / STN defaults
// -----------------------------------------------------------------------------
//  The OBDLink EX powers on at 115200 baud (verified: ScanTool FRPM / default
//  STN UART rate). We MUST open the FTDI link at this rate or the adapter never
//  sees valid AT commands and init fails. The Settings screen can renegotiate
//  higher (up to 2 Mbps) at runtime via STBR once the link is up.
#define OBD_DEFAULT_BAUD         115200
#define OBD_FALLBACK_BAUD        38400     // pre-v2 firmware default

//  Milliseconds with no successful response before we declare the link dead
//  and kick the auto-reconnect path.
#define OBD_LINK_TIMEOUT_MS      2500

//  How long to wait for a single command's terminating prompt ('>') character.
#define OBD_CMD_TIMEOUT_MS       1000

// -----------------------------------------------------------------------------
//  MAP / boost sensor scaling
// -----------------------------------------------------------------------------
//  Full-scale of the manifold-absolute-pressure sensor, in bar. A factory NA
//  sensor reads ~1.05 bar; forced-induction builds commonly fit a 2.5 or 3 bar
//  sensor. This sets the top of the Boost and MAP gauges (and their warn/danger
//  zones scale from it).
//
//  IMPORTANT: standard OBD PID 0x0B (MAP) returns a SINGLE byte, so it saturates
//  at 255 kPa (~22.3 psi boost) regardless of the physical sensor. To read the
//  full 3-bar range above that, source boost from the enhanced 2-byte "Turbo
//  Boost" PID via the custom-PID loader (obd/custom_pids).
#define MAP_SENSOR_BAR           3.0f
#define SEA_LEVEL_KPA            101.325f
#define KPA_TO_PSI               0.1450377f

// -----------------------------------------------------------------------------
//  Logging
// -----------------------------------------------------------------------------
#define SD_MOUNT_POINT           "/sdcard"
#define LOG_FLUSH_EVERY_N_ROWS   64        // batch fsync to spare the SD card

// -----------------------------------------------------------------------------
//  UI refresh
// -----------------------------------------------------------------------------
#define UI_GAUGE_REFRESH_MS      33        // ~30 fps gauge/sparkline updates
#define UI_SNIFFER_DRAIN_MS      50        // sniffer view batch-drain cadence
#define UI_SNIFFER_MAX_ROWS      200       // visible scrollback before recycling

// -----------------------------------------------------------------------------
//  Audible alerts (Tab5 speaker)
// -----------------------------------------------------------------------------
//  Thresholds that fire a spoken/beeped warning. Each alert plays
//  /sdcard/sounds/<name>.wav if present (drop in your own voice clip, e.g. a
//  "knock knock knock" recording), else a synthesized fallback beep. Re-arms
//  only after the value drops back below (threshold - hysteresis), and won't
//  repeat within the cooldown.
#define ALERT_VOLUME_PCT         70
#define ALERT_COOLDOWN_MS        8000
#define ALERT_KNOCK_DEG          3.0f      // active knock retard [deg]
#define ALERT_BOOST_PSI          25.0f     // overboost [psi]
#define ALERT_COOLANT_F          240.0f    // coolant over-temp [degF]
#define ALERT_OIL_F              270.0f    // oil over-temp [degF]

// -----------------------------------------------------------------------------
//  GPS / GNSS  (u-blox SAM-M10Q on Port A = UART)
// -----------------------------------------------------------------------------
//  Port A on the Tab5 breaks out GPIO53/54. The SAM-M10Q is a UART NMEA-0183
//  device: its TX (NMEA out) is wired to GPS_UART_RX_PIN, and the P4's
//  GPS_UART_TX_PIN -> module RX carries our config. The ESP32-P4 GPIO matrix
//  lets UART1 route to any pins. UART0 is the debug console, so GPS uses UART1.
//
//  GPS_AUTOCONFIG: on boot the driver sends UBX-CFG-VALSET to set the update
//  rate (GPS_NAV_RATE_HZ) and switch the module to GPS_BAUD. It sends the config
//  first at GPS_FACTORY_BAUD (to convert a fresh 9600 module) then again at the
//  target baud, and the reader auto-detects the baud (cycling GPS_BAUD <->
//  factory) if no valid NMEA arrives - so it self-heals from any module state
//  and never bricks the link. A bare u-blox module ships at 9600 baud / 1 Hz.
#define GPS_ENABLED              1
#define GPS_UART_NUM             1         // UART1 (UART0 = console)
#define GPS_UART_RX_PIN          53        // P4 GPIO <- module TX  (Port A)
#define GPS_UART_TX_PIN          54        // P4 GPIO -> module RX  (Port A)
#define GPS_BAUD                 115200    // operating baud (autoconfig target)
#define GPS_AUTOCONFIG           1         // push rate+baud to the module on boot
#define GPS_NAV_RATE_HZ          10        // fixes/sec (10 Hz -> ~100 ms timing)
#define GPS_FACTORY_BAUD         9600      // module's default; used to bootstrap

//  Port A (and the other external connectors) get 5V only when EXT5V_EN is
//  asserted: PI4IOE5V6408 #1 (I2C 0x43, the BSP's first IO expander), pin 2,
//  active-high - the same rail M5.Power.setExtOutput() toggles. The Espressif
//  BSP never drives it, so the port is dead until we do. Set to 0 if the module
//  is powered externally. (The pin is IO_EXPANDER_PIN_NUM_2 on bsp_io_expander.)
#define GPS_ENABLE_EXT5V         1

//  Prefer GPS ground speed over OBD VSS for the 0-60 / quarter-mile timers when
//  a fix is available. Accurate trap speed, but only as fine as the GPS update
//  rate (1 Hz at the default baud). Set to 0 to always time off OBD VSS.
#define PERF_USE_GPS             1

