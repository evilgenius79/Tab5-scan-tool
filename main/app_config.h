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

