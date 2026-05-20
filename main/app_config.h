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
//  The OBDLink EX negotiates up to 2 Mbps over its FTDI link. Start fast; the
//  Settings screen can renegotiate via STBR.
#define OBD_DEFAULT_BAUD         2000000
#define OBD_FALLBACK_BAUD        115200

//  Milliseconds with no successful response before we declare the link dead
//  and kick the auto-reconnect path.
#define OBD_LINK_TIMEOUT_MS      2500

//  How long to wait for a single command's terminating prompt ('>') character.
#define OBD_CMD_TIMEOUT_MS       1000

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
