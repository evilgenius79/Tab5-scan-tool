// =============================================================================
//  sd_logger.h - High-speed CSV telemetry / CAN logger to the Tab5 microSD.
// -----------------------------------------------------------------------------
//  Sits behind the EventBus logger ring (for raw frames) and the telemetry
//  snapshot (for decoded values). When logging is enabled it opens a fresh,
//  timestamped CSV on the SD card and streams rows, batching fsync() to spare
//  the card's flash. Toggling logging_enabled false flushes and closes.
//
//  Two file shapes, chosen by the active ObdMode at start:
//    * telemetry-*.csv : one row per sample, decoded engineering units.
//    * canlog-*.csv    : one row per CAN frame (timestamp, id, dlc, bytes).
// =============================================================================
#pragma once

// Mounts the SD card (via the Tab5 BSP) and spawns the logger task. Safe to
// call once from app_main. If the card is absent the task idles and retries.
void sd_logger_start();

// Ensure the Tab5 microSD is mounted (idempotent). Returns true if the card is
// available. Shared so other subsystems (e.g. the DTC database) can read files
// off the card without racing the logger's own lazy mount.
bool sd_card_ensure_mounted();
