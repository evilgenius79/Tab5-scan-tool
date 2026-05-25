// =============================================================================
//  alerts.h - audible warnings through the Tab5 speaker.
// -----------------------------------------------------------------------------
//  A low-priority task watches the live telemetry and plays a warning when a
//  parameter crosses a threshold (knock, overboost, over-temp, MIL set). Each
//  alert plays /sdcard/sounds/<name>.wav if the file exists - so you can drop
//  in your own voice clip - otherwise it falls back to a synthesized beep.
//
//  WAV files should be PCM (uncompressed): 16-bit, mono or stereo, any common
//  sample rate (8-48 kHz). The codec is opened to match the file's format.
// =============================================================================
#pragma once

namespace alerts {

// Bring up the speaker codec and start the monitor task. Call once after the
// BSP/I2C bus is up (i.e. after ui_init), from app_main.
void init();

// Master mute. Persisted by the Settings screen; defaults enabled.
void set_enabled(bool on);
bool enabled();

// Play a named alert now (looks up /sdcard/sounds/<name>.wav, else beeps).
// Exposed so the UI can offer a "test sound" button.
void play(const char* name);

} // namespace alerts
