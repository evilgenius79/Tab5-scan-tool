// =============================================================================
//  gps.h - u-blox SAM-M10Q GNSS reader (UART / NMEA-0183).
// -----------------------------------------------------------------------------
//  Parses RMC + GGA sentences from a GPS module on an external UART port and
//  publishes a GpsFix snapshot into the EventBus. Pins/baud are set in
//  app_config.h (GPS_*). A no-op when GPS_ENABLED is 0.
// =============================================================================
#pragma once

#include <cstdint>

namespace gps {

// Attach to the BSP I2C bus and spawn the NMEA reader task on the I/O core.
// Safe to call once at boot, after the EventBus and BSP I2C are initialized.
void start();

// --- GPX track recording (handled entirely on the GPS task) -----------------
// Request start/stop of a GPX track log on the SD card. The reader task opens
// /sdcard/track-<ms>.gpx, appends a <trkpt> per fix, and closes it on stop.
void        track_set(bool on);
bool        track_active();      // true once the file is actually open
uint32_t    track_points();      // trackpoints written this session
const char* track_path();        // current/last GPX filename ("" if none)

// --- Link status (for the GPS screen) ---------------------------------------
uint32_t    link_baud();         // UART baud the reader is currently locked to
float       fix_hz();            // measured position-fix update rate (Hz)

} // namespace gps
