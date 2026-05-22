// =============================================================================
//  dtc_lookup.h - Map a numeric DTC (e.g. "P0301") to a human-readable meaning.
// -----------------------------------------------------------------------------
//  The ECU only ever returns the numeric code; the description is always a
//  client-side lookup. This module is a hybrid source:
//
//    1. A curated table of common generic SAE J1979 codes compiled into flash,
//       so descriptions work with no SD card present.
//    2. An optional /sdcard/dtc_db.csv that adds manufacturer-specific codes
//       (P1xxx etc.) and can override/extend the built-in text without a
//       reflash. SD entries take precedence over the embedded table.
//
//  CSV format (one code per line, first comma splits code from text):
//      P0301,Cylinder 1 Misfire Detected
//      # lines beginning with '#' and blank lines are ignored
// =============================================================================
#pragma once

namespace dtc {

// Attempt to load the SD database (idempotent). The embedded table needs no
// init. Safe to call repeatedly: it retries the SD read until it succeeds once
// (e.g. after the card is mounted), then becomes a no-op. Performs file I/O, so
// call it from a user-action context (the DTC READ handler), not a hot loop.
void ensureLoaded();

// Return a description for a DTC code, or nullptr if unknown. Lookup order is
// SD database first, then the embedded generic table.
const char* describe(const char* code);

} // namespace dtc
