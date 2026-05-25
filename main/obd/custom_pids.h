// =============================================================================
//  custom_pids.h - User/profile-defined PIDs (manufacturer-specific).
// -----------------------------------------------------------------------------
//  Standard OBD PIDs are universal and built in. Manufacturer parameters (knock
//  retard, charge-air temp, fuel pressure, ...) are NOT standard, so they are
//  defined as data, not code:
//
//    * If /sdcard/custom_pids.csv exists it is loaded (works for ANY make).
//    * Otherwise a small built-in Ford EcoBoost default set is used.
//
//  CSV columns (one PID per line, '#' comments allowed):
//    name,request,bytes,signed,scale,offset,unit
//  e.g.   Knock Retard,220318,1,1,0.0625,0,deg
//  value = (signed?signed_raw:raw) * scale + offset
//
//  IMPORTANT: the built-in Ford values are community FORScan figures and should
//  be verified against your vehicle - edit the CSV to correct them.
// =============================================================================
#pragma once

#include <cstdint>
#include <cstddef>

struct CustomPid {
    char     name[20];
    char     request[10];   // hex request, e.g. "220318"
    uint8_t  mode;          // parsed from request (0x22, 0x01, ...)
    uint16_t pid;           // parsed from request
    uint8_t  bytes;         // data bytes to use (1 or 2)
    bool     is_signed;
    float    scale;
    float    offset;
    char     unit[8];
};

namespace custpid {

// Load definitions: SD /sdcard/custom_pids.csv if present (any make), else the
// built-in Ford EcoBoost defaults - but ONLY when `make` is a Ford/Lincoln, so
// Ford-specific scaling is never applied to another marque. Pass the
// VIN-decoded manufacturer ("" if unknown). Idempotent.
void        load(const char* make);
size_t      count();
const CustomPid& def(size_t i);
const char* source();                    // "SD" / "Ford defaults" / "none"

// Live values (thread-safe): OBD task writes, UI reads.
void  setValue(size_t i, float v);
bool  getValue(size_t i, float& out);

} // namespace custpid
