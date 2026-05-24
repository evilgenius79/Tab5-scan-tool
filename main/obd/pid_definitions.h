// =============================================================================
//  pid_definitions.h - PID catalog: how to request and decode each parameter.
// -----------------------------------------------------------------------------
//  Each PidDef bundles the request bytes, the number of data bytes expected in
//  the response, and a decode function that writes the engineering value into
//  the shared TelemetryState.
//
//  STANDARD vs ENHANCED
//  --------------------
//  The Mode 01 PIDs (RPM, MAP, IAT, ...) are SAE J1979 standard and decode
//  identically across vehicles. The "enhanced" entries (knock retard, charge
//  air temp, HPFP pressure, ignition advance) are manufacturer-specific: they
//  ride on a non-standard header/mode and the scaling differs per platform.
//  The values below are illustrative (modeled on a common VAG/EA888 turbo
//  layout) and MUST be retuned per target ECU - they are isolated here so a
//  developer can drop in their own definitions without touching the poller.
// =============================================================================
#pragma once

#include <cstdint>
#include <cmath>
#include "core/telemetry_state.h"

// Decoder signature: receives the response data bytes (A=data[0], B=data[1]...)
// and the byte count, and mutates the telemetry snapshot in place.
using PidDecoder = void (*)(const uint8_t* d, uint8_t n, TelemetryState& t);

struct PidDef {
    const char* name;        // human label (UI / CSV header)
    const char* request;     // full STPX/Mode+PID request payload, hex
    uint8_t     resp_bytes;  // expected data bytes in the reply
    bool        enhanced;    // true => manufacturer-specific, retune required
    PidDecoder  decode;
};

// --- Standard SAE J1979 decoders --------------------------------------------
namespace dec {

inline void rpm(const uint8_t* d, uint8_t n, TelemetryState& t) {
    if (n >= 2) t.rpm = ((d[0] << 8) | d[1]) / 4.0f;
}
inline void speed(const uint8_t* d, uint8_t n, TelemetryState& t) {
    if (n >= 1) t.speed_kph = d[0];
}
inline void coolant(const uint8_t* d, uint8_t n, TelemetryState& t) {
    if (n >= 1) t.coolant_c = (int)d[0] - 40;
}
inline void iat(const uint8_t* d, uint8_t n, TelemetryState& t) {
    if (n >= 1) t.intake_air_c = (int)d[0] - 40;
}
inline void map_kpa(const uint8_t* d, uint8_t n, TelemetryState& t) {
    if (n >= 1) {
        t.map_kpa = d[0];
        // Gauge boost: MAP above ~101.3 kPa baro, converted to psi.
        t.boost_psi = (t.map_kpa - 101.3f) * 0.1450377f;
    }
}
inline void throttle(const uint8_t* d, uint8_t n, TelemetryState& t) {
    if (n >= 1) t.throttle_pct = d[0] * 100.0f / 255.0f;
}
inline void load(const uint8_t* d, uint8_t n, TelemetryState& t) {
    if (n >= 1) t.engine_load = d[0] * 100.0f / 255.0f;   // calculated load [%]
}
inline void baro(const uint8_t* d, uint8_t n, TelemetryState& t) {
    if (n >= 1) {
        t.baro_kpa = d[0];                                // absolute baro [kPa]
        // With a real baro reading, boost = MAP - baro is exact.
        if (t.map_kpa > 0) t.boost_psi = (t.map_kpa - t.baro_kpa) * 0.1450377f;
    }
}
inline void battery(const uint8_t* d, uint8_t n, TelemetryState& t) {
    if (n >= 2) t.battery_v = ((d[0] << 8) | d[1]) / 1000.0f; // module voltage
}
// Wideband AFR derived from commanded equivalence ratio (PID 44) * stoich.
inline void afr(const uint8_t* d, uint8_t n, TelemetryState& t) {
    if (n >= 2) {
        float lambda = ((d[0] << 8) | d[1]) * 2.0f / 65536.0f;
        t.afr = lambda * 14.7f;
    }
}

// Manufacturer-specific parameters (knock retard, charge-air temp, HPFP) are
// not standard PIDs - they are handled by the data-driven custom-PID loader
// (obd/custom_pids), not hard-coded decoders here.
inline void ign_adv(const uint8_t* d, uint8_t n, TelemetryState& t) {
    if (n >= 1) t.ignition_adv_deg = (d[0] / 2.0f) - 64.0f;  // SAE PID 0E scaling
}

} // namespace dec

// --- The polling catalog ----------------------------------------------------
//  Order matters: this is the round-robin poll sequence. Hot, fast-moving
//  parameters (RPM, MAP, knock) appear first / more often in obd_task's
//  scheduler. `request` strings are the data payloads handed to STPX.
static const PidDef kPidCatalog[] = {
    // name             request   bytes  enhanced  decoder
    { "RPM",            "010C",   2,     false,    dec::rpm        },
    { "MAP",            "010B",   1,     false,    dec::map_kpa    },
    { "Speed",          "010D",   1,     false,    dec::speed      },
    { "Throttle",       "0111",   1,     false,    dec::throttle   },
    { "IgnAdv",         "010E",   1,     false,    dec::ign_adv    },
    { "Coolant",        "0105",   1,     false,    dec::coolant    },
    { "IAT",            "010F",   1,     false,    dec::iat        },
    { "EngLoad",        "0104",   1,     false,    dec::load       },
    { "AFR",            "0144",   2,     false,    dec::afr        },
    { "Baro",           "0133",   1,     false,    dec::baro       },
    { "Battery",        "ATRV",   2,     false,    dec::battery    },

    // NOTE: real manufacturer parameters (knock retard, charge-air temp, HPFP)
    // are NOT standard OBD PIDs - they need a per-vehicle profile (e.g. Ford
    // EcoBoost UDS PIDs). The old 22F40C/F445/F446 entries here were just the
    // UDS *mirror* of standard PIDs (DID 0xF4xx == OBD PID 0xxx) and reported
    // mislabeled data, so they were removed. See the vehicle-profile work.
};

static constexpr size_t kPidCount = sizeof(kPidCatalog) / sizeof(kPidCatalog[0]);
