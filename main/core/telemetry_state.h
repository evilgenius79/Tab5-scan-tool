// =============================================================================
//  telemetry_state.h - The shared, live telemetry snapshot.
// -----------------------------------------------------------------------------
//  This is the single source of truth the OBD task writes and the UI / logger
//  read. Access is mediated by the event_bus (mutex-guarded copy in/out) so no
//  module touches these fields directly across a thread boundary.
// =============================================================================
#pragma once

#include <cstdint>

struct TelemetryState {
    // --- Standard SAE PIDs ---------------------------------------------------
    float    rpm;                 // engine speed             [rpm]
    float    speed_kph;           // vehicle speed (VSS)      [km/h]
    float    coolant_c;           // ECT                      [degC]
    float    intake_air_c;        // IAT                      [degC]
    float    map_kpa;             // manifold abs pressure    [kPa]
    float    throttle_pct;        // throttle position        [%]
    float    afr;                 // air/fuel ratio (from O2/wideband)
    float    battery_v;           // control module voltage   [V]

    // --- Custom / enhanced PIDs (turbo / performance builds) -----------------
    float    boost_psi;           // gauge boost = MAP - baro  [psi]
    float    charge_air_c;        // CACT / post-intercooler  [degC]
    float    knock_retard_deg;    // active knock retard      [deg]
    float    hpfp_pressure_bar;   // high-pressure fuel pump  [bar]
    float    ignition_adv_deg;    // commanded ign advance    [deg]

    // --- Derived performance metrics ----------------------------------------
    float    accel_0_60_s;        // last 0-60 mph time       [s] (0 = none yet)
    float    quarter_mile_s;      // last 1/4 mile ET         [s]
    float    quarter_mile_trap;   // trap speed               [mph]

    // --- Health / freshness --------------------------------------------------
    uint64_t last_update_us;      // timestamp of newest field write
    uint64_t last_good_pid_us;    // timestamp of last successfully decoded PID
    uint32_t poll_hz;             // measured poll loop frequency
    uint32_t frames_dropped;      // ring-buffer overflow counter
};
