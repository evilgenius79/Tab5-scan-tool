// =============================================================================
//  obd_task.cpp - dual-mode OBD/USB orchestrator.
// =============================================================================
#include "obd/obd_task.h"
#include "obd/stn_commands.h"
#include "obd/stn_parser.h"
#include "obd/pid_definitions.h"
#include "obd/vin_decode.h"
#include "obd/custom_pids.h"

#include "usb/usb_host_cdc.h"
#include "usb/obd_link.h"

#include "core/event_bus.h"
#include "app_config.h"

#include <string>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "ObdTask";

namespace {

// --- Module-local I/O objects (single instance, lives on the I/O task) ------
UsbHostCdc      g_usb;
ObdLink         g_link(g_usb);
TelemetryState  g_telem{};         // working copy; published to the bus each loop
bool            g_reinit_needed = false;  // set when the adapter is seen echoing
                                          // (it reset and lost ATE0) - re-init

// Performance-run capture state.
struct PerfRun {
    bool     armed       = false;
    bool     running     = false;
    uint64_t start_us    = 0;
    bool     hit_60      = false;
    float    last_speed  = 0.0f;
} g_perf;

// --- Connection callback (from USB daemon context) --------------------------
void onConnChange(bool connected) {
    auto& bus = EventBus::instance();
    bus.link.store(connected ? LinkState::Enumerating : LinkState::Disconnected);
}

// ---------------------------------------------------------------------------
//  Mode A: poll one PID from the catalog and fold the result into g_telem.
//  Uses STPX for standard PIDs and a raw service request for enhanced ones.
// ---------------------------------------------------------------------------
void pollPid(const PidDef& pid) {
    bool ok = false;
    std::string resp;

    if (strncmp(pid.request, "ATRV", 4) == 0) {
        // Battery voltage is an adapter command (voltage at OBD pin 16), not an
        // OBD request. Skip any non-numeric prefix before parsing "14.2V".
        resp = g_link.sendCommand("ATRV", &ok);
        const char* p = resp.c_str();
        while (*p && (*p < '0' || *p > '9') && *p != '.') ++p;
        float v = strtof(p, nullptr);
        if (ok && v > 1.0f) g_telem.battery_v = v;
        static uint64_t s_batt_log = 0;
        const uint64_t bnow = esp_timer_get_time();
        if (bnow - s_batt_log > 2000000) {   // ~1 line / 2s
            ESP_LOGI("Poll", "Battery       req=ATRV -> '%s' = %.2fV",
                     resp.c_str(), v);
            s_batt_log = bnow;
        }
        return;
    }

    // Send the plain OBD request (e.g. "010C"). Plain Mode 01/22 requests are
    // universally supported; we previously wrapped these in STPX, whose syntax
    // is finicky and was returning errors. The adapter answers with the
    // positive-response echo + data (e.g. "41 0C 1A F8").
    resp = g_link.sendCommand(pid.request, &ok);

    // If the reply begins with the request itself, command echo is ON - the
    // adapter reset and lost its ATE0 config. Flag for re-initialization.
    if (resp.rfind(pid.request, 0) == 0) g_reinit_needed = true;

    // Throttled poll trace (~3 lines/s) -> console + SD diag.log. Lets an
    // in-vehicle session be debugged afterward: shows the adapter's raw reply
    // and whether it parsed, without flooding the log on every poll.
    static uint64_t s_last_log = 0;
    const uint64_t now = esp_timer_get_time();
    const bool do_log = (now - s_last_log) > 300000;

    if (!ok || stn::isErrorResponse(resp)) {
        if (do_log) {
            ESP_LOGI("Poll", "%-13s req=%s -> '%s' (no data)",
                     pid.name, pid.request, resp.c_str());
            s_last_log = now;
        }
        return;
    }

    // Decode the request mode/pid from the catalog string (e.g. "010C").
    uint8_t  mode = (uint8_t)strtoul(std::string(pid.request).substr(0, 2).c_str(), nullptr, 16);
    uint16_t pidn = (uint16_t)strtoul(std::string(pid.request).substr(2).c_str(), nullptr, 16);

    uint8_t data[16];
    int n = stn::parsePidResponse(resp, mode, pidn, data, sizeof(data));
    if (do_log) {
        ESP_LOGI("Poll", "%-13s req=%s -> '%s' n=%d",
                 pid.name, pid.request, resp.c_str(), n);
        s_last_log = now;
    }
    if (n > 0 && pid.decode) {
        pid.decode(data, (uint8_t)n, g_telem);
        g_telem.last_good_pid_us = esp_timer_get_time();   // freshness for UI/diag
    }
}

// ---------------------------------------------------------------------------
//  Poll one user/profile-defined custom PID and store its scaled value.
// ---------------------------------------------------------------------------
void pollCustom(size_t idx) {
    const CustomPid& c = custpid::def(idx);
    bool ok = false;
    std::string resp = g_link.sendCommand(c.request, &ok);
    if (!ok || stn::isErrorResponse(resp)) return;

    uint8_t data[16];
    int n = stn::parsePidResponse(resp, c.mode, c.pid, data, sizeof(data));
    if (n < (int)c.bytes || c.bytes == 0) return;

    int32_t raw;
    if (c.bytes >= 2) {
        raw = (data[0] << 8) | data[1];
        if (c.is_signed && (raw & 0x8000)) raw -= 0x10000;
    } else {
        raw = data[0];
        if (c.is_signed && (raw & 0x80)) raw -= 0x100;
    }
    custpid::setValue(idx, raw * c.scale + c.offset);
}

// ---------------------------------------------------------------------------
//  Performance timer. Triggered by StartPerfRun; uses VSS (speed_kph) to clock
//  0-60 mph and the 1/4 mile. Distance is integrated from speed each loop.
// ---------------------------------------------------------------------------
float g_perf_distance_m = 0.0f;
uint64_t g_perf_last_us = 0;

void updatePerf() {
    const float mph = g_telem.speed_kph * 0.621371f;
    const uint64_t now = esp_timer_get_time();

    if (g_perf.armed && !g_perf.running) {
        // Launch when we start moving from a near stop.
        if (mph > 1.0f) {
            g_perf.running   = true;
            g_perf.start_us  = now;
            g_perf.hit_60    = false;
            g_perf_distance_m = 0.0f;
            g_perf_last_us   = now;
            ESP_LOGI(TAG, "perf run launched");
        }
        return;
    }
    if (!g_perf.running) return;

    // Integrate distance (trapezoidal) for the 1/4 mile (402.336 m).
    float dt = (now - g_perf_last_us) / 1e6f;
    g_perf_distance_m += (g_telem.speed_kph / 3.6f) * dt;
    g_perf_last_us = now;

    if (!g_perf.hit_60 && mph >= 60.0f) {
        g_perf.hit_60 = true;
        g_telem.accel_0_60_s = (now - g_perf.start_us) / 1e6f;
        ESP_LOGI(TAG, "0-60 = %.2fs", g_telem.accel_0_60_s);
    }
    if (g_perf_distance_m >= 402.336f) {
        g_telem.quarter_mile_s    = (now - g_perf.start_us) / 1e6f;
        g_telem.quarter_mile_trap = mph;
        g_perf.running = g_perf.armed = false;
        ESP_LOGI(TAG, "1/4 mile = %.2fs @ %.1f mph",
                 g_telem.quarter_mile_s, g_telem.quarter_mile_trap);
    }
}

// ---------------------------------------------------------------------------
//  Fuel economy + trip computer. Derives instantaneous MPG from MAF and the
//  air/fuel ratio, then integrates trip distance and fuel used each poll.
//    fuel_mass_rate = MAF / AFR                         [g/s]
//    fuel_vol_rate  = fuel_mass_rate / gasoline_density [gal/s]
//    MPG            = mph / (gal/s * 3600)
//  Gasoline density ~2789 g/gal (737 g/L). Uses live AFR when valid, else
//  stoichiometric 14.7. Accumulates only with the engine running; reset by
//  CmdType::ResetPeaks.
// ---------------------------------------------------------------------------
uint64_t g_trip_last_us = 0;
float    g_trip_time_acc = 0.0f;   // fractional-second accumulator

void updateTrip() {
    const uint64_t now = esp_timer_get_time();
    if (g_trip_last_us == 0) { g_trip_last_us = now; return; }
    const float dt = (now - g_trip_last_us) / 1e6f;
    g_trip_last_us = now;
    if (dt <= 0.0f || dt > 5.0f) return;   // skip gaps (mode switch, scan, ...)

    const float mph = g_telem.speed_kph * 0.621371f;
    const float afr = (g_telem.afr > 5.0f) ? g_telem.afr : 14.7f;
    constexpr float kGramsPerGal = 2789.0f;

    if (g_telem.maf_gps > 0.0f) {
        const float gal_s  = (g_telem.maf_gps / afr) / kGramsPerGal;
        const float gal_hr = gal_s * 3600.0f;
        g_telem.mpg_instant = (gal_hr > 1e-4f && mph > 0.5f) ? (mph / gal_hr) : 0.0f;
        g_telem.trip_distance_mi += (mph / 3600.0f) * dt;
        g_telem.trip_fuel_gal    += gal_s * dt;
        g_telem.trip_mpg = (g_telem.trip_fuel_gal > 1e-4f)
                         ? g_telem.trip_distance_mi / g_telem.trip_fuel_gal : 0.0f;
    }
    if (g_telem.rpm > 0.0f) {
        g_trip_time_acc += dt;
        if (g_trip_time_acc >= 1.0f) {
            g_telem.trip_time_s += (uint32_t)g_trip_time_acc;
            g_trip_time_acc -= (float)(uint32_t)g_trip_time_acc;
        }
    }
}

// ---------------------------------------------------------------------------
//  Decode Mode 01 PID 01 (I/M readiness). Data bytes A,B,C,D:
//    A: bit7 MIL on, bits0-6 stored-DTC count.
//    B: bit3 ignition type (1=compression/diesel); continuous monitors -
//       supported in bits0-2, "not complete" in bits4-6.
//    C/D: non-continuous monitors - C=supported, D=not complete (bit per type).
// ---------------------------------------------------------------------------
void decodeReadiness(const uint8_t* d, ReadinessInfo& r) {
    const uint8_t A = d[0], B = d[1], C = d[2], D = d[3];
    r.mil_on      = A & 0x80;
    r.dtc_count   = A & 0x7F;
    r.compression = B & 0x08;
    r.mon_count   = 0;

    auto add = [&](const char* name, bool supported, bool not_complete) {
        if (r.mon_count >= ReadinessInfo::MAX_MON) return;
        r.mon[r.mon_count].name  = name;
        r.mon[r.mon_count].state = !supported ? MonState::NotSupported
                                  : (not_complete ? MonState::NotReady
                                                  : MonState::Ready);
        r.mon_count++;
    };

    // Continuous monitors (byte B).
    add("Misfire",     B & 0x01, B & 0x10);
    add("Fuel System", B & 0x02, B & 0x20);
    add("Components",  B & 0x04, B & 0x40);

    // Non-continuous monitors (C supported / D not-complete), per ignition type.
    static const char* kSpark[8] = {
        "Catalyst", "Heated Catalyst", "Evap System", "Secondary Air",
        "A/C Refrig", "O2 Sensor", "O2 Heater", "EGR/VVT" };
    static const char* kDiesel[8] = {
        "NMHC Cat", "NOx/SCR Mon", "(reserved)", "Boost Pressure",
        "(reserved)", "Exhaust Sensor", "PM Filter", "EGR/VVT" };
    const char** names = r.compression ? kDiesel : kSpark;
    for (int i = 0; i < 8; ++i) add(names[i], C & (1 << i), D & (1 << i));

    r.valid = true;
}

// ---------------------------------------------------------------------------
//  Ford control-module map for enhanced UDS access (response header = req+8).
//  Used by both the module DTC scan (0x19) and clear (0x14).
// ---------------------------------------------------------------------------
struct ModuleAddr { const char* name; uint16_t req; };
const ModuleAddr g_fordModules[] = {
    { "PCM (Engine)",   0x7E0 }, { "TCM (Trans)",    0x7E1 },
    { "ABS",            0x760 }, { "Airbag (RCM)",   0x737 },
    { "BCM (Body)",     0x726 }, { "Cluster (IPC)",  0x720 },
    { "Pwr Steering",   0x730 }, { "HVAC",           0x733 },
    { "Park Aid (PAM)", 0x736 }, { "Restraints",     0x727 },
};
constexpr size_t kFordModuleCount = sizeof(g_fordModules) / sizeof(g_fordModules[0]);

// Restore OBD functional broadcast addressing after physical-module access.
//  Order matters: drain any late multi-frame bytes still arriving from the
//  last physically-addressed request FIRST, otherwise they leak into the
//  re-addressing commands (or the first few PID polls) as garbage / "STOPPED".
static void restoreFunctionalAddressing() {
    bool ok = false;
    g_link.drainUntilQuiet(150, 1500);   // flush stray ISO-TP tail frames
    g_link.sendCommand("ATCRA", &ok);    // clear the receive-address filter
    g_link.sendCommand("ATAR", &ok);     // automatic receive address
    g_link.sendCommand("ATSH7DF", &ok);  // OBD functional request header
    g_link.drainUntilQuiet(100, 500);    // and once more for good measure
}

// ---------------------------------------------------------------------------
//  Apply a UI command. Returns the (possibly changed) desired mode.
// ---------------------------------------------------------------------------
void applyCommand(const ObdCommand& cmd) {
    auto& bus = EventBus::instance();
    bool ok = false;
    char buf[48];

    switch (cmd.type) {
    case CmdType::SetMode:
        bus.mode.store((ObdMode)cmd.arg0);
        ESP_LOGI(TAG, "mode -> %d", cmd.arg0);
        break;

    case CmdType::SetSnifferFilter:
        // STFAP <pattern>,<mask>. arg_u32 = CAN id; mask covers 11 or 29 bits.
        g_link.sendCommand(STN_CMD_FILTER_CLEAR_ALL, &ok);
        snprintf(buf, sizeof(buf), "%s%lX,%s", STN_CMD_FILTER_PASS_ADD,
                 (unsigned long)cmd.arg_u32, cmd.arg0 ? "1FFFFFFF" : "7FF");
        g_link.sendCommand(buf, &ok);
        ESP_LOGI(TAG, "sniffer pass filter set: %lX", (unsigned long)cmd.arg_u32);
        break;

    case CmdType::ClearSnifferFilter:
        g_link.sendCommand(STN_CMD_FILTER_CLEAR_ALL, &ok);
        break;

    case CmdType::ClearDtcs:
        g_link.sendCommand(OBD_MODE_CLEAR_DTC, &ok);
        ESP_LOGI(TAG, "DTCs cleared (%s)", ok ? "ok" : "fail");
        break;

    case CmdType::ReadDtcs: {
        bus.dtc_read_active.store(true);
        DtcRecord recs[MAX_DTCS];
        size_t n = 0;
        // Stored (Mode 03), pending (Mode 07) and permanent (Mode 0A).
        std::string r3 = g_link.sendCommand(OBD_MODE_READ_DTC, &ok);     // "03"
        n += stn::parseDtcs(r3, recs + n, MAX_DTCS - n, 0x43, DTC_STORED);
        std::string r7 = g_link.sendCommand(OBD_MODE_PENDING_DTC, &ok);  // "07"
        n += stn::parseDtcs(r7, recs + n, MAX_DTCS - n, 0x47, DTC_PENDING);
        std::string rA = g_link.sendCommand("0A", &ok);                  // permanent
        n += stn::parseDtcs(rA, recs + n, MAX_DTCS - n, 0x4A, DTC_PERMANENT);
        bus.setDtcs(recs, n);
        bus.dtc_read_active.store(false);
        ESP_LOGI(TAG, "read %u DTCs (stored+pending+permanent)", (unsigned)n);
        break;
    }

    case CmdType::ReadVin: {
        // VIN is a multi-frame ISO-TP reply. Headers are already off globally
        // (set in initialize()), which keeps the reassembled output clean - do
        // NOT turn them back on or the single-frame PID polling parser breaks.
        // Give it a long timeout: the multi-frame reply (and any protocol
        // re-search) takes well over the 1s default.
        std::string resp = g_link.sendCommand(OBD_MODE_VIN, &ok, 5000);

        VehicleInfo info{};
        if (ok && stn::parseVin(resp, info.vin)) {
            info.valid = true;
            if (const char* mk = vin::manufacturer(info.vin)) {
                strncpy(info.manufacturer, mk, sizeof(info.manufacturer) - 1);
            }
            info.model_year = vin::model_year(info.vin);
            ESP_LOGI(TAG, "VIN %s (%s %d)", info.vin,
                     info.manufacturer[0] ? info.manufacturer : "?",
                     info.model_year);
        } else {
            ESP_LOGW(TAG, "VIN read failed");
        }

        // Calibration ID (Mode 09 PID 04) and ECU name (PID 0A) - both
        // multi-frame ASCII, same long-timeout handling as the VIN.
        std::string cal = g_link.sendCommand("0904", &ok, 5000);
        stn::parseMode09Ascii(cal, 0x04, info.cal_id, sizeof(info.cal_id));
        std::string ecu = g_link.sendCommand("090A", &ok, 5000);
        stn::parseMode09Ascii(ecu, 0x0A, info.ecu_name, sizeof(info.ecu_name));
        if (info.cal_id[0])   ESP_LOGI(TAG, "CALID: %s", info.cal_id);
        if (info.ecu_name[0]) ESP_LOGI(TAG, "ECU:   %s", info.ecu_name);

        bus.setVehicleInfo(info);
        break;
    }

    case CmdType::ReadReadiness: {
        // Mode 01 PID 01: MIL + stored-DTC count + emissions-monitor readiness.
        std::string resp = g_link.sendCommand("0101", &ok, 2000);
        uint8_t d[8];
        int n = stn::parsePidResponse(resp, 0x01, 0x01, d, sizeof(d));
        ReadinessInfo r{};
        if (n >= 4) decodeReadiness(d, r);
        bus.setReadiness(r);
        ESP_LOGI(TAG, "readiness: MIL=%d DTCs=%d monitors=%d",
                 r.mil_on, r.dtc_count, r.mon_count);
        break;
    }

    case CmdType::ScanModules: {
        // Enhanced multi-module DTC scan. Address each Ford control module
        // directly by its CAN header and run UDS ReadDTCInformation (19 02 FF).
        const ModuleAddr* kModules = g_fordModules;
        const size_t nmod = kFordModuleCount;

        bus.module_scan_active.store(true);
        ModuleResult results[MAX_MODULES];
        size_t mcount = 0;
        char cmd[16];

        for (size_t m = 0; m < nmod && mcount < MAX_MODULES; ++m) {
            // Point the adapter at this module: TX header + RX filter.
            snprintf(cmd, sizeof(cmd), "ATSH%03X", kModules[m].req);
            g_link.sendCommand(cmd, &ok);
            snprintf(cmd, sizeof(cmd), "ATCRA%03X", (kModules[m].req + 8) & 0xFFF);
            g_link.sendCommand(cmd, &ok);

            // UDS ReadDTCInformation, reportDTCByStatusMask, mask 0xFF (all).
            std::string resp = g_link.sendCommand("1902FF", &ok, 1500);

            ModuleResult& r = results[mcount];
            memset(&r, 0, sizeof(r));
            snprintf(r.name, sizeof(r.name), "%s", kModules[m].name);
            r.req_id    = kModules[m].req;

            bool uds_ok = ok && !stn::isErrorResponse(resp);
            if (uds_ok) {
                r.responded = true;
                r.dtc_count = (uint8_t)stn::parseUdsDtcs(resp, r.dtcs, 8);
            } else {
                // Fallback: many Ford powertrain modules (PCM 7E0 / TCM 7E1)
                // are pre-UDS and reject service 0x19 (negative "7F 19 ..") or
                // stay silent. They still answer the legacy OBD Mode 03 stored-
                // DTC query when addressed physically. Try that before giving up.
                std::string r03 = g_link.sendCommand("03", &ok, 1500);
                bool obd_ok = ok && !stn::isErrorResponse(r03);
                r.responded = obd_ok;
                r.dtc_count = obd_ok
                    ? (uint8_t)stn::parseDtcs(r03, r.dtcs, 8, 0x43, DTC_STORED) : 0;
                ESP_LOGI(TAG, "module %-14s %03X UDS '%s' -> Mode03 '%s'",
                         r.name, r.req_id, resp.c_str(), r03.c_str());
            }
            ESP_LOGI(TAG, "module %-14s %03X: %s, %u DTCs (raw '%s')", r.name,
                     r.req_id, r.responded ? "responded" : "no response",
                     r.dtc_count, resp.c_str());
            mcount++;
        }

        restoreFunctionalAddressing();   // so normal PID polling resumes
        bus.setModuleResults(results, mcount);
        bus.module_scan_active.store(false);
        ESP_LOGI(TAG, "module scan complete (%u modules probed)", (unsigned)mcount);
        break;
    }

    case CmdType::ClearModuleDtcs: {
        // Clear DTCs in every Ford module via UDS ClearDiagnosticInformation
        // (service 0x14, group 0xFFFFFF = all). Then re-scan to refresh the UI.
        bus.module_scan_active.store(true);
        char cmd[16];
        for (size_t m = 0; m < kFordModuleCount; ++m) {
            snprintf(cmd, sizeof(cmd), "ATSH%03X", g_fordModules[m].req);
            g_link.sendCommand(cmd, &ok);
            snprintf(cmd, sizeof(cmd), "ATCRA%03X", (g_fordModules[m].req + 8) & 0xFFF);
            g_link.sendCommand(cmd, &ok);
            std::string resp = g_link.sendCommand("14FFFFFF", &ok, 1500);
            ESP_LOGI(TAG, "clear %-14s %03X -> '%s'", g_fordModules[m].name,
                     g_fordModules[m].req, resp.c_str());
        }
        restoreFunctionalAddressing();
        // Also clear the standard emissions DTCs / MIL on the PCM.
        g_link.sendCommand(OBD_MODE_CLEAR_DTC, &ok);
        bus.module_scan_active.store(false);
        ESP_LOGI(TAG, "module clear complete");
        // Trigger a fresh scan so the screen reflects cleared state.
        ObdCommand rescan{ CmdType::ScanModules, 0, 0 };
        bus.sendCommand(rescan, 0);
        break;
    }

    case CmdType::SetBaud:
        g_link.setBaud(cmd.arg_u32);
        bus.current_baud.store(cmd.arg_u32);
        break;

    case CmdType::SelectBus:
        g_link.selectBus((CanBus)cmd.arg0);
        bus.bus.store((CanBus)cmd.arg0);
        break;

    case CmdType::StartPerfRun:
        g_perf.armed   = true;
        g_perf.running = false;
        g_telem.accel_0_60_s = g_telem.quarter_mile_s = 0;
        ESP_LOGI(TAG, "perf run armed");
        break;

    case CmdType::Reconnect:
        g_usb.close();   // hotplug task will reopen
        break;

    case CmdType::ResetPeaks:
        g_telem.peak_rpm = g_telem.peak_boost_psi = 0;
        g_telem.peak_coolant_c = g_telem.top_speed_kph = 0;
        // Reset the trip computer alongside the session peaks.
        g_telem.trip_distance_mi = g_telem.trip_fuel_gal = 0;
        g_telem.trip_mpg = g_telem.mpg_instant = 0;
        g_telem.trip_time_s = 0;
        g_trip_time_acc = 0.0f;
        ESP_LOGI(TAG, "session peaks + trip reset");
        break;
    }
}

// ---------------------------------------------------------------------------
//  Sniffer pump: read monitor lines, parse, fan out to the rings. Runs in a
//  tight loop while mode == Sniffing and not frozen, yielding to the command
//  queue between batches.
// ---------------------------------------------------------------------------
void runSniffer() {
    auto& bus = EventBus::instance();

    // Kick off the monitor. STM applies active filters; STMA monitors all.
    bus.snifferRing().clear();
    g_link.sendRaw(STN_CMD_MONITOR_ALL);
    ESP_LOGI(TAG, "sniffer started");

    std::string line;
    can_frame_t frame;
    while (bus.mode.load() == ObdMode::Sniffing) {
        // Drain pending commands without leaving sniff mode unless told.
        ObdCommand cmd;
        while (bus.recvCommand(cmd, 0)) {
            if (cmd.type == CmdType::SetSnifferFilter ||
                cmd.type == CmdType::ClearSnifferFilter) {
                // Re-arming a filter requires bouncing the monitor.
                g_link.stopMonitor();
                bus.sniffer_filter_active.store(cmd.type == CmdType::SetSnifferFilter);
                applyCommand(cmd);
                g_link.sendRaw(bus.sniffer_filter_active.load() ? STN_CMD_MONITOR_FILTERED
                                                                : STN_CMD_MONITOR_ALL);
            } else {
                applyCommand(cmd);
            }
            if (bus.mode.load() != ObdMode::Sniffing) break;
        }

        if (bus.sniffer_frozen.load()) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (g_link.readLine(line, 30)) {
            if (stn::parseCanFrame(line, frame)) {
                if (!bus.snifferRing().push(frame)) g_telem.frames_dropped++;
                if (bus.logging_enabled.load()) bus.loggerRing().push(frame);
            }
        }
    }

    g_link.stopMonitor();
    ESP_LOGI(TAG, "sniffer stopped");
}

// ---------------------------------------------------------------------------
//  The task body.
// ---------------------------------------------------------------------------
void obdTask(void*) {
    auto& bus = EventBus::instance();

    if (!g_usb.init(USB_RX_STREAM_BYTES, onConnChange)) {
        ESP_LOGE(TAG, "USB init failed; task halting");
        vTaskDelete(nullptr);
        return;
    }

    size_t   pid_idx     = 0;
    size_t   cust_idx    = 0;
    uint32_t poll_count  = 0;
    uint64_t hz_window   = esp_timer_get_time();

    while (true) {
        // --- Connection management ------------------------------------------
        if (!g_usb.isOpen()) {
            bus.link.store(LinkState::Disconnected);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (bus.link.load() == LinkState::Enumerating) {
            bus.link.store(LinkState::Initializing);
            if (g_link.initialize()) {
                bus.link.store(LinkState::Online);
                bus.current_baud.store(OBD_DEFAULT_BAUD);
                custpid::load();   // SD custom_pids.csv or built-in Ford defaults
                // Auto-populate identity + emissions status on connect, like a
                // real scan tool, so Vehicle Info / I/M Readiness / the Home
                // MIL badge fill in without the user tapping each screen first.
                applyCommand(ObdCommand{ CmdType::ReadReadiness, 0, 0 });
                applyCommand(ObdCommand{ CmdType::ReadVin, 0, 0 });
            } else {
                bus.link.store(LinkState::Error);
                vTaskDelay(pdMS_TO_TICKS(500));
                g_usb.close();           // force reconnect
                continue;
            }
        }
        if (bus.link.load() != LinkState::Online) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // --- Service one command per loop (sniffer drains its own) ----------
        ObdCommand cmd;
        if (bus.recvCommand(cmd, 0)) applyCommand(cmd);

        // --- Dispatch on mode ----------------------------------------------
        switch (bus.mode.load()) {
        case ObdMode::Sniffing:
            runSniffer();                // blocks until mode changes
            break;

        case ObdMode::Polling: {
            // Recover if the adapter reset and lost its config (echo came back).
            if (g_reinit_needed) {
                ESP_LOGW(TAG, "adapter echo detected - re-initializing");
                g_reinit_needed = false;
                g_link.initialize();
                break;
            }
            pollPid(kPidCatalog[pid_idx]);
            pid_idx = (pid_idx + 1) % kPidCount;
            // Interleave one custom/profile PID per loop (knock, CACT, ...).
            if (custpid::count() > 0) {
                pollCustom(cust_idx);
                cust_idx = (cust_idx + 1) % custpid::count();
            }
            poll_count++;
            updatePerf();
            updateTrip();
            break;
        }

        case ObdMode::Idle:
        default:
            vTaskDelay(pdMS_TO_TICKS(50));
            break;
        }

        // --- Session peak-hold ----------------------------------------------
        if (g_telem.rpm        > g_telem.peak_rpm)       g_telem.peak_rpm = g_telem.rpm;
        if (g_telem.boost_psi  > g_telem.peak_boost_psi) g_telem.peak_boost_psi = g_telem.boost_psi;
        if (g_telem.coolant_c  > g_telem.peak_coolant_c) g_telem.peak_coolant_c = g_telem.coolant_c;
        if (g_telem.speed_kph  > g_telem.top_speed_kph)  g_telem.top_speed_kph = g_telem.speed_kph;

        // --- Publish telemetry + measure poll rate --------------------------
        g_telem.last_update_us = esp_timer_get_time();
        const uint64_t elapsed = g_telem.last_update_us - hz_window;
        if (elapsed >= 1000000ULL) {
            g_telem.poll_hz = (uint32_t)(poll_count * 1000000ULL / elapsed);
            poll_count = 0;
            hz_window  = g_telem.last_update_us;
        }
        bus.publish(g_telem);
    }
}

} // namespace

void obd_task_start() {
    xTaskCreatePinnedToCore(obdTask, "obd_task", STACK_OBD_TASK, nullptr,
                            PRIO_OBD_TASK, nullptr, APP_CORE_IO);
}
