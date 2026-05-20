// =============================================================================
//  obd_task.cpp - dual-mode OBD/USB orchestrator.
// =============================================================================
#include "obd/obd_task.h"
#include "obd/stn_commands.h"
#include "obd/stn_parser.h"
#include "obd/pid_definitions.h"

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
        // Battery voltage is an adapter command, not an OBD request.
        resp = g_link.sendCommand("ATRV", &ok);
        if (ok && !resp.empty()) {
            float v = strtof(resp.c_str(), nullptr);   // "14.2V"
            if (v > 1.0f) g_telem.battery_v = v;
        }
        return;
    }

    // STPX wrapper: "STPX d:<request>, r:1" issues the OBD request with one
    // expected response, minimizing adapter round-trip latency.
    std::string cmd = std::string(STN_CMD_POLL_PREFIX) + "d:" + pid.request + ", r:1";
    resp = g_link.sendCommand(cmd, &ok);
    if (!ok) return;

    // Decode the request mode/pid from the catalog string (e.g. "010C").
    uint8_t  mode = (uint8_t)strtoul(std::string(pid.request).substr(0, 2).c_str(), nullptr, 16);
    uint16_t pidn = (uint16_t)strtoul(std::string(pid.request).substr(2).c_str(), nullptr, 16);

    uint8_t data[16];
    int n = stn::parsePidResponse(resp, mode, pidn, data, sizeof(data));
    if (n > 0 && pid.decode) {
        pid.decode(data, (uint8_t)n, g_telem);
    }
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
        std::string resp = g_link.sendCommand(OBD_MODE_READ_DTC, &ok);
        DtcRecord recs[MAX_DTCS];
        size_t n = stn::parseDtcs(resp, recs, MAX_DTCS);
        bus.setDtcs(recs, n);
        ESP_LOGI(TAG, "read %u DTCs", (unsigned)n);
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
            pollPid(kPidCatalog[pid_idx]);
            pid_idx = (pid_idx + 1) % kPidCount;
            poll_count++;
            updatePerf();
            break;
        }

        case ObdMode::Idle:
        default:
            vTaskDelay(pdMS_TO_TICKS(50));
            break;
        }

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
