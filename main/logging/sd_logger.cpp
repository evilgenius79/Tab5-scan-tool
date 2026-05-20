// =============================================================================
//  sd_logger.cpp - microSD CSV logging task.
// =============================================================================
#include "logging/sd_logger.h"
#include "core/event_bus.h"
#include "app_config.h"

#include <cstdio>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

// Tab5 BSP exposes SD mount helpers.
#include "bsp/esp-bsp.h"

static const char* TAG = "SdLogger";

namespace {

bool   g_card_mounted = false;
FILE*  g_file         = nullptr;
bool   g_logging_can  = false;     // file shape currently open
uint32_t g_rows       = 0;

// Open a new timestamped CSV and write the header for the current mode.
bool openFile(bool can_mode) {
    const uint64_t t = esp_timer_get_time() / 1000;   // ms since boot
    char path[64];
    snprintf(path, sizeof(path), SD_MOUNT_POINT "/%s-%llu.csv",
             can_mode ? "canlog" : "telemetry", (unsigned long long)t);

    g_file = fopen(path, "w");
    if (!g_file) {
        ESP_LOGE(TAG, "fopen('%s') failed", path);
        return false;
    }
    g_logging_can = can_mode;
    g_rows = 0;

    if (can_mode) {
        fprintf(g_file, "timestamp_us,can_id,extended,dlc,b0,b1,b2,b3,b4,b5,b6,b7\n");
    } else {
        fprintf(g_file,
            "timestamp_us,rpm,speed_kph,map_kpa,boost_psi,afr,knock_retard_deg,"
            "charge_air_c,coolant_c,iat_c,throttle_pct,ign_adv_deg,hpfp_bar,"
            "battery_v\n");
    }
    ESP_LOGI(TAG, "logging to %s", path);
    return true;
}

void closeFile() {
    if (g_file) {
        fflush(g_file);
        fclose(g_file);
        g_file = nullptr;
        ESP_LOGI(TAG, "log closed (%u rows)", (unsigned)g_rows);
    }
}

void writeTelemetryRow(const TelemetryState& t) {
    fprintf(g_file,
        "%llu,%.0f,%.1f,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f\n",
        (unsigned long long)t.last_update_us,
        t.rpm, t.speed_kph, t.map_kpa, t.boost_psi, t.afr, t.knock_retard_deg,
        t.charge_air_c, t.coolant_c, t.intake_air_c, t.throttle_pct,
        t.ignition_adv_deg, t.hpfp_pressure_bar, t.battery_v);
}

void writeFrameRow(const can_frame_t& f) {
    fprintf(g_file, "%llu,%lX,%d,%u",
            (unsigned long long)f.timestamp_us, (unsigned long)f.id,
            f.extended ? 1 : 0, f.dlc);
    for (int i = 0; i < 8; ++i) {
        if (i < f.dlc) fprintf(g_file, ",%02X", f.data[i]);
        else           fprintf(g_file, ",");
    }
    fputc('\n', g_file);
}

void maybeFlush() {
    if (++g_rows % LOG_FLUSH_EVERY_N_ROWS == 0) fflush(g_file);
}

void loggerTask(void*) {
    auto& bus = EventBus::instance();

    // Mount the card once; retry slowly if absent so a later insert works.
    while (!g_card_mounted) {
        if (bsp_sdcard_mount() == ESP_OK) {
            g_card_mounted = true;
            ESP_LOGI(TAG, "microSD mounted at %s", SD_MOUNT_POINT);
        } else {
            ESP_LOGW(TAG, "no microSD; retrying");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    can_frame_t frame;
    uint64_t last_telem_us = 0;

    while (true) {
        const bool want = bus.logging_enabled.load();

        if (want && !g_file) {
            // Pick file shape from the mode at the moment logging starts.
            openFile(bus.mode.load() == ObdMode::Sniffing);
        } else if (!want && g_file) {
            // Drain any remaining frames before closing for a clean tail.
            while (bus.loggerRing().pop(frame)) writeFrameRow(frame);
            closeFile();
        }

        if (!g_file) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

        if (g_logging_can) {
            // High-speed frame drain: pull a batch, then yield briefly.
            int drained = 0;
            while (drained < 256 && bus.loggerRing().pop(frame)) {
                writeFrameRow(frame);
                maybeFlush();
                drained++;
            }
            if (drained == 0) vTaskDelay(pdMS_TO_TICKS(5));
        } else {
            // Telemetry sampling at a fixed cadence (~50 Hz).
            const uint64_t now = esp_timer_get_time();
            if (now - last_telem_us >= 20000) {
                writeTelemetryRow(bus.snapshot());
                maybeFlush();
                last_telem_us = now;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

} // namespace

void sd_logger_start() {
    xTaskCreatePinnedToCore(loggerTask, "sd_logger", STACK_SD_LOGGER, nullptr,
                            PRIO_SD_LOGGER, nullptr, APP_CORE_IO);
}
