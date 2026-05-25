// =============================================================================
//  sd_logger.cpp - microSD CSV logging task.
// =============================================================================
#include "logging/sd_logger.h"
#include "core/event_bus.h"
#include "obd/custom_pids.h"
#include "obd/dtc_lookup.h"
#include "app_config.h"

#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <unistd.h>     // fsync()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
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

// --- Diagnostic log: mirror the ESP-IDF console (esp_log) to a file on SD so
// an in-vehicle session can be reviewed afterward without a serial cable. ---
FILE*             g_diag_file   = nullptr;
SemaphoreHandle_t g_diag_mtx    = nullptr;
vprintf_like_t    g_prev_vprintf = nullptr;   // original (UART) sink

// esp_log hook: emit to the original UART sink AND append to the SD file.
// The non-recursive mutex (taken with 0 timeout) also blocks reentrancy if a
// FATFS write itself logs an error.
int diag_vprintf(const char* fmt, va_list ap) {
    va_list ap2;
    va_copy(ap2, ap);
    int r = g_prev_vprintf ? g_prev_vprintf(fmt, ap) : vprintf(fmt, ap);
    if (g_diag_file && g_diag_mtx &&
        xSemaphoreTake(g_diag_mtx, 0) == pdTRUE) {
        vfprintf(g_diag_file, fmt, ap2);
        xSemaphoreGive(g_diag_mtx);
    }
    va_end(ap2);
    return r;
}

// Open the diagnostic log and install the hook. Call once, after SD is mounted.
void diag_log_init() {
    if (g_diag_file) return;
    // Persist across boots: APPEND so a module scan from any session is kept.
    // Auto-trim (start fresh) if the file has grown past ~1 MB so the card
    // doesn't fill up over time.
    const char* mode = "a";
    FILE* probe = fopen(SD_MOUNT_POINT "/diag.log", "r");
    if (probe) {
        fseek(probe, 0, SEEK_END);
        if (ftell(probe) > 1024 * 1024) mode = "w";   // too big -> truncate
        fclose(probe);
    }
    g_diag_file = fopen(SD_MOUNT_POINT "/diag.log", mode);
    if (!g_diag_file) {
        ESP_LOGW(TAG, "could not open diag.log");
        return;
    }
    g_diag_mtx = xSemaphoreCreateMutex();
    // Boot separator so sessions are easy to tell apart in the appended file.
    fprintf(g_diag_file, "\n==== boot (uptime %llu ms) ====\n",
            (unsigned long long)(esp_timer_get_time() / 1000));
    g_prev_vprintf = esp_log_set_vprintf(diag_vprintf);
    ESP_LOGI(TAG, "diagnostic log -> " SD_MOUNT_POINT "/diag.log (append)");
}

void diag_log_flush() {
    if (g_diag_file && g_diag_mtx &&
        xSemaphoreTake(g_diag_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
        fflush(g_diag_file);                 // C stdio buffer -> FATFS
        fsync(fileno(g_diag_file));          // commit FAT directory entry/size
        long sz = ftell(g_diag_file);
        xSemaphoreGive(g_diag_mtx);
        // One-shot confirmation that bytes are actually landing on the card.
        static bool reported = false;
        if (!reported) { reported = true; ESP_LOGI(TAG, "diag.log committed (%ld bytes)", sz); }
    }
}

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
        // Standard columns are always populated. The old fixed knock_retard_deg/
        // charge_air_c/hpfp_bar columns were dropped: those TelemetryState fields
        // are never written (manufacturer params live in the custom-PID store),
        // so they only ever logged zeros. Instead, append one column per loaded
        // custom/profile PID with its real name + unit, pulled live from custpid.
        fprintf(g_file,
            "timestamp_us,rpm,speed_kph,map_kpa,boost_psi,afr,eng_load_pct,"
            "maf_gps,coolant_c,iat_c,throttle_pct,ign_adv_deg,battery_v,mpg,"
            "stft_pct,ltft_pct,fuel_level_pct,oil_c,ambient_c,run_time_s");
        for (size_t i = 0; i < custpid::count(); ++i) {
            const CustomPid& c = custpid::def(i);
            fprintf(g_file, ",%s[%s]", c.name, c.unit);
        }
        fputc('\n', g_file);
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
        "%llu,%.0f,%.1f,%.1f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.1f,"
        "%.1f,%.1f,%.0f,%.0f,%.0f,%u",
        (unsigned long long)t.last_update_us,
        t.rpm, t.speed_kph, t.map_kpa, t.boost_psi, t.afr, t.engine_load,
        t.maf_gps, t.coolant_c, t.intake_air_c, t.throttle_pct,
        t.ignition_adv_deg, t.battery_v, t.mpg_instant,
        t.short_fuel_trim, t.long_fuel_trim, t.fuel_level_pct,
        t.oil_temp_c, t.ambient_c, (unsigned)t.run_time_s);
    // Live custom/profile PID values (knock retard, charge-air temp, boost, ...).
    for (size_t i = 0; i < custpid::count(); ++i) {
        float v;
        if (custpid::getValue(i, v)) fprintf(g_file, ",%.2f", v);
        else                         fputc(',', g_file);   // not yet polled
    }
    fputc('\n', g_file);
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
    if (++g_rows % LOG_FLUSH_EVERY_N_ROWS == 0) {
        fflush(g_file);
        fsync(fileno(g_file));   // commit FAT entry so key-off doesn't lose the file
    }
}

void loggerTask(void*) {
    auto& bus = EventBus::instance();

    // Bring the diagnostic log up immediately (single mount attempt) so the
    // adapter connect/init/poll sequence is captured for an in-vehicle review.
    if (sd_card_ensure_mounted()) {
        diag_log_init();
        // Preload the SD DTC description database now (card is mounted) so code
        // meanings + the Settings count are ready before the first DTC read,
        // and the read itself isn't stalled by ~9k lines of file I/O. Runs once
        // here on this low-priority task; the UI's later ensureLoaded() calls
        // become no-ops (g_sdResolved is already set).
        dtc::ensureLoaded();
    }

    can_frame_t frame;
    uint64_t last_telem_us = 0;
    uint64_t last_logged_update_us = 0;   // dedup: skip unchanged snapshots
    uint64_t last_diag_flush_us = 0;

    while (true) {
        // Flush the diag log about once a second so an abrupt power-off (key
        // off / unplug) loses at most ~1s of log tail.
        const uint64_t now_us = esp_timer_get_time();
        if (now_us - last_diag_flush_us >= 1000000ULL) {
            diag_log_flush();
            last_diag_flush_us = now_us;
        }

        const bool want = bus.logging_enabled.load();
        // Only start a log once the adapter is online - otherwise the file fills
        // with zero rows during boot / on the bench before any data exists.
        const bool online = bus.link.load() == LinkState::Online;

        if (want && online && !g_file) {
            // Mount on first use; if the card/LDO isn't available, drop the
            // logging request so the UI switch reflects reality.
            if (sd_card_ensure_mounted()) {
                openFile(bus.mode.load() == ObdMode::Sniffing);
            } else {
                bus.logging_enabled.store(false);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
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
            // Telemetry sampling. Poll one row per actual data refresh: the OBD
            // task advances last_update_us once per poll loop (~8-9 Hz over USB),
            // so sampling on a fixed 50 Hz timer previously wrote each snapshot
            // 5-7x with an identical timestamp. Dedup on last_update_us to emit
            // exactly one row per fresh poll. Still skip the all-zero pre-data
            // state (last_update_us == 0).
            const uint64_t now = esp_timer_get_time();
            if (now - last_telem_us >= 20000) {
                TelemetryState t = bus.snapshot();
                if (t.last_update_us != 0 &&
                    t.last_update_us != last_logged_update_us) {
                    writeTelemetryRow(t);
                    maybeFlush();
                    last_logged_update_us = t.last_update_us;
                }
                last_telem_us = now;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

} // namespace

// Lazy mount: attempted only on demand because bsp_sdcard_mount() powers the
// card via an on-chip LDO channel, and probing it in a boot-time retry loop
// spams LDO-acquire errors when no card is present.
bool sd_card_ensure_mounted() {
    if (g_card_mounted) return true;
    if (bsp_sdcard_mount() == ESP_OK) {
        g_card_mounted = true;
        ESP_LOGI(TAG, "microSD mounted at %s", SD_MOUNT_POINT);
        return true;
    }
    ESP_LOGW(TAG, "microSD mount failed (no card / LDO unavailable)");
    return false;
}

void sd_logger_start() {
    xTaskCreatePinnedToCore(loggerTask, "sd_logger", STACK_SD_LOGGER, nullptr,
                            PRIO_SD_LOGGER, nullptr, APP_CORE_IO);
}
