// =============================================================================
//  main.cpp - Tab5 Scan Tool entry point.
// -----------------------------------------------------------------------------
//  Boot sequence:
//    1. NVS (settings persistence) + the EventBus (shared state, rings, queues).
//    2. UI: bring up the display/LVGL and build the six screens. esp_lvgl_port
//       owns the LVGL task pinned to APP_CORE_UI.
//    3. UI refresh timers (live data -> widgets).
//    4. OBD/USB task on APP_CORE_IO (transport + dual-mode state machine).
//    5. SD logger task.
//
//  After spawning the tasks, app_main returns; FreeRTOS keeps the system alive.
// =============================================================================
#include "app_config.h"
#include "core/event_bus.h"
#include "ui/ui.h"
#include "obd/obd_task.h"
#include "logging/sd_logger.h"
#include "audio/alerts.h"
#include "gps/gps.h"
#include "sensors/imu.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "core/tab5_power.h"

static const char* TAG = "main";

static const char* reset_reason_str(esp_reset_reason_t r) {
    switch (r) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_EXT:       return "external pin";
    case ESP_RST_SW:        return "software restart";
    case ESP_RST_PANIC:     return "PANIC / exception";
    case ESP_RST_INT_WDT:   return "INTERRUPT watchdog";
    case ESP_RST_TASK_WDT:  return "TASK watchdog";
    case ESP_RST_WDT:       return "other watchdog";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (voltage dip)";
    case ESP_RST_DEEPSLEEP: return "deep-sleep wake";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "unknown";
    }
}

// Periodic heap report so a slow leak shows up as a downward trend in diag.log
// (and "min ever free" catches transient low-water marks near a crash).
static void heap_report_cb(void*) {
    ESP_LOGI(TAG, "heap: internal free=%u min=%u | psram free=%u min=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM));
}

extern "C" void app_main(void) {
    ESP_LOGW(TAG, "Tab5 Scan Tool starting; last reset: %s",
             reset_reason_str(esp_reset_reason()));

    // --- 1. NVS + shared state ---------------------------------------------
    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    if (!EventBus::instance().init()) {
        ESP_LOGE(TAG, "EventBus init failed - halting");
        return;
    }

    // --- 2. UI --------------------------------------------------------------
    ui_init();                 // brings up the BSP (display, I2C, IO expanders)
    tab5pwr::enable_charging(); // turn on the IP2326 charger (needs the I2C bus)
    alerts::init();             // speaker + threshold-driven audible warnings
    ui_task_start();

    // --- 3. I/O + logging tasks --------------------------------------------
    obd_task_start();
    sd_logger_start();
    gps::start();               // GNSS reader on the external UART (if enabled)
    imu::start();               // BMI270 accel + drag-tree launch detection

    ESP_LOGI(TAG, "boot complete; free PSRAM=%u internal=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    // Heap trend every 30 s -> diag.log, to spot a leak behind periodic reboots.
    const esp_timer_create_args_t ht = { heap_report_cb, nullptr,
                                         ESP_TIMER_TASK, "heap", false };
    esp_timer_handle_t h;
    if (esp_timer_create(&ht, &h) == ESP_OK)
        esp_timer_start_periodic(h, 30ULL * 1000 * 1000);
}
