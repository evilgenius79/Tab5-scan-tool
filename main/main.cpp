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

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char* TAG = "main";

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Tab5 Scan Tool starting");

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
    ui_init();
    ui_task_start();

    // --- 3. I/O + logging tasks --------------------------------------------
    obd_task_start();
    sd_logger_start();

    ESP_LOGI(TAG, "boot complete; free PSRAM=%u internal=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}
