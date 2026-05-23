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
#include "bsp/esp-bsp.h"
#include "esp_io_expander.h"

static const char* TAG = "main";

// -----------------------------------------------------------------------------
//  Enable the Tab5's internal battery charger. The IP2326 charge controller is
//  gated by pin 7 of the second PI4IOE5V6408 IO expander (0x44); the Tab5 only
//  charges once powered on and this pin is driven high. (Pin 5 = quick-charge
//  enable, active-low; left high for the default 500 mA rate. Pin 6 = charge
//  status input.) Without this the tablet runs only on USB and never charges.
// -----------------------------------------------------------------------------
static void enable_battery_charging() {
    esp_io_expander_handle_t e2 = bsp_io_expander1_init();
    if (!e2) { ESP_LOGW(TAG, "charger: IO expander unavailable"); return; }
    esp_io_expander_set_dir(e2, IO_EXPANDER_PIN_NUM_7, IO_EXPANDER_OUTPUT);
    esp_io_expander_set_output_mode(e2, IO_EXPANDER_PIN_NUM_7,
                                    IO_EXPANDER_OUTPUT_MODE_PUSH_PULL);
    esp_io_expander_set_level(e2, IO_EXPANDER_PIN_NUM_7, 1);   // charge enable
    ESP_LOGI(TAG, "battery charging enabled");
}

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
    ui_init();                 // brings up the BSP (display, I2C, IO expanders)
    enable_battery_charging(); // turn on the IP2326 charger (needs the I2C bus)
    ui_task_start();

    // --- 3. I/O + logging tasks --------------------------------------------
    obd_task_start();
    sd_logger_start();

    ESP_LOGI(TAG, "boot complete; free PSRAM=%u internal=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}
