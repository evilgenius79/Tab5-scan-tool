// =============================================================================
//  tab5_power.cpp - Tab5 IP2326 charger enable / status via the IO expander.
// =============================================================================
#include "core/tab5_power.h"

#include "bsp/esp-bsp.h"
#include "esp_io_expander.h"
#include "esp_log.h"

static const char* TAG = "Tab5Pwr";

// Charger pins on the second PI4IOE5V6408 expander (I2C 0x44):
//   pin 7 = charge enable (drive high to charge)
//   pin 6 = charge status (HIGH = charging)
//   pin 5 = quick-charge enable, active-low (left high = default 500 mA)
namespace tab5pwr {

static esp_io_expander_handle_t expander() { return bsp_io_expander1_init(); }

void enable_charging() {
    esp_io_expander_handle_t h = expander();
    if (!h) { ESP_LOGW(TAG, "IO expander unavailable; cannot enable charging"); return; }

    esp_io_expander_set_dir(h, IO_EXPANDER_PIN_NUM_6, IO_EXPANDER_INPUT);   // status in
    esp_io_expander_set_dir(h, IO_EXPANDER_PIN_NUM_7, IO_EXPANDER_OUTPUT);  // enable out
    esp_io_expander_set_output_mode(h, IO_EXPANDER_PIN_NUM_7,
                                    IO_EXPANDER_OUTPUT_MODE_PUSH_PULL);
    esp_io_expander_set_level(h, IO_EXPANDER_PIN_NUM_7, 1);                 // charge ON
    ESP_LOGI(TAG, "battery charging enabled (status pin=%d)", is_charging());
}

bool is_charging() {
    esp_io_expander_handle_t h = expander();
    if (!h) return false;
    uint32_t level = 0;
    if (esp_io_expander_get_level(h, IO_EXPANDER_PIN_NUM_6, &level) == ESP_OK)
        return level != 0;
    return false;
}

} // namespace tab5pwr
