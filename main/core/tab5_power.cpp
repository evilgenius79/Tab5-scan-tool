// =============================================================================
//  tab5_power.cpp - Tab5 IP2326 charger enable / status via the IO expander.
// =============================================================================
#include "core/tab5_power.h"

#include "bsp/esp-bsp.h"
#include "esp_io_expander.h"
#include "driver/i2c_master.h"
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

// --- INA226 battery monitor (I2C 0x41) --------------------------------------
static i2c_master_dev_handle_t s_ina = nullptr;

static i2c_master_dev_handle_t ina() {
    if (s_ina) return s_ina;
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (!bus) return nullptr;
    i2c_device_config_t cfg = {};
    cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg.device_address  = 0x41;          // INA226 on the Tab5
    cfg.scl_speed_hz    = 400000;
    if (i2c_master_bus_add_device(bus, &cfg, &s_ina) != ESP_OK) s_ina = nullptr;
    return s_ina;
}

float battery_voltage() {
    i2c_master_dev_handle_t d = ina();
    if (!d) return 0.0f;
    // Bus-voltage register 0x02, LSB = 1.25 mV (calibration-independent).
    uint8_t reg = 0x02, rx[2] = {0, 0};
    if (i2c_master_transmit_receive(d, &reg, 1, rx, 2, 100) != ESP_OK) return 0.0f;
    uint16_t raw = (uint16_t)((rx[0] << 8) | rx[1]);
    return raw * 0.00125f;
}

int battery_percent() {
    float v = battery_voltage();
    if (v < 1.0f) return -1;                 // INA226 not responding
    float cell = v / 2.0f;                   // 2S pack
    // Piecewise Li-ion open-circuit SoC curve (per cell). Rough but reasonable.
    struct P { float v; int pct; };
    static const P k[] = {
        {4.20f,100},{4.10f,90},{4.00f,80},{3.90f,65},{3.80f,55},
        {3.70f,40},{3.60f,25},{3.50f,15},{3.40f,8},{3.30f,3},{3.00f,0},
    };
    if (cell >= k[0].v) return 100;
    for (size_t i = 1; i < sizeof(k)/sizeof(k[0]); ++i) {
        if (cell >= k[i].v) {
            float f = (cell - k[i].v) / (k[i-1].v - k[i].v);
            return (int)(k[i].pct + f * (k[i-1].pct - k[i].pct) + 0.5f);
        }
    }
    return 0;
}

} // namespace tab5pwr
