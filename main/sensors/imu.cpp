// =============================================================================
//  imu.cpp - BMI270 accelerometer + drag-launch detection.
// =============================================================================
#include "sensors/imu.h"
#include "core/event_bus.h"
#include "app_config.h"

#include "bsp/esp-bsp.h"
#include "iot_sensor_hub.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_log.h"

#include <cmath>

static const char* TAG = "IMU";

namespace {

// Launch is declared when the acceleration vector deviates from the captured
// at-rest (gravity) baseline by more than this magnitude, in G.
constexpr float    LAUNCH_THRESHOLD_G = 0.15f;
constexpr int      BASELINE_SAMPLES   = 16;   // averaged while staged + still

// Launch-detector state (touched only by the sensor-hub event handler).
bool     g_was_armed   = false;
bool     g_base_ready  = false;
bool     g_launched    = false;
int      g_base_n      = 0;
float    g_bx = 0, g_by = 0, g_bz = 0;        // baseline gravity vector

void onAccel(float ax, float ay, float az) {
    auto& bus = EventBus::instance();
    bus.imu_ax.store(ax);
    bus.imu_ay.store(ay);
    bus.imu_az.store(az);

    const bool armed = bus.perf_armed.load();
    if (!armed) { g_was_armed = false; return; }

    // (Re)arm when armed rises, or when a new run clears perf_launch_us back to 0
    // after we'd already detected a launch (so repeat runs recapture the baseline
    // even if the previous run never finished and left perf_armed set).
    if (!g_was_armed || (g_launched && bus.perf_launch_us.load() == 0)) {
        g_base_ready = false; g_launched = false; g_base_n = 0;
        g_bx = g_by = g_bz = 0;
    }
    g_was_armed = armed;
    if (g_launched) return;

    if (!g_base_ready) {                       // average the at-rest vector first
        g_bx += ax; g_by += ay; g_bz += az;
        if (++g_base_n >= BASELINE_SAMPLES) {
            g_bx /= g_base_n; g_by /= g_base_n; g_bz /= g_base_n;
            g_base_ready = true;
        }
        return;
    }

    const float dx = ax - g_bx, dy = ay - g_by, dz = az - g_bz;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) > LAUNCH_THRESHOLD_G) {
        bus.perf_launch_us.store((uint64_t)esp_timer_get_time());
        g_launched = true;
    }
}

void sensorEventHandler(void*, esp_event_base_t, int32_t event_id, void* event_data) {
    if (event_id != SENSOR_ACCE_DATA_READY) return;
    auto* d = static_cast<sensor_data_t*>(event_data);
    onAccel(d->acce.x, d->acce.y, d->acce.z);
}

} // namespace

namespace imu {

void start() {
    // The sensor hub posts data-ready events on the default event loop.
    esp_err_t le = esp_event_loop_create_default();
    if (le != ESP_OK && le != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event loop create failed (%s)", esp_err_to_name(le));
        return;
    }

    bsp_sensor_config_t cfg = {};
    cfg.type   = IMU_ID;
    cfg.mode   = MODE_POLLING;
    cfg.period = 10;                            // ms -> ~100 Hz accel sampling

    sensor_handle_t h = nullptr;
    if (bsp_sensor_init(&cfg, &h) != ESP_OK || !h) {
        ESP_LOGW(TAG, "BMI270 init failed; launch will fall back to speed");
        return;
    }
    iot_sensor_handler_register(h, sensorEventHandler, nullptr);
    if (iot_sensor_start(h) != ESP_OK) {
        ESP_LOGW(TAG, "sensor hub start failed");
        return;
    }
    EventBus::instance().imu_present.store(true);
    ESP_LOGI(TAG, "BMI270 started (100 Hz, launch threshold %.2fg)", LAUNCH_THRESHOLD_G);
}

} // namespace imu
