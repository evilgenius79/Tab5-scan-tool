// =============================================================================
//  ui_task.cpp - periodic live-data refresh driven by LVGL timers.
// -----------------------------------------------------------------------------
//  esp_lvgl_port already owns the LVGL task (rendering + input). We don't spawn
//  another OS task; instead we register lv_timers, whose callbacks run inside
//  that LVGL task. That means no cross-thread locking is needed when touching
//  widgets, and the EventBus accessors we call are themselves thread-safe.
//
//  Two cadences:
//    * fast timer (UI_GAUGE_REFRESH_MS): gauges/sparklines on the dash + the
//      sniffer drain, but only for whichever screen is currently visible.
//    * slow timer (250 ms): low-rate screens (diagnostics, logging, settings).
// =============================================================================
#include "ui/ui.h"
#include "ui/screens/screens.h"
#include "app_config.h"

#include "esp_lvgl_port.h"

// Refresh only the visible screen to save cycles; off-screen widgets keep
// their last state and refresh the instant their tab is shown.
static void fast_refresh_cb(lv_timer_t*) {
    switch ((ScreenId)ui_active_screen()) {
    case ScreenId::Dash:     screen_dash_update();     break;
    case ScreenId::Sniffer:  screen_sniffer_update();  break;
    case ScreenId::LiveData: screen_livedata_update(); break;
    default: break;
    }
}

static void slow_refresh_cb(lv_timer_t*) {
    switch ((ScreenId)ui_active_screen()) {
    case ScreenId::Performance: screen_performance_update(); break;
    case ScreenId::Diagnostics: screen_diagnostics_update(); break;
    case ScreenId::Readiness:   screen_readiness_update();   break;
    case ScreenId::Vehicle:     screen_vehicle_update();     break;
    case ScreenId::Modules:     screen_modules_update();     break;
    case ScreenId::Logging:     screen_logging_update();     break;
    case ScreenId::Settings:    screen_settings_update();    break;
    default: break;
    }
}

void ui_task_start() {
    lvgl_port_lock(0);
    lv_timer_create(fast_refresh_cb, UI_GAUGE_REFRESH_MS, nullptr);
    lv_timer_create(slow_refresh_cb, 250, nullptr);
    lvgl_port_unlock();
}
