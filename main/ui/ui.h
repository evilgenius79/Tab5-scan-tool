// =============================================================================
//  ui.h - Top-level UI bring-up and screen registry.
// -----------------------------------------------------------------------------
//  ui_init() brings up the Tab5 display via the BSP, installs esp_lvgl_port
//  (which owns the LVGL task pinned to APP_CORE_UI), applies the carbon theme,
//  and builds the six-screen tab layout. ui_task_start() registers the
//  periodic lv_timers that pump live data into the widgets.
// =============================================================================
#pragma once

// Top-level screens, in tab order.
enum class ScreenId : int {
    Dash = 0,      // live telemetry: radial gauges + sparklines
    Performance,   // 0-60 / quarter-mile timers
    Diagnostics,   // DTC table read/clear
    Readiness,     // I/M emissions readiness monitors
    Logging,       // CSV logging toggle + status
    Sniffer,       // raw CAN frame monitor + filters
    Settings,      // baud / brightness / bus selection
    _Count
};

// Bring up display + LVGL + screens. Call once from app_main (on APP_CORE_UI).
void ui_init();

// Register the live-update lv_timers. Call after ui_init().
void ui_task_start();

// Returns the index of the currently visible tab (for update gating).
int ui_active_screen();
