// =============================================================================
//  screens.h - Per-screen builders and live-update entry points.
// -----------------------------------------------------------------------------
//  Each screen owns a translation unit. create() populates a tab page; update()
//  is called from the UI refresh timer (only for the visible screen) to fold
//  fresh EventBus data into the widgets. All create()/update() calls happen on
//  the LVGL task, so no explicit locking is needed inside them.
// =============================================================================
#pragma once

#include "lvgl.h"

// Screen 1 - Live Telemetry Dash (RPM/Boost gauges, knock/AFR sparklines).
void screen_dash_create(lv_obj_t* parent);
void screen_dash_update(void);

// Screen 2 - Performance Tracker (0-60, 1/4 mile).
void screen_performance_create(lv_obj_t* parent);
void screen_performance_update(void);

// Screen 3 - Diagnostics (DTC table).
void screen_diagnostics_create(lv_obj_t* parent);
void screen_diagnostics_update(void);

// Screen 4 - Data Logging (CSV toggle + status).
void screen_logging_create(lv_obj_t* parent);
void screen_logging_update(void);

// Screen 5 - CAN Sniffer & Hacker (raw frame stream, filters, freeze).
void screen_sniffer_create(lv_obj_t* parent);
void screen_sniffer_update(void);

// Screen 6 - Settings (baud, brightness, HS/MS CAN).
void screen_settings_create(lv_obj_t* parent);
void screen_settings_update(void);

// Screen 7 - I/M Readiness Monitors (Mode 01 PID 01).
void screen_readiness_create(lv_obj_t* parent);
void screen_readiness_update(void);
