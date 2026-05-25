// =============================================================================
//  screen_performance.cpp - Screen 2: Performance Tracker.
// -----------------------------------------------------------------------------
//  0-60 mph and 1/4 mile timing, clocked by the OBD task off VSS. The UI just
//  arms the run (StartPerfRun command) and displays the live speed plus the
//  captured results. The actual timing logic lives in obd_task to keep it on
//  the high-priority I/O core where VSS samples are freshest.
// =============================================================================
#include "ui/screens/screens.h"
#include "ui/ui_theme.h"
#include "core/event_bus.h"

#include <cstdio>

namespace {

lv_obj_t* g_speed_val = nullptr;
lv_obj_t* g_t060_val  = nullptr;
lv_obj_t* g_qmile_val = nullptr;
lv_obj_t* g_trap_val  = nullptr;
lv_obj_t* g_arm_btn   = nullptr;
lv_obj_t* g_arm_lbl   = nullptr;
lv_obj_t* g_pk_rpm    = nullptr;
lv_obj_t* g_pk_boost  = nullptr;
lv_obj_t* g_pk_cool   = nullptr;
lv_obj_t* g_pk_speed  = nullptr;
lv_obj_t* g_mpg_now   = nullptr;
lv_obj_t* g_trip_mpg  = nullptr;
lv_obj_t* g_trip_mi   = nullptr;
lv_obj_t* g_trip_gal  = nullptr;
lv_obj_t* g_trip_time = nullptr;

// Reusable "stat card": big value over a dim caption.
lv_obj_t* stat_card(lv_obj_t* parent, const char* caption, const char* init) {
    lv_obj_t* card = ui_make_panel(parent, nullptr);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_t* val = lv_label_create(card);
    lv_obj_add_style(val, &st_value_big, 0);
    lv_label_set_text(val, init);
    lv_obj_t* cap = lv_label_create(card);
    lv_obj_add_style(cap, &st_label_dim, 0);
    lv_label_set_text(cap, caption);
    return val;
}

void arm_clicked_cb(lv_event_t*) {
    ObdCommand cmd{ CmdType::StartPerfRun, 0, 0 };
    EventBus::instance().sendCommand(cmd, 0);
    lv_label_set_text(g_arm_lbl, LV_SYMBOL_GPS " ARMED - LAUNCH!");
    lv_obj_set_style_border_color(g_arm_btn, COL_MAGENTA, 0);
    lv_obj_set_style_text_color(g_arm_btn, COL_MAGENTA, 0);
}

void reset_peaks_cb(lv_event_t*) {
    ObdCommand cmd{ CmdType::ResetPeaks, 0, 0 };
    EventBus::instance().sendCommand(cmd, 0);
}

} // namespace

void screen_performance_create(lv_obj_t* parent) {
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(parent, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(parent, 16, 0);
    lv_obj_set_style_pad_row(parent, 16, 0);

    // Live speed banner.
    lv_obj_t* banner = ui_make_panel(parent, "VEHICLE SPEED");
    lv_obj_set_width(banner, lv_pct(100));
    lv_obj_set_height(banner, 120);
    g_speed_val = lv_label_create(banner);
    lv_obj_add_style(g_speed_val, &st_value_big, 0);
    lv_obj_set_style_text_font(g_speed_val, &lv_font_montserrat_48, 0);
    lv_label_set_text(g_speed_val, "0 mph");
    lv_obj_center(g_speed_val);

    // Result cards row.
    lv_obj_t* row = lv_obj_create(parent);
    lv_obj_add_style(row, &st_screen, 0);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_height(row, 130);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    g_t060_val  = stat_card(row, "0-60 MPH (s)", "--.--");
    g_qmile_val = stat_card(row, "1/4 MILE (s)", "--.--");
    g_trap_val  = stat_card(row, "TRAP (mph)",   "---");

    // Session peaks row.
    lv_obj_t* prow = lv_obj_create(parent);
    lv_obj_add_style(prow, &st_screen, 0);
    lv_obj_set_width(prow, lv_pct(100));
    lv_obj_set_height(prow, 130);
    lv_obj_set_flex_flow(prow, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(prow, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(prow, LV_OBJ_FLAG_SCROLLABLE);
    g_pk_rpm   = stat_card(prow, "PEAK RPM",     "0");
    g_pk_boost = stat_card(prow, "PEAK BOOST",   "0.0");
    g_pk_cool  = stat_card(prow, "MAX COOL (F)", "0");
    g_pk_speed = stat_card(prow, "TOP SPEED",    "0");

    // Fuel economy / trip computer row.
    lv_obj_t* erow = lv_obj_create(parent);
    lv_obj_add_style(erow, &st_screen, 0);
    lv_obj_set_width(erow, lv_pct(100));
    lv_obj_set_height(erow, 120);
    lv_obj_set_flex_flow(erow, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(erow, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(erow, LV_OBJ_FLAG_SCROLLABLE);
    g_mpg_now   = stat_card(erow, "MPG (NOW)",   "--.-");
    g_trip_mpg  = stat_card(erow, "TRIP MPG",    "--.-");
    g_trip_mi   = stat_card(erow, "TRIP (MI)",   "0.0");
    g_trip_gal  = stat_card(erow, "FUEL (GAL)",  "0.00");
    g_trip_time = stat_card(erow, "TRIP TIME",   "0:00");

    // Button row: arm run + reset peaks.
    lv_obj_t* brow = lv_obj_create(parent);
    lv_obj_add_style(brow, &st_screen, 0);
    lv_obj_set_width(brow, lv_pct(100));
    lv_obj_set_height(brow, 70);
    lv_obj_set_flex_flow(brow, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(brow, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(brow, LV_OBJ_FLAG_SCROLLABLE);

    g_arm_btn = lv_btn_create(brow);
    lv_obj_add_style(g_arm_btn, &st_accent_btn, 0);
    lv_obj_set_size(g_arm_btn, 300, 60);
    lv_obj_add_event_cb(g_arm_btn, arm_clicked_cb, LV_EVENT_CLICKED, nullptr);
    g_arm_lbl = lv_label_create(g_arm_btn);
    lv_label_set_text(g_arm_lbl, LV_SYMBOL_PLAY " ARM RUN");
    lv_obj_center(g_arm_lbl);

    lv_obj_t* rst = lv_btn_create(brow);
    lv_obj_add_style(rst, &st_accent_btn, 0);
    lv_obj_set_size(rst, 240, 60);
    lv_obj_add_event_cb(rst, reset_peaks_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* rl = lv_label_create(rst);
    lv_label_set_text(rl, LV_SYMBOL_REFRESH " RESET SESSION");
    lv_obj_center(rl);
}

void screen_performance_update(void) {
    TelemetryState t = EventBus::instance().snapshot();
    char buf[24];

    snprintf(buf, sizeof(buf), "%.0f mph", t.speed_kph * 0.621371f);
    lv_label_set_text(g_speed_val, buf);

    // Session peaks (USA units).
    snprintf(buf, sizeof(buf), "%.0f", t.peak_rpm);
    lv_label_set_text(g_pk_rpm, buf);
    snprintf(buf, sizeof(buf), "%.1f", t.peak_boost_psi);
    lv_label_set_text(g_pk_boost, buf);
    snprintf(buf, sizeof(buf), "%.0f", t.peak_coolant_c * 1.8f + 32.0f);
    lv_label_set_text(g_pk_cool, buf);
    snprintf(buf, sizeof(buf), "%.0f", t.top_speed_kph * 0.621371f);
    lv_label_set_text(g_pk_speed, buf);

    // Fuel economy / trip computer.
    if (t.mpg_instant > 0) snprintf(buf, sizeof(buf), "%.1f", t.mpg_instant);
    else                   snprintf(buf, sizeof(buf), "--.-");
    lv_label_set_text(g_mpg_now, buf);
    if (t.trip_mpg > 0) snprintf(buf, sizeof(buf), "%.1f", t.trip_mpg);
    else                snprintf(buf, sizeof(buf), "--.-");
    lv_label_set_text(g_trip_mpg, buf);
    snprintf(buf, sizeof(buf), "%.1f", t.trip_distance_mi);
    lv_label_set_text(g_trip_mi, buf);
    snprintf(buf, sizeof(buf), "%.2f", t.trip_fuel_gal);
    lv_label_set_text(g_trip_gal, buf);
    snprintf(buf, sizeof(buf), "%u:%02u", (unsigned)(t.trip_time_s / 60),
             (unsigned)(t.trip_time_s % 60));
    lv_label_set_text(g_trip_time, buf);

    if (t.accel_0_60_s > 0) {
        snprintf(buf, sizeof(buf), "%.2f", t.accel_0_60_s);
        lv_label_set_text(g_t060_val, buf);
    }
    if (t.quarter_mile_s > 0) {
        snprintf(buf, sizeof(buf), "%.2f", t.quarter_mile_s);
        lv_label_set_text(g_qmile_val, buf);
        snprintf(buf, sizeof(buf), "%.0f", t.quarter_mile_trap);
        lv_label_set_text(g_trap_val, buf);
        // Run complete: reset the arm button affordance.
        lv_label_set_text(g_arm_lbl, LV_SYMBOL_PLAY " ARM RUN");
        lv_obj_set_style_border_color(g_arm_btn, COL_CYAN, 0);
        lv_obj_set_style_text_color(g_arm_btn, COL_CYAN, 0);
    }
}
