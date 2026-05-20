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
    lv_obj_set_height(banner, 160);
    g_speed_val = lv_label_create(banner);
    lv_obj_add_style(g_speed_val, &st_value_big, 0);
    lv_obj_set_style_text_font(g_speed_val, &lv_font_montserrat_48, 0);
    lv_label_set_text(g_speed_val, "0 mph");
    lv_obj_center(g_speed_val);

    // Result cards row.
    lv_obj_t* row = lv_obj_create(parent);
    lv_obj_add_style(row, &st_screen, 0);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_height(row, 160);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    g_t060_val  = stat_card(row, "0-60 MPH (s)", "--.--");
    g_qmile_val = stat_card(row, "1/4 MILE (s)", "--.--");
    g_trap_val  = stat_card(row, "TRAP (mph)",   "---");

    // Arm button.
    g_arm_btn = lv_btn_create(parent);
    lv_obj_add_style(g_arm_btn, &st_accent_btn, 0);
    lv_obj_set_size(g_arm_btn, 320, 64);
    lv_obj_add_event_cb(g_arm_btn, arm_clicked_cb, LV_EVENT_CLICKED, nullptr);
    g_arm_lbl = lv_label_create(g_arm_btn);
    lv_label_set_text(g_arm_lbl, LV_SYMBOL_PLAY " ARM RUN");
    lv_obj_center(g_arm_lbl);
}

void screen_performance_update(void) {
    TelemetryState t = EventBus::instance().snapshot();
    char buf[24];

    snprintf(buf, sizeof(buf), "%.0f mph", t.speed_kph * 0.621371f);
    lv_label_set_text(g_speed_val, buf);

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
