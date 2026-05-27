// =============================================================================
//  screen_tree.cpp - Drag-strip Christmas tree + reaction timer.
// -----------------------------------------------------------------------------
//  A full ("sportsman", .500) tree: tap STAGE, a random pre-light delay, then
//  the three amber bulbs cascade 0.5 s apart and green lights 0.5 s after the
//  last amber. Reaction time = (your launch) - (green on), where the launch is
//  timestamped by the BMI270 (EventBus::perf_launch_us) for ~10 ms accuracy.
//  Moving before green is a red-light foul. The same launch starts the 0-60 /
//  quarter-mile run (timed in obd_task), shown here once captured.
// =============================================================================
#include "ui/screens/screens.h"
#include "ui/ui.h"
#include "ui/ui_theme.h"
#include "core/event_bus.h"

#include "esp_timer.h"
#include "esp_random.h"
#include <cstdio>

namespace {

enum class Tree { Idle, Staging, Ambers, Green, Result };

Tree     g_state = Tree::Idle;
uint64_t g_stage_us = 0;     // when STAGE was pressed
uint64_t g_amber_us = 0;     // when the amber cascade started
uint64_t g_green_us = 0;     // when green lit
uint32_t g_delay_ms = 0;     // randomized pre-light delay

lv_obj_t* g_stage_bulb = nullptr;
lv_obj_t* g_amber[3]   = {nullptr, nullptr, nullptr};
lv_obj_t* g_green_bulb = nullptr;
lv_obj_t* g_red_bulb   = nullptr;
lv_obj_t* g_rt_lbl     = nullptr;   // big reaction-time readout
lv_obj_t* g_hint_lbl   = nullptr;
lv_obj_t* g_t060_val   = nullptr;
lv_obj_t* g_qmile_val  = nullptr;
lv_obj_t* g_trap_val   = nullptr;

constexpr lv_color_t off_amber() { return lv_color_hex(0x3A2A00); }
constexpr lv_color_t off_green() { return lv_color_hex(0x06351F); }
constexpr lv_color_t off_red()   { return lv_color_hex(0x3A0A0A); }

lv_obj_t* make_bulb(lv_obj_t* parent, int size, lv_color_t off) {
    lv_obj_t* b = lv_obj_create(parent);
    lv_obj_set_size(b, size, size);
    lv_obj_set_style_radius(b, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(b, 2, 0);
    lv_obj_set_style_border_color(b, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_color(b, off, 0);
    lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
    return b;
}

void light(lv_obj_t* b, lv_color_t on, bool lit, lv_color_t off) {
    lv_obj_set_style_bg_color(b, lit ? on : off, 0);
}

void all_off() {
    light(g_stage_bulb, COL_CYAN, false, lv_color_hex(0x0A2A33));
    for (auto* a : g_amber) light(a, COL_AMBER, false, off_amber());
    light(g_green_bulb, COL_GREEN, false, off_green());
    light(g_red_bulb,   COL_RED,   false, off_red());
}

void set_rt_text(const char* s, lv_color_t c) {
    lv_label_set_text(g_rt_lbl, s);
    lv_obj_set_style_text_color(g_rt_lbl, c, 0);
}

void stage_cb(lv_event_t*) {
    EventBus::instance().sendCommand(ObdCommand{CmdType::StartPerfRun, 0, 0}, 0);
    all_off();
    light(g_stage_bulb, COL_CYAN, true, lv_color_hex(0x0A2A33));
    set_rt_text("READY", COL_TEXT_DIM);
    lv_label_set_text(g_hint_lbl, "staged - launch on green");
    g_stage_us  = (uint64_t)esp_timer_get_time();
    g_delay_ms  = 700 + (esp_random() % 1000);   // 0.7-1.7 s anti-anticipation
    g_state     = Tree::Staging;
}

void reset_cb(lv_event_t*) {
    g_state = Tree::Idle;
    all_off();
    set_rt_text("--", COL_TEXT_DIM);
    lv_label_set_text(g_hint_lbl, "tap STAGE to start");
}

// 25 ms animation/timing tick (registered in create()).
void tick_cb(lv_timer_t*) {
    if (g_state == Tree::Idle || g_state == Tree::Result) return;
    const uint64_t now    = (uint64_t)esp_timer_get_time();
    const uint64_t launch = EventBus::instance().perf_launch_us.load();

    // Foul: any launch before green is a red light.
    if ((g_state == Tree::Staging || g_state == Tree::Ambers) && launch != 0) {
        all_off();
        light(g_red_bulb, COL_RED, true, off_red());
        set_rt_text("RED LIGHT", COL_RED);
        lv_label_set_text(g_hint_lbl, "left before green - foul");
        g_state = Tree::Result;
        return;
    }

    switch (g_state) {
    case Tree::Staging:
        if (now - g_stage_us >= (uint64_t)g_delay_ms * 1000) {
            g_amber_us = now;
            g_state = Tree::Ambers;
        }
        break;
    case Tree::Ambers: {
        const uint64_t e = now - g_amber_us;            // cascade 0.5 s apart
        light(g_amber[0], COL_AMBER, e >= 0,       off_amber());
        light(g_amber[1], COL_AMBER, e >= 500000,  off_amber());
        light(g_amber[2], COL_AMBER, e >= 1000000, off_amber());
        if (e >= 1500000ULL) {
            g_green_us = now;
            EventBus::instance().perf_green_us.store(now);
            light(g_green_bulb, COL_GREEN, true, off_green());
            lv_label_set_text(g_hint_lbl, "GO!");
            g_state = Tree::Green;
        }
        break;
    }
    case Tree::Green:
        if (launch != 0) {
            float rt = (float)((int64_t)launch - (int64_t)g_green_us) / 1e6f;
            char buf[24];
            snprintf(buf, sizeof(buf), "%.3f", rt);
            set_rt_text(buf, rt < 0.20f ? COL_GREEN : COL_AMBER);
            lv_label_set_text(g_hint_lbl, "reaction time (s)");
            g_state = Tree::Result;
        }
        break;
    default: break;
    }
}

lv_obj_t* stat_card(lv_obj_t* parent, const char* caption) {
    lv_obj_t* card = ui_make_panel(parent, nullptr);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_t* val = lv_label_create(card);
    lv_obj_add_style(val, &st_value_big, 0);
    lv_label_set_text(val, "--.--");
    lv_obj_t* cap = lv_label_create(card);
    lv_obj_add_style(cap, &st_label_dim, 0);
    lv_label_set_text(cap, caption);
    return val;
}

} // namespace

void screen_tree_create(lv_obj_t* parent) {
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(parent, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(parent, 16, 0);

    // --- Left: the tree (vertical bulb stack) -------------------------------
    lv_obj_t* tree = lv_obj_create(parent);
    lv_obj_add_style(tree, &st_screen, 0);
    lv_obj_set_size(tree, 150, lv_pct(100));
    lv_obj_set_flex_flow(tree, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(tree, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(tree, 10, 0);
    lv_obj_clear_flag(tree, LV_OBJ_FLAG_SCROLLABLE);

    g_stage_bulb = make_bulb(tree, 34, lv_color_hex(0x0A2A33));
    g_amber[0]   = make_bulb(tree, 60, off_amber());
    g_amber[1]   = make_bulb(tree, 60, off_amber());
    g_amber[2]   = make_bulb(tree, 60, off_amber());
    g_green_bulb = make_bulb(tree, 60, off_green());
    g_red_bulb   = make_bulb(tree, 34, off_red());

    // --- Right: reaction time + run results + controls ----------------------
    lv_obj_t* col = lv_obj_create(parent);
    lv_obj_add_style(col, &st_screen, 0);
    lv_obj_set_flex_grow(col, 1);
    lv_obj_set_height(col, lv_pct(100));
    lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(col, 14, 0);
    lv_obj_clear_flag(col, LV_OBJ_FLAG_SCROLLABLE);

    g_rt_lbl = lv_label_create(col);
    lv_obj_set_style_text_font(g_rt_lbl, &lv_font_montserrat_48, 0);
    set_rt_text("--", COL_TEXT_DIM);

    g_hint_lbl = lv_label_create(col);
    lv_obj_set_style_text_font(g_hint_lbl, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(g_hint_lbl, COL_TEXT_DIM, 0);
    lv_label_set_text(g_hint_lbl, "tap STAGE to start");

    lv_obj_t* row = lv_obj_create(col);
    lv_obj_add_style(row, &st_screen, 0);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_height(row, 130);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    g_t060_val  = stat_card(row, "0-60 MPH (s)");
    g_qmile_val = stat_card(row, "1/4 MILE (s)");
    g_trap_val  = stat_card(row, "TRAP (mph)");

    lv_obj_t* brow = lv_obj_create(col);
    lv_obj_add_style(brow, &st_screen, 0);
    lv_obj_set_width(brow, lv_pct(100));
    lv_obj_set_height(brow, 80);
    lv_obj_set_flex_flow(brow, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(brow, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(brow, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* stage = lv_btn_create(brow);
    lv_obj_add_style(stage, &st_accent_btn, 0);
    lv_obj_set_size(stage, 240, 64);
    lv_obj_add_event_cb(stage, stage_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* sl = lv_label_create(stage);
    lv_label_set_text(sl, LV_SYMBOL_PLAY " STAGE");
    lv_obj_center(sl);

    lv_obj_t* rst = lv_btn_create(brow);
    lv_obj_add_style(rst, &st_accent_btn, 0);
    lv_obj_set_size(rst, 200, 64);
    lv_obj_add_event_cb(rst, reset_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* rl = lv_label_create(rst);
    lv_label_set_text(rl, LV_SYMBOL_REFRESH " RESET");
    lv_obj_center(rl);

    all_off();
    lv_timer_create(tick_cb, 25, nullptr);
}

void screen_tree_update(void) {
    TelemetryState t = EventBus::instance().snapshot();
    char buf[16];
    if (t.accel_0_60_s > 0) {
        snprintf(buf, sizeof(buf), "%.2f", t.accel_0_60_s);
        lv_label_set_text(g_t060_val, buf);
    }
    if (t.quarter_mile_s > 0) {
        snprintf(buf, sizeof(buf), "%.2f", t.quarter_mile_s);
        lv_label_set_text(g_qmile_val, buf);
        snprintf(buf, sizeof(buf), "%.0f", t.quarter_mile_trap);
        lv_label_set_text(g_trap_val, buf);
    }
}
