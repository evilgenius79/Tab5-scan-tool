// =============================================================================
//  screen_readiness.cpp - I/M Readiness Monitors (OBD Mode 01 PID 01).
// -----------------------------------------------------------------------------
//  The classic "will it pass smog" view: MIL (check-engine) status, stored-DTC
//  count, and each emissions monitor's readiness - READY / NOT READY / N/A.
//  A REFRESH button re-reads; the OBD task decodes PID 01 into ReadinessInfo.
// =============================================================================
#include "ui/screens/screens.h"
#include "ui/ui_theme.h"
#include "core/event_bus.h"

#include <cstdio>

namespace {

lv_obj_t* g_mil_lbl   = nullptr;
lv_obj_t* g_dtc_lbl   = nullptr;
lv_obj_t* g_table     = nullptr;
uint8_t   g_shown     = 255;   // last rendered monitor count (avoid redraw spam)
bool      g_last_valid = false;

void request_read() {
    ObdCommand c{ CmdType::ReadReadiness, 0, 0 };
    EventBus::instance().sendCommand(c, 0);
    g_shown = 255;             // force refresh on next update
}

void refresh_cb(lv_event_t*) { request_read(); }

} // namespace

void screen_readiness_create(lv_obj_t* parent) {
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 12, 0);
    lv_obj_set_style_pad_row(parent, 10, 0);

    // --- Status bar: MIL lamp + DTC count + refresh -------------------------
    lv_obj_t* bar = lv_obj_create(parent);
    lv_obj_add_style(bar, &st_screen, 0);
    lv_obj_set_width(bar, lv_pct(100));
    lv_obj_set_height(bar, 70);
    lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(bar, 18, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

    g_mil_lbl = lv_label_create(bar);
    lv_obj_set_style_text_font(g_mil_lbl, &lv_font_montserrat_20, 0);
    lv_label_set_text(g_mil_lbl, LV_SYMBOL_WARNING " MIL: --");

    g_dtc_lbl = lv_label_create(bar);
    lv_obj_set_style_text_font(g_dtc_lbl, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(g_dtc_lbl, COL_TEXT, 0);
    lv_label_set_text(g_dtc_lbl, "DTCs: --");

    lv_obj_t* rb = lv_btn_create(bar);
    lv_obj_add_style(rb, &st_accent_btn, 0);
    lv_obj_add_event_cb(rb, refresh_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* rl = lv_label_create(rb);
    lv_label_set_text(rl, LV_SYMBOL_REFRESH " READ");
    lv_obj_center(rl);

    // --- Monitor table ------------------------------------------------------
    g_table = lv_table_create(parent);
    lv_obj_set_width(g_table, lv_pct(100));
    lv_obj_set_flex_grow(g_table, 1);
    lv_table_set_column_count(g_table, 2);
    lv_table_set_column_width(g_table, 0, 360);
    lv_table_set_column_width(g_table, 1, 220);
    lv_table_set_cell_value(g_table, 0, 0, "MONITOR");
    lv_table_set_cell_value(g_table, 0, 1, "STATUS");
    lv_obj_set_style_text_font(g_table, &lv_font_montserrat_18, LV_PART_ITEMS);
    lv_obj_set_style_bg_color(g_table, COL_PANEL, LV_PART_ITEMS);
    lv_obj_set_style_text_color(g_table, COL_TEXT, LV_PART_ITEMS);
    lv_obj_set_style_border_color(g_table, COL_GRID, LV_PART_ITEMS);
    lv_obj_set_style_pad_ver(g_table, 6, LV_PART_ITEMS);

    request_read();   // kick off an initial read when the screen is built
}

void screen_readiness_update(void) {
    ReadinessInfo r = EventBus::instance().getReadiness();
    if (r.valid == g_last_valid && r.mon_count == g_shown) return;
    g_shown = r.mon_count;
    g_last_valid = r.valid;

    char buf[32];
    if (!r.valid) {
        lv_label_set_text(g_mil_lbl, LV_SYMBOL_WARNING " MIL: --");
        lv_obj_set_style_text_color(g_mil_lbl, COL_TEXT_DIM, 0);
        lv_label_set_text(g_dtc_lbl, "DTCs: --");
        lv_table_set_row_count(g_table, 1);
        return;
    }

    lv_label_set_text(g_mil_lbl, r.mil_on ? LV_SYMBOL_WARNING " MIL: ON"
                                          : LV_SYMBOL_OK " MIL: OFF");
    lv_obj_set_style_text_color(g_mil_lbl, r.mil_on ? COL_RED : COL_GREEN, 0);
    snprintf(buf, sizeof(buf), "DTCs: %u", r.dtc_count);
    lv_label_set_text(g_dtc_lbl, buf);

    lv_table_set_row_count(g_table, r.mon_count + 1);
    for (uint8_t i = 0; i < r.mon_count; ++i) {
        lv_table_set_cell_value(g_table, i + 1, 0, r.mon[i].name);
        const char* s; (void)s;
        switch (r.mon[i].state) {
            case MonState::Ready:        s = LV_SYMBOL_OK " READY";   break;
            case MonState::NotReady:     s = "NOT READY";             break;
            default:                     s = "n/a";                   break;
        }
        lv_table_set_cell_value(g_table, i + 1, 1, s);
    }
}
