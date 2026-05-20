// =============================================================================
//  screen_logging.cpp - Screen 4: high-speed CSV logging control.
// -----------------------------------------------------------------------------
//  A single master toggle drives EventBus::logging_enabled, which the SD logger
//  task observes to open/close files. The file shape (telemetry vs raw CAN) is
//  chosen automatically from the active OBD mode, so this screen also surfaces
//  what will be logged and live status (link, frames dropped, poll rate).
// =============================================================================
#include "ui/screens/screens.h"
#include "ui/ui_theme.h"
#include "core/event_bus.h"

#include <cstdio>

namespace {

lv_obj_t* g_switch    = nullptr;
lv_obj_t* g_mode_lbl  = nullptr;
lv_obj_t* g_stat_lbl  = nullptr;

void toggle_cb(lv_event_t* e) {
    bool on = lv_obj_has_state((lv_obj_t*)lv_event_get_target(e), LV_STATE_CHECKED);
    EventBus::instance().logging_enabled.store(on);
}

} // namespace

void screen_logging_create(lv_obj_t* parent) {
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 20, 0);
    lv_obj_set_style_pad_row(parent, 18, 0);

    // Master toggle panel.
    lv_obj_t* panel = ui_make_panel(parent, "DATA LOGGING");
    lv_obj_set_width(panel, lv_pct(100));
    lv_obj_set_height(panel, 160);

    g_switch = lv_switch_create(panel);
    lv_obj_set_size(g_switch, 110, 56);
    lv_obj_align(g_switch, LV_ALIGN_LEFT_MID, 10, 10);
    lv_obj_set_style_bg_color(g_switch, COL_GRID, LV_PART_MAIN);
    lv_obj_set_style_bg_color(g_switch, COL_GREEN, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_add_event_cb(g_switch, toggle_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t* hint = lv_label_create(panel);
    lv_obj_add_style(hint, &st_value_big, 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_28, 0);
    lv_label_set_text(hint, "Record to microSD");
    lv_obj_align(hint, LV_ALIGN_LEFT_MID, 150, 10);

    // What-will-be-logged + status panel.
    lv_obj_t* info = ui_make_panel(parent, "STATUS");
    lv_obj_set_width(info, lv_pct(100));
    lv_obj_set_flex_grow(info, 1);

    g_mode_lbl = lv_label_create(info);
    lv_obj_add_style(g_mode_lbl, &st_label_dim, 0);
    lv_obj_set_style_text_font(g_mode_lbl, &lv_font_montserrat_18, 0);
    lv_label_set_text(g_mode_lbl, "Capture: --");
    lv_obj_align(g_mode_lbl, LV_ALIGN_TOP_LEFT, 0, 40);

    g_stat_lbl = lv_label_create(info);
    lv_obj_add_style(g_stat_lbl, &st_label_dim, 0);
    lv_obj_set_style_text_font(g_stat_lbl, &lv_font_montserrat_16, 0);
    lv_label_set_text(g_stat_lbl, "");
    lv_obj_align(g_stat_lbl, LV_ALIGN_TOP_LEFT, 0, 80);
}

void screen_logging_update(void) {
    auto& bus = EventBus::instance();
    TelemetryState t = bus.snapshot();

    const bool sniff = bus.mode.load() == ObdMode::Sniffing;
    char buf[160];
    snprintf(buf, sizeof(buf), "Capture: %s",
             sniff ? "RAW CAN frames (canlog-*.csv)"
                   : "Decoded telemetry (telemetry-*.csv)");
    lv_label_set_text(g_mode_lbl, buf);

    const char* link =
        bus.link.load() == LinkState::Online ? "ONLINE" : "OFFLINE";
    snprintf(buf, sizeof(buf),
             "Link: %s    Poll: %u Hz    Dropped frames: %u\n"
             "Logging: %s",
             link, (unsigned)t.poll_hz, (unsigned)t.frames_dropped,
             bus.logging_enabled.load() ? "ACTIVE" : "stopped");
    lv_label_set_text(g_stat_lbl, buf);
}
