// =============================================================================
//  screen_dash.cpp - Screen 1: Live Telemetry Dash (configurable gauges).
// -----------------------------------------------------------------------------
//  A grid of radial gauges. Each gauge slot has a dropdown to choose which
//  parameter it displays (RPM, Boost, AFR, timing, temps, ...), shows a numeric
//  readout + unit, and the assignments persist in NVS across reboots. Built on
//  lv_arc (display-only) so it compiles cleanly on LVGL 9.
// =============================================================================
#include "ui/screens/screens.h"
#include "ui/ui_theme.h"
#include "core/event_bus.h"

#include <cstdio>
#include <cstring>
#include <cstdint>
#include "nvs.h"

namespace {

// --- Available gauge channels (parameter metadata) ----------------------------
struct ChMeta { const char* name; const char* unit; float min; float max; int dec; };
const ChMeta kCh[] = {
    { "RPM",         "rpm",  0,    8000, 0 },   // 0
    { "Boost",       "psi", -15,   35,   1 },   // 1
    { "AFR",         "",     8,    20,   1 },   // 2
    { "Ign Timing",  "deg", -10,   50,   1 },   // 3
    { "Coolant",     "C",   -20,   130,  0 },   // 4
    { "Intake Air",  "C",   -20,   120,  0 },   // 5
    { "Speed",       "km/h", 0,    220,  0 },   // 6
    { "MAP",         "kPa",  0,    255,  0 },   // 7
    { "Throttle",    "%",    0,    100,  0 },   // 8
    { "Eng Load",    "%",    0,    100,  0 },   // 9
    { "Battery",     "V",    8,    16,   1 },   // 10
    { "Barometric",  "kPa",  80,   110,  0 },   // 11
};
constexpr int kChCount = sizeof(kCh) / sizeof(kCh[0]);

float channelValue(int ch, const TelemetryState& t) {
    switch (ch) {
        case 0:  return t.rpm;
        case 1:  return t.boost_psi;
        case 2:  return t.afr;
        case 3:  return t.ignition_adv_deg;
        case 4:  return t.coolant_c;
        case 5:  return t.intake_air_c;
        case 6:  return t.speed_kph;
        case 7:  return t.map_kpa;
        case 8:  return t.throttle_pct;
        case 9:  return t.engine_load;
        case 10: return t.battery_v;
        case 11: return t.baro_kpa;
        default: return 0;
    }
}

constexpr int NUM_GAUGES = 6;
uint8_t  g_slot[NUM_GAUGES] = { 0, 1, 2, 3, 4, 10 };  // default channel per slot

struct Gauge { lv_obj_t* arc; lv_obj_t* value; lv_obj_t* unit; lv_obj_t* dd; };
Gauge      g_gauge[NUM_GAUGES];
nvs_handle_t g_nvs = 0;
char       g_options[256];   // newline-joined channel names for the dropdowns

void persist() {
    if (g_nvs) { nvs_set_blob(g_nvs, "slots", g_slot, NUM_GAUGES); nvs_commit(g_nvs); }
}

// Apply a channel to a slot: re-title the dropdown range stays 0..1000 (mapped).
void applyChannel(int slot) {
    const ChMeta& m = kCh[g_slot[slot]];
    lv_label_set_text(g_gauge[slot].unit, m.unit);
}

void dd_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    uint16_t sel = lv_dropdown_get_selected((lv_obj_t*)lv_event_get_target(e));
    if (sel < kChCount) {
        g_slot[slot] = (uint8_t)sel;
        applyChannel(slot);
        persist();
    }
}

void makeGauge(lv_obj_t* parent, int slot) {
    lv_obj_t* panel = ui_make_panel(parent, nullptr);
    lv_obj_set_grid_cell(panel, LV_GRID_ALIGN_STRETCH, slot % 3, 1,
                         LV_GRID_ALIGN_STRETCH, slot / 3, 1);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    Gauge& g = g_gauge[slot];

    // Channel selector dropdown (top of the panel).
    g.dd = lv_dropdown_create(panel);
    lv_dropdown_set_options(g.dd, g_options);
    lv_dropdown_set_selected(g.dd, g_slot[slot]);
    lv_obj_set_width(g.dd, lv_pct(90));
    lv_obj_align(g.dd, LV_ALIGN_TOP_MID, 0, -4);
    lv_obj_set_style_text_font(g.dd, &lv_font_montserrat_16, 0);
    lv_obj_add_event_cb(g.dd, dd_cb, LV_EVENT_VALUE_CHANGED, (void*)(intptr_t)slot);

    g.arc = lv_arc_create(panel);
    lv_obj_set_size(g.arc, 180, 180);
    lv_obj_align(g.arc, LV_ALIGN_CENTER, 0, 18);
    lv_arc_set_rotation(g.arc, 135);
    lv_arc_set_bg_angles(g.arc, 0, 270);
    lv_arc_set_range(g.arc, 0, 1000);          // value mapped into 0..1000
    lv_arc_set_value(g.arc, 0);
    lv_obj_remove_flag(g.arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_style(g.arc, nullptr, LV_PART_KNOB);
    lv_obj_set_style_arc_color(g.arc, COL_GRID, LV_PART_MAIN);
    lv_obj_set_style_arc_width(g.arc, 12, LV_PART_MAIN);
    lv_obj_set_style_arc_color(g.arc, COL_CYAN, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(g.arc, 12, LV_PART_INDICATOR);

    g.value = lv_label_create(panel);
    lv_obj_set_style_text_font(g.value, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(g.value, COL_TEXT, 0);
    lv_label_set_text(g.value, "0");
    lv_obj_align(g.value, LV_ALIGN_CENTER, 0, 12);

    g.unit = lv_label_create(panel);
    lv_obj_add_style(g.unit, &st_label_dim, 0);
    lv_obj_align(g.unit, LV_ALIGN_CENTER, 0, 44);

    applyChannel(slot);
}

} // namespace

void screen_dash_create(lv_obj_t* parent) {
    // Persisted gauge assignments.
    if (nvs_open("dash", NVS_READWRITE, &g_nvs) == ESP_OK) {
        size_t sz = NUM_GAUGES;
        nvs_get_blob(g_nvs, "slots", g_slot, &sz);   // leaves defaults if absent
    }

    // Build the dropdown option string once.
    g_options[0] = '\0';
    for (int i = 0; i < kChCount; ++i) {
        strncat(g_options, kCh[i].name, sizeof(g_options) - strlen(g_options) - 2);
        if (i + 1 < kChCount) strncat(g_options, "\n", sizeof(g_options) - strlen(g_options) - 1);
    }

    lv_obj_set_style_pad_all(parent, 8, 0);
    static int32_t col_dsc[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
    static int32_t row_dsc[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
    lv_obj_set_grid_dsc_array(parent, col_dsc, row_dsc);
    lv_obj_set_layout(parent, LV_LAYOUT_GRID);

    for (int i = 0; i < NUM_GAUGES; ++i) makeGauge(parent, i);
}

void screen_dash_update(void) {
    TelemetryState t = EventBus::instance().snapshot();
    char buf[24];
    for (int i = 0; i < NUM_GAUGES; ++i) {
        const ChMeta& m = kCh[g_slot[i]];
        float v = channelValue(g_slot[i], t);
        // Map value into the 0..1000 arc range (clamped).
        float frac = (v - m.min) / (m.max - m.min);
        if (frac < 0) frac = 0;
        if (frac > 1) frac = 1;
        lv_arc_set_value(g_gauge[i].arc, (int)(frac * 1000));
        snprintf(buf, sizeof(buf), "%.*f", m.dec, v);
        lv_label_set_text(g_gauge[i].value, buf);
    }
}
