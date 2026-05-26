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
#include "obd/custom_pids.h"   // enhanced Turbo Boost PID for the Boost gauge
#include "app_config.h"        // MAP_SENSOR_BAR / SEA_LEVEL_KPA / KPA_TO_PSI

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cctype>
#include "nvs.h"

namespace {

// --- Available gauge channels (parameter metadata) ----------------------------
//  warn/danger are thresholds for the colour zones; low_bad=true means LOW values
//  are the danger (e.g. battery). A huge threshold disables the zone (neutral).
struct ChMeta {
    const char* name; const char* unit; float min; float max; int dec;
    float warn; float danger; bool low_bad;
};
constexpr float NONE = 1e9f;   // disables a colour zone

// Boost/MAP gauge scaling derived from the configured MAP-sensor full scale
// (app_config.h). A 3-bar sensor -> 300 kPa absolute -> ~43.5 psi MAP, with a
// ~28.8 psi gauge-boost ceiling once atmospheric is subtracted. Overboost
// warn/danger zones scale as a fraction of that ceiling so swapping the sensor
// size reflows the gauge automatically.
constexpr float kMapFullKpa  = MAP_SENSOR_BAR * 100.0f;
constexpr float kMapFullPsi  = kMapFullKpa * KPA_TO_PSI;
constexpr float kBoostMaxPsi = (kMapFullKpa - SEA_LEVEL_KPA) * KPA_TO_PSI;

// USA/imperial units throughout (mph, degF, psi).
const ChMeta kCh[] = {
    { "RPM",        "rpm",  0,  8000, 0,  6000,  6600, false },  // 0 redline
    { "Boost",      "psi",-15,  kBoostMaxPsi, 1,                  // 1 overboost
                    kBoostMaxPsi * 0.80f, kBoostMaxPsi * 0.93f, false },
    { "AFR",        "",     8,  20,   1,  NONE,  NONE, false },  // 2 (context-dep)
    { "Ign Timing", "deg",-10,  50,   1,  NONE,  NONE, false },  // 3
    { "Coolant",    "F",    0,  260,  0,  220,   240,  false },  // 4 hot
    { "IAT1 (Intake)","F",  0,  250,  0,  150,   180,  false },  // 5 heat soak
    { "Speed",      "mph",  0,  140,  0,  NONE,  NONE, false },  // 6
    { "MAP",        "psi",  0,  kMapFullPsi, 1, NONE, NONE, false }, // 7 (3-bar)
    { "Throttle",   "%",    0,  100,  0,  NONE,  NONE, false },  // 8
    { "Eng Load",   "%",    0,  100,  0,  NONE,  NONE, false },  // 9
    { "Battery",    "V",    8,  16,   1,  12.0f, 11.5f,true  },  // 10 low = bad
    { "Barometric", "psi", 10,  16,   1,  NONE,  NONE, false },  // 11
    { "Mass Airflow","g/s", 0,  300,  0,  NONE,  NONE, false },  // 12
    { "MPG",        "mpg",  0,  60,   1,  NONE,  NONE, false },  // 13 instant
    { "Short Fuel Tr","%", -25, 25,   1,  NONE,  NONE, false },  // 14 STFT
    { "Long Fuel Tr", "%", -25, 25,   1,  NONE,  NONE, false },  // 15 LTFT
    { "Fuel Level",  "%",   0,  100,  0,  15,    5,    true  },  // 16 low = bad
    { "Oil Temp",    "F",   0,  300,  0,  250,   270,  false },  // 17 hot
    { "Ambient",     "F",  -20, 120,  0,  NONE,  NONE, false },  // 18
    { "IAT2 (Charge)","F",  0,  300,  0,  160,   185,  false },  // 19 charge-air temp
};
constexpr int kChCount = sizeof(kCh) / sizeof(kCh[0]);

// Colour for a value given its channel's warn/danger zones.
lv_color_t zoneColor(const ChMeta& m, float v) {
    if (m.warn == NONE) return COL_CYAN;
    if (m.low_bad) {
        if (v <= m.danger) return COL_RED;
        if (v <= m.warn)   return COL_AMBER;
    } else {
        if (v >= m.danger) return COL_RED;
        if (v >= m.warn)   return COL_AMBER;
    }
    return COL_CYAN;
}

inline float c_to_f(float c)    { return c * 1.8f + 32.0f; }
inline float kpa_to_psi(float k){ return k * 0.1450377f; }

// Case-insensitive "does `s` contain `sub`" (tiny, avoids strcasestr).
bool containsCI(const char* s, const char* sub) {
    for (; *s; ++s) {
        const char* a = s; const char* b = sub;
        while (*a && *b && (tolower((unsigned char)*a) == tolower((unsigned char)*b))) { ++a; ++b; }
        if (!*b) return true;
    }
    return false;
}

// Find the custom/profile PID that represents turbo boost (matched by name),
// so the Boost gauge can prefer the enhanced 2-byte reading over the 1-byte,
// 255 kPa-capped MAP derivation. Re-searched until the custom PIDs finish
// loading (custpid::load runs once the adapter is online); -1 = none defined.
int boostCustomIdx() {
    static int idx = -1;
    if (idx < 0 && custpid::count() > 0) {
        for (size_t i = 0; i < custpid::count(); ++i) {
            if (containsCI(custpid::def(i).name, "boost")) { idx = (int)i; break; }
        }
    }
    return idx;
}

// IAT2 = post-intercooler charge-air temp. Standard OBD exposes only IAT1
// (PID 0x0F); the second sensor is a manufacturer PID, so match the charge-air
// custom/profile PID by name (e.g. Ford "Charge Air Temp"). -1 = none defined.
int chargeAirCustomIdx() {
    static int idx = -1;
    if (idx < 0 && custpid::count() > 0) {
        for (size_t i = 0; i < custpid::count(); ++i) {
            const char* nm = custpid::def(i).name;
            if (containsCI(nm, "charge") || containsCI(nm, "cact") ||
                containsCI(nm, "iat2")) { idx = (int)i; break; }
        }
    }
    return idx;
}

float channelValue(int ch, const TelemetryState& t) {
    switch (ch) {
        case 0:  return t.rpm;
        case 1: {                                     // Boost (psi)
            // Prefer the enhanced Turbo Boost PID; it isn't capped at the
            // single-byte MAP ceiling (~22 psi). Fall back to MAP-derived boost
            // when no boost PID is defined or it hasn't been polled yet.
            int bi = boostCustomIdx();
            float bv;
            if (bi >= 0 && custpid::getValue(bi, bv)) return bv;
            return t.boost_psi;
        }
        case 2:  return t.afr;
        case 3:  return t.ignition_adv_deg;
        case 4:  return c_to_f(t.coolant_c);
        case 5:  return c_to_f(t.intake_air_c);
        case 6:  return t.speed_kph * 0.621371f;      // mph
        case 7:  return kpa_to_psi(t.map_kpa);
        case 8:  return t.throttle_pct;
        case 9:  return t.engine_load;
        case 10: return t.battery_v;
        case 11: return kpa_to_psi(t.baro_kpa);
        case 12: return t.maf_gps;
        case 13: return t.mpg_instant;
        case 14: return t.short_fuel_trim;
        case 15: return t.long_fuel_trim;
        case 16: return t.fuel_level_pct;
        case 17: return c_to_f(t.oil_temp_c);
        case 18: return c_to_f(t.ambient_c);
        case 19: {                                    // IAT2 / charge-air temp
            // The Ford charge-air custom PID already outputs degF; if none is
            // defined, fall back to the (standard-decode) charge_air_c field.
            int ci = chargeAirCustomIdx();
            float cv;
            if (ci >= 0 && custpid::getValue(ci, cv)) return cv;
            return c_to_f(t.charge_air_c);
        }
        default: return 0;
    }
}

constexpr int NUM_GAUGES = 6;
uint8_t  g_slot[NUM_GAUGES] = { 0, 1, 2, 3, 4, 10 };  // default channel per slot

struct Gauge { lv_obj_t* panel; lv_obj_t* arc; lv_obj_t* value; lv_obj_t* unit;
               lv_obj_t* dd; lv_obj_t* spark; lv_chart_series_t* sser; };
Gauge      g_gauge[NUM_GAUGES];
nvs_handle_t g_nvs = 0;
char       g_options[256];   // newline-joined channel names for the dropdowns

lv_obj_t*  g_parent     = nullptr;   // the tab page (grid container)
lv_obj_t*  g_layout_btn = nullptr;   // floating layout-cycle button
lv_obj_t*  g_layout_lbl = nullptr;

// Selectable gauge layouts: 6 (3x2), 4 (2x2), or 2 (1x2 stacked). Fewer gauges
// => larger arcs. cols drives the grid; rows is always 2.
struct LayoutCfg { uint8_t visible; uint8_t cols; int16_t arc; const char* label; };
const LayoutCfg kLayouts[] = {
    { 6, 3, 160, "6 GAUGES" },
    { 4, 2, 210, "4 GAUGES" },
    { 2, 1, 250, "2 GAUGES" },
};
constexpr int kLayoutCount = sizeof(kLayouts) / sizeof(kLayouts[0]);
uint8_t g_layout = 0;

// Grid column templates (rows are always two equal tracks).
int32_t g_cols3[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
int32_t g_cols2[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
int32_t g_cols1[] = { LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
int32_t g_rows2[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };

void persist() {
    if (g_nvs) {
        nvs_set_blob(g_nvs, "slots", g_slot, NUM_GAUGES);
        nvs_set_u8(g_nvs, "layout", g_layout);
        nvs_commit(g_nvs);
    }
}

// Apply a channel to a slot: show the unit plus the gauge's scale range so the
// reading always has a reference (e.g. "psi  -15..35").
// Sparkline values are scaled x10 into the chart's integer axis to keep one
// decimal of resolution for fractional channels (psi/AFR).
constexpr float kSparkScale = 10.0f;

void applyChannel(int slot) {
    const ChMeta& m = kCh[g_slot[slot]];
    char u[36];
    snprintf(u, sizeof(u), "%s  %g..%g", m.unit, m.min, m.max);
    lv_label_set_text(g_gauge[slot].unit, u);

    // Re-scale and clear the sparkline history for the new channel.
    if (g_gauge[slot].spark) {
        lv_chart_set_range(g_gauge[slot].spark, LV_CHART_AXIS_PRIMARY_Y,
                           (int32_t)(m.min * kSparkScale),
                           (int32_t)(m.max * kSparkScale));
        lv_chart_set_all_value(g_gauge[slot].spark, g_gauge[slot].sser,
                               LV_CHART_POINT_NONE);
    }
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
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    Gauge& g = g_gauge[slot];
    g.panel = panel;

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

    // Trend sparkline pinned to the bottom of the panel: a scrolling history of
    // this gauge's value so you can see where it's been, not just "now".
    g.spark = lv_chart_create(panel);
    lv_obj_set_size(g.spark, lv_pct(88), 40);
    lv_obj_align(g.spark, LV_ALIGN_BOTTOM_MID, 0, -6);
    lv_chart_set_type(g.spark, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(g.spark, 48);
    lv_chart_set_update_mode(g.spark, LV_CHART_UPDATE_MODE_SHIFT);
    lv_chart_set_div_line_count(g.spark, 0, 0);
    lv_obj_set_style_bg_opa(g.spark, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g.spark, 0, 0);
    lv_obj_set_style_size(g.spark, 0, 0, LV_PART_INDICATOR);   // no point markers
    lv_obj_set_style_line_width(g.spark, 2, LV_PART_ITEMS);
    lv_obj_remove_flag(g.spark, LV_OBJ_FLAG_CLICKABLE);
    g.sser = lv_chart_add_series(g.spark, COL_CYAN, LV_CHART_AXIS_PRIMARY_Y);

    applyChannel(slot);
}

// Reflow the grid for the current layout: re-template the columns, place the
// visible gauges, hide the rest, and resize the arcs.
void applyLayout() {
    const LayoutCfg& L = kLayouts[g_layout];
    int32_t* cols = (L.cols == 3) ? g_cols3 : (L.cols == 2) ? g_cols2 : g_cols1;
    lv_obj_set_grid_dsc_array(g_parent, cols, g_rows2);

    for (int i = 0; i < NUM_GAUGES; ++i) {
        if (i < L.visible) {
            lv_obj_clear_flag(g_gauge[i].panel, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_grid_cell(g_gauge[i].panel,
                                 LV_GRID_ALIGN_STRETCH, i % L.cols, 1,
                                 LV_GRID_ALIGN_STRETCH, i / L.cols, 1);
            lv_obj_set_size(g_gauge[i].arc, L.arc, L.arc);
        } else {
            lv_obj_add_flag(g_gauge[i].panel, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (g_layout_lbl) lv_label_set_text(g_layout_lbl, L.label);
}

void layout_cb(lv_event_t*) {
    g_layout = (g_layout + 1) % kLayoutCount;
    applyLayout();
    persist();
}

} // namespace

void screen_dash_create(lv_obj_t* parent) {
    // Persisted gauge assignments + layout choice.
    if (nvs_open("dash", NVS_READWRITE, &g_nvs) == ESP_OK) {
        size_t sz = NUM_GAUGES;
        nvs_get_blob(g_nvs, "slots", g_slot, &sz);   // leaves defaults if absent
        nvs_get_u8(g_nvs, "layout", &g_layout);      // leaves default (0) if absent
    }
    // Clamp persisted channel indices: a stale blob (e.g. from a build with more
    // channels) must not index kCh out of range in applyChannel().
    for (int i = 0; i < NUM_GAUGES; ++i)
        if (g_slot[i] >= kChCount) g_slot[i] = 0;
    if (g_layout >= kLayoutCount) g_layout = 0;

    // Build the dropdown option string once.
    g_options[0] = '\0';
    for (int i = 0; i < kChCount; ++i) {
        strncat(g_options, kCh[i].name, sizeof(g_options) - strlen(g_options) - 2);
        if (i + 1 < kChCount) strncat(g_options, "\n", sizeof(g_options) - strlen(g_options) - 1);
    }

    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 6, 0);
    lv_obj_set_style_pad_row(parent, 6, 0);

    // Dash menu bar across the top: the gauge-layout cycle button lives here so
    // it never overlaps a gauge's channel dropdown.
    lv_obj_t* bar = lv_obj_create(parent);
    lv_obj_add_style(bar, &st_screen, 0);
    lv_obj_set_width(bar, lv_pct(100));
    lv_obj_set_height(bar, 48);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    g_layout_btn = lv_btn_create(bar);
    lv_obj_add_style(g_layout_btn, &st_accent_btn, 0);
    lv_obj_add_event_cb(g_layout_btn, layout_cb, LV_EVENT_CLICKED, nullptr);
    g_layout_lbl = lv_label_create(g_layout_btn);
    lv_obj_center(g_layout_lbl);

    // Gauge grid fills the rest of the page.
    lv_obj_t* grid = lv_obj_create(parent);
    lv_obj_add_style(grid, &st_screen, 0);
    lv_obj_set_width(grid, lv_pct(100));
    lv_obj_set_flex_grow(grid, 1);
    lv_obj_set_style_pad_all(grid, 0, 0);
    lv_obj_clear_flag(grid, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_layout(grid, LV_LAYOUT_GRID);
    g_parent = grid;

    for (int i = 0; i < NUM_GAUGES; ++i) makeGauge(grid, i);

    applyLayout();   // place gauges + set the button label for the saved layout
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

        // Colour the arc + readout by warning/danger zone.
        lv_color_t col = zoneColor(m, v);
        lv_obj_set_style_arc_color(g_gauge[i].arc, col, LV_PART_INDICATOR);
        lv_obj_set_style_text_color(g_gauge[i].value, col, 0);

        // Push the value onto the trend sparkline (same zone colour).
        lv_chart_set_next_value(g_gauge[i].spark, g_gauge[i].sser,
                                (int32_t)(v * kSparkScale));
        lv_chart_set_series_color(g_gauge[i].spark, g_gauge[i].sser, col);
    }
}
