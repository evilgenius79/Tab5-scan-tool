// =============================================================================
//  screen_livedata.cpp - Live Data list (scan-tool "data stream" view).
// -----------------------------------------------------------------------------
//  A scrollable table of every parameter the tool decodes, updated live from
//  the shared TelemetryState. This is the classic generic-OBD "Live Data"
//  screen - all values in one place, numeric, fast-refreshing.
// =============================================================================
#include "ui/screens/screens.h"
#include "ui/ui_theme.h"
#include "core/event_bus.h"
#include "obd/custom_pids.h"

#include <cstdio>

namespace {

lv_obj_t* g_table = nullptr;

// One row per parameter. The order here is the display order.
enum Row {
    R_RPM, R_SPEED, R_MAP, R_BOOST, R_THROTTLE, R_LOAD, R_MAF, R_IGN, R_COOLANT,
    R_IAT, R_AFR, R_BARO, R_BATT, R_MPG, R_TRIPMPG, R_TRIPMI, R_COUNT
};

const char* kNames[R_COUNT] = {
    "Engine RPM", "Vehicle Speed", "MAP", "Boost", "Throttle", "Engine Load",
    "Mass Air Flow", "Ignition Adv", "Coolant Temp", "Intake Air Temp", "AFR",
    "Barometric", "Battery", "MPG (now)", "Trip MPG", "Trip Distance",
};

// Session min/max recording (persists while the app runs; paused only while
// this screen is off-view). Covers the standard rows plus the custom PIDs.
constexpr int MAXR = R_COUNT + 16;
float g_min[MAXR];
float g_max[MAXR];
bool  g_seen[MAXR];

void track(int row, float v) {
    if (row < 0 || row >= MAXR) return;
    if (!g_seen[row] || v < g_min[row]) g_min[row] = v;
    if (!g_seen[row] || v > g_max[row]) g_max[row] = v;
    g_seen[row] = true;
}

} // namespace

void screen_livedata_create(lv_obj_t* parent) {
    lv_obj_set_style_pad_all(parent, 12, 0);

    lv_obj_t* title = lv_label_create(parent);
    lv_obj_add_style(title, &st_title, 0);
    lv_label_set_text(title, "LIVE DATA");
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 4, 0);

    for (int i = 0; i < MAXR; ++i) g_seen[i] = false;   // reset min/max record

    g_table = lv_table_create(parent);
    lv_obj_set_width(g_table, lv_pct(100));
    lv_obj_set_height(g_table, lv_pct(92));
    lv_obj_align(g_table, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_table_set_column_count(g_table, 4);
    lv_table_set_column_width(g_table, 0, 330);
    lv_table_set_column_width(g_table, 1, 240);
    lv_table_set_column_width(g_table, 2, 175);
    lv_table_set_column_width(g_table, 3, 175);
    lv_table_set_row_count(g_table, R_COUNT + 1);
    lv_table_set_cell_value(g_table, 0, 0, "PARAMETER");
    lv_table_set_cell_value(g_table, 0, 1, "VALUE");
    lv_table_set_cell_value(g_table, 0, 2, "MIN");
    lv_table_set_cell_value(g_table, 0, 3, "MAX");
    for (int i = 0; i < R_COUNT; ++i)
        lv_table_set_cell_value(g_table, i + 1, 0, kNames[i]);

    lv_obj_set_style_text_font(g_table, &lv_font_montserrat_18, LV_PART_ITEMS);
    lv_obj_set_style_bg_color(g_table, COL_PANEL, LV_PART_ITEMS);
    lv_obj_set_style_text_color(g_table, COL_TEXT, LV_PART_ITEMS);
    lv_obj_set_style_border_color(g_table, COL_GRID, LV_PART_ITEMS);
    lv_obj_set_style_pad_ver(g_table, 6, LV_PART_ITEMS);
}

void screen_livedata_update(void) {
    TelemetryState t = EventBus::instance().snapshot();
    char b[24];
    // Write the live value, record min/max, and render the MIN/MAX columns.
    // `prec` controls the min/max number format so e.g. RPM isn't shown as
    // "1253.0". Trip-total rows (distance) skip min/max (monotonic).
    auto set = [&](int row, const char* fmt, float v, int prec, bool minmax) {
        snprintf(b, sizeof(b), fmt, v);
        lv_table_set_cell_value(g_table, row + 1, 1, b);
        if (!minmax) return;
        track(row, v);
        snprintf(b, sizeof(b), "%.*f", prec, g_min[row]);
        lv_table_set_cell_value(g_table, row + 1, 2, b);
        snprintf(b, sizeof(b), "%.*f", prec, g_max[row]);
        lv_table_set_cell_value(g_table, row + 1, 3, b);
    };
    set(R_RPM,      "%.0f rpm",  t.rpm,                      0, true);
    set(R_SPEED,    "%.0f mph",  t.speed_kph * 0.621371f,    0, true);
    set(R_MAP,      "%.1f psi",  t.map_kpa * 0.1450377f,     1, true);
    set(R_BOOST,    "%.1f psi",  t.boost_psi,                1, true);
    set(R_THROTTLE, "%.0f %%",   t.throttle_pct,             0, true);
    set(R_LOAD,     "%.0f %%",   t.engine_load,              0, true);
    set(R_MAF,      "%.1f g/s",  t.maf_gps,                  1, true);
    set(R_IGN,      "%.1f deg",  t.ignition_adv_deg,         1, true);
    set(R_COOLANT,  "%.0f F",    t.coolant_c * 1.8f + 32.0f, 0, true);
    set(R_IAT,      "%.0f F",    t.intake_air_c * 1.8f + 32, 0, true);
    set(R_AFR,      "%.1f",      t.afr,                      1, true);
    set(R_BARO,     "%.1f psi",  t.baro_kpa * 0.1450377f,    1, true);
    set(R_BATT,     "%.2f V",    t.battery_v,                2, true);
    set(R_MPG,      "%.1f mpg",  t.mpg_instant,              1, true);
    set(R_TRIPMPG,  "%.1f mpg",  t.trip_mpg,                 1, false);
    set(R_TRIPMI,   "%.2f mi",   t.trip_distance_mi,         2, false);

    // Append manufacturer/custom PIDs (loaded from SD or the Ford defaults).
    size_t cc = custpid::count();
    lv_table_set_row_count(g_table, R_COUNT + 1 + cc);
    for (size_t i = 0; i < cc; ++i) {
        const int row = R_COUNT + (int)i;
        const CustomPid& c = custpid::def(i);
        lv_table_set_cell_value(g_table, row + 1, 0, c.name);
        float v;
        if (custpid::getValue(i, v)) {
            snprintf(b, sizeof(b), "%.2f %s", v, c.unit);
            lv_table_set_cell_value(g_table, row + 1, 1, b);
            track(row, v);
            snprintf(b, sizeof(b), "%.2f", g_min[row]);
            lv_table_set_cell_value(g_table, row + 1, 2, b);
            snprintf(b, sizeof(b), "%.2f", g_max[row]);
            lv_table_set_cell_value(g_table, row + 1, 3, b);
        } else {
            lv_table_set_cell_value(g_table, row + 1, 1, "--");
        }
    }
}
