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
    R_RPM, R_SPEED, R_MAP, R_BOOST, R_THROTTLE, R_LOAD, R_IGN, R_COOLANT,
    R_IAT, R_AFR, R_BARO, R_BATT, R_COUNT
};

const char* kNames[R_COUNT] = {
    "Engine RPM", "Vehicle Speed", "MAP", "Boost", "Throttle", "Engine Load",
    "Ignition Adv", "Coolant Temp", "Intake Air Temp", "AFR",
    "Barometric", "Battery",
};

} // namespace

void screen_livedata_create(lv_obj_t* parent) {
    lv_obj_set_style_pad_all(parent, 12, 0);

    lv_obj_t* title = lv_label_create(parent);
    lv_obj_add_style(title, &st_title, 0);
    lv_label_set_text(title, "LIVE DATA");
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 4, 0);

    g_table = lv_table_create(parent);
    lv_obj_set_width(g_table, lv_pct(100));
    lv_obj_set_height(g_table, lv_pct(92));
    lv_obj_align(g_table, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_table_set_column_count(g_table, 2);
    lv_table_set_column_width(g_table, 0, 360);
    lv_table_set_column_width(g_table, 1, 300);
    lv_table_set_row_count(g_table, R_COUNT + 1);
    lv_table_set_cell_value(g_table, 0, 0, "PARAMETER");
    lv_table_set_cell_value(g_table, 0, 1, "VALUE");
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
    auto set = [&](int row, const char* fmt, float v) {
        snprintf(b, sizeof(b), fmt, v);
        lv_table_set_cell_value(g_table, row + 1, 1, b);
    };
    set(R_RPM,      "%.0f rpm",  t.rpm);
    set(R_SPEED,    "%.0f km/h", t.speed_kph);
    set(R_MAP,      "%.0f kPa",  t.map_kpa);
    set(R_BOOST,    "%.1f psi",  t.boost_psi);
    set(R_THROTTLE, "%.0f %%",   t.throttle_pct);
    set(R_LOAD,     "%.0f %%",   t.engine_load);
    set(R_IGN,      "%.1f deg",  t.ignition_adv_deg);
    set(R_COOLANT,  "%.0f C",    t.coolant_c);
    set(R_IAT,      "%.0f C",    t.intake_air_c);
    set(R_AFR,      "%.1f",      t.afr);
    set(R_BARO,     "%.0f kPa",  t.baro_kpa);
    set(R_BATT,     "%.2f V",    t.battery_v);

    // Append manufacturer/custom PIDs (loaded from SD or the Ford defaults).
    size_t cc = custpid::count();
    lv_table_set_row_count(g_table, R_COUNT + 1 + cc);
    for (size_t i = 0; i < cc; ++i) {
        const CustomPid& c = custpid::def(i);
        lv_table_set_cell_value(g_table, R_COUNT + 1 + i, 0, c.name);
        float v;
        if (custpid::getValue(i, v)) snprintf(b, sizeof(b), "%.2f %s", v, c.unit);
        else                          snprintf(b, sizeof(b), "--");
        lv_table_set_cell_value(g_table, R_COUNT + 1 + i, 1, b);
    }
}
