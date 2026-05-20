// =============================================================================
//  screen_dash.cpp - Screen 1: Live Telemetry Dash.
// -----------------------------------------------------------------------------
//  Two large radial gauges (RPM, Boost) built from lv_arc + centered readouts,
//  plus two real-time sparkline charts (Knock Retard, AFR). Gauges use the
//  neon palette; the knock chart shifts amber->red as retard climbs.
//
//  Built with lv_arc (not the removed lv_meter) so it compiles cleanly on
//  LVGL 9. The arc renders the value ring; a center label shows the number.
// =============================================================================
#include "ui/screens/screens.h"
#include "ui/ui_theme.h"
#include "core/event_bus.h"

#include <cstdio>

namespace {

// --- Gauge widget bundle ----------------------------------------------------
struct Gauge {
    lv_obj_t* arc   = nullptr;
    lv_obj_t* value = nullptr;   // big number
    lv_obj_t* unit  = nullptr;   // caption
};

Gauge      g_rpm, g_boost;
lv_obj_t*  g_knock_chart = nullptr;
lv_chart_series_t* g_knock_ser = nullptr;
lv_obj_t*  g_afr_chart   = nullptr;
lv_chart_series_t* g_afr_ser = nullptr;

constexpr int SPARK_POINTS = 120;   // ~4 s of history at 30 fps

// Build a radial gauge into `parent`. `range_max` scales the arc sweep.
Gauge make_gauge(lv_obj_t* parent, const char* unit_txt, lv_color_t accent,
                 int range_max) {
    Gauge g;
    g.arc = lv_arc_create(parent);
    lv_obj_set_size(g.arc, 260, 260);
    lv_obj_center(g.arc);
    lv_arc_set_rotation(g.arc, 135);
    lv_arc_set_bg_angles(g.arc, 0, 270);
    lv_arc_set_range(g.arc, 0, range_max);
    lv_arc_set_value(g.arc, 0);
    lv_obj_remove_flag(g.arc, LV_OBJ_FLAG_CLICKABLE);     // display-only
    lv_obj_remove_style(g.arc, nullptr, LV_PART_KNOB);     // hide drag knob

    // Track + indicator styling: dim carbon track, neon value arc.
    lv_obj_set_style_arc_color(g.arc, COL_GRID, LV_PART_MAIN);
    lv_obj_set_style_arc_width(g.arc, 16, LV_PART_MAIN);
    lv_obj_set_style_arc_color(g.arc, accent, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(g.arc, 16, LV_PART_INDICATOR);

    g.value = lv_label_create(parent);
    lv_obj_add_style(g.value, &st_value_big, 0);
    lv_label_set_text(g.value, "0");
    lv_obj_align(g.value, LV_ALIGN_CENTER, 0, -6);

    g.unit = lv_label_create(parent);
    lv_obj_add_style(g.unit, &st_label_dim, 0);
    lv_label_set_text(g.unit, unit_txt);
    lv_obj_align(g.unit, LV_ALIGN_CENTER, 0, 34);
    return g;
}

// Build a sparkline chart with one line series.
lv_chart_series_t* make_spark(lv_obj_t* parent, lv_color_t color,
                              lv_obj_t** out_chart, int y_min, int y_max) {
    lv_obj_t* chart = lv_chart_create(parent);
    lv_obj_set_size(chart, lv_pct(100), lv_pct(70));
    lv_obj_align(chart, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart, SPARK_POINTS);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, y_min, y_max);
    lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_SHIFT);
    lv_chart_set_div_line_count(chart, 3, 0);

    lv_obj_set_style_bg_color(chart, COL_BG, 0);
    lv_obj_set_style_border_width(chart, 0, 0);
    lv_obj_set_style_line_color(chart, COL_GRID, LV_PART_MAIN);   // gridlines
    lv_obj_set_style_size(chart, 0, 0, LV_PART_INDICATOR);        // no points

    lv_chart_series_t* ser =
        lv_chart_add_series(chart, color, LV_CHART_AXIS_PRIMARY_Y);
    *out_chart = chart;
    return ser;
}

} // namespace

void screen_dash_create(lv_obj_t* parent) {
    lv_obj_set_style_pad_all(parent, 10, 0);
    // 2x2 grid: gauges on top row, sparklines on bottom row.
    static int32_t col_dsc[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
    static int32_t row_dsc[] = { LV_GRID_FR(3), LV_GRID_FR(2), LV_GRID_TEMPLATE_LAST };
    lv_obj_set_grid_dsc_array(parent, col_dsc, row_dsc);
    lv_obj_set_layout(parent, LV_LAYOUT_GRID);

    lv_obj_t* p;

    p = ui_make_panel(parent, "ENGINE RPM");
    lv_obj_set_grid_cell(p, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    g_rpm = make_gauge(p, "RPM", COL_CYAN, 8000);

    p = ui_make_panel(parent, "BOOST");
    lv_obj_set_grid_cell(p, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    g_boost = make_gauge(p, "PSI", COL_MAGENTA, 35);

    p = ui_make_panel(parent, "KNOCK RETARD");
    lv_obj_set_grid_cell(p, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 1, 1);
    g_knock_ser = make_spark(p, COL_RED, &g_knock_chart, 0, 15);

    p = ui_make_panel(parent, "AFR (LAMBDA)");
    lv_obj_set_grid_cell(p, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_STRETCH, 1, 1);
    g_afr_ser = make_spark(p, COL_GREEN, &g_afr_chart, 100, 200);  // AFR*10
}

void screen_dash_update(void) {
    TelemetryState t = EventBus::instance().snapshot();
    char buf[16];

    // RPM gauge.
    lv_arc_set_value(g_rpm.arc, (int)t.rpm);
    snprintf(buf, sizeof(buf), "%.0f", t.rpm);
    lv_label_set_text(g_rpm.value, buf);

    // Boost gauge (clamp negative vacuum to 0 on the dial).
    int boost = t.boost_psi > 0 ? (int)t.boost_psi : 0;
    lv_arc_set_value(g_boost.arc, boost);
    snprintf(buf, sizeof(buf), "%.1f", t.boost_psi);
    lv_label_set_text(g_boost.value, buf);

    // Knock sparkline; recolor the line as retard worsens.
    lv_chart_set_next_value(g_knock_chart, g_knock_ser, (int)t.knock_retard_deg);
    lv_color_t kc = t.knock_retard_deg >= 6 ? COL_RED
                  : t.knock_retard_deg >= 2 ? COL_AMBER : COL_GREEN;
    lv_chart_set_series_color(g_knock_chart, g_knock_ser, kc);

    // AFR sparkline (stored as AFR*10 for integer y-axis).
    lv_chart_set_next_value(g_afr_chart, g_afr_ser, (int)(t.afr * 10));
}
