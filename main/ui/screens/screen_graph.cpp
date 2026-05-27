// =============================================================================
//  screen_graph.cpp - scrolling live trend graph for a selectable parameter.
// -----------------------------------------------------------------------------
//  A single lv_chart line that scrolls left as new samples arrive, with a
//  dropdown to pick which channel to plot and a large current-value readout.
//  Useful for watching boost/AFR/knock build during a pull, where the gauges
//  only show the instantaneous value.
// =============================================================================
#include "ui/screens/screens.h"
#include "ui/ui_theme.h"
#include "core/event_bus.h"
#include "obd/custom_pids.h"

#include <cstdio>
#include <cmath>
#include <cctype>

namespace {

// Channels plottable here. Values are scaled x10 into the chart's integer axis
// so fractional channels (psi/AFR) keep one decimal of resolution.
struct GMeta { const char* name; float min; float max; int dec; const char* unit; };
const GMeta kG[] = {
    { "RPM",          0, 8000, 0, "rpm" },
    { "Boost",      -15,   30, 1, "psi" },
    { "Coolant",      0,  260, 0, "F"   },
    { "IAT",          0,  250, 0, "F"   },
    { "AFR",          8,   20, 1, ""    },
    { "Throttle",     0,  100, 0, "%"   },
    { "Speed",        0,  160, 0, "mph" },
    { "MAF",          0,  300, 0, "g/s" },
    { "Knock",       -5,   15, 1, "deg" },
    { "Timing",     -10,   50, 1, "deg" },
};
constexpr int kGCount = sizeof(kG) / sizeof(kG[0]);
constexpr int kPoints = 120;
constexpr float kScale = 10.0f;

lv_obj_t*           g_chart = nullptr;
lv_chart_series_t*  g_ser   = nullptr;
lv_obj_t*           g_value = nullptr;
lv_obj_t*           g_dd    = nullptr;
int                 g_ch    = 1;   // default: Boost

bool customByName(const char* sub, float& out) {
    for (size_t i = 0; i < custpid::count(); ++i) {
        const char* nm = custpid::def(i).name;
        for (const char* s = nm; *s; ++s) {
            const char* a = s; const char* b = sub;
            while (*a && *b && tolower((unsigned char)*a) == tolower((unsigned char)*b)) { ++a; ++b; }
            if (!*b) return custpid::getValue(i, out);
        }
    }
    return false;
}

float gValue(int ch, const TelemetryState& t) {
    float v;
    switch (ch) {
        case 0: return t.rpm;
        case 1: if (customByName("boost", v)) return v; return t.boost_psi;
        case 2: return t.coolant_c * 1.8f + 32.0f;
        case 3: return t.intake_air_c * 1.8f + 32.0f;
        case 4: return t.afr;
        case 5: return t.throttle_pct;
        case 6: return t.speed_kph * 0.621371f;
        case 7: return t.maf_gps;
        case 8: if (customByName("knock", v)) return v; return 0;  // knock: custom PID only
        case 9: return t.ignition_adv_deg;
        default: return 0;
    }
}

void applyChannel() {
    const GMeta& m = kG[g_ch];
    lv_chart_set_range(g_chart, LV_CHART_AXIS_PRIMARY_Y,
                       (int32_t)(m.min * kScale), (int32_t)(m.max * kScale));
    lv_chart_set_all_value(g_chart, g_ser, LV_CHART_POINT_NONE);  // clear history
}

void dd_cb(lv_event_t* e) {
    int sel = lv_dropdown_get_selected((lv_obj_t*)lv_event_get_target(e));
    if (sel >= 0 && sel < kGCount) { g_ch = sel; applyChannel(); }
}

} // namespace

void screen_graph_create(lv_obj_t* parent) {
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 12, 0);
    lv_obj_set_style_pad_row(parent, 8, 0);

    // Top row: channel dropdown + current value.
    lv_obj_t* bar = lv_obj_create(parent);
    lv_obj_add_style(bar, &st_screen, 0);
    lv_obj_set_width(bar, lv_pct(100));
    lv_obj_set_height(bar, 60);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    g_dd = lv_dropdown_create(bar);
    char opts[160];
    opts[0] = '\0';
    for (int i = 0; i < kGCount; ++i) {
        strncat(opts, kG[i].name, sizeof(opts) - strlen(opts) - 2);
        if (i + 1 < kGCount) strncat(opts, "\n", sizeof(opts) - strlen(opts) - 1);
    }
    lv_dropdown_set_options(g_dd, opts);
    lv_dropdown_set_selected(g_dd, g_ch);
    lv_obj_set_width(g_dd, 240);
    lv_obj_add_event_cb(g_dd, dd_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    g_value = lv_label_create(bar);
    lv_obj_set_style_text_font(g_value, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(g_value, COL_CYAN, 0);
    lv_label_set_text(g_value, "--");

    // The chart fills the rest.
    g_chart = lv_chart_create(parent);
    lv_obj_set_width(g_chart, lv_pct(100));
    lv_obj_set_flex_grow(g_chart, 1);
    lv_chart_set_type(g_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(g_chart, kPoints);
    lv_chart_set_update_mode(g_chart, LV_CHART_UPDATE_MODE_SHIFT);
    lv_chart_set_div_line_count(g_chart, 6, 0);
    lv_obj_set_style_bg_color(g_chart, COL_PANEL, 0);
    lv_obj_set_style_border_color(g_chart, COL_GRID, 0);
    lv_obj_set_style_line_color(g_chart, COL_GRID, LV_PART_MAIN);
    lv_obj_set_style_size(g_chart, 0, 0, LV_PART_INDICATOR);   // no point markers
    g_ser = lv_chart_add_series(g_chart, COL_CYAN, LV_CHART_AXIS_PRIMARY_Y);

    applyChannel();
}

void screen_graph_update(void) {
    if (!g_chart) return;
    TelemetryState t = EventBus::instance().snapshot();
    const GMeta& m = kG[g_ch];
    float v = gValue(g_ch, t);
    lv_chart_set_next_value(g_chart, g_ser, (int32_t)lroundf(v * kScale));

    char buf[24];
    snprintf(buf, sizeof(buf), "%.*f %s", m.dec, v, m.unit);
    lv_label_set_text(g_value, buf);
}
