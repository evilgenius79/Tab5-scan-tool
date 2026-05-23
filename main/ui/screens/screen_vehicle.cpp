// =============================================================================
//  screen_vehicle.cpp - Vehicle Information (OBD Mode 09).
// -----------------------------------------------------------------------------
//  A proper "Vehicle Info" page: VIN with decoded manufacturer + model year,
//  calibration ID(s) (PID 04) and ECU name (PID 0A). READ triggers the OBD
//  task's Mode 09 sequence; results land in EventBus VehicleInfo.
// =============================================================================
#include "ui/screens/screens.h"
#include "ui/ui_theme.h"
#include "core/event_bus.h"

#include <cstdio>
#include <cstring>

namespace {

struct Field { lv_obj_t* caption; lv_obj_t* value; };
Field     g_vin, g_make, g_year, g_cal, g_ecu;
lv_obj_t* g_status = nullptr;
bool      g_shown  = false;

// A labeled row: dim caption above a bright value, inside a carbon panel.
Field make_field(lv_obj_t* parent, const char* caption) {
    lv_obj_t* panel = ui_make_panel(parent, nullptr);
    lv_obj_set_width(panel, lv_pct(100));
    lv_obj_set_height(panel, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(panel, 10, 0);

    Field f;
    f.caption = lv_label_create(panel);
    lv_obj_add_style(f.caption, &st_label_dim, 0);
    lv_label_set_text(f.caption, caption);

    f.value = lv_label_create(panel);
    lv_obj_set_style_text_color(f.value, COL_TEXT, 0);
    lv_obj_set_style_text_font(f.value, &lv_font_montserrat_20, 0);
    lv_label_set_long_mode(f.value, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(f.value, lv_pct(100));
    lv_label_set_text(f.value, "--");
    return f;
}

void read_cb(lv_event_t*) {
    ObdCommand c{ CmdType::ReadVin, 0, 0 };
    EventBus::instance().sendCommand(c, 0);
    lv_label_set_text(g_status, "Reading vehicle info...");
    g_shown = false;
}

} // namespace

void screen_vehicle_create(lv_obj_t* parent) {
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 12, 0);
    lv_obj_set_style_pad_row(parent, 8, 0);

    lv_obj_t* bar = lv_obj_create(parent);
    lv_obj_add_style(bar, &st_screen, 0);
    lv_obj_set_width(bar, lv_pct(100));
    lv_obj_set_height(bar, 60);
    lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(bar, 14, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* rb = lv_btn_create(bar);
    lv_obj_add_style(rb, &st_accent_btn, 0);
    lv_obj_add_event_cb(rb, read_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* rl = lv_label_create(rb);
    lv_label_set_text(rl, LV_SYMBOL_DOWNLOAD " READ VEHICLE INFO");
    lv_obj_center(rl);

    g_status = lv_label_create(bar);
    lv_obj_add_style(g_status, &st_label_dim, 0);
    lv_label_set_text(g_status, "Idle");

    g_vin  = make_field(parent, "VIN");
    g_make = make_field(parent, "MANUFACTURER");
    g_year = make_field(parent, "MODEL YEAR");
    g_cal  = make_field(parent, "CALIBRATION ID");
    g_ecu  = make_field(parent, "ECU NAME");
}

void screen_vehicle_update(void) {
    VehicleInfo v = EventBus::instance().getVehicleInfo();
    if (g_shown || !v.valid) return;
    g_shown = true;

    char buf[24];
    lv_label_set_text(g_vin.value,  v.vin[0] ? v.vin : "--");
    lv_label_set_text(g_make.value, v.manufacturer[0] ? v.manufacturer : "Unknown");
    if (v.model_year > 0) { snprintf(buf, sizeof(buf), "%d", v.model_year);
                            lv_label_set_text(g_year.value, buf); }
    else                  { lv_label_set_text(g_year.value, "Unknown"); }
    lv_label_set_text(g_cal.value, v.cal_id[0]   ? v.cal_id   : "--");
    lv_label_set_text(g_ecu.value, v.ecu_name[0] ? v.ecu_name : "--");
    lv_label_set_text(g_status, LV_SYMBOL_OK " Read OK");
}
