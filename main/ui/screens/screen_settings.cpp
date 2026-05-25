// =============================================================================
//  screen_settings.cpp - Screen 6: Settings.
// -----------------------------------------------------------------------------
//  Adapter UART baud (STBR), screen brightness (BSP backlight PWM), and the
//  HS-CAN / MS-CAN network selector. Changes are pushed to the OBD task via
//  commands; brightness is applied directly through the BSP. A live link
//  status row and a force-reconnect button round it out.
// =============================================================================
#include "ui/screens/screens.h"
#include "ui/ui_theme.h"
#include "core/event_bus.h"
#include "obd/dtc_lookup.h"
#include "obd/custom_pids.h"

#include "bsp/esp-bsp.h"
#include "nvs.h"
#include <cstdio>

namespace {

lv_obj_t* g_baud_dd   = nullptr;
lv_obj_t* g_bus_sw    = nullptr;
lv_obj_t* g_bright_sl = nullptr;
lv_obj_t* g_link_lbl  = nullptr;
lv_obj_t* g_data_lbl  = nullptr;
nvs_handle_t g_nvs    = 0;
constexpr int DEFAULT_BRIGHTNESS = 80;

// Baud options must track this list when decoding the dropdown selection.
const uint32_t kBauds[] = { 115200, 230400, 500000, 1000000, 2000000 };

void baud_cb(lv_event_t* e) {
    uint16_t sel = lv_dropdown_get_selected((lv_obj_t*)lv_event_get_target(e));
    if (sel < sizeof(kBauds) / sizeof(kBauds[0])) {
        ObdCommand c{ CmdType::SetBaud, 0, kBauds[sel] };
        EventBus::instance().sendCommand(c, 0);
    }
}

void bus_cb(lv_event_t* e) {
    bool ms = lv_obj_has_state((lv_obj_t*)lv_event_get_target(e), LV_STATE_CHECKED);
    ObdCommand c{ CmdType::SelectBus, (uint8_t)(ms ? CanBus::MS_CAN
                                                   : CanBus::HS_CAN), 0 };
    EventBus::instance().sendCommand(c, 0);
}

void bright_cb(lv_event_t* e) {
    int v = lv_slider_get_value((lv_obj_t*)lv_event_get_target(e));
    bsp_display_brightness_set(v);    // 0..100 %
    if (g_nvs) { nvs_set_u8(g_nvs, "bright", (uint8_t)v); nvs_commit(g_nvs); }
}

void reconnect_cb(lv_event_t*) {
    ObdCommand c{ CmdType::Reconnect, 0, 0 };
    EventBus::instance().sendCommand(c, 0);
}

// A labeled settings row inside a panel.
lv_obj_t* setting_row(lv_obj_t* parent, const char* label) {
    lv_obj_t* row = lv_obj_create(parent);
    lv_obj_add_style(row, &st_panel, 0);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_height(row, 90);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* l = lv_label_create(row);
    lv_obj_set_style_text_color(l, COL_TEXT, 0);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_20, 0);
    lv_label_set_text(l, label);
    return row;
}

} // namespace

void screen_settings_create(lv_obj_t* parent) {
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 16, 0);
    lv_obj_set_style_pad_row(parent, 12, 0);

    // Restore persisted brightness and apply it immediately.
    uint8_t saved_bright = DEFAULT_BRIGHTNESS;
    if (nvs_open("settings", NVS_READWRITE, &g_nvs) == ESP_OK)
        nvs_get_u8(g_nvs, "bright", &saved_bright);
    bsp_display_brightness_set(saved_bright);

    // --- Baud rate ----------------------------------------------------------
    lv_obj_t* row = setting_row(parent, "USB Baud Rate");
    g_baud_dd = lv_dropdown_create(row);
    lv_dropdown_set_options(g_baud_dd, "115200\n230400\n500000\n1000000\n2000000");
    lv_dropdown_set_selected(g_baud_dd, 0);   // adapter opens at 115200 (OBD_DEFAULT_BAUD)
    lv_obj_set_width(g_baud_dd, 220);
    lv_obj_add_event_cb(g_baud_dd, baud_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    // --- CAN network --------------------------------------------------------
    row = setting_row(parent, "CAN Network  (off = HS-CAN, on = MS-CAN)");
    g_bus_sw = lv_switch_create(row);
    lv_obj_set_size(g_bus_sw, 90, 46);
    lv_obj_set_style_bg_color(g_bus_sw, COL_CYAN, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_add_event_cb(g_bus_sw, bus_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    // --- Brightness ---------------------------------------------------------
    row = setting_row(parent, "Screen Brightness");
    g_bright_sl = lv_slider_create(row);
    lv_slider_set_range(g_bright_sl, 10, 100);
    lv_slider_set_value(g_bright_sl, saved_bright, LV_ANIM_OFF);
    lv_obj_set_width(g_bright_sl, 300);
    lv_obj_set_style_bg_color(g_bright_sl, COL_GRID, LV_PART_MAIN);
    lv_obj_set_style_bg_color(g_bright_sl, COL_CYAN, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(g_bright_sl, COL_CYAN, LV_PART_KNOB);
    lv_obj_add_event_cb(g_bright_sl, bright_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    // --- Link status + reconnect -------------------------------------------
    row = setting_row(parent, "Adapter Link");
    g_link_lbl = lv_label_create(row);
    lv_obj_set_style_text_font(g_link_lbl, &lv_font_montserrat_18, 0);
    lv_label_set_text(g_link_lbl, "...");

    lv_obj_t* rc = lv_btn_create(row);
    lv_obj_add_style(rc, &st_accent_btn, 0);
    lv_obj_add_event_cb(rc, reconnect_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* rl = lv_label_create(rc);
    lv_label_set_text(rl, LV_SYMBOL_REFRESH " RECONNECT");
    lv_obj_center(rl);

    // --- SD data files (DTC database + custom PIDs) -------------------------
    // Try loading the SD DTC database now so the count reflects reality; the
    // card is mounted early by the logger, so this usually succeeds here.
    dtc::ensureLoaded();
    row = setting_row(parent, "SD Data Files");
    g_data_lbl = lv_label_create(row);
    lv_obj_set_style_text_font(g_data_lbl, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(g_data_lbl, COL_CYAN, 0);
    lv_label_set_text(g_data_lbl, "...");
}

void screen_settings_update(void) {
    auto& bus = EventBus::instance();
    const char* txt; lv_color_t col;
    switch (bus.link.load()) {
    case LinkState::Online:       txt = LV_SYMBOL_OK " ONLINE";        col = COL_GREEN; break;
    case LinkState::Initializing: txt = "INITIALIZING...";             col = COL_AMBER; break;
    case LinkState::Enumerating:  txt = "ENUMERATING...";              col = COL_AMBER; break;
    case LinkState::Error:        txt = LV_SYMBOL_WARNING " ERROR";    col = COL_RED;   break;
    default:                      txt = LV_SYMBOL_CLOSE " NO ADAPTER"; col = COL_TEXT_DIM; break;
    }
    lv_label_set_text(g_link_lbl, txt);
    lv_obj_set_style_text_color(g_link_lbl, col, 0);

    // SD data-file status: DTC descriptions and custom PIDs, with their source.
    if (g_data_lbl) {
        size_t dsd = dtc::sd_count();
        size_t dn  = dsd ? dsd : dtc::embedded_count();
        size_t pn  = custpid::count();
        char buf[80];
        snprintf(buf, sizeof(buf), "DTC %u (%s)   PIDs %u (%s)",
                 (unsigned)dn, dsd ? "SD" : "built-in",
                 (unsigned)pn, custpid::source());
        lv_label_set_text(g_data_lbl, buf);
    }
}
