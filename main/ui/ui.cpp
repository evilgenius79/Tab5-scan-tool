// =============================================================================
//  ui.cpp - display bring-up, tab layout, and screen registration.
// =============================================================================
#include "ui/ui.h"
#include "ui/ui_theme.h"
#include "ui/screens/screens.h"
#include "app_config.h"
#include "core/tab5_power.h"
#include "core/event_bus.h"

#include "bsp/esp-bsp.h"
#include "esp_lvgl_port.h"
#include "esp_log.h"

#include <cstdio>

static const char* TAG = "UI";

static lv_obj_t* g_tabview = nullptr;
static int       g_active  = 0;

// Global top status bar (battery + MIL), shown above the tabview on every page.
static lv_obj_t* g_topbar  = nullptr;
static lv_obj_t* g_bat_lbl = nullptr;
static lv_obj_t* g_mil_lbl = nullptr;
static constexpr int TOPBAR_H = 44;

// Tab labels with glyphs (Montserrat symbol font) for a polished nav rail.
static const char* kTabNames[(int)ScreenId::_Count] = {
    LV_SYMBOL_HOME    " HOME",
    LV_SYMBOL_CHARGE  " DASH",
    LV_SYMBOL_GPS     " PERF",
    LV_SYMBOL_WARNING " DTC",
    LV_SYMBOL_OK      " I/M",
    LV_SYMBOL_LIST    " VEH",
    LV_SYMBOL_EYE_OPEN" LIVE",
    LV_SYMBOL_WARNING " MOD",
    LV_SYMBOL_SD_CARD " LOG",
    LV_SYMBOL_LIST    " CAN",
    LV_SYMBOL_IMAGE   " GRAPH",
    LV_SYMBOL_GPS     " GPS",
    LV_SYMBOL_SETTINGS" SET",
};

static void tab_changed_cb(lv_event_t* e) {
    (void)e;
    g_active = lv_tabview_get_tab_active(g_tabview);
}

int ui_active_screen() { return g_active; }

void ui_goto_screen(int screen_index) {
    if (g_tabview) {
        lv_tabview_set_active(g_tabview, (uint32_t)screen_index, LV_ANIM_ON);
        g_active = screen_index;
    }
}

void ui_init() {
    // --- Display + LVGL port -------------------------------------------------
    // The Tab5 BSP configures the MIPI-DSI panel, GT911 touch, and installs
    // esp_lvgl_port. We pin the LVGL task to the UI core and give it generous
    // double buffers in PSRAM for the 1280x720 panel.
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size   = BSP_LCD_H_RES * 120,      // partial-render buffer
        .double_buffer = true,
        .flags = {
            .buff_dma    = false,
            .buff_spiram = true,
        },
    };
    cfg.lvgl_port_cfg.task_affinity = APP_CORE_UI;
    cfg.lvgl_port_cfg.task_priority = PRIO_UI_TASK;
    cfg.lvgl_port_cfg.task_stack    = STACK_UI_TASK;

    lv_display_t* disp = bsp_display_start_with_config(&cfg);
    if (!disp) {
        ESP_LOGE(TAG, "display start failed");
        return;
    }
    bsp_display_backlight_on();

    // All LVGL object creation must hold the port lock.
    lvgl_port_lock(0);

    ui_theme_init(disp);

    lv_obj_t* scr = lv_screen_active();
    lv_obj_add_style(scr, &st_screen, 0);

    // Stack a global status bar above the tabview. Flex (not absolute sizing)
    // keeps this correct regardless of the panel's portrait-native rotation.
    lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(scr, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_all(scr, 0, 0);
    lv_obj_set_style_pad_row(scr, 0, 0);

    // --- Global top bar: battery + MIL, visible on every screen -------------
    g_topbar = lv_obj_create(scr);
    lv_obj_set_width(g_topbar, lv_pct(100));
    lv_obj_set_height(g_topbar, TOPBAR_H);
    lv_obj_set_style_bg_color(g_topbar, COL_PANEL, 0);
    lv_obj_set_style_border_width(g_topbar, 0, 0);
    lv_obj_set_style_radius(g_topbar, 0, 0);
    lv_obj_set_style_pad_hor(g_topbar, 12, 0);
    lv_obj_set_style_pad_ver(g_topbar, 4, 0);
    lv_obj_clear_flag(g_topbar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(g_topbar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(g_topbar, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    g_mil_lbl = lv_label_create(g_topbar);
    lv_obj_set_style_text_font(g_mil_lbl, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(g_mil_lbl, COL_TEXT_DIM, 0);
    lv_label_set_text(g_mil_lbl, LV_SYMBOL_WARNING " status --");

    g_bat_lbl = lv_label_create(g_topbar);
    lv_obj_set_style_text_font(g_bat_lbl, &lv_font_montserrat_20, 0);
    lv_label_set_text(g_bat_lbl, LV_SYMBOL_BATTERY_FULL);

    // --- Tabview with a left-side nav rail (fills the rest of the height) ----
    g_tabview = lv_tabview_create(scr);
    lv_obj_set_width(g_tabview, lv_pct(100));
    lv_obj_set_flex_grow(g_tabview, 1);
    lv_tabview_set_tab_bar_position(g_tabview, LV_DIR_LEFT);
    lv_tabview_set_tab_bar_size(g_tabview, 150);
    lv_obj_set_style_bg_color(g_tabview, COL_BG, 0);

    // Style the nav rail (the tab buttons container).
    lv_obj_t* bar = lv_tabview_get_tab_bar(g_tabview);
    lv_obj_set_style_bg_color(bar, COL_PANEL, 0);
    lv_obj_set_style_text_color(bar, COL_TEXT_DIM, 0);
    lv_obj_set_style_text_color(bar, COL_CYAN, LV_STATE_CHECKED);
    lv_obj_set_style_text_font(bar, &lv_font_montserrat_16, 0);

    lv_obj_add_event_cb(g_tabview, tab_changed_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    // --- Build each screen into its tab page --------------------------------
    lv_obj_t* page;
    page = lv_tabview_add_tab(g_tabview, kTabNames[(int)ScreenId::Home]);
    screen_home_create(page);

    page = lv_tabview_add_tab(g_tabview, kTabNames[(int)ScreenId::Dash]);
    screen_dash_create(page);

    page = lv_tabview_add_tab(g_tabview, kTabNames[(int)ScreenId::Performance]);
    screen_performance_create(page);

    page = lv_tabview_add_tab(g_tabview, kTabNames[(int)ScreenId::Diagnostics]);
    screen_diagnostics_create(page);

    page = lv_tabview_add_tab(g_tabview, kTabNames[(int)ScreenId::Readiness]);
    screen_readiness_create(page);

    page = lv_tabview_add_tab(g_tabview, kTabNames[(int)ScreenId::Vehicle]);
    screen_vehicle_create(page);

    page = lv_tabview_add_tab(g_tabview, kTabNames[(int)ScreenId::LiveData]);
    screen_livedata_create(page);

    page = lv_tabview_add_tab(g_tabview, kTabNames[(int)ScreenId::Modules]);
    screen_modules_create(page);

    page = lv_tabview_add_tab(g_tabview, kTabNames[(int)ScreenId::Logging]);
    screen_logging_create(page);

    page = lv_tabview_add_tab(g_tabview, kTabNames[(int)ScreenId::Sniffer]);
    screen_sniffer_create(page);

    page = lv_tabview_add_tab(g_tabview, kTabNames[(int)ScreenId::Graph]);
    screen_graph_create(page);

    page = lv_tabview_add_tab(g_tabview, kTabNames[(int)ScreenId::Gps]);
    screen_gps_create(page);

    page = lv_tabview_add_tab(g_tabview, kTabNames[(int)ScreenId::Settings]);
    screen_settings_create(page);

    lvgl_port_unlock();
    ESP_LOGI(TAG, "UI built (%d screens)", (int)ScreenId::_Count);
}

void ui_topbar_update(void) {
    if (!g_bat_lbl) return;

    // --- Battery (right) ----------------------------------------------------
    const int  pct      = tab5pwr::battery_percent();
    const bool charging = tab5pwr::is_charging();
    const char* icon = charging ? LV_SYMBOL_CHARGE
                     : pct < 0   ? LV_SYMBOL_BATTERY_FULL
                     : pct >= 80 ? LV_SYMBOL_BATTERY_FULL
                     : pct >= 55 ? LV_SYMBOL_BATTERY_3
                     : pct >= 30 ? LV_SYMBOL_BATTERY_2
                     : pct >= 12 ? LV_SYMBOL_BATTERY_1
                                 : LV_SYMBOL_BATTERY_EMPTY;
    char buf[40];
    if (pct < 0) snprintf(buf, sizeof(buf), "%s %s", icon,
                          charging ? "Charging" : "Battery");
    else         snprintf(buf, sizeof(buf), "%s %d%%%s", icon, pct,
                          charging ? " +" : "");
    lv_label_set_text(g_bat_lbl, buf);
    lv_obj_set_style_text_color(g_bat_lbl,
        charging ? COL_GREEN : (pct >= 0 && pct < 15 ? COL_RED : COL_TEXT), 0);

    // --- MIL / Check-Engine (left) from the auto-read I/M readiness ---------
    if (g_mil_lbl) {
        ReadinessInfo r = EventBus::instance().getReadiness();
        char mbuf[48];
        if (!r.valid) {
            lv_label_set_text(g_mil_lbl, LV_SYMBOL_WARNING " status --");
            lv_obj_set_style_text_color(g_mil_lbl, COL_TEXT_DIM, 0);
        } else if (r.mil_on) {
            snprintf(mbuf, sizeof(mbuf), LV_SYMBOL_WARNING " CHECK ENGINE  (%u DTC)",
                     (unsigned)r.dtc_count);
            lv_label_set_text(g_mil_lbl, mbuf);
            lv_obj_set_style_text_color(g_mil_lbl, COL_RED, 0);
        } else if (r.dtc_count > 0) {
            snprintf(mbuf, sizeof(mbuf), LV_SYMBOL_WARNING " %u stored code%s",
                     (unsigned)r.dtc_count, r.dtc_count == 1 ? "" : "s");
            lv_label_set_text(g_mil_lbl, mbuf);
            lv_obj_set_style_text_color(g_mil_lbl, COL_AMBER, 0);
        } else {
            lv_label_set_text(g_mil_lbl, LV_SYMBOL_OK " No MIL");
            lv_obj_set_style_text_color(g_mil_lbl, COL_GREEN, 0);
        }
    }
}
