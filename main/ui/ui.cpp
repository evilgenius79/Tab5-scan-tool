// =============================================================================
//  ui.cpp - display bring-up, tab layout, and screen registration.
// =============================================================================
#include "ui/ui.h"
#include "ui/ui_theme.h"
#include "ui/screens/screens.h"
#include "app_config.h"

#include "bsp/esp-bsp.h"
#include "esp_lvgl_port.h"
#include "esp_log.h"

static const char* TAG = "UI";

static lv_obj_t* g_tabview = nullptr;
static int       g_active  = 0;

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

    // --- Tabview with a left-side nav rail ----------------------------------
    g_tabview = lv_tabview_create(scr);
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

    page = lv_tabview_add_tab(g_tabview, kTabNames[(int)ScreenId::Settings]);
    screen_settings_create(page);

    lvgl_port_unlock();
    ESP_LOGI(TAG, "UI built (%d screens)", (int)ScreenId::_Count);
}
