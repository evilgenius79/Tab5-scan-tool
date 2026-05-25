// =============================================================================
//  screen_home.cpp - Home menu (scan-tool main menu).
// -----------------------------------------------------------------------------
//  A grid of large function cards, like a real scan tool's home screen. Tapping
//  a card jumps to that function's tab (ui_goto_screen). The left tab rail stays
//  available for quick switching, so navigation works both ways.
// =============================================================================
#include "ui/screens/screens.h"
#include "ui/ui.h"
#include "ui/ui_theme.h"
#include "core/tab5_power.h"
#include "core/event_bus.h"

#include <cstdint>   // intptr_t
#include <cstdio>    // snprintf

namespace {

struct MenuItem { const char* icon; const char* label; ScreenId target; lv_color_t accent; };

const MenuItem kItems[] = {
    { LV_SYMBOL_CHARGE,   "Live Dash",     ScreenId::Dash,        COL_CYAN    },
    { LV_SYMBOL_EYE_OPEN, "Live Data",     ScreenId::LiveData,    COL_CYAN    },
    { LV_SYMBOL_GPS,      "Performance",   ScreenId::Performance, COL_MAGENTA },
    { LV_SYMBOL_WARNING,  "Trouble Codes", ScreenId::Diagnostics, COL_AMBER   },
    { LV_SYMBOL_WARNING,  "Module Scan",   ScreenId::Modules,     COL_RED     },
    { LV_SYMBOL_OK,       "I/M Readiness", ScreenId::Readiness,   COL_GREEN   },
    { LV_SYMBOL_LIST,     "Vehicle Info",  ScreenId::Vehicle,     COL_CYAN    },
    { LV_SYMBOL_SD_CARD,  "Data Logging",  ScreenId::Logging,     COL_GREEN   },
    { LV_SYMBOL_LIST,     "CAN Sniffer",   ScreenId::Sniffer,     COL_MAGENTA },
    { LV_SYMBOL_SETTINGS, "Settings",      ScreenId::Settings,    COL_TEXT_DIM},
};

lv_obj_t* g_charge = nullptr;   // charge-status indicator in the header
lv_obj_t* g_mil    = nullptr;   // MIL / stored-DTC status badge in the header

void card_cb(lv_event_t* e) {
    int target = (int)(intptr_t)lv_event_get_user_data(e);
    ui_goto_screen(target);
}

} // namespace

void screen_home_create(lv_obj_t* parent) {
    lv_obj_set_style_pad_all(parent, 16, 0);

    lv_obj_t* title = lv_label_create(parent);
    lv_obj_add_style(title, &st_title, 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_label_set_text(title, "TAB5 SCAN TOOL");
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 4, 0);

    // Charge-status indicator (top-right of the header).
    g_charge = lv_label_create(parent);
    lv_obj_set_style_text_font(g_charge, &lv_font_montserrat_20, 0);
    lv_obj_align(g_charge, LV_ALIGN_TOP_RIGHT, -8, 0);
    lv_label_set_text(g_charge, LV_SYMBOL_BATTERY_FULL);

    // MIL / Check-Engine badge (top-center) - filled by the auto readiness read.
    g_mil = lv_label_create(parent);
    lv_obj_set_style_text_font(g_mil, &lv_font_montserrat_20, 0);
    lv_obj_align(g_mil, LV_ALIGN_TOP_MID, 0, 2);
    lv_obj_set_style_text_color(g_mil, COL_TEXT_DIM, 0);
    lv_label_set_text(g_mil, LV_SYMBOL_WARNING " status --");

    // Card grid: flex-wrap so it lays out responsively.
    lv_obj_t* grid = lv_obj_create(parent);
    lv_obj_add_style(grid, &st_screen, 0);
    lv_obj_set_size(grid, lv_pct(100), lv_pct(88));
    lv_obj_align(grid, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(grid, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(grid, 6, 0);
    lv_obj_set_style_pad_row(grid, 14, 0);
    lv_obj_set_style_pad_column(grid, 14, 0);

    for (const auto& it : kItems) {
        lv_obj_t* card = lv_btn_create(grid);
        lv_obj_add_style(card, &st_panel, 0);
        lv_obj_set_size(card, 320, 150);
        lv_obj_set_style_border_color(card, it.accent, 0);
        lv_obj_set_style_border_width(card, 2, 0);
        lv_obj_add_event_cb(card, card_cb, LV_EVENT_CLICKED,
                            (void*)(intptr_t)(int)it.target);

        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);

        lv_obj_t* icon = lv_label_create(card);
        lv_obj_set_style_text_font(icon, &lv_font_montserrat_28, 0);
        lv_obj_set_style_text_color(icon, it.accent, 0);
        lv_label_set_text(icon, it.icon);

        lv_obj_t* lbl = lv_label_create(card);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(lbl, COL_TEXT, 0);
        lv_label_set_text(lbl, it.label);
    }
}

void screen_home_update(void) {
    if (!g_charge) return;
    const int pct = tab5pwr::battery_percent();
    const bool charging = tab5pwr::is_charging();
    char buf[40];

    // Pick a battery glyph by level (charging always shows the charge bolt).
    const char* icon = charging ? LV_SYMBOL_CHARGE
                     : pct < 0   ? LV_SYMBOL_BATTERY_FULL
                     : pct >= 80 ? LV_SYMBOL_BATTERY_FULL
                     : pct >= 55 ? LV_SYMBOL_BATTERY_3
                     : pct >= 30 ? LV_SYMBOL_BATTERY_2
                     : pct >= 12 ? LV_SYMBOL_BATTERY_1
                                 : LV_SYMBOL_BATTERY_EMPTY;

    if (pct < 0) snprintf(buf, sizeof(buf), "%s %s", icon,
                          charging ? "Charging" : "Battery");
    else         snprintf(buf, sizeof(buf), "%s %d%%%s", icon, pct,
                          charging ? " +" : "");
    lv_label_set_text(g_charge, buf);
    lv_obj_set_style_text_color(g_charge,
        charging ? COL_GREEN : (pct >= 0 && pct < 15 ? COL_RED : COL_TEXT), 0);

    // MIL / Check-Engine badge from the (auto-read) I/M readiness snapshot.
    if (g_mil) {
        ReadinessInfo r = EventBus::instance().getReadiness();
        char mbuf[48];
        if (!r.valid) {
            lv_label_set_text(g_mil, LV_SYMBOL_WARNING " status --");
            lv_obj_set_style_text_color(g_mil, COL_TEXT_DIM, 0);
        } else if (r.mil_on) {
            snprintf(mbuf, sizeof(mbuf), LV_SYMBOL_WARNING " CHECK ENGINE  (%u DTC)",
                     (unsigned)r.dtc_count);
            lv_label_set_text(g_mil, mbuf);
            lv_obj_set_style_text_color(g_mil, COL_RED, 0);
        } else if (r.dtc_count > 0) {
            snprintf(mbuf, sizeof(mbuf), LV_SYMBOL_WARNING " %u stored code%s",
                     (unsigned)r.dtc_count, r.dtc_count == 1 ? "" : "s");
            lv_label_set_text(g_mil, mbuf);
            lv_obj_set_style_text_color(g_mil, COL_AMBER, 0);
        } else {
            lv_label_set_text(g_mil, LV_SYMBOL_OK " No MIL");
            lv_obj_set_style_text_color(g_mil, COL_GREEN, 0);
        }
    }
}
