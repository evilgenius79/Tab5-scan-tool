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

#include <cstdint>   // intptr_t

namespace {

struct MenuItem { const char* icon; const char* label; ScreenId target; lv_color_t accent; };

const MenuItem kItems[] = {
    { LV_SYMBOL_CHARGE,   "Live Dash",     ScreenId::Dash,        COL_CYAN    },
    { LV_SYMBOL_EYE_OPEN, "Live Data",     ScreenId::LiveData,    COL_CYAN    },
    { LV_SYMBOL_GPS,      "Performance",   ScreenId::Performance, COL_MAGENTA },
    { LV_SYMBOL_WARNING,  "Trouble Codes", ScreenId::Diagnostics, COL_AMBER   },
    { LV_SYMBOL_OK,       "I/M Readiness", ScreenId::Readiness,   COL_GREEN   },
    { LV_SYMBOL_LIST,     "Vehicle Info",  ScreenId::Vehicle,     COL_CYAN    },
    { LV_SYMBOL_SD_CARD,  "Data Logging",  ScreenId::Logging,     COL_GREEN   },
    { LV_SYMBOL_LIST,     "CAN Sniffer",   ScreenId::Sniffer,     COL_MAGENTA },
    { LV_SYMBOL_SETTINGS, "Settings",      ScreenId::Settings,    COL_TEXT_DIM},
};

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

void screen_home_update(void) { /* static menu - nothing to refresh */ }
