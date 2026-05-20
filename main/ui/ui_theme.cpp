// =============================================================================
//  ui_theme.cpp - shared style construction.
// =============================================================================
#include "ui/ui_theme.h"

lv_style_t st_screen;
lv_style_t st_panel;
lv_style_t st_title;
lv_style_t st_value_big;
lv_style_t st_label_dim;
lv_style_t st_accent_btn;

void ui_theme_init(lv_display_t* disp) {
    // Apply LVGL's built-in dark theme as a base, then override with our
    // carbon palette via explicit styles. Cyan is the primary accent.
    lv_theme_t* base = lv_theme_default_init(
        disp, COL_CYAN, COL_MAGENTA, /*dark=*/true, LV_FONT_DEFAULT);
    lv_disp_set_theme(disp, base);

    // --- Root screen ---------------------------------------------------------
    lv_style_init(&st_screen);
    lv_style_set_bg_color(&st_screen, COL_BG);
    lv_style_set_bg_opa(&st_screen, LV_OPA_COVER);
    lv_style_set_text_color(&st_screen, COL_TEXT);
    lv_style_set_pad_all(&st_screen, 0);

    // --- Carbon panel / card -------------------------------------------------
    lv_style_init(&st_panel);
    lv_style_set_bg_color(&st_panel, COL_PANEL);
    lv_style_set_bg_opa(&st_panel, LV_OPA_COVER);
    lv_style_set_radius(&st_panel, 14);
    lv_style_set_border_width(&st_panel, 1);
    lv_style_set_border_color(&st_panel, COL_GRID);
    lv_style_set_pad_all(&st_panel, 12);
    // Subtle shadow lifts the card off the carbon background.
    lv_style_set_shadow_width(&st_panel, 16);
    lv_style_set_shadow_color(&st_panel, lv_color_black());
    lv_style_set_shadow_opa(&st_panel, LV_OPA_40);

    // --- Section title -------------------------------------------------------
    lv_style_init(&st_title);
    lv_style_set_text_color(&st_title, COL_CYAN);
    lv_style_set_text_font(&st_title, &lv_font_montserrat_20);
    lv_style_set_text_letter_space(&st_title, 2);

    // --- Big numeric readout -------------------------------------------------
    lv_style_init(&st_value_big);
    lv_style_set_text_color(&st_value_big, COL_TEXT);
    lv_style_set_text_font(&st_value_big, &lv_font_montserrat_48);

    // --- Dimmed caption ------------------------------------------------------
    lv_style_init(&st_label_dim);
    lv_style_set_text_color(&st_label_dim, COL_TEXT_DIM);
    lv_style_set_text_font(&st_label_dim, &lv_font_montserrat_14);

    // --- Neon button ---------------------------------------------------------
    lv_style_init(&st_accent_btn);
    lv_style_set_bg_color(&st_accent_btn, COL_PANEL_HI);
    lv_style_set_bg_opa(&st_accent_btn, LV_OPA_COVER);
    lv_style_set_border_width(&st_accent_btn, 2);
    lv_style_set_border_color(&st_accent_btn, COL_CYAN);
    lv_style_set_text_color(&st_accent_btn, COL_CYAN);
    lv_style_set_radius(&st_accent_btn, 10);
    lv_style_set_pad_hor(&st_accent_btn, 18);
    lv_style_set_pad_ver(&st_accent_btn, 10);
}

lv_obj_t* ui_make_panel(lv_obj_t* parent, const char* title) {
    lv_obj_t* panel = lv_obj_create(parent);
    lv_obj_add_style(panel, &st_panel, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    if (title) {
        lv_obj_t* lbl = lv_label_create(panel);
        lv_obj_add_style(lbl, &st_title, 0);
        lv_label_set_text(lbl, title);
        lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 0, 0);
    }
    return panel;
}
