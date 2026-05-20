// =============================================================================
//  ui_theme.h - "Carbon" dark-mode design system for the scan tool.
// -----------------------------------------------------------------------------
//  High-contrast motorsport aesthetic: near-black carbon background, cyan and
//  magenta neon accents, amber/red warning ramps. Centralizing the palette and
//  reusable styles keeps the six screens visually consistent and lets us
//  re-skin in one place.
// =============================================================================
#pragma once

#include "lvgl.h"

// --- Palette ----------------------------------------------------------------
#define COL_BG          lv_color_hex(0x0A0C10)   // carbon black
#define COL_PANEL       lv_color_hex(0x14181F)   // raised panel
#define COL_PANEL_HI    lv_color_hex(0x1E242E)   // panel hover / header
#define COL_GRID        lv_color_hex(0x2A313C)   // chart gridlines / borders
#define COL_TEXT        lv_color_hex(0xE6EDF3)   // primary text
#define COL_TEXT_DIM    lv_color_hex(0x8A93A0)   // secondary text
#define COL_CYAN        lv_color_hex(0x00E5FF)   // primary neon accent
#define COL_MAGENTA     lv_color_hex(0xFF2EC4)   // secondary neon accent
#define COL_GREEN       lv_color_hex(0x29F0A0)   // ok / good
#define COL_AMBER       lv_color_hex(0xFFB02E)   // caution
#define COL_RED         lv_color_hex(0xFF3B57)   // alarm / knock

// --- Shared style handles (initialized by ui_theme_init) --------------------
extern lv_style_t st_screen;     // root screen background
extern lv_style_t st_panel;      // rounded carbon panel/card
extern lv_style_t st_title;      // section title text
extern lv_style_t st_value_big;  // large numeric readout
extern lv_style_t st_label_dim;  // dimmed caption text
extern lv_style_t st_accent_btn; // neon-outlined button

// Build the shared styles and register the dark theme on `disp`.
void ui_theme_init(lv_display_t* disp);

// Convenience: create a titled carbon panel inside `parent`. Returns the panel
// (content goes inside it). `title` may be nullptr for a bare panel.
lv_obj_t* ui_make_panel(lv_obj_t* parent, const char* title);
