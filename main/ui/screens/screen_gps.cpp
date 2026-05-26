// =============================================================================
//  screen_gps.cpp - GPS / GNSS status + GPX track recording.
// -----------------------------------------------------------------------------
//  Shows the live fix from the SAM-M10Q (lat/lon, speed, heading, altitude,
//  satellites, HDOP) and a RECORD toggle that writes a GPX track to the SD card
//  via the GPS task. Position/speed timing is consumed elsewhere (Performance);
//  this screen is the at-a-glance status + track-log control.
// =============================================================================
#include "ui/screens/screens.h"
#include "ui/ui_theme.h"
#include "core/event_bus.h"
#include "gps/gps.h"

#include <cstdio>

namespace {

lv_obj_t* g_fix_lbl   = nullptr;
lv_obj_t* g_coord_lbl = nullptr;
lv_obj_t* g_speed_lbl = nullptr;
lv_obj_t* g_alt_lbl   = nullptr;
lv_obj_t* g_link_lbl  = nullptr;
lv_obj_t* g_rec_btn   = nullptr;
lv_obj_t* g_rec_lbl   = nullptr;
lv_obj_t* g_trk_lbl   = nullptr;

// Course over ground -> 16-point compass (N, NNE, NE, ...).
const char* cardinal(float deg) {
    static const char* dirs[16] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                                    "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
    if (deg < 0.0f) deg += 360.0f;
    int idx = (int)((deg + 11.25f) / 22.5f) & 15;
    return dirs[idx];
}

lv_obj_t* info_row(lv_obj_t* parent, const char* caption) {
    lv_obj_t* row = ui_make_panel(parent, nullptr);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_height(row, 76);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* cap = lv_label_create(row);
    lv_obj_add_style(cap, &st_label_dim, 0);
    lv_label_set_text(cap, caption);
    lv_obj_t* val = lv_label_create(row);
    lv_obj_set_style_text_font(val, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(val, COL_TEXT, 0);
    lv_label_set_text(val, "--");
    return val;
}

void rec_cb(lv_event_t*) {
    gps::track_set(!gps::track_active());   // toggle; GPS task opens/closes
}

} // namespace

void screen_gps_create(lv_obj_t* parent) {
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 16, 0);
    lv_obj_set_style_pad_row(parent, 12, 0);

    g_fix_lbl   = info_row(parent, "FIX");
    g_coord_lbl = info_row(parent, "POSITION");
    g_speed_lbl = info_row(parent, "SPEED / HEADING");
    g_alt_lbl   = info_row(parent, "ALTITUDE / HDOP");
    g_link_lbl  = info_row(parent, "LINK");

    // Track recording control.
    lv_obj_t* trow = ui_make_panel(parent, "GPX TRACK LOG");
    lv_obj_set_width(trow, lv_pct(100));
    lv_obj_set_height(trow, 110);
    lv_obj_set_flex_flow(trow, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(trow, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(trow, LV_OBJ_FLAG_SCROLLABLE);

    g_trk_lbl = lv_label_create(trow);
    lv_obj_set_style_text_font(g_trk_lbl, &lv_font_montserrat_18, 0);
    lv_label_set_text(g_trk_lbl, "Idle");

    g_rec_btn = lv_btn_create(trow);
    lv_obj_add_style(g_rec_btn, &st_accent_btn, 0);
    lv_obj_set_size(g_rec_btn, 220, 64);
    lv_obj_add_event_cb(g_rec_btn, rec_cb, LV_EVENT_CLICKED, nullptr);
    g_rec_lbl = lv_label_create(g_rec_btn);
    lv_label_set_text(g_rec_lbl, LV_SYMBOL_PLAY " RECORD");
    lv_obj_center(g_rec_lbl);
}

void screen_gps_update(void) {
    GpsFix g = EventBus::instance().getGps();
    char buf[64];

    if (!g.valid) {
        lv_label_set_text(g_fix_lbl, "no module");
        lv_obj_set_style_text_color(g_fix_lbl, COL_TEXT_DIM, 0);
    } else if (!g.has_fix) {
        snprintf(buf, sizeof(buf), "acquiring  (%u sats)", (unsigned)g.sats);
        lv_label_set_text(g_fix_lbl, buf);
        lv_obj_set_style_text_color(g_fix_lbl, COL_AMBER, 0);
    } else {
        snprintf(buf, sizeof(buf), LV_SYMBOL_OK " 3D fix  (%u sats)", (unsigned)g.sats);
        lv_label_set_text(g_fix_lbl, buf);
        lv_obj_set_style_text_color(g_fix_lbl, COL_GREEN, 0);
    }

    if (g.has_fix) {
        snprintf(buf, sizeof(buf), "%.5f, %.5f", g.lat, g.lon);
        lv_label_set_text(g_coord_lbl, buf);
        snprintf(buf, sizeof(buf), "%.0f mph   %s",
                 g.speed_kph * 0.621371f, cardinal(g.course_deg));
        lv_label_set_text(g_speed_lbl, buf);
        snprintf(buf, sizeof(buf), "%.0f ft   HDOP %.1f",
                 g.alt_m * 3.28084f, g.hdop);
        lv_label_set_text(g_alt_lbl, buf);
    }

    // Link: actual UART baud the reader locked to + measured fix rate.
    snprintf(buf, sizeof(buf), "%u baud   %.1f Hz",
             (unsigned)gps::link_baud(), gps::fix_hz());
    lv_label_set_text(g_link_lbl, buf);

    // Track recording status.
    bool rec = gps::track_active();
    if (rec) {
        snprintf(buf, sizeof(buf), LV_SYMBOL_SD_CARD " REC  %u pts",
                 (unsigned)gps::track_points());
        lv_label_set_text(g_trk_lbl, buf);
        lv_obj_set_style_text_color(g_trk_lbl, COL_RED, 0);
        lv_label_set_text(g_rec_lbl, LV_SYMBOL_STOP " STOP");
    } else {
        lv_label_set_text(g_trk_lbl, "Idle");
        lv_obj_set_style_text_color(g_trk_lbl, COL_TEXT_DIM, 0);
        lv_label_set_text(g_rec_lbl, LV_SYMBOL_PLAY " RECORD");
    }
}
