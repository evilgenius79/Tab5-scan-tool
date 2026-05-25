// =============================================================================
//  screen_diagnostics.cpp - Screen 3: DTC read / clear.
// -----------------------------------------------------------------------------
//  A scrollable table of stored DTCs with READ and CLEAR actions. READ asks the
//  OBD task to run Mode 03 and parse the codes into the EventBus DTC table;
//  CLEAR runs Mode 04 (and turns off the MIL). A confirm step guards CLEAR so a
//  stray touch doesn't wipe freeze-frame data.
// =============================================================================
#include "ui/screens/screens.h"
#include "ui/ui_theme.h"
#include "core/event_bus.h"
#include "obd/dtc_lookup.h"

#include <cstdio>

namespace {

lv_obj_t* g_table  = nullptr;
lv_obj_t* g_status = nullptr;
lv_obj_t* g_vin    = nullptr;
size_t    g_shown  = (size_t)-1;   // last rendered count (avoids needless redraw)
bool      g_vin_shown = false;     // whether a VIN has been rendered yet
bool      g_reading = false;       // a READ is in progress (for completion text)
bool      g_saw_active = false;    // observed the OBD task's read flag go true
bool      g_ff_reading = false;    // a freeze-frame read is in progress
bool      g_ff_saw_active = false; // observed the freeze-frame read flag go true

void vin_cb(lv_event_t*) {
    ObdCommand c{ CmdType::ReadVin, 0, 0 };
    EventBus::instance().sendCommand(c, 0);
    lv_label_set_text(g_vin, "VIN: reading...");
    g_vin_shown = false;           // force refresh once the result lands
}

void read_cb(lv_event_t*) {
    ObdCommand c{ CmdType::ReadDtcs, 0, 0 };
    EventBus::instance().sendCommand(c, 0);
    // Load the SD description database (if a card with dtc_db.csv is present)
    // now, on this user action, so meanings are ready when the codes arrive.
    dtc::ensureLoaded();
    lv_label_set_text(g_status, LV_SYMBOL_REFRESH " Reading DTCs...");
    g_shown = (size_t)-1;          // force refresh on next update
    g_reading = true;
    g_saw_active = false;
}

void freeze_cb(lv_event_t*) {
    ObdCommand c{ CmdType::ReadFreezeFrame, 0, 0 };
    EventBus::instance().sendCommand(c, 0);
    lv_label_set_text(g_status, LV_SYMBOL_REFRESH " Reading freeze frame...");
    g_ff_reading = true;
    g_ff_saw_active = false;
}

void clear_confirm_cb(lv_event_t* e) {
    auto* mbox = (lv_obj_t*)lv_event_get_user_data(e);
    ObdCommand c{ CmdType::ClearDtcs, 0, 0 };
    EventBus::instance().sendCommand(c, 0);
    lv_label_set_text(g_status, "DTCs cleared");
    lv_msgbox_close(mbox);
    g_shown = (size_t)-1;
}

void clear_cb(lv_event_t*) {
    // Confirmation dialog - clearing also wipes freeze-frame data.
    lv_obj_t* mbox = lv_msgbox_create(nullptr);
    lv_msgbox_add_title(mbox, "Clear DTCs?");
    lv_msgbox_add_text(mbox, "This erases stored codes and freeze-frame data.");
    lv_obj_t* ok = lv_msgbox_add_footer_button(mbox, "Clear");
    lv_msgbox_add_close_button(mbox);
    lv_obj_add_event_cb(ok, clear_confirm_cb, LV_EVENT_CLICKED, mbox);
}

} // namespace

void screen_diagnostics_create(lv_obj_t* parent) {
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 12, 0);
    lv_obj_set_style_pad_row(parent, 10, 0);

    // Action bar.
    lv_obj_t* bar = lv_obj_create(parent);
    lv_obj_add_style(bar, &st_screen, 0);
    lv_obj_set_width(bar, lv_pct(100));
    lv_obj_set_height(bar, 64);
    lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(bar, 12, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* read_btn = lv_btn_create(bar);
    lv_obj_add_style(read_btn, &st_accent_btn, 0);
    lv_obj_add_event_cb(read_btn, read_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* rl = lv_label_create(read_btn);
    lv_label_set_text(rl, LV_SYMBOL_REFRESH " READ");
    lv_obj_center(rl);

    lv_obj_t* clr_btn = lv_btn_create(bar);
    lv_obj_add_style(clr_btn, &st_accent_btn, 0);
    lv_obj_set_style_border_color(clr_btn, COL_RED, 0);
    lv_obj_set_style_text_color(clr_btn, COL_RED, 0);
    lv_obj_add_event_cb(clr_btn, clear_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* cl = lv_label_create(clr_btn);
    lv_label_set_text(cl, LV_SYMBOL_TRASH " CLEAR");
    lv_obj_center(cl);

    lv_obj_t* vin_btn = lv_btn_create(bar);
    lv_obj_add_style(vin_btn, &st_accent_btn, 0);
    lv_obj_add_event_cb(vin_btn, vin_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* vl = lv_label_create(vin_btn);
    lv_label_set_text(vl, LV_SYMBOL_LIST " VIN");
    lv_obj_center(vl);

    lv_obj_t* ff_btn = lv_btn_create(bar);
    lv_obj_add_style(ff_btn, &st_accent_btn, 0);
    lv_obj_add_event_cb(ff_btn, freeze_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* fl = lv_label_create(ff_btn);
    lv_label_set_text(fl, LV_SYMBOL_IMAGE " FREEZE");
    lv_obj_center(fl);

    g_status = lv_label_create(bar);
    lv_obj_add_style(g_status, &st_label_dim, 0);
    lv_label_set_text(g_status, "Idle");

    // Vehicle identification (filled by the VIN button).
    g_vin = lv_label_create(parent);
    lv_obj_add_style(g_vin, &st_label_dim, 0);
    lv_obj_set_width(g_vin, lv_pct(100));
    lv_label_set_text(g_vin, "VIN: --");

    // DTC table.
    g_table = lv_table_create(parent);
    lv_obj_set_width(g_table, lv_pct(100));
    lv_obj_set_flex_grow(g_table, 1);
    lv_table_set_column_count(g_table, 3);
    lv_table_set_column_width(g_table, 0, 120);
    lv_table_set_column_width(g_table, 1, 140);
    lv_table_set_column_width(g_table, 2, 660);
    lv_table_set_cell_value(g_table, 0, 0, "CODE");
    lv_table_set_cell_value(g_table, 0, 1, "TYPE");
    lv_table_set_cell_value(g_table, 0, 2, "DESCRIPTION");

    lv_obj_set_style_bg_color(g_table, COL_PANEL, LV_PART_ITEMS);
    lv_obj_set_style_text_color(g_table, COL_TEXT, LV_PART_ITEMS);
    lv_obj_set_style_border_color(g_table, COL_GRID, LV_PART_ITEMS);
    lv_obj_set_style_text_color(g_table, COL_CYAN, LV_PART_ITEMS | LV_STATE_DEFAULT);
}

void screen_diagnostics_update(void) {
    // Detect READ completion via the OBD task's flag (true while reading) so we
    // can show a clear "complete" message even when there are zero codes.
    bool active = EventBus::instance().dtc_read_active.load();
    if (active) g_saw_active = true;
    const bool just_done = (g_reading && g_saw_active && !active);
    if (just_done) g_reading = false;

    // Freeze-frame read completion -> pop a results dialog.
    bool ff_active = EventBus::instance().freeze_read_active.load();
    if (ff_active) g_ff_saw_active = true;
    if (g_ff_reading && g_ff_saw_active && !ff_active) {
        g_ff_reading = false;
        FreezeFrame ff = EventBus::instance().getFreezeFrame();
        lv_obj_t* mbox = lv_msgbox_create(nullptr);
        lv_msgbox_add_title(mbox, "Freeze Frame (Mode 02)");
        char body[256];
        const bool has_data = ff.valid && (ff.dtc[0] || ff.rpm > 0 || ff.coolant_c != 0);
        if (!has_data) {
            snprintf(body, sizeof(body),
                     "No freeze-frame data stored.\n"
                     "(The ECU captures one only when an emissions DTC is set.)");
        } else {
            snprintf(body, sizeof(body),
                     "Triggered by: %s\n\n"
                     "RPM: %.0f      Load: %.0f%%\n"
                     "Coolant: %.0f F   Intake: %.0f F\n"
                     "Speed: %.0f mph   Throttle: %.0f%%\n"
                     "MAP: %.0f kPa     Timing: %.1f deg\n"
                     "MAF: %.1f g/s",
                     ff.dtc[0] ? ff.dtc : "(unknown)",
                     ff.rpm, ff.load, ff.coolant_c * 1.8f + 32.0f,
                     ff.intake_air_c * 1.8f + 32.0f, ff.speed_kph * 0.621371f,
                     ff.throttle_pct, ff.map_kpa, ff.ign_adv_deg, ff.maf_gps);
        }
        lv_msgbox_add_text(mbox, body);
        lv_msgbox_add_close_button(mbox);
        lv_label_set_text(g_status, LV_SYMBOL_OK " Freeze frame read");
    }

    // Refresh the VIN line once a read has produced a result.
    if (!g_vin_shown) {
        VehicleInfo v = EventBus::instance().getVehicleInfo();
        if (v.valid) {
            char line[96];
            if (v.manufacturer[0] && v.model_year) {
                snprintf(line, sizeof(line), "VIN: %s   (%d %s)",
                         v.vin, v.model_year, v.manufacturer);
            } else if (v.manufacturer[0]) {
                snprintf(line, sizeof(line), "VIN: %s   (%s)",
                         v.vin, v.manufacturer);
            } else {
                snprintf(line, sizeof(line), "VIN: %s", v.vin);
            }
            lv_label_set_text(g_vin, line);
            g_vin_shown = true;
        }
    }

    DtcRecord recs[MAX_DTCS];
    size_t n = EventBus::instance().getDtcs(recs, MAX_DTCS);

    // Announce completion (covers the zero-codes case where the table is same).
    if (just_done) {
        char s[44];
        if (n == 0) snprintf(s, sizeof(s), LV_SYMBOL_OK " Read complete - no codes");
        else        snprintf(s, sizeof(s), LV_SYMBOL_OK " Read complete - %u code%s",
                             (unsigned)n, n == 1 ? "" : "s");
        lv_label_set_text(g_status, s);
    }

    if (n == g_shown && !just_done) return;   // nothing changed
    g_shown = n;

    lv_table_set_row_count(g_table, n + 1);
    if (n == 0) {
        lv_table_set_cell_value(g_table, 1, 0, "--");
        lv_table_set_cell_value(g_table, 1, 1, "");
        lv_table_set_cell_value(g_table, 1, 2, "No trouble codes (stored / pending / permanent)");
        lv_table_set_row_count(g_table, 2);
        return;
    }
    for (size_t i = 0; i < n; ++i) {
        lv_table_set_cell_value(g_table, i + 1, 0, recs[i].code);
        const char* type = (recs[i].status & DTC_PERMANENT) ? "Permanent"
                         : (recs[i].status & DTC_PENDING)   ? "Pending"
                                                            : "Stored";
        lv_table_set_cell_value(g_table, i + 1, 1, type);
        const char* desc = dtc::describe(recs[i].code);
        lv_table_set_cell_value(g_table, i + 1, 2,
                                desc ? desc : "(unknown - add to dtc_db.csv)");
    }
}
