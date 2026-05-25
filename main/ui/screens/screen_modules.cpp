// =============================================================================
//  screen_modules.cpp - Enhanced multi-module DTC scan (UDS service 0x19).
// -----------------------------------------------------------------------------
//  Beyond generic OBD (powertrain only), this reads fault codes from every
//  control module the tool knows how to address (ABS, airbag, BCM, cluster...).
//  SCAN sends UDS 0x19 to each module by CAN header; results land in EventBus.
//  NOTE: module addressing is manufacturer-specific - the built-in map targets
//  Ford; other makes need their own profile.
// =============================================================================
#include "ui/screens/screens.h"
#include "ui/ui_theme.h"
#include "core/event_bus.h"
#include "obd/dtc_lookup.h"

#include <cstdio>
#include <cstring>

namespace {

lv_obj_t* g_table  = nullptr;
lv_obj_t* g_status = nullptr;
bool      g_was_scanning = false;

void scan_cb(lv_event_t*) {
    ObdCommand c{ CmdType::ScanModules, 0, 0 };
    EventBus::instance().sendCommand(c, 0);
    lv_label_set_text(g_status, LV_SYMBOL_REFRESH " Scanning all modules...");
}

void clear_confirm_cb(lv_event_t* e) {
    auto* mbox = (lv_obj_t*)lv_event_get_user_data(e);
    ObdCommand c{ CmdType::ClearModuleDtcs, 0, 0 };
    EventBus::instance().sendCommand(c, 0);
    lv_label_set_text(g_status, LV_SYMBOL_TRASH " Clearing all modules...");
    lv_msgbox_close(mbox);
}

void clear_cb(lv_event_t*) {
    lv_obj_t* mbox = lv_msgbox_create(nullptr);
    lv_msgbox_add_title(mbox, "Clear ALL module codes?");
    lv_msgbox_add_text(mbox, "Erases DTCs in every module (incl. airbag/ABS) and "
                             "freeze-frame data. Only do this with the engine on.");
    lv_obj_t* ok = lv_msgbox_add_footer_button(mbox, "Clear All");
    lv_msgbox_add_close_button(mbox);
    lv_obj_add_event_cb(ok, clear_confirm_cb, LV_EVENT_CLICKED, mbox);
}

} // namespace

void screen_modules_create(lv_obj_t* parent) {
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 12, 0);
    lv_obj_set_style_pad_row(parent, 10, 0);

    lv_obj_t* bar = lv_obj_create(parent);
    lv_obj_add_style(bar, &st_screen, 0);
    lv_obj_set_width(bar, lv_pct(100));
    lv_obj_set_height(bar, 60);
    lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(bar, 14, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* sb = lv_btn_create(bar);
    lv_obj_add_style(sb, &st_accent_btn, 0);
    lv_obj_add_event_cb(sb, scan_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* sl = lv_label_create(sb);
    lv_label_set_text(sl, LV_SYMBOL_REFRESH " SCAN ALL MODULES");
    lv_obj_center(sl);

    lv_obj_t* cb = lv_btn_create(bar);
    lv_obj_add_style(cb, &st_accent_btn, 0);
    lv_obj_set_style_border_color(cb, COL_RED, 0);
    lv_obj_set_style_text_color(cb, COL_RED, 0);
    lv_obj_add_event_cb(cb, clear_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* cl = lv_label_create(cb);
    lv_label_set_text(cl, LV_SYMBOL_TRASH " CLEAR ALL");
    lv_obj_center(cl);

    g_status = lv_label_create(bar);
    lv_obj_add_style(g_status, &st_label_dim, 0);
    lv_label_set_text(g_status, "Idle - tap SCAN (engine on)");

    g_table = lv_table_create(parent);
    lv_obj_set_width(g_table, lv_pct(100));
    lv_obj_set_flex_grow(g_table, 1);
    lv_table_set_column_count(g_table, 3);
    lv_table_set_column_width(g_table, 0, 220);
    lv_table_set_column_width(g_table, 1, 140);
    lv_table_set_column_width(g_table, 2, 540);
    lv_table_set_cell_value(g_table, 0, 0, "MODULE");
    lv_table_set_cell_value(g_table, 0, 1, "STATUS");
    lv_table_set_cell_value(g_table, 0, 2, "CODES");
    lv_obj_set_style_text_font(g_table, &lv_font_montserrat_16, LV_PART_ITEMS);
    lv_obj_set_style_bg_color(g_table, COL_PANEL, LV_PART_ITEMS);
    lv_obj_set_style_text_color(g_table, COL_TEXT, LV_PART_ITEMS);
    lv_obj_set_style_border_color(g_table, COL_GRID, LV_PART_ITEMS);
    lv_obj_set_style_pad_ver(g_table, 5, LV_PART_ITEMS);
}

void screen_modules_update(void) {
    auto& bus = EventBus::instance();
    bool scanning = bus.module_scan_active.load();

    // Only re-render when a scan finishes (scanning edge) to avoid churn.
    if (scanning) { g_was_scanning = true; return; }
    if (!g_was_scanning) return;
    g_was_scanning = false;
    lv_label_set_text(g_status, LV_SYMBOL_OK " Scan complete");

    ModuleResult mods[MAX_MODULES];
    size_t n = bus.getModuleResults(mods, MAX_MODULES);
    lv_table_set_row_count(g_table, n + 1);

    char codes[512];
    for (size_t i = 0; i < n; ++i) {
        lv_table_set_cell_value(g_table, i + 1, 0, mods[i].name);

        const char* status;
        if (!mods[i].responded)        status = "--";          // module absent
        else if (mods[i].dtc_count==0) status = LV_SYMBOL_OK " OK";
        else                           status = LV_SYMBOL_WARNING " FAULT";
        lv_table_set_cell_value(g_table, i + 1, 1, status);

        // One "CODE  Description" per line so faults are readable at a glance.
        codes[0] = '\0';
        for (uint8_t d = 0; d < mods[i].dtc_count; ++d) {
            size_t len = strlen(codes);
            const char* desc = dtc::describe(mods[i].dtcs[d].code);
            snprintf(codes + len, sizeof(codes) - len, "%s%s%s%s",
                     d ? "\n" : "", mods[i].dtcs[d].code,
                     desc ? "  " : "", desc ? desc : "");
        }
        lv_table_set_cell_value(g_table, i + 1, 2,
                                mods[i].dtc_count ? codes
                                : (mods[i].responded ? "no codes" : ""));
    }
}
