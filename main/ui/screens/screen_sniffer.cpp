// =============================================================================
//  screen_sniffer.cpp - Screen 5: CAN Sniffer & Hacker.
// -----------------------------------------------------------------------------
//  Real-time raw CAN frame view with the carbon/neon aesthetic:
//    * A fixed-height lv_table acting as a monospace scrollback (newest on top)
//      fed by draining the EventBus sniffer ring in batches.
//    * START/STOP toggles the OBD task into Sniffing mode (STMA/STM).
//    * FREEZE pauses the on-screen view (and the producer's ring writes) so a
//      fast-moving ID can be inspected without the display scrolling.
//    * A hex keypad sets an STMF-style pass filter by CAN ID.
//
//  Performance: lv_table copies cell strings internally, so we can render from
//  scratch buffers each tick. We bound the visible window to UI_SNIFFER_MAX_ROWS
//  and only fully re-render when new frames arrived and we are not frozen.
// =============================================================================
#include "ui/screens/screens.h"
#include "ui/ui_theme.h"
#include "core/event_bus.h"
#include "app_config.h"

#include <cstdio>
#include <cstring>

namespace {

lv_obj_t* g_table     = nullptr;
lv_obj_t* g_freeze_lbl= nullptr;
lv_obj_t* g_run_lbl   = nullptr;
lv_obj_t* g_filter_ta = nullptr;
lv_obj_t* g_kb        = nullptr;
lv_obj_t* g_stat_lbl  = nullptr;

// Local newest-first ring of formatted rows.
struct Row { char id[10]; char meta[12]; char data[40]; };
Row    g_rows[UI_SNIFFER_MAX_ROWS];
size_t g_head  = 0;     // index of newest row
size_t g_count = 0;
uint32_t g_total = 0;   // lifetime frames seen on screen

void format_row(const can_frame_t& f, Row& r) {
    snprintf(r.id, sizeof(r.id), f.extended ? "%08lX" : "%03lX",
             (unsigned long)f.id);
    snprintf(r.meta, sizeof(r.meta), "[%u]", f.dlc);
    int o = 0;
    for (int i = 0; i < f.dlc && o < (int)sizeof(r.data) - 3; ++i) {
        o += snprintf(r.data + o, sizeof(r.data) - o, "%02X ", f.data[i]);
    }
}

// --- Controls ---------------------------------------------------------------
void run_cb(lv_event_t*) {
    auto& bus = EventBus::instance();
    bool sniffing = bus.mode.load() == ObdMode::Sniffing;
    ObdCommand c{ CmdType::SetMode, (uint8_t)(sniffing ? ObdMode::Idle
                                                       : ObdMode::Sniffing), 0 };
    bus.sendCommand(c, 0);
    lv_label_set_text(g_run_lbl, sniffing ? LV_SYMBOL_PLAY " START"
                                          : LV_SYMBOL_STOP " STOP");
}

void freeze_cb(lv_event_t*) {
    auto& bus = EventBus::instance();
    bool frozen = !bus.sniffer_frozen.load();
    bus.sniffer_frozen.store(frozen);
    lv_label_set_text(g_freeze_lbl, frozen ? LV_SYMBOL_PLAY " RESUME"
                                           : LV_SYMBOL_PAUSE " FREEZE");
}

void apply_filter_cb(lv_event_t*) {
    const char* txt = lv_textarea_get_text(g_filter_ta);
    auto& bus = EventBus::instance();
    if (txt && txt[0]) {
        uint32_t id = (uint32_t)strtoul(txt, nullptr, 16);
        ObdCommand c{ CmdType::SetSnifferFilter, (uint8_t)(id > 0x7FF), id };
        bus.sendCommand(c, 0);
    } else {
        ObdCommand c{ CmdType::ClearSnifferFilter, 0, 0 };
        bus.sendCommand(c, 0);
    }
}

void clear_view_cb(lv_event_t*) {
    g_head = g_count = g_total = 0;
    EventBus::instance().snifferRing().clear();
    lv_table_set_row_count(g_table, 1);
}

// Show/hide the hex keyboard when the filter field gains/loses focus.
void filter_focus_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_FOCUSED) lv_obj_remove_flag(g_kb, LV_OBJ_FLAG_HIDDEN);
    else if (code == LV_EVENT_DEFOCUSED) lv_obj_add_flag(g_kb, LV_OBJ_FLAG_HIDDEN);
}

lv_obj_t* make_btn(lv_obj_t* parent, const char* txt, lv_color_t accent,
                   lv_event_cb_t cb, lv_obj_t** out_lbl) {
    lv_obj_t* b = lv_btn_create(parent);
    lv_obj_add_style(b, &st_accent_btn, 0);
    lv_obj_set_style_border_color(b, accent, 0);
    lv_obj_set_style_text_color(b, accent, 0);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* l = lv_label_create(b);
    lv_label_set_text(l, txt);
    lv_obj_center(l);
    if (out_lbl) *out_lbl = l;
    return b;
}

} // namespace

void screen_sniffer_create(lv_obj_t* parent) {
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 10, 0);
    lv_obj_set_style_pad_row(parent, 8, 0);

    // --- Control bar --------------------------------------------------------
    lv_obj_t* bar = lv_obj_create(parent);
    lv_obj_add_style(bar, &st_screen, 0);
    lv_obj_set_width(bar, lv_pct(100));
    lv_obj_set_height(bar, 60);
    lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(bar, 10, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

    make_btn(bar, LV_SYMBOL_PLAY  " START",  COL_GREEN,   run_cb,        &g_run_lbl);
    make_btn(bar, LV_SYMBOL_PAUSE " FREEZE", COL_AMBER,   freeze_cb,     &g_freeze_lbl);
    make_btn(bar, LV_SYMBOL_TRASH " CLEAR",  COL_RED,     clear_view_cb, nullptr);

    // Filter input.
    g_filter_ta = lv_textarea_create(bar);
    lv_textarea_set_one_line(g_filter_ta, true);
    lv_textarea_set_placeholder_text(g_filter_ta, "CAN ID hex (e.g. 7E8)");
    lv_textarea_set_max_length(g_filter_ta, 8);
    lv_obj_set_width(g_filter_ta, 220);
    lv_obj_set_style_text_font(g_filter_ta, &lv_font_montserrat_18, 0);
    lv_obj_add_event_cb(g_filter_ta, filter_focus_cb, LV_EVENT_ALL, nullptr);

    make_btn(bar, LV_SYMBOL_OK " FILTER", COL_CYAN, apply_filter_cb, nullptr);

    g_stat_lbl = lv_label_create(bar);
    lv_obj_add_style(g_stat_lbl, &st_label_dim, 0);
    lv_label_set_text(g_stat_lbl, "0 frames");

    // --- Frame table --------------------------------------------------------
    g_table = lv_table_create(parent);
    lv_obj_set_width(g_table, lv_pct(100));
    lv_obj_set_flex_grow(g_table, 1);
    lv_table_set_column_count(g_table, 3);
    lv_table_set_column_width(g_table, 0, 140);   // ID
    lv_table_set_column_width(g_table, 1, 80);    // DLC
    lv_table_set_column_width(g_table, 2, 700);   // data bytes
    lv_table_set_cell_value(g_table, 0, 0, "CAN ID");
    lv_table_set_cell_value(g_table, 0, 1, "DLC");
    lv_table_set_cell_value(g_table, 0, 2, "DATA");

    // Monospace-ish, dense rows on carbon.
    lv_obj_set_style_text_font(g_table, &lv_font_montserrat_16, LV_PART_ITEMS);
    lv_obj_set_style_bg_color(g_table, COL_PANEL, LV_PART_ITEMS);
    lv_obj_set_style_text_color(g_table, COL_GREEN, LV_PART_ITEMS);
    lv_obj_set_style_border_color(g_table, COL_GRID, LV_PART_ITEMS);
    lv_obj_set_style_pad_ver(g_table, 4, LV_PART_ITEMS);

    // --- Hex keyboard (hidden until the filter field is focused) ------------
    g_kb = lv_keyboard_create(parent);
    lv_keyboard_set_mode(g_kb, LV_KEYBOARD_MODE_NUMBER);
    lv_keyboard_set_textarea(g_kb, g_filter_ta);
    lv_obj_add_flag(g_kb, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_height(g_kb, lv_pct(40));
}

void screen_sniffer_update(void) {
    auto& bus = EventBus::instance();

    // Status line is always fresh.
    char sbuf[48];
    snprintf(sbuf, sizeof(sbuf), "%u shown / ring %u%s",
             (unsigned)g_total, (unsigned)bus.snifferRing().size(),
             bus.sniffer_frozen.load() ? "  [FROZEN]" : "");
    lv_label_set_text(g_stat_lbl, sbuf);

    if (bus.sniffer_frozen.load()) return;

    // Drain a bounded batch from the ring into our newest-first row store.
    can_frame_t f;
    int drained = 0;
    while (drained < 128 && bus.snifferRing().pop(f)) {
        g_head = (g_head + UI_SNIFFER_MAX_ROWS - 1) % UI_SNIFFER_MAX_ROWS;
        format_row(f, g_rows[g_head]);
        if (g_count < UI_SNIFFER_MAX_ROWS) g_count++;
        g_total++;
        drained++;
    }
    if (drained == 0) return;

    // Re-render the visible window (row 0 is the header).
    lv_table_set_row_count(g_table, g_count + 1);
    for (size_t i = 0; i < g_count; ++i) {
        const Row& r = g_rows[(g_head + i) % UI_SNIFFER_MAX_ROWS];
        lv_table_set_cell_value(g_table, i + 1, 0, r.id);
        lv_table_set_cell_value(g_table, i + 1, 1, r.meta);
        lv_table_set_cell_value(g_table, i + 1, 2, r.data);
    }
}
