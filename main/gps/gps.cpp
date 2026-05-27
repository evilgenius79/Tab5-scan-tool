// =============================================================================
//  gps.cpp - u-blox SAM-M10Q GNSS reader (UART, NMEA-0183).
// -----------------------------------------------------------------------------
//  The module is wired to Port A (GPIO53/54) as a UART NMEA device. We read the
//  byte stream, validate the NMEA checksum, and parse RMC (position / speed /
//  course / date / time / validity) + GGA (altitude, sats, HDOP, fix quality),
//  publishing the accumulated GpsFix into the EventBus.
//
//  The same task owns an optional GPX track log so all file + UART access stays
//  single-threaded on the I/O core (the UI only flips an atomic request flag).
//
//  NMEA fields can be empty (",,"), so we split manually rather than with
//  strtok_r, which would collapse consecutive commas and misalign the columns.
// =============================================================================
#include "gps/gps.h"
#include "app_config.h"

#if GPS_ENABLED

#include "core/event_bus.h"
#include "logging/sd_logger.h"   // sd_card_ensure_mounted()

#include "bsp/esp-bsp.h"         // bsp_io_expander_init + EXT5V enable
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>

static const char* TAG = "GPS";

namespace {

constexpr uart_port_t kUart = (uart_port_t)GPS_UART_NUM;

GpsFix g_fix{};   // accumulated across RMC/GGA; republished after each sentence

// --- GPX track recording state (touched only by the GPS task, except the
//     atomics, which the UI reads/writes) ----------------------------------
std::atomic<bool>     g_track_want{false};   // UI request
std::atomic<bool>     g_track_on{false};     // file actually open
std::atomic<uint32_t> g_track_pts{0};
FILE*    g_track_fp   = nullptr;
char     g_track_file[64] = {0};
uint64_t g_track_last_us = 0;

// --- NMEA parsing -----------------------------------------------------------

// XOR checksum of the bytes between '$' and '*'. Returns true if it matches the
// two hex digits after '*'. Sentences without a checksum are rejected.
bool checksumOk(const char* s) {
    if (s[0] != '$') return false;
    uint8_t sum = 0;
    size_t i = 1;
    for (; s[i] && s[i] != '*'; ++i) sum ^= (uint8_t)s[i];
    if (s[i] != '*') return false;
    char hex[3] = { s[i + 1], s[i + 2], 0 };
    if (!hex[0] || !hex[1]) return false;
    return (uint8_t)strtol(hex, nullptr, 16) == sum;
}

// Split a mutable NMEA body into comma-separated fields (preserving empties).
// Stops at '*' (checksum). Returns the field count. fields[] point into buf.
int splitFields(char* buf, char** fields, int max) {
    int n = 0;
    fields[n++] = buf;
    for (char* p = buf; *p; ++p) {
        if (*p == '*') { *p = '\0'; break; }
        if (*p == ',') {
            *p = '\0';
            if (n < max) fields[n++] = p + 1;
        }
    }
    return n;
}

// "ddmm.mmmm" + hemisphere -> signed decimal degrees. Empty field -> 0.
double nmeaCoord(const char* v, const char* hemi, bool* ok) {
    if (!v || !*v) return 0.0;
    double raw = atof(v);
    double deg = (double)((int)(raw / 100.0));   // leading dd / ddd
    double min = raw - deg * 100.0;
    double dec = deg + min / 60.0;
    if (hemi && (*hemi == 'S' || *hemi == 'W')) dec = -dec;
    *ok = true;
    return dec;
}

void parseTime(const char* v, GpsFix& f) {
    if (!v || strlen(v) < 6) return;
    f.utc_h = (uint8_t)((v[0] - '0') * 10 + (v[1] - '0'));
    f.utc_m = (uint8_t)((v[2] - '0') * 10 + (v[3] - '0'));
    f.utc_s = (uint8_t)((v[4] - '0') * 10 + (v[5] - '0'));
}

void parseDate(const char* v, GpsFix& f) {   // RMC date field "ddmmyy"
    if (!v || strlen(v) < 6) return;
    f.utc_day  = (uint8_t)((v[0] - '0') * 10 + (v[1] - '0'));
    f.utc_mon  = (uint8_t)((v[2] - '0') * 10 + (v[3] - '0'));
    f.utc_year = (uint16_t)(2000 + (v[4] - '0') * 10 + (v[5] - '0'));
}

// Match the 3-char sentence type after the 2-char talker id (GP/GN/GL/GA/...).
// talker_sentence is the first field WITHOUT the leading '$', e.g. "GNRMC".
bool isType(const char* talker_sentence, const char* type3) {
    size_t n = strlen(talker_sentence);
    if (n < 5) return false;
    return strncmp(talker_sentence + (n - 3), type3, 3) == 0;
}

// --- GPX track log (all calls on the GPS task) ------------------------------
void trackOpen() {
    if (!sd_card_ensure_mounted()) { g_track_want.store(false); return; }
    uint64_t ms = esp_timer_get_time() / 1000;
    snprintf(g_track_file, sizeof(g_track_file), SD_MOUNT_POINT "/track-%llu.gpx",
             (unsigned long long)ms);
    g_track_fp = fopen(g_track_file, "w");
    if (!g_track_fp) {
        ESP_LOGW(TAG, "could not open %s", g_track_file);
        g_track_want.store(false);
        return;
    }
    fprintf(g_track_fp,
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<gpx version=\"1.1\" creator=\"Tab5 Scan Tool\" "
        "xmlns=\"http://www.topografix.com/GPX/1/1\">\n<trk><trkseg>\n");
    g_track_pts.store(0);
    g_track_last_us = 0;
    g_track_on.store(true);
    ESP_LOGI(TAG, "track recording -> %s", g_track_file);
}

void trackClose() {
    if (g_track_fp) {
        fprintf(g_track_fp, "</trkseg></trk></gpx>\n");
        fclose(g_track_fp);
        g_track_fp = nullptr;
        ESP_LOGI(TAG, "track closed (%u pts)", (unsigned)g_track_pts.load());
    }
    g_track_on.store(false);
}

void trackAppend(const GpsFix& f) {
    if (!g_track_fp || !f.has_fix) return;
    uint64_t now = esp_timer_get_time();
    if (g_track_last_us && now - g_track_last_us < 900000ULL) return;  // ~1 Hz
    g_track_last_us = now;
    char tbuf[40] = {0};
    if (f.utc_year)
        snprintf(tbuf, sizeof(tbuf),
                 "<time>%04u-%02u-%02uT%02u:%02u:%02uZ</time>",
                 (unsigned)f.utc_year, (unsigned)f.utc_mon, (unsigned)f.utc_day,
                 (unsigned)f.utc_h, (unsigned)f.utc_m, (unsigned)f.utc_s);
    fprintf(g_track_fp, "<trkpt lat=\"%.6f\" lon=\"%.6f\"><ele>%.1f</ele>%s</trkpt>\n",
            f.lat, f.lon, f.alt_m, tbuf);
    uint32_t n = g_track_pts.load() + 1;
    g_track_pts.store(n);
    if ((n & 0x0F) == 0) fflush(g_track_fp);   // commit every 16 points
}

unsigned g_nmea_ok = 0;   // count of checksum-valid NMEA sentences (diagnostic)
unsigned g_fix_seq = 0;   // ++ per published RMC fix (for rate measurement)
std::atomic<uint32_t> g_link_baud{GPS_BAUD};
std::atomic<float>    g_fix_hz{0.0f};

void parseLine(char* line) {
    if (!checksumOk(line)) return;
    ++g_nmea_ok;

    char* fields[24];
    int n = splitFields(line + 1, fields, 24);   // skip '$'
    if (n < 1) return;

    if (isType(fields[0], "RMC") && n >= 10) {
        // $..RMC,time,status,lat,N/S,lon,E/W,speed_kn,course,date,...
        parseTime(fields[1], g_fix);
        bool valid_fix = (fields[2][0] == 'A');
        g_fix.has_fix = valid_fix;
        if (valid_fix) {
            bool a = false, b = false;
            g_fix.lat = nmeaCoord(fields[3], fields[4], &a);
            g_fix.lon = nmeaCoord(fields[5], fields[6], &b);
            g_fix.speed_kph  = (float)(atof(fields[7]) * 1.852);   // knots -> kph
            g_fix.course_deg = (float)atof(fields[8]);
        }
        parseDate(fields[9], g_fix);
        g_fix.valid = true;
        EventBus::instance().setGps(g_fix);
        EventBus::instance().gps_present.store(true);
        ++g_fix_seq;
        trackAppend(g_fix);
    } else if (isType(fields[0], "GGA") && n >= 10) {
        // $..GGA,time,lat,N/S,lon,E/W,fixqual,numsat,hdop,alt,M,...
        int fixqual = atoi(fields[6]);
        g_fix.sats  = (uint8_t)atoi(fields[7]);
        g_fix.hdop  = (float)atof(fields[8]);
        g_fix.alt_m = (float)atof(fields[9]);
        if (fixqual > 0) g_fix.has_fix = true;
        g_fix.valid = true;
        EventBus::instance().setGps(g_fix);
        EventBus::instance().gps_present.store(true);
    }
}

// --- UART transport ---------------------------------------------------------

void feed(const uint8_t* buf, int len, char* line, size_t& line_len, size_t cap) {
    for (int i = 0; i < len; ++i) {
        char c = (char)buf[i];
        if (c == '\n' || c == '\r') {
            if (line_len > 0) { line[line_len] = '\0'; parseLine(line); line_len = 0; }
        } else if (line_len < cap - 1) {
            line[line_len++] = c;
        } else {
            line_len = 0;                      // overrun: resync on next newline
        }
    }
}

#if GPS_AUTOCONFIG
// Frame + write a UBX message over the UART (8-bit Fletcher checksum).
void sendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
    uint8_t hdr[6] = {0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    uint8_t cka = 0, ckb = 0;
    for (int i = 2; i < 6; ++i)  { cka += hdr[i];     ckb += cka; }
    for (int i = 0; i < len; ++i){ cka += payload[i]; ckb += cka; }
    uint8_t tail[2] = {cka, ckb};
    uart_write_bytes(kUart, (const char*)hdr, 6);
    uart_write_bytes(kUart, (const char*)payload, len);
    uart_write_bytes(kUart, (const char*)tail, 2);
    uart_wait_tx_done(kUart, pdMS_TO_TICKS(100));
}

// Wait for a UBX-ACK-ACK(+1) / ACK-NAK(-1) to a CFG message; 0 if none arrive.
// Confirms the module actually received and accepted our command (i.e. our TX
// line reaches the module's RX), which a baud change alone can't verify.
int waitUbxAck(int ms) {
    uint64_t end = esp_timer_get_time() + (uint64_t)ms * 1000;
    int st = 0; uint8_t id = 0, b = 0;
    while (esp_timer_get_time() < end) {
        if (uart_read_bytes(kUart, &b, 1, pdMS_TO_TICKS(20)) <= 0) continue;
        switch (st) {
        case 0: st = (b == 0xB5) ? 1 : 0; break;
        case 1: st = (b == 0x62) ? 2 : 0; break;
        case 2: st = (b == 0x05) ? 3 : 0; break;          // ACK class
        case 3: id = b; st = (b == 0x01 || b == 0x00) ? 4 : 0; break;
        case 4: st = (b == 0x02) ? 5 : 0; break;          // len = 2
        case 5: st = (b == 0x00) ? 6 : 0; break;
        case 6: st = (b == 0x06) ? 7 : 0; break;          // acked class = CFG
        case 7: return id == 0x01 ? 1 : -1;               // acked msg id byte
        }
    }
    return 0;
}

// True if NMEA ("$G"/"$P") is seen within `ms` at the current baud.
bool sawNmea(int ms) {
    uint64_t end = esp_timer_get_time() + (uint64_t)ms * 1000;
    uint8_t buf[128];
    bool dollar = false;
    while (esp_timer_get_time() < end) {
        int r = uart_read_bytes(kUart, buf, sizeof(buf), pdMS_TO_TICKS(40));
        for (int i = 0; i < r; ++i) {
            if (buf[i] == '$') dollar = true;
            else if (dollar && (buf[i] == 'G' || buf[i] == 'P')) return true;
        }
    }
    return false;
}

void valsetMeas(uint16_t ms) {   // CFG-RATE-MEAS (+ CFG-RATE-NAV = 1), RAM layer
    uint8_t p[] = {0x00, 0x01, 0x00, 0x00,
                   0x01, 0x00, 0x21, 0x30, (uint8_t)ms, (uint8_t)(ms >> 8),
                   0x02, 0x00, 0x21, 0x30, 0x01, 0x00};
    sendUbx(0x06, 0x8A, p, sizeof(p));
}

void valsetBaud(uint32_t b) {    // CFG-UART1-BAUDRATE (U4), RAM layer
    uint8_t p[] = {0x00, 0x01, 0x00, 0x00,
                   0x01, 0x00, 0x52, 0x40,
                   (uint8_t)b, (uint8_t)(b >> 8), (uint8_t)(b >> 16), (uint8_t)(b >> 24)};
    sendUbx(0x06, 0x8A, p, sizeof(p));
}

// Detect the module's current baud, verify our TX reaches it (ACK), then raise
// the baud and set the target rate. Ordering matters: baud first, rate second,
// so we never command 10 Hz on a 9600 link (which would flood / truncate).
void gpsConfigure() {
    static const uint32_t cands[2] = { GPS_BAUD, GPS_FACTORY_BAUD };
    uint32_t cur = 0;
    for (uint32_t b : cands) {
        uart_set_baudrate(kUart, b);
        uart_flush_input(kUart);
        if (sawNmea(500)) { cur = b; break; }
    }
    if (!cur) {
        ESP_LOGW(TAG, "config: no NMEA at %d or %d baud - skipping",
                 GPS_BAUD, GPS_FACTORY_BAUD);
        uart_set_baudrate(kUart, GPS_BAUD);
        return;
    }
    uart_set_baudrate(kUart, cur);

    // TX health check: a harmless 1 Hz command, and see if the module ACKs it.
    uart_flush_input(kUart);
    valsetMeas(1000);
    int ack = waitUbxAck(400);
    ESP_LOGI(TAG, "config: module at %u baud, command ack=%s", (unsigned)cur,
             ack > 0 ? "ACK" : ack < 0 ? "NAK" : "NONE");
    if (ack == 0)
        ESP_LOGW(TAG, "module not acknowledging commands - check TX wiring "
                      "(GPIO%d -> module RX); rate/baud config will not apply",
                 GPS_UART_TX_PIN);

    // Raise baud, then set the real rate at the new baud.
    if (cur != GPS_BAUD) {
        valsetBaud(GPS_BAUD);
        vTaskDelay(pdMS_TO_TICKS(150));
        uart_set_baudrate(kUart, GPS_BAUD);
    }
    uart_flush_input(kUart);
    valsetMeas(1000 / GPS_NAV_RATE_HZ);
    waitUbxAck(400);
    uart_flush_input(kUart);
    ESP_LOGI(TAG, "config: requested %d Hz @ %d baud", GPS_NAV_RATE_HZ, GPS_BAUD);
}
#endif

void gpsTask(void*) {
    char    line[128];
    size_t  line_len = 0;
    uint8_t buf[256];

#if GPS_AUTOCONFIG
    gpsConfigure();
#endif
    // Auto-baud: if no valid NMEA arrives, cycle target <-> factory baud.
    const uint32_t kBauds[] = { GPS_BAUD, GPS_FACTORY_BAUD };
    size_t baud_idx = 0;
    g_link_baud.store(kBauds[baud_idx]);

    // Fix-rate measurement (published over the 1 s window).
    uint64_t hz_t0   = esp_timer_get_time();
    unsigned hz_seq0 = 0;

    // RX diagnostic: every 5 s report bytes received vs valid sentences parsed,
    // so a wiring fault (0 bytes) is distinguishable from a baud mismatch
    // (bytes arrive but none checksum-valid). Drops out once GPS is healthy.
    uint64_t next_diag = esp_timer_get_time() + 5000000ULL;
    uint32_t rx_window = 0;
    char     sample[48] = {0};

    while (true) {
        // Reconcile track-record requests (open/close happen on this task only).
        if (g_track_want.load() && !g_track_on.load())      trackOpen();
        else if (!g_track_want.load() && g_track_on.load()) trackClose();

        // Blocks up to 200 ms; returns sooner once a chunk has arrived.
        int n = uart_read_bytes(kUart, buf, sizeof(buf), pdMS_TO_TICKS(200));
        if (n > 0) {
            rx_window += (uint32_t)n;
            if (sample[0] == 0) {            // capture one printable sample/window
                size_t k = 0;
                for (int i = 0; i < n && k < sizeof(sample) - 1; ++i)
                    sample[k++] = (buf[i] >= 32 && buf[i] < 127) ? (char)buf[i] : '.';
                sample[k] = 0;
            }
            feed(buf, n, line, line_len, sizeof(line));
        }

        uint64_t now = esp_timer_get_time();
        if (now - hz_t0 >= 1000000ULL) {            // publish measured fix rate
            g_fix_hz.store((g_fix_seq - hz_seq0) * 1e6f / (now - hz_t0));
            hz_seq0 = g_fix_seq;
            hz_t0   = now;
        }
        if (g_nmea_ok < 4 && now >= next_diag) {    // quiet once clearly working
            next_diag = now + 5000000ULL;
            if (g_nmea_ok > 0) {
                ESP_LOGI(TAG, "rx OK: %u valid NMEA sentences so far", g_nmea_ok);
            } else if (rx_window > 0) {
                // Bytes arrive but none parse -> wrong baud; try the next one.
                baud_idx = (baud_idx + 1) % (sizeof(kBauds) / sizeof(kBauds[0]));
                uart_set_baudrate(kUart, kBauds[baud_idx]);
                g_link_baud.store(kBauds[baud_idx]);
                ESP_LOGW(TAG, "rx %u bytes/5s but 0 valid NMEA (sample: \"%s\") - "
                              "retrying at %u baud", (unsigned)rx_window, sample,
                         (unsigned)kBauds[baud_idx]);
            } else {
                ESP_LOGW(TAG, "rx 0 bytes on GPIO%d - no data; check wiring or "
                              "swap RX<->TX (set GPS_UART_RX_PIN=%d)",
                         GPS_UART_RX_PIN, GPS_UART_TX_PIN);
            }
            rx_window = 0;
            sample[0] = 0;
        }
    }
}

} // namespace

namespace gps {

// Drive EXT5V_EN (PI4IOE5V6408 #1 @0x43, pin 2, active-high) so Port A gets 5V.
void enableExt5v() {
    esp_io_expander_handle_t exp = bsp_io_expander_init();
    if (!exp) { ESP_LOGW(TAG, "IO expander init failed; Port A may stay unpowered"); return; }
    esp_io_expander_set_dir(exp, IO_EXPANDER_PIN_NUM_2, IO_EXPANDER_OUTPUT);
    esp_io_expander_set_output_mode(exp, IO_EXPANDER_PIN_NUM_2, IO_EXPANDER_OUTPUT_MODE_PUSH_PULL);
    esp_io_expander_set_level(exp, IO_EXPANDER_PIN_NUM_2, 1);
    ESP_LOGI(TAG, "EXT_5V enabled (Port A power on)");
}

void start() {
#if GPS_ENABLE_EXT5V
    enableExt5v();
#endif
    uart_config_t cfg = {};
    cfg.baud_rate  = GPS_BAUD;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_driver_install(kUart, 2048, 0, 0, nullptr, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed (%s); GPS disabled",
                 esp_err_to_name(err));
        return;
    }
    uart_param_config(kUart, &cfg);
    uart_set_pin(kUart, GPS_UART_TX_PIN, GPS_UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    xTaskCreatePinnedToCore(gpsTask, "gps", 4096, nullptr,
                            PRIO_SD_LOGGER, nullptr, APP_CORE_IO);
    ESP_LOGI(TAG, "GPS reader started (UART%d, RX=%d TX=%d, %d baud)",
             GPS_UART_NUM, GPS_UART_RX_PIN, GPS_UART_TX_PIN, GPS_BAUD);
}

void        track_set(bool on) { g_track_want.store(on); }
bool        track_active()     { return g_track_on.load(); }
uint32_t    track_points()     { return g_track_pts.load(); }
const char* track_path()       { return g_track_file; }
uint32_t    link_baud()        { return g_link_baud.load(); }
float       fix_hz()           { return g_fix_hz.load(); }

} // namespace gps

#else   // GPS_ENABLED == 0

#include <cstdint>
namespace gps {
void        start() {}
void        track_set(bool)   {}
bool        track_active()    { return false; }
uint32_t    track_points()    { return 0; }
const char* track_path()      { return ""; }
uint32_t    link_baud()       { return 0; }
float       fix_hz()          { return 0.0f; }
}

#endif
