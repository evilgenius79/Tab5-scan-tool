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

void parseLine(char* line) {
    if (!checksumOk(line)) return;

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

void gpsTask(void*) {
    char    line[128];
    size_t  line_len = 0;
    uint8_t buf[256];

    while (true) {
        // Reconcile track-record requests (open/close happen on this task only).
        if (g_track_want.load() && !g_track_on.load())      trackOpen();
        else if (!g_track_want.load() && g_track_on.load()) trackClose();

        // Blocks up to 200 ms; returns sooner once a chunk has arrived.
        int n = uart_read_bytes(kUart, buf, sizeof(buf), pdMS_TO_TICKS(200));
        if (n > 0) feed(buf, n, line, line_len, sizeof(line));
    }
}

} // namespace

namespace gps {

void start() {
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

} // namespace gps

#else   // GPS_ENABLED == 0

#include <cstdint>
namespace gps {
void        start() {}
void        track_set(bool)   {}
bool        track_active()    { return false; }
uint32_t    track_points()    { return 0; }
const char* track_path()      { return ""; }
}

#endif
