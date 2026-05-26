// =============================================================================
//  gps.cpp - u-blox SAM-M10Q GNSS reader (I2C / DDC, NMEA-0183).
// -----------------------------------------------------------------------------
//  The module sits on Port A = the Tab5 shared I2C bus. u-blox exposes its
//  message stream over the DDC interface at address 0x42:
//    * register 0xFD/0xFE: 16-bit count of bytes pending in the stream
//    * register 0xFF     : the stream itself (NMEA + UBX); 0xFF == idle filler
//  We drain that stream on a fixed cadence, validate the NMEA checksum, and
//  parse RMC (position/speed/course/date/time/validity) + GGA (altitude, sats,
//  HDOP, fix quality), publishing the accumulated GpsFix into the EventBus.
//
//  The same task owns an optional GPX track log so all file + I2C access stays
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

#include "bsp/esp-bsp.h"
#include "driver/i2c_master.h"
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

i2c_master_dev_handle_t g_dev = nullptr;
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

// --- I2C / DDC transport ----------------------------------------------------

bool readAvail(uint16_t* avail) {
    uint8_t reg = 0xFD, cnt[2] = {0, 0};
    if (i2c_master_transmit_receive(g_dev, &reg, 1, cnt, 2, 100) != ESP_OK)
        return false;
    *avail = (uint16_t)((cnt[0] << 8) | cnt[1]);
    return true;
}

void feed(const uint8_t* buf, int len, char* line, size_t& line_len, size_t cap) {
    for (int i = 0; i < len; ++i) {
        char c = (char)buf[i];
        if (c == (char)0xFF) continue;        // DDC idle filler
        if (c == '\n' || c == '\r') {
            if (line_len > 0) { line[line_len] = '\0'; parseLine(line); line_len = 0; }
        } else if (line_len < cap - 1) {
            line[line_len++] = c;
        } else {
            line_len = 0;                      // overrun: resync on next newline
        }
    }
}

// Build + write a UBX frame to the module (computes the Fletcher checksum).
void sendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
    uint8_t buf[64];
    if (len + 8 > (int)sizeof(buf)) return;
    int p = 0;
    buf[p++] = 0xB5; buf[p++] = 0x62; buf[p++] = cls; buf[p++] = id;
    buf[p++] = (uint8_t)(len & 0xFF); buf[p++] = (uint8_t)(len >> 8);
    for (int i = 0; i < len; ++i) buf[p++] = payload[i];
    uint8_t cka = 0, ckb = 0;
    for (int i = 2; i < p; ++i) { cka += buf[i]; ckb += cka; }
    buf[p++] = cka; buf[p++] = ckb;
    i2c_master_transmit(g_dev, buf, p, 100);
}

// Best-effort: set the M10 measurement rate to 100 ms (10 Hz) via UBX-CFG-VALSET
// in RAM (key CFG-RATE-MEAS = 0x30210001, U2). If the module ignores it, it
// stays at its default rate - GPS timing just runs slower.
void configure10Hz() {
    const uint8_t payload[] = {
        0x00, 0x01, 0x00, 0x00,             // version, layer=RAM, reserved
        0x01, 0x00, 0x21, 0x30,             // key 0x30210001 (little-endian)
        0x64, 0x00                          // value = 100 ms
    };
    sendUbx(0x06, 0x8A, payload, sizeof(payload));
}

void gpsTask(void*) {
    char    line[128];
    size_t  line_len = 0;
    uint8_t reg_ff = 0xFF;
    uint8_t chunk[96];

    configure10Hz();

    while (true) {
        // Reconcile track-record requests (open/close happen on this task only).
        if (g_track_want.load() && !g_track_on.load())      trackOpen();
        else if (!g_track_want.load() && g_track_on.load()) trackClose();

        uint16_t avail = 0;
        if (readAvail(&avail) && avail != 0 && avail != 0xFFFF) {
            while (avail > 0) {
                int want = avail < sizeof(chunk) ? (int)avail : (int)sizeof(chunk);
                if (i2c_master_transmit_receive(g_dev, &reg_ff, 1, chunk, want, 100)
                        != ESP_OK)
                    break;
                feed(chunk, want, line, line_len, sizeof(line));
                avail -= (uint16_t)want;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(GPS_POLL_MS));
    }
}

} // namespace

namespace gps {

void start() {
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (!bus) {
        ESP_LOGE(TAG, "BSP I2C bus not initialized; GPS disabled");
        return;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = GPS_I2C_ADDR;
    dev_cfg.scl_speed_hz    = 400000;        // M10 DDC tops out at 400 kHz

    if (i2c_master_bus_add_device(bus, &dev_cfg, &g_dev) != ESP_OK) {
        ESP_LOGE(TAG, "i2c add device 0x%02X failed; GPS disabled", GPS_I2C_ADDR);
        return;
    }

    if (i2c_master_probe(bus, GPS_I2C_ADDR, 200) == ESP_OK)
        ESP_LOGI(TAG, "SAM-M10Q present at I2C 0x%02X", GPS_I2C_ADDR);
    else
        ESP_LOGW(TAG, "no ACK from I2C 0x%02X - check Port A wiring", GPS_I2C_ADDR);

    xTaskCreatePinnedToCore(gpsTask, "gps", 4096, nullptr,
                            PRIO_SD_LOGGER, nullptr, APP_CORE_IO);
    ESP_LOGI(TAG, "GPS reader started (DDC, %d ms poll)", GPS_POLL_MS);
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
