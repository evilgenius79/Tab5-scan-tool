// =============================================================================
//  alerts.cpp - threshold-driven audible warnings via the Tab5 speaker.
// =============================================================================
#include "audio/alerts.h"
#include "app_config.h"
#include "core/event_bus.h"
#include "obd/custom_pids.h"

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <cctype>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp/esp-bsp.h"
#include "esp_codec_dev.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "Alerts";

namespace {

esp_codec_dev_handle_t g_spk     = nullptr;
volatile bool          g_enabled = true;

// ---------------------------------------------------------------------------
//  Low-level PCM playback. Opens the codec for the clip's format, streams the
//  samples, then closes. Blocks for the clip duration - runs on the alert task.
// ---------------------------------------------------------------------------
void playPcm(const uint8_t* data, size_t len, uint32_t rate,
             uint8_t bits, uint8_t ch) {
    if (!g_spk || !data || !len) return;
    esp_codec_dev_sample_info_t fs = {};
    fs.bits_per_sample = bits;
    fs.channel         = ch;
    fs.channel_mask    = 0;
    fs.sample_rate     = rate;
    fs.mclk_multiple   = 0;
    if (esp_codec_dev_open(g_spk, &fs) != 0) {
        ESP_LOGW(TAG, "codec open failed");
        return;
    }
    // esp_codec_dev_write copies the whole buffer (blocking).
    esp_codec_dev_write(g_spk, (void*)data, (int)len);
    esp_codec_dev_close(g_spk);
}

// Synthesized fallback: a short sine tone, 16-bit mono @16 kHz.
void beep(uint32_t freq_hz, uint32_t ms) {
    constexpr uint32_t kRate = 16000;
    const size_t samples = (size_t)kRate * ms / 1000;
    int16_t* buf = (int16_t*)malloc(samples * sizeof(int16_t));
    if (!buf) return;
    const float w = 2.0f * (float)M_PI * freq_hz / kRate;
    for (size_t i = 0; i < samples; ++i) {
        // 60% amplitude with a short fade-out to avoid a click at the end.
        float env = (i > samples - 400) ? (float)(samples - i) / 400.0f : 1.0f;
        buf[i] = (int16_t)(0.6f * 32767.0f * env * sinf(w * i));
    }
    playPcm((uint8_t*)buf, samples * sizeof(int16_t), kRate, 16, 1);
    free(buf);
}

// Read a little-endian integer from a buffer.
uint32_t rd(const uint8_t* p, int n) {
    uint32_t v = 0;
    for (int i = 0; i < n; ++i) v |= (uint32_t)p[i] << (8 * i);
    return v;
}

// Play a PCM WAV file from SD. Returns false if absent/unparseable so the
// caller can fall back to a beep. Parses the RIFF chunks (doesn't assume a
// fixed 44-byte header).
bool playWavFile(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;

    uint8_t hdr[12];
    if (fread(hdr, 1, 12, f) != 12 ||
        memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) {
        fclose(f);
        return false;
    }

    uint32_t rate = 0, data_size = 0;
    uint8_t  bits = 16, ch = 1;
    bool     have_fmt = false, have_data = false;
    uint8_t  cb[8];
    while (fread(cb, 1, 8, f) == 8) {
        uint32_t id = rd(cb, 4);            // chunk id (as LE bytes)
        uint32_t sz = rd(cb + 4, 4);
        if (memcmp(cb, "fmt ", 4) == 0) {
            uint8_t fmt[16];
            uint32_t want = sz < 16 ? sz : 16;
            if (fread(fmt, 1, want, f) != want) break;
            ch   = (uint8_t)rd(fmt + 2, 2);
            rate = rd(fmt + 4, 4);
            bits = (uint8_t)rd(fmt + 14, 2);
            if (sz > want) fseek(f, sz - want, SEEK_CUR);
            have_fmt = true;
        } else if (memcmp(cb, "data", 4) == 0) {
            data_size = sz;
            have_data = true;
            break;                          // data follows; stream it below
        } else {
            fseek(f, sz + (sz & 1), SEEK_CUR);   // skip (chunks are word-aligned)
        }
        (void)id;
    }

    if (!have_fmt || !have_data || rate == 0 || (bits != 16 && bits != 8)) {
        fclose(f);
        return false;
    }

    esp_codec_dev_sample_info_t fs = {};
    fs.bits_per_sample = bits;
    fs.channel         = ch ? ch : 1;
    fs.channel_mask    = 0;
    fs.sample_rate     = rate;
    if (!g_spk || esp_codec_dev_open(g_spk, &fs) != 0) {
        fclose(f);
        return false;
    }

    uint8_t buf[2048];
    uint32_t remaining = data_size;
    while (remaining) {
        size_t want = remaining < sizeof(buf) ? remaining : sizeof(buf);
        size_t got  = fread(buf, 1, want, f);
        if (!got) break;
        esp_codec_dev_write(g_spk, buf, (int)got);
        remaining -= got;
    }
    esp_codec_dev_close(g_spk);
    fclose(f);
    return true;
}

// Find a custom/profile PID value by a name substring (case-insensitive).
bool customByName(const char* sub, float& out) {
    for (size_t i = 0; i < custpid::count(); ++i) {
        const char* nm = custpid::def(i).name;
        // crude case-insensitive contains
        for (const char* s = nm; *s; ++s) {
            const char* a = s; const char* b = sub;
            while (*a && *b && tolower((unsigned char)*a) == tolower((unsigned char)*b)) { ++a; ++b; }
            if (!*b) return custpid::getValue(i, out);
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
//  Alert definitions + edge state.
// ---------------------------------------------------------------------------
struct AlertDef { const char* name; uint32_t beep_hz; uint32_t beep_ms; };
enum { A_KNOCK, A_BOOST, A_COOLANT, A_OIL, A_MIL, A_COUNT };
const AlertDef kAlerts[A_COUNT] = {
    { "knock",   900, 250 },
    { "boost",   700, 400 },
    { "coolant", 500, 600 },
    { "oil",     500, 600 },
    { "mil",     440, 500 },
};
struct EdgeState { bool armed; uint64_t last_us; };
EdgeState g_state[A_COUNT] = {};

void fire(int a) {
    g_state[a].armed   = false;
    g_state[a].last_us = (uint64_t)esp_timer_get_time();
    alerts::play(kAlerts[a].name);
}

// Evaluate one alert: `active` = condition met, `clear` = back below re-arm
// level. Fires on the active edge, respecting cooldown; re-arms once clear.
void evaluate(int a, bool active, bool clear) {
    const uint64_t now = (uint64_t)esp_timer_get_time();
    if (clear) g_state[a].armed = true;
    if (active && g_state[a].armed &&
        (now - g_state[a].last_us) > (uint64_t)ALERT_COOLDOWN_MS * 1000) {
        fire(a);
    }
}

void alertTask(void*) {
    // Start armed; don't fire on the very first sample.
    for (auto& s : g_state) { s.armed = true; s.last_us = 0; }

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(200));
        if (!g_enabled) continue;

        TelemetryState t = EventBus::instance().snapshot();

        // Knock: prefer the enhanced knock-retard custom PID.
        float knock = 0;
        if (customByName("knock", knock))
            evaluate(A_KNOCK, knock >= ALERT_KNOCK_DEG, knock < ALERT_KNOCK_DEG - 1.0f);

        // Boost: prefer the enhanced boost PID, else MAP-derived.
        float boost;
        if (!customByName("boost", boost)) boost = t.boost_psi;
        evaluate(A_BOOST, boost >= ALERT_BOOST_PSI, boost < ALERT_BOOST_PSI - 2.0f);

        const float coolant_f = t.coolant_c * 1.8f + 32.0f;
        evaluate(A_COOLANT, coolant_f >= ALERT_COOLANT_F, coolant_f < ALERT_COOLANT_F - 8.0f);

        const float oil_f = t.oil_temp_c * 1.8f + 32.0f;
        if (t.oil_temp_c != 0)   // only if the PID is actually reporting
            evaluate(A_OIL, oil_f >= ALERT_OIL_F, oil_f < ALERT_OIL_F - 8.0f);

        ReadinessInfo r = EventBus::instance().getReadiness();
        evaluate(A_MIL, r.valid && r.mil_on, r.valid && !r.mil_on);
    }
}

} // namespace

namespace alerts {

void init() {
    if (bsp_audio_init(nullptr) != ESP_OK) {
        ESP_LOGW(TAG, "audio init failed - alerts disabled");
        return;
    }
    g_spk = bsp_audio_codec_speaker_init();
    if (!g_spk) {
        ESP_LOGW(TAG, "speaker codec init failed - alerts disabled");
        return;
    }
    esp_codec_dev_set_out_vol(g_spk, ALERT_VOLUME_PCT);
    xTaskCreatePinnedToCore(alertTask, "alerts", 4096, nullptr,
                            tskIDLE_PRIORITY + 1, nullptr, APP_CORE_IO);
    ESP_LOGI(TAG, "audible alerts ready");
}

void set_enabled(bool on) { g_enabled = on; }
bool enabled()            { return g_enabled; }

void play(const char* name) {
    if (!g_spk || !name) return;
    char path[64];
    snprintf(path, sizeof(path), SD_MOUNT_POINT "/sounds/%s.wav", name);
    if (playWavFile(path)) return;
    // Fallback beep keyed off the alert name.
    for (const auto& a : kAlerts) {
        if (strcmp(a.name, name) == 0) { beep(a.beep_hz, a.beep_ms); return; }
    }
    beep(800, 250);
}

} // namespace alerts
