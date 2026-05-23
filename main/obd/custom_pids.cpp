// =============================================================================
//  custom_pids.cpp - load + store manufacturer/user PID definitions.
// =============================================================================
#include "obd/custom_pids.h"
#include "app_config.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char* TAG = "CustomPid";

namespace {

constexpr size_t MAX_CUSTOM = 16;
CustomPid         g_defs[MAX_CUSTOM];
float             g_values[MAX_CUSTOM];
bool              g_valid[MAX_CUSTOM];
size_t            g_count  = 0;
const char*       g_source = "none";
SemaphoreHandle_t g_mtx    = nullptr;
bool              g_loaded = false;

// Built-in Ford EcoBoost defaults (COMMUNITY FORScan values - verify per car).
// Fields: name, request, bytes, signed, scale, offset, unit.
const CustomPid kFordDefaults[] = {
    { "Knock Retard",    "220318", 0,0, 1, true,  0.0625f,   0.0f, "deg" },
    { "Charge Air Temp", "22F40F", 0,0, 1, false, 1.0f,    -40.0f, "C"   },
    { "Turbo Boost",     "220466", 0,0, 2, false, 0.0011328f, -14.7f, "psi" },
};

// Parse the request string's leading mode + pid (e.g. "220318" -> 0x22,0x0318).
void parseRequest(CustomPid& p) {
    char m[3] = { p.request[0], p.request[1], 0 };
    p.mode = (uint8_t)strtoul(m, nullptr, 16);
    p.pid  = (uint16_t)strtoul(p.request + 2, nullptr, 16);
}

void addDef(const CustomPid& src) {
    if (g_count >= MAX_CUSTOM) return;
    g_defs[g_count] = src;
    parseRequest(g_defs[g_count]);
    g_valid[g_count] = false;
    g_count++;
}

// Parse one CSV line into a CustomPid. Returns false on comment/blank/malformed.
bool parseCsvLine(char* line) {
    while (*line == ' ' || *line == '\t') ++line;
    if (*line == '#' || *line == '\0' || *line == '\r' || *line == '\n') return false;

    char* tok[7]; int n = 0;
    for (char* s = strtok(line, ","); s && n < 7; s = strtok(nullptr, ",")) tok[n++] = s;
    if (n < 7) return false;

    CustomPid p{};
    snprintf(p.name, sizeof(p.name), "%s", tok[0]);
    snprintf(p.request, sizeof(p.request), "%s", tok[1]);
    p.bytes     = (uint8_t)atoi(tok[2]);
    p.is_signed = atoi(tok[3]) != 0;
    p.scale     = strtof(tok[4], nullptr);
    p.offset    = strtof(tok[5], nullptr);
    snprintf(p.unit, sizeof(p.unit), "%s", tok[6]);
    // strip trailing CR/LF/space from unit
    for (char* c = p.unit; *c; ++c) if (*c=='\r'||*c=='\n') { *c = 0; break; }
    addDef(p);
    return true;
}

bool loadFromSd() {
    FILE* f = fopen(SD_MOUNT_POINT "/custom_pids.csv", "r");
    if (!f) return false;
    char line[160];
    while (fgets(line, sizeof(line), f)) parseCsvLine(line);
    fclose(f);
    return g_count > 0;
}

} // namespace

namespace custpid {

void load() {
    if (g_loaded) return;
    g_loaded = true;
    if (!g_mtx) g_mtx = xSemaphoreCreateMutex();

    if (loadFromSd()) {
        g_source = "SD";
    } else {
        for (const auto& d : kFordDefaults) addDef(d);
        g_source = "Ford defaults";
    }
    ESP_LOGI(TAG, "loaded %u custom PIDs (%s)", (unsigned)g_count, g_source);
}

size_t           count()        { return g_count; }
const CustomPid& def(size_t i)  { return g_defs[i]; }
const char*      source()       { return g_source; }

void setValue(size_t i, float v) {
    if (i >= g_count || !g_mtx) return;
    if (xSemaphoreTake(g_mtx, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_values[i] = v; g_valid[i] = true;
        xSemaphoreGive(g_mtx);
    }
}

bool getValue(size_t i, float& out) {
    if (i >= g_count || !g_mtx) return false;
    bool ok = false;
    if (xSemaphoreTake(g_mtx, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (g_valid[i]) { out = g_values[i]; ok = true; }
        xSemaphoreGive(g_mtx);
    }
    return ok;
}

} // namespace custpid
