// =============================================================================
//  dtc_lookup.cpp - hybrid (flash + SD) DTC description lookup.
// =============================================================================
#include "obd/dtc_lookup.h"
#include "app_config.h"            // SD_MOUNT_POINT
#include "logging/sd_logger.h"     // sd_card_ensure_mounted()

#include <cstdio>
#include <cstring>
#include <cctype>
#include <algorithm>

#include "esp_heap_caps.h"
#include "esp_log.h"

static const char* TAG = "DtcLookup";

namespace {

// ---------------------------------------------------------------------------
//  Embedded generic SAE J1979 codes. Curated common subset - works with no SD
//  card. The long tail and all manufacturer-specific codes live in the SD CSV.
//  Kept sorted by code so a binary search would work, but the table is small
//  enough that a linear scan is fine.
// ---------------------------------------------------------------------------
struct Entry { const char* code; const char* desc; };

constexpr Entry kGeneric[] = {
    { "P0010", "Camshaft Position Actuator A Circuit (Bank 1)" },
    { "P0011", "Camshaft Position - Timing Over-Advanced (Bank 1)" },
    { "P0014", "Camshaft Position B - Timing Over-Advanced (Bank 1)" },
    { "P0016", "Crankshaft/Camshaft Position Correlation (Bank 1 Sensor A)" },
    { "P0017", "Crankshaft/Camshaft Position Correlation (Bank 1 Sensor B)" },
    { "P0030", "HO2S Heater Control Circuit (Bank 1 Sensor 1)" },
    { "P0070", "Ambient Air Temperature Sensor Circuit" },
    { "P0087", "Fuel Rail/System Pressure Too Low" },
    { "P0088", "Fuel Rail/System Pressure Too High" },
    { "P0089", "Fuel Pressure Regulator Performance" },
    { "P008A", "Low Pressure Fuel System Pressure Too Low" },
    { "P008B", "Low Pressure Fuel System Pressure Too High" },
    { "P0096", "Intake Air Temperature Sensor 2 Circuit Range/Performance" },
    { "P0100", "Mass or Volume Air Flow Circuit" },
    { "P0101", "Mass Air Flow Circuit Range/Performance" },
    { "P0102", "Mass Air Flow Circuit Low Input" },
    { "P0103", "Mass Air Flow Circuit High Input" },
    { "P0106", "MAP/Barometric Pressure Circuit Range/Performance" },
    { "P0107", "MAP/Barometric Pressure Circuit Low Input" },
    { "P0108", "MAP/Barometric Pressure Circuit High Input" },
    { "P0111", "Intake Air Temperature Circuit Range/Performance" },
    { "P0112", "Intake Air Temperature Circuit Low Input" },
    { "P0113", "Intake Air Temperature Circuit High Input" },
    { "P0116", "Engine Coolant Temperature Circuit Range/Performance" },
    { "P0117", "Engine Coolant Temperature Circuit Low Input" },
    { "P0118", "Engine Coolant Temperature Circuit High Input" },
    { "P0120", "Throttle/Pedal Position Sensor A Circuit" },
    { "P0121", "Throttle/Pedal Position Sensor A Range/Performance" },
    { "P0128", "Coolant Thermostat (Below Regulating Temperature)" },
    { "P0130", "O2 Sensor Circuit (Bank 1 Sensor 1)" },
    { "P0131", "O2 Sensor Circuit Low Voltage (Bank 1 Sensor 1)" },
    { "P0132", "O2 Sensor Circuit High Voltage (Bank 1 Sensor 1)" },
    { "P0133", "O2 Sensor Circuit Slow Response (Bank 1 Sensor 1)" },
    { "P0134", "O2 Sensor Circuit No Activity Detected (Bank 1 Sensor 1)" },
    { "P0135", "O2 Sensor Heater Circuit (Bank 1 Sensor 1)" },
    { "P0137", "O2 Sensor Circuit Low Voltage (Bank 1 Sensor 2)" },
    { "P0140", "O2 Sensor Circuit No Activity Detected (Bank 1 Sensor 2)" },
    { "P0171", "System Too Lean (Bank 1)" },
    { "P0172", "System Too Rich (Bank 1)" },
    { "P0174", "System Too Lean (Bank 2)" },
    { "P0175", "System Too Rich (Bank 2)" },
    { "P0182", "Fuel Temperature Sensor A Circuit Low" },
    { "P0190", "Fuel Rail Pressure Sensor Circuit" },
    { "P0201", "Injector Circuit/Open - Cylinder 1" },
    { "P0217", "Engine Coolant Over Temperature Condition" },
    { "P0219", "Engine Overspeed Condition" },
    { "P0234", "Turbocharger/Supercharger Overboost Condition" },
    { "P0236", "Turbocharger Boost Sensor A Circuit Range/Performance" },
    { "P0238", "Turbocharger/Supercharger Boost Sensor A Circuit High" },
    { "P0299", "Turbocharger/Supercharger Underboost Condition" },
    { "P0300", "Random/Multiple Cylinder Misfire Detected" },
    { "P0301", "Cylinder 1 Misfire Detected" },
    { "P0302", "Cylinder 2 Misfire Detected" },
    { "P0303", "Cylinder 3 Misfire Detected" },
    { "P0304", "Cylinder 4 Misfire Detected" },
    { "P0305", "Cylinder 5 Misfire Detected" },
    { "P0306", "Cylinder 6 Misfire Detected" },
    { "P0307", "Cylinder 7 Misfire Detected" },
    { "P0308", "Cylinder 8 Misfire Detected" },
    { "P0325", "Knock Sensor 1 Circuit (Bank 1)" },
    { "P0327", "Knock Sensor 1 Circuit Low (Bank 1)" },
    { "P0335", "Crankshaft Position Sensor A Circuit" },
    { "P0336", "Crankshaft Position Sensor A Range/Performance" },
    { "P0340", "Camshaft Position Sensor A Circuit (Bank 1)" },
    { "P0341", "Camshaft Position Sensor A Range/Performance (Bank 1)" },
    { "P0351", "Ignition Coil A Primary/Secondary Circuit" },
    { "P0400", "Exhaust Gas Recirculation Flow" },
    { "P0401", "Exhaust Gas Recirculation Flow Insufficient" },
    { "P0420", "Catalyst System Efficiency Below Threshold (Bank 1)" },
    { "P0430", "Catalyst System Efficiency Below Threshold (Bank 2)" },
    { "P0440", "Evaporative Emission System" },
    { "P0442", "Evaporative Emission System Leak Detected (Small Leak)" },
    { "P0443", "Evaporative Emission System Purge Control Valve Circuit" },
    { "P0446", "Evaporative Emission System Vent Control Circuit" },
    { "P0455", "Evaporative Emission System Leak Detected (Large Leak)" },
    { "P0456", "Evaporative Emission System Leak Detected (Very Small Leak)" },
    { "P0500", "Vehicle Speed Sensor A" },
    { "P0505", "Idle Air Control System" },
    { "P0506", "Idle Air Control System RPM Lower Than Expected" },
    { "P0507", "Idle Air Control System RPM Higher Than Expected" },
    { "P0520", "Engine Oil Pressure Sensor/Switch Circuit" },
    { "P0562", "System Voltage Low" },
    { "P0563", "System Voltage High" },
    { "P0600", "Serial Communication Link" },
    { "P0601", "Internal Control Module Memory Check Sum Error" },
    { "P0606", "ECM/PCM Processor" },
    { "P0700", "Transmission Control System (MIL Request)" },
    { "P0715", "Input/Turbine Speed Sensor Circuit" },
    { "P0720", "Output Speed Sensor Circuit" },
    { "P0730", "Incorrect Gear Ratio" },
    { "P2196", "O2 Sensor Signal Biased/Stuck Rich (Bank 1 Sensor 1)" },
    { "P2270", "O2 Sensor Signal Stuck Lean (Bank 1 Sensor 2)" },
    { "U0073", "Control Module Communication Bus A Off" },
    { "U0100", "Lost Communication With ECM/PCM A" },
    { "U0101", "Lost Communication With TCM" },
    { "U0121", "Lost Communication With ABS Control Module" },
    { "U0140", "Lost Communication With Body Control Module" },
    { "U0155", "Lost Communication With Instrument Panel Cluster (IPC)" },
    { "U0401", "Invalid Data Received From ECM/PCM A" },
    { "U0415", "Invalid Data Received From ABS Control Module" },
    { "C0035", "Left Front Wheel Speed Sensor Circuit" },
    { "C0040", "Right Front Wheel Speed Sensor Circuit" },
    { "B0001", "Driver Frontal Stage 1 Deployment Control" },
};
constexpr size_t kGenericCount = sizeof(kGeneric) / sizeof(kGeneric[0]);

const char* findGeneric(const char* code) {
    for (const auto& e : kGeneric) {
        if (std::strcmp(e.code, code) == 0) return e.desc;
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
//  SD-loaded database. The whole CSV is read into one PSRAM buffer; the index
//  points into it (code/desc are null-terminated in place), so we make few
//  allocations and keep internal SRAM free.
// ---------------------------------------------------------------------------
constexpr size_t kSdMaxEntries = 16000;  // full Ford DTC DB is ~9.2k codes
constexpr size_t kSdMaxFileBytes = 4 * 1024 * 1024;   // sanity cap

char*  g_buf = nullptr;
Entry* g_idx = nullptr;
size_t g_idxCount = 0;
bool   g_sdResolved = false;   // true once we've definitively loaded or given up

void* psramAlloc(size_t bytes) {
    void* p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
    return p ? p : heap_caps_malloc(bytes, MALLOC_CAP_DEFAULT);
}

const char* findSd(const char* code) {
    if (!g_idx || g_idxCount == 0) return nullptr;
    size_t lo = 0, hi = g_idxCount;          // binary search; g_idx is sorted
    while (lo < hi) {
        size_t mid = lo + (hi - lo) / 2;
        int c = std::strcmp(code, g_idx[mid].code);
        if (c == 0) return g_idx[mid].desc;
        if (c < 0) hi = mid; else lo = mid + 1;
    }
    return nullptr;
}

void loadSdDatabase() {
    FILE* f = std::fopen(SD_MOUNT_POINT "/dtc_db.csv", "rb");
    if (!f) {
        ESP_LOGI(TAG, "no %s/dtc_db.csv - using embedded table only",
                 SD_MOUNT_POINT);
        return;   // card mounted but no file: nothing to add
    }

    std::fseek(f, 0, SEEK_END);
    long sz = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (sz <= 0 || (size_t)sz > kSdMaxFileBytes) {
        ESP_LOGW(TAG, "dtc_db.csv size %ld out of range - skipping", sz);
        std::fclose(f);
        return;
    }

    g_buf = static_cast<char*>(psramAlloc((size_t)sz + 1));
    Entry* idx = static_cast<Entry*>(psramAlloc(kSdMaxEntries * sizeof(Entry)));
    if (!g_buf || !idx) {
        ESP_LOGE(TAG, "out of memory loading dtc_db.csv");
        std::fclose(f);
        std::free(g_buf); g_buf = nullptr; std::free(idx);
        return;
    }

    size_t n = std::fread(g_buf, 1, (size_t)sz, f);
    std::fclose(f);
    g_buf[n] = '\0';

    // Parse line by line, splitting on the first comma. Tokenise in place.
    size_t count = 0;
    char* p = g_buf;
    while (*p && count < kSdMaxEntries) {
        char* line = p;
        while (*p && *p != '\n') ++p;
        if (*p == '\n') *p++ = '\0';

        // Trim trailing CR and leading whitespace.
        char* end = line + std::strlen(line);
        while (end > line && (end[-1] == '\r' || end[-1] == ' ')) *--end = '\0';
        while (*line == ' ' || *line == '\t') ++line;
        if (*line == '\0' || *line == '#') continue;

        char* comma = std::strchr(line, ',');
        if (!comma) continue;
        *comma = '\0';
        char* desc = comma + 1;
        while (*desc == ' ' || *desc == '\t') ++desc;
        if (*line == '\0' || *desc == '\0') continue;

        // Normalise the code to upper case so lookups (always upper) match.
        for (char* c = line; *c; ++c) *c = (char)std::toupper((unsigned char)*c);

        idx[count].code = line;
        idx[count].desc = desc;
        ++count;
    }

    std::sort(idx, idx + count, [](const Entry& a, const Entry& b) {
        return std::strcmp(a.code, b.code) < 0;
    });

    g_idx = idx;
    g_idxCount = count;
    ESP_LOGI(TAG, "loaded %u DTC descriptions from SD", (unsigned)count);
}

} // namespace

namespace dtc {

void ensureLoaded() {
    if (g_sdResolved) return;
    if (!sd_card_ensure_mounted()) return;   // no card yet - retry next time
    loadSdDatabase();
    g_sdResolved = true;                      // card was available; don't re-probe
}

const char* describe(const char* code) {
    if (!code || !*code) return nullptr;
    if (const char* d = findSd(code)) return d;
    return findGeneric(code);
}

} // namespace dtc
