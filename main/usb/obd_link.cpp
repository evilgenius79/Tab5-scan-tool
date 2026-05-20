// =============================================================================
//  obd_link.cpp - OBDLink EX / STN session layer.
// =============================================================================
#include "usb/obd_link.h"
#include "obd/stn_commands.h"
#include "app_config.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "ObdLink";

// Strip leading/trailing whitespace and CR/LF from a response blob.
static std::string trim(const std::string& s) {
    size_t b = 0, e = s.size();
    while (b < e && (isspace((unsigned char)s[b]) || s[b] == '>')) b++;
    while (e > b && (isspace((unsigned char)s[e - 1]) || s[e - 1] == '>')) e--;
    return s.substr(b, e - b);
}

// ---------------------------------------------------------------------------
//  Initialization handshake.
// ---------------------------------------------------------------------------
//  The OBDLink EX boots in ELM327-compatible mode. We harden it for high-rate
//  polling: kill echo (ATE0) and linefeeds (ATL0) to halve the bytes on the
//  wire, disable spaces (ATS0), enable adaptive timing (ATAT2), and let the
//  adapter auto-detect the protocol (ATSP0). STN-specific niceties follow.
// ---------------------------------------------------------------------------
bool ObdLink::initialize() {
    ESP_LOGI(TAG, "initializing adapter");

    bool ok = false;
    // Reset to a known state. ATZ takes ~1s; give it room.
    sendCommand(STN_CMD_RESET, &ok);
    vTaskDelay(pdMS_TO_TICKS(1000));

    struct Step { const char* cmd; const char* desc; };
    static const Step seq[] = {
        { STN_CMD_ECHO_OFF,      "echo off"        },  // ATE0
        { STN_CMD_LINEFEED_OFF,  "linefeed off"    },  // ATL0
        { STN_CMD_SPACES_OFF,    "spaces off"      },  // ATS0
        { STN_CMD_HEADERS_ON,    "headers on"      },  // ATH1 (needed for sniffing)
        { STN_CMD_ADAPTIVE_T2,   "adaptive timing" },  // ATAT2
        { STN_CMD_PROTO_AUTO,    "auto protocol"   },  // ATSP0
    };

    for (const auto& step : seq) {
        std::string resp = sendCommand(step.cmd, &ok);
        ESP_LOGI(TAG, "  %-16s -> '%s'%s", step.desc, resp.c_str(),
                 ok ? "" : " (TIMEOUT)");
        if (!ok) return false;
    }

    // Probe protocol/voltage to confirm a live ECU link.
    std::string proto = sendCommand(STN_CMD_DESCRIBE_PROTO, &ok);
    ESP_LOGI(TAG, "protocol: %s", proto.c_str());
    return ok;
}

// ---------------------------------------------------------------------------
//  Synchronous command/response.
// ---------------------------------------------------------------------------
std::string ObdLink::sendCommand(const std::string& cmd, bool* ok) {
    usb_.flushRx();                 // discard any stale bytes
    std::string line = cmd + "\r";
    usb_.write(reinterpret_cast<const uint8_t*>(line.data()), line.size(),
               OBD_CMD_TIMEOUT_MS);

    std::string acc;
    bool got = readUntil('>', acc, OBD_CMD_TIMEOUT_MS);
    if (ok) *ok = got;

    // The reply echoes nothing (ATE0) so `acc` is the bare answer + prompt.
    return trim(acc);
}

void ObdLink::sendRaw(const std::string& cmd) {
    std::string line = cmd + "\r";
    usb_.write(reinterpret_cast<const uint8_t*>(line.data()), line.size(),
               OBD_CMD_TIMEOUT_MS);
}

void ObdLink::stopMonitor() {
    // Any byte halts an STN monitor; a CR is the conventional choice.
    const uint8_t stop = '\r';
    usb_.write(&stop, 1, OBD_CMD_TIMEOUT_MS);
    // Drain residual frames + the returning prompt.
    std::string acc;
    readUntil('>', acc, OBD_CMD_TIMEOUT_MS);
    line_acc_.clear();
    usb_.flushRx();
}

// ---------------------------------------------------------------------------
//  Streaming line reader (monitor mode). Accumulates bytes across calls and
//  yields one CR-terminated line at a time.
// ---------------------------------------------------------------------------
bool ObdLink::readLine(std::string& out, uint32_t timeout_ms) {
    const uint64_t deadline = esp_timer_get_time() + (uint64_t)timeout_ms * 1000;

    while (true) {
        // Emit a buffered line if one is already complete.
        size_t nl = line_acc_.find('\r');
        if (nl != std::string::npos) {
            out = line_acc_.substr(0, nl);
            line_acc_.erase(0, nl + 1);
            // Skip empty / lone-LF artifacts.
            if (!out.empty() && out != "\n") return true;
            continue;
        }

        int64_t remaining = (int64_t)deadline - (int64_t)esp_timer_get_time();
        if (remaining <= 0) return false;

        uint8_t buf[256];
        size_t n = usb_.read(buf, sizeof(buf), remaining / 1000 + 1);
        if (n) line_acc_.append(reinterpret_cast<char*>(buf), n);
    }
}

bool ObdLink::readUntil(char terminator, std::string& acc, uint32_t timeout_ms) {
    const uint64_t deadline = esp_timer_get_time() + (uint64_t)timeout_ms * 1000;
    uint8_t buf[256];
    while (true) {
        int64_t remaining = (int64_t)deadline - (int64_t)esp_timer_get_time();
        if (remaining <= 0) return false;

        size_t n = usb_.read(buf, sizeof(buf), remaining / 1000 + 1);
        if (n) {
            acc.append(reinterpret_cast<char*>(buf), n);
            if (acc.find(terminator) != std::string::npos) return true;
        }
    }
}

// ---------------------------------------------------------------------------
//  Bus / baud configuration.
// ---------------------------------------------------------------------------
bool ObdLink::selectBus(CanBus bus) {
    bool ok = false;
    // HS-CAN: ISO 15765-4 11-bit @500k (protocol 6). MS-CAN on the OBDLink EX
    // is reached via the SW-CAN/MS pins using protocol B with a custom config.
    if (bus == CanBus::HS_CAN) {
        sendCommand("ATSP6", &ok);
    } else {
        // Protocol B = user CAN; set 125 kbps via STN protocol params.
        sendCommand("STP32", &ok);          // switch to MS-CAN protocol slot
        sendCommand("STCSWM2", &ok);        // single-wire/MS mode where wired
    }
    return ok;
}

bool ObdLink::setBaud(uint32_t baud) {
    // STBR sets the adapter UART baud. The adapter echoes "OK" at the OLD baud,
    // then both sides switch. We confirm with STI at the new rate.
    bool ok = false;
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "STBR%u", (unsigned)baud);
    sendRaw(cmd);
    vTaskDelay(pdMS_TO_TICKS(50));
    usb_.setBaud(baud);                     // host side follows
    vTaskDelay(pdMS_TO_TICKS(50));
    std::string id = sendCommand(STN_CMD_DEVICE_ID, &ok);
    ESP_LOGI(TAG, "baud switch to %u -> id '%s'", (unsigned)baud, id.c_str());
    return ok;
}
