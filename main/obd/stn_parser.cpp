// =============================================================================
//  stn_parser.cpp - text -> structured decode for STN/ELM responses.
// =============================================================================
#include "obd/stn_parser.h"
#include "esp_timer.h"

#include <cctype>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <algorithm>
#include <vector>

namespace {

// Parse a run of hex tokens (with or without spaces) into a byte vector.
// Non-hex characters terminate a token; whitespace separates tokens.
std::vector<uint8_t> hexBytes(const std::string& s) {
    std::vector<uint8_t> out;
    int hi = -1;
    for (char c : s) {
        int v;
        if (c >= '0' && c <= '9') v = c - '0';
        else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
        else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
        else { hi = -1; continue; }   // separator: reset nibble pairing
        if (hi < 0) { hi = v; }
        else { out.push_back((uint8_t)((hi << 4) | v)); hi = -1; }
    }
    return out;
}

uint32_t parseHexId(const std::string& tok) {
    return (uint32_t)strtoul(tok.c_str(), nullptr, 16);
}

} // namespace

namespace stn {

bool isErrorResponse(const std::string& resp) {
    static const char* errs[] = {
        "NO DATA", "ERROR", "UNABLE", "BUS INIT", "CAN ERROR",
        "STOPPED", "?", "SEARCHING", "BUFFER FULL"
    };
    for (const char* e : errs) {
        if (resp.find(e) != std::string::npos) return true;
    }
    return resp.empty();
}

// ---------------------------------------------------------------------------
//  Monitor-mode CAN frame line. With ATH1 + ATS0 the OBDLink emits lines like:
//     7E8104100BE7F0801   (no spaces)   or
//     7E8 10 41 00 BE 7F  (spaced)
//  We treat the first 3 (11-bit) or 8 (29-bit) hex nibbles as the ID and the
//  remainder as data bytes. STN also prefixes a length nibble in some formats;
//  we accept the common "<ID> <bytes...>" shape and the no-space packed shape.
// ---------------------------------------------------------------------------
bool parseCanFrame(const std::string& line, can_frame_t& out) {
    if (line.empty() || isErrorResponse(line)) return false;

    // Tokenize on whitespace.
    std::vector<std::string> tok;
    {
        std::string cur;
        for (char c : line) {
            if (isspace((unsigned char)c)) { if (!cur.empty()) { tok.push_back(cur); cur.clear(); } }
            else cur.push_back(c);
        }
        if (!cur.empty()) tok.push_back(cur);
    }
    if (tok.empty()) return false;

    memset(&out, 0, sizeof(out));
    out.timestamp_us = esp_timer_get_time();

    if (tok.size() == 1) {
        // Packed, no-space form: first 3 chars = 11-bit ID (or 8 = 29-bit).
        const std::string& s = tok[0];
        if (s.size() < 4) return false;
        size_t id_len = (s.size() >= 8 && s.size() % 2 == 0 && s.size() > 19) ? 8 : 3;
        out.extended = (id_len == 8);
        out.id = parseHexId(s.substr(0, id_len));
        auto bytes = hexBytes(s.substr(id_len));
        out.dlc = (uint8_t)std::min<size_t>(bytes.size(), 8);
        memcpy(out.data, bytes.data(), out.dlc);
        return out.dlc > 0;
    }

    // Spaced form: tok[0] is the ID, the rest are data bytes.
    out.id = parseHexId(tok[0]);
    out.extended = (tok[0].size() > 3);
    size_t n = 0;
    for (size_t i = 1; i < tok.size() && n < 8; ++i) {
        auto b = hexBytes(tok[i]);
        for (uint8_t v : b) { if (n < 8) out.data[n++] = v; }
    }
    out.dlc = (uint8_t)n;
    return out.dlc > 0;
}

// ---------------------------------------------------------------------------
//  Service-mode reply. Strips the positive-response echo and returns data.
//  Handles both Mode 01 (2-byte echo: 41 PID) and Mode 22 (3-byte: 62 PIDhi
//  PIDlo). The special "ATRV" voltage reply (e.g. "14.2V") is handled by the
//  caller, not here.
// ---------------------------------------------------------------------------
int parsePidResponse(const std::string& resp, uint8_t expect_mode,
                     uint16_t expect_pid, uint8_t* data, size_t max) {
    if (isErrorResponse(resp)) return -1;

    auto bytes = hexBytes(resp);
    if (bytes.empty()) return -1;

    const uint8_t pos_mode = expect_mode | 0x40;
    // Find the response echo within the byte stream (multi-frame responses may
    // include header/length bytes ahead of the echo).
    for (size_t i = 0; i + 1 < bytes.size(); ++i) {
        if (bytes[i] != pos_mode) continue;

        size_t pid_len = (expect_mode >= 0x22) ? 2 : 1;
        if (i + pid_len >= bytes.size()) continue;

        uint16_t pid = bytes[i + 1];
        if (pid_len == 2) pid = (pid << 8) | bytes[i + 2];
        if (pid != expect_pid) continue;

        size_t start = i + 1 + pid_len;
        size_t count = std::min(bytes.size() - start, max);
        memcpy(data, &bytes[start], count);
        return (int)count;
    }
    return -1;
}

// ---------------------------------------------------------------------------
//  Mode 03 DTC decode. Each code is 2 bytes; the top 2 bits select the letter
//  domain (P/C/B/U) and the next 2 bits the first digit.
// ---------------------------------------------------------------------------
size_t parseDtcs(const std::string& resp, DtcRecord* out, size_t max,
                 uint8_t resp_echo, uint8_t status_tag) {
    if (isErrorResponse(resp)) return 0;
    auto bytes = hexBytes(resp);

    // Skip the positive-response echo (43 stored / 47 pending / 4A permanent)
    // plus an optional DTC-count byte that some ECUs prepend.
    size_t i = 0;
    if (!bytes.empty() && bytes[0] == resp_echo) {
        i = 1;
        if ((bytes.size() - i) % 2 == 1 && i < bytes.size()) ++i;  // count byte
    }

    static const char domain[4] = { 'P', 'C', 'B', 'U' };
    size_t count = 0;
    for (; i + 1 < bytes.size() && count < max; i += 2) {
        uint16_t raw = (bytes[i] << 8) | bytes[i + 1];
        if (raw == 0) continue;   // padding
        DtcRecord& r = out[count];
        r.code[0] = domain[(raw >> 14) & 0x3];
        r.code[1] = '0' + ((raw >> 12) & 0x3);
        snprintf(&r.code[2], 4, "%03X", raw & 0x0FFF);
        r.status = status_tag;
        count++;
    }
    return count;
}

// ---------------------------------------------------------------------------
//  Mode 09 PID 02 VIN decode. After ISO-TP reassembly (CAN auto-format on,
//  headers off) the data stream contains the positive-response echo 49 02,
//  then a NODI count byte (01), then the 17 ASCII VIN characters. We scan for
//  the 49 02 echo to skip any leading framing (e.g. the ELM length line), then
//  copy the printable bytes that follow.
// ---------------------------------------------------------------------------
bool parseVin(const std::string& resp, char* out) {
    out[0] = '\0';
    if (isErrorResponse(resp)) return false;
    auto bytes = hexBytes(resp);

    for (size_t i = 0; i + 1 < bytes.size(); ++i) {
        if (bytes[i] != 0x49 || bytes[i + 1] != 0x02) continue;

        size_t start = i + 2;
        // Skip the NODI (number-of-data-items) byte when present.
        if (start < bytes.size() && bytes[start] == 0x01) ++start;

        size_t n = 0;
        for (size_t j = start; j < bytes.size() && n < 17; ++j) {
            uint8_t c = bytes[j];
            if (c < 0x20 || c > 0x7E) break;   // stop at non-printable framing
            out[n++] = (char)c;
        }
        out[n] = '\0';
        return n == 17;
    }
    return false;
}

// ---------------------------------------------------------------------------
//  UDS ReadDTCInformation (0x19 / 0x02). Response: 59 02 <availMask> then
//  4-byte records: 3 DTC bytes + 1 status byte. The first two DTC bytes encode
//  the P/C/B/U code exactly like OBD; the third is the failure-type byte (FTB),
//  appended as "-NN".
// ---------------------------------------------------------------------------
size_t parseUdsDtcs(const std::string& resp, DtcRecord* out, size_t max) {
    if (isErrorResponse(resp)) return 0;
    auto bytes = hexBytes(resp);

    // Find the 59 02 positive-response echo, then skip the availability byte.
    size_t i = 0;
    for (; i + 1 < bytes.size(); ++i)
        if (bytes[i] == 0x59 && bytes[i + 1] == 0x02) break;
    if (i + 1 >= bytes.size()) return 0;
    i += 3;   // skip 59, 02, availability-mask

    static const char domain[4] = { 'P', 'C', 'B', 'U' };
    size_t count = 0;
    for (; i + 2 < bytes.size() && count < max; i += 4) {  // 3 DTC + 1 status
        uint16_t raw = (bytes[i] << 8) | bytes[i + 1];
        uint8_t  ftb = bytes[i + 2];
        if (raw == 0) continue;
        DtcRecord& r = out[count];
        r.code[0] = domain[(raw >> 14) & 0x3];
        r.code[1] = '0' + ((raw >> 12) & 0x3);
        snprintf(&r.code[2], 4, "%03X", raw & 0x0FFF);
        (void)ftb;                       // failure-type byte available if needed
        r.status = DTC_STORED;
        count++;
    }
    return count;
}

// ---------------------------------------------------------------------------
//  Mode 02 freeze-frame PID. Response "42 <pid> [frame#] <data...>". Different
//  ECUs/adapters do or don't echo the frame-number byte after the PID, so we
//  return the trailing expect_len bytes after the echo - correct either way for
//  a single-PID request (the data sits at the end of the line).
// ---------------------------------------------------------------------------
int parseFreezeFrame(const std::string& resp, uint8_t pid, uint8_t expect_len,
                     uint8_t* data, size_t max) {
    if (isErrorResponse(resp)) return -1;
    auto bytes = hexBytes(resp);
    for (size_t i = 0; i + 1 < bytes.size(); ++i) {
        if (bytes[i] != 0x42 || bytes[i + 1] != pid) continue;
        size_t avail = bytes.size() - (i + 2);          // bytes after the echo
        if (avail < expect_len) return -1;
        size_t start = bytes.size() - expect_len;        // trailing data bytes
        size_t cnt   = std::min<size_t>(expect_len, max);
        memcpy(data, &bytes[start], cnt);
        return (int)cnt;
    }
    return -1;
}

// ---------------------------------------------------------------------------
//  Generic Mode 09 ASCII reply (CALID PID 04, ECU name PID 0A, ...). Same
//  framing as VIN: find the 49 <pid> echo, skip the NODI count byte, then copy
//  printable ASCII. NUL bytes (CALID field padding) are skipped; a run of other
//  non-printable framing ends the current field with a single space separator.
// ---------------------------------------------------------------------------
size_t parseMode09Ascii(const std::string& resp, uint8_t pid, char* out, size_t cap) {
    out[0] = '\0';
    if (cap < 2 || isErrorResponse(resp)) return 0;
    auto bytes = hexBytes(resp);

    for (size_t i = 0; i + 1 < bytes.size(); ++i) {
        if (bytes[i] != 0x49 || bytes[i + 1] != pid) continue;

        size_t start = i + 2;
        // Skip the data-item count byte (small value) when present.
        if (start < bytes.size() && bytes[start] <= 0x10) ++start;

        size_t n = 0;
        bool gap = false;
        for (size_t j = start; j < bytes.size() && n < cap - 1; ++j) {
            uint8_t c = bytes[j];
            if (c == 0x00) { gap = (n > 0); continue; }   // field padding
            if (c < 0x20 || c > 0x7E) { gap = (n > 0); continue; }
            if (gap) { out[n++] = ' '; gap = false; if (n >= cap - 1) break; }
            out[n++] = (char)c;
        }
        out[n] = '\0';
        return n;
    }
    return 0;
}

} // namespace stn
