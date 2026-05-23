// =============================================================================
//  stn_parser.h - Decode raw STN/ELM text into structured data.
// -----------------------------------------------------------------------------
//  Two responsibilities:
//    1. parseCanFrame()  - turn a monitor-mode line (e.g. "7E8 04 41 0C 1A F8")
//                          into a can_frame_t for the sniffer/logger.
//    2. parsePidResponse() - extract the data bytes from a Mode 01/22 reply so
//                          a PidDecoder can convert them to engineering units.
//    3. parseDtcs()      - decode a Mode 03 reply into P/C/B/U trouble codes.
//
//  All functions are stateless and allocation-light (operate on caller buffers
//  / std::string) so they are safe to call from the OBD task hot loop.
// =============================================================================
#pragma once

#include <cstdint>
#include <string>
#include "core/can_types.h"
#include "core/event_bus.h"   // DtcRecord

namespace stn {

// Parse one line of monitor output into `out`. Returns false if the line is
// not a valid frame (status text, prompt, empty, etc.).
bool parseCanFrame(const std::string& line, can_frame_t& out);

// Parse a service-mode reply. `expect_mode` is the request mode byte (e.g.
// 0x01) and `expect_pid` the PID; the parser validates the positive-response
// echo (mode|0x40, pid) and copies the trailing data bytes into `data`.
// Returns the number of data bytes, or -1 on a NO DATA / error / mismatch.
int parsePidResponse(const std::string& resp, uint8_t expect_mode,
                     uint16_t expect_pid, uint8_t* data, size_t max);

// Decode a DTC response (Mode 03 stored / 07 pending / 0A permanent) into DTC
// records, tagging each with `status_tag`. `resp_echo` is the positive-response
// byte to skip (0x43 / 0x47 / 0x4A). Returns the count written.
size_t parseDtcs(const std::string& resp, DtcRecord* out, size_t max,
                 uint8_t resp_echo = 0x43, uint8_t status_tag = DTC_STORED);

// Decode a Mode 09 PID 02 (VIN) response. The 17 ASCII characters are written
// to `out` (must hold at least 18 bytes, NUL-terminated). Returns true if a
// full 17-character VIN was extracted. Call with the adapter's CAN
// auto-formatting on and headers off so the multi-frame ISO-TP reply is
// already reassembled.
bool parseVin(const std::string& resp, char* out);

// Decode a generic Mode 09 ASCII reply (e.g. PID 04 calibration ID, PID 0A ECU
// name). Scans for the 49 <pid> positive-response echo, skips the data-item
// count byte, and copies the printable ASCII (NUL padding stripped) into `out`
// (capacity `cap`, NUL-terminated). Returns the number of characters written.
size_t parseMode09Ascii(const std::string& resp, uint8_t pid, char* out, size_t cap);

// Helper: true if the response is an adapter error token (NO DATA, ?, etc.).
bool isErrorResponse(const std::string& resp);

} // namespace stn
