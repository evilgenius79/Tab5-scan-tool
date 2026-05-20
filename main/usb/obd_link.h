// =============================================================================
//  obd_link.h - OBDLink EX session layer over the USB CDC transport.
// -----------------------------------------------------------------------------
//  Knows how to talk the ELM327/STN dialect:
//    * Line-oriented request/response framed by the '>' prompt character.
//    * AT/STN initialization handshake (echo off, headers, protocol).
//    * Synchronous sendCommand() returning the device's reply text.
//    * Streaming readLine() for monitor (sniffer) mode where the adapter
//      free-runs without prompts between frames.
//
//  This class is transport-agnostic above UsbHostCdc: swap the transport and
//  the STN dialect handling is unchanged.
// =============================================================================
#pragma once

#include <cstdint>
#include <cstddef>
#include <string>

#include "usb/usb_host_cdc.h"
#include "core/can_types.h"

class ObdLink {
public:
    explicit ObdLink(UsbHostCdc& transport) : usb_(transport) {}

    // Run the AT/STN init handshake. Call after the transport reports the
    // adapter is open. Returns true once the device acknowledges and reports
    // a valid protocol. Safe to re-run on reconnect.
    bool initialize();

    // Send `cmd` (no trailing CR needed) and collect the response up to the
    // '>' prompt. Returns the trimmed response; `ok` reflects whether the
    // adapter answered before OBD_CMD_TIMEOUT_MS. Echo and whitespace lines
    // are stripped.
    std::string sendCommand(const std::string& cmd, bool* ok = nullptr);

    // Fire-and-forget: write a command without waiting for a prompt. Used to
    // kick off monitor modes (STMA/STMF) that stream until interrupted.
    void sendRaw(const std::string& cmd);

    // Interrupt a running monitor by sending a single byte (any char) which
    // the STN firmware treats as "stop monitoring", then drain to the prompt.
    void stopMonitor();

    // Pull the next complete line from the stream (monitor mode). Returns false
    // on timeout with no full line available. The returned line excludes CR/LF.
    bool readLine(std::string& out, uint32_t timeout_ms);

    // Select powertrain (HS) vs body (MS) CAN by reconfiguring the protocol.
    bool selectBus(CanBus bus);

    // Change adapter UART baud via STBR, then re-sync the host side.
    bool setBaud(uint32_t baud);

private:
    // Read raw bytes until `terminator` seen or timeout. Appends to `acc`.
    bool readUntil(char terminator, std::string& acc, uint32_t timeout_ms);

    UsbHostCdc& usb_;
    std::string line_acc_;   // partial-line accumulator for readLine()
};
