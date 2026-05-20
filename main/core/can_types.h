// =============================================================================
//  can_types.h - POD types shared across the OBD, UI, and logging layers.
// =============================================================================
#pragma once

#include <cstdint>
#include <cstddef>

// -----------------------------------------------------------------------------
//  A single decoded CAN frame as emitted by the OBDLink EX monitor modes.
//  Kept as a flat POD so it can be memcpy'd through the lock-free frame ring.
// -----------------------------------------------------------------------------
struct can_frame_t {
    uint64_t timestamp_us;   // esp_timer_get_time() at capture
    uint32_t id;             // 11-bit or 29-bit CAN identifier
    uint8_t  dlc;            // data length code (0..8)
    bool     extended;       // true => 29-bit identifier
    uint8_t  data[8];        // payload bytes
};

// -----------------------------------------------------------------------------
//  Operating modes for the OBD task state machine.
// -----------------------------------------------------------------------------
enum class ObdMode : uint8_t {
    Idle,        // connected, no active traffic
    Polling,     // Mode A: STPX/PID request loop
    Sniffing,    // Mode B: STMA/STMF raw monitor
};

// -----------------------------------------------------------------------------
//  Link lifecycle state for the USB/OBD connection (drives the Settings UI).
// -----------------------------------------------------------------------------
enum class LinkState : uint8_t {
    Disconnected,   // no USB device present
    Enumerating,    // device attached, opening VCP
    Initializing,   // running AT/STN init sequence
    Online,         // ready for commands
    Error,          // fault; auto-reconnect pending
};

// -----------------------------------------------------------------------------
//  CAN bus selection (OBDLink EX supports network switching via STP/PP).
// -----------------------------------------------------------------------------
enum class CanBus : uint8_t {
    HS_CAN,   // ISO 15765-4 / 500 kbps powertrain
    MS_CAN,   // medium-speed body CAN (125 kbps), if vehicle/adapter wired
};
