// =============================================================================
//  event_bus.h - Central, thread-safe coordination hub.
// -----------------------------------------------------------------------------
//  Everything that crosses a task boundary goes through here:
//    * TelemetryState   - mutex-guarded snapshot (OBD writes, UI/logger read)
//    * Command queue    - UI -> OBD task requests (mode switch, clear DTC, ...)
//    * Frame rings      - OBD -> sniffer UI and OBD -> SD logger fan-out
//    * Link/mode status - atomics the UI can poll cheaply every frame
//
//  The bus is a process-wide singleton initialized once in app_main before any
//  task starts, so the pointers/handles it hands out are stable for the life
//  of the program.
// =============================================================================
#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "core/can_types.h"
#include "core/telemetry_state.h"
#include "core/ring_buffer.h"

// -----------------------------------------------------------------------------
//  UI -> OBD task commands.
// -----------------------------------------------------------------------------
enum class CmdType : uint8_t {
    SetMode,         // arg0 = ObdMode
    SetSnifferFilter,// arg_u32 = CAN id, arg0 = mask-width nibble (STMF)
    ClearSnifferFilter,
    ClearDtcs,
    ReadDtcs,
    SetBaud,         // arg_u32 = baud
    SelectBus,       // arg0 = CanBus
    StartPerfRun,    // arm the 0-60 / quarter-mile capture
    Reconnect,       // force USB re-enumeration
    ReadVin,         // OBD Mode 09 PID 02 (vehicle VIN)
};

struct ObdCommand {
    CmdType  type;
    uint8_t  arg0;
    uint32_t arg_u32;
};

// -----------------------------------------------------------------------------
//  Diagnostic trouble code record (populated by ReadDtcs).
// -----------------------------------------------------------------------------
struct DtcRecord {
    char     code[6];     // e.g. "P0301"
    uint8_t  status;      // pending/confirmed/permanent bitfield
};
static constexpr size_t MAX_DTCS = 64;

// -----------------------------------------------------------------------------
//  Vehicle identification (populated by ReadVin). VIN comes from the car over
//  OBD; manufacturer/year are decoded offline from the VIN itself.
// -----------------------------------------------------------------------------
struct VehicleInfo {
    char vin[18];           // 17 chars + NUL; empty string if not yet read
    char manufacturer[28];  // decoded from the WMI (chars 1-3), "" if unknown
    int  model_year;        // decoded from char 10, 0 if unknown
    bool valid;             // true once a VIN has been read and parsed
};

// -----------------------------------------------------------------------------
//  The bus.
// -----------------------------------------------------------------------------
class EventBus {
public:
    static EventBus& instance();

    // Call exactly once, before spawning tasks. Returns false on alloc failure.
    bool init();

    // --- Telemetry snapshot (mutex-guarded copy semantics) ------------------
    void          publish(const TelemetryState& s);   // OBD task
    TelemetryState snapshot();                          // UI / logger

    // --- Command channel (UI -> OBD) ----------------------------------------
    bool sendCommand(const ObdCommand& cmd, TickType_t wait = 0);   // UI side
    bool recvCommand(ObdCommand& out, TickType_t wait);             // OBD side

    // --- Frame fan-out ------------------------------------------------------
    FrameRing& snifferRing() { return sniffer_ring_; }
    FrameRing& loggerRing()  { return logger_ring_; }

    // --- Lightweight status (lock-free) -------------------------------------
    std::atomic<LinkState> link{LinkState::Disconnected};
    // Default to Polling so the live dashboard populates as soon as the adapter
    // is online. The OBD task only acts on this once link == Online, and the
    // Sniffer screen flips it to Sniffing on demand.
    std::atomic<ObdMode>   mode{ObdMode::Polling};
    std::atomic<CanBus>    bus{CanBus::HS_CAN};
    std::atomic<bool>      logging_enabled{false};
    std::atomic<bool>      sniffer_frozen{false};
    std::atomic<bool>      sniffer_filter_active{false};
    std::atomic<uint32_t>  current_baud{0};

    // --- DTC table (guarded by dtc_mtx_) ------------------------------------
    void  setDtcs(const DtcRecord* recs, size_t count);
    size_t getDtcs(DtcRecord* out, size_t max);

    // --- Vehicle info (guarded by veh_mtx_) ---------------------------------
    void        setVehicleInfo(const VehicleInfo& info);
    VehicleInfo getVehicleInfo();

private:
    EventBus() = default;

    SemaphoreHandle_t   telem_mtx_  = nullptr;
    TelemetryState      telem_{};

    QueueHandle_t       cmd_queue_  = nullptr;

    FrameRing           sniffer_ring_;
    FrameRing           logger_ring_;

    SemaphoreHandle_t   dtc_mtx_    = nullptr;
    DtcRecord           dtcs_[MAX_DTCS]{};
    size_t              dtc_count_  = 0;

    SemaphoreHandle_t   veh_mtx_    = nullptr;
    VehicleInfo         vehicle_{};
};
