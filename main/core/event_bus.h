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
    ReadReadiness,   // OBD Mode 01 PID 01 (MIL + I/M readiness monitors)
    ScanModules,     // enhanced: UDS 0x19 read-DTC across all known modules
    ClearModuleDtcs, // enhanced: UDS 0x14 clear-DTC across all known modules
    ResetPeaks,      // zero the session peak-hold values
    ReadFreezeFrame, // OBD Mode 02: sensor conditions captured when a DTC set
    DiscoverPids,    // OBD Mode 01 PID 00/20/40...: enumerate supported PIDs
};

struct ObdCommand {
    CmdType  type;
    uint8_t  arg0;
    uint32_t arg_u32;
};

// -----------------------------------------------------------------------------
//  Diagnostic trouble code record (populated by ReadDtcs).
// -----------------------------------------------------------------------------
// DTC type tags stored in DtcRecord::status.
static constexpr uint8_t DTC_STORED    = 0x01;  // Mode 03 (confirmed)
static constexpr uint8_t DTC_PENDING   = 0x02;  // Mode 07
static constexpr uint8_t DTC_PERMANENT = 0x04;  // Mode 0A

struct DtcRecord {
    char     code[6];     // e.g. "P0301"
    uint8_t  status;      // DTC_STORED / DTC_PENDING / DTC_PERMANENT
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
    char cal_id[40];        // Mode 09 PID 04 calibration ID(s), "" if unread
    char ecu_name[24];      // Mode 09 PID 0A ECU name, "" if unread
    bool valid;             // true once a VIN has been read and parsed
};

// -----------------------------------------------------------------------------
//  I/M readiness (OBD Mode 01 PID 01): MIL status, stored-DTC count, and the
//  emissions-monitor readiness states - the classic "will it pass smog" view.
// -----------------------------------------------------------------------------
enum class MonState : uint8_t { NotSupported = 0, Ready = 1, NotReady = 2 };

struct ReadinessMonitor {
    const char* name;     // points to a static string literal (safe to copy)
    MonState    state;
};

// -----------------------------------------------------------------------------
//  Enhanced multi-module DTC scan (UDS service 0x19). One result per control
//  module addressed directly by its CAN header.
// -----------------------------------------------------------------------------
struct ModuleResult {
    char      name[18];
    uint16_t  req_id;            // request CAN header (e.g. 0x7E0, 0x760)
    bool      responded;         // module answered the UDS request
    uint8_t   dtc_count;
    DtcRecord dtcs[8];           // up to 8 codes shown per module
};
static constexpr size_t MAX_MODULES = 14;

// -----------------------------------------------------------------------------
//  Freeze-frame snapshot (OBD Mode 02): the sensor values the ECU latched at
//  the instant a fault was confirmed. `dtc` is the code that triggered it.
// -----------------------------------------------------------------------------
struct FreezeFrame {
    bool  valid = false;
    char  dtc[6] = {0};       // triggering DTC, e.g. "P0301" ("" if none)
    float rpm = 0, load = 0, coolant_c = 0, speed_kph = 0, map_kpa = 0;
    float throttle_pct = 0, ign_adv_deg = 0, intake_air_c = 0, maf_gps = 0;
};

struct ReadinessInfo {
    bool             valid = false;
    bool             mil_on = false;       // malfunction indicator lamp
    uint8_t          dtc_count = 0;        // stored DTC count
    bool             compression = false;  // true = diesel, false = spark/gas
    static constexpr int MAX_MON = 11;
    ReadinessMonitor mon[MAX_MON]{};
    uint8_t          mon_count = 0;
};

// -----------------------------------------------------------------------------
//  Supported standard PIDs (OBD Mode 01 PID 00/20/40/...). A bit per PID number
//  1..0xFF; the poll scheduler uses this to skip PIDs the ECU doesn't implement,
//  and the UI can list what's available. `bits` is MSB-first per byte:
//  PID p supported == bits[p>>3] & (0x80 >> (p&7)).
// -----------------------------------------------------------------------------
struct SupportedPids {
    uint8_t bits[32] = {0};
    size_t  count = 0;
    bool    valid = false;

    bool has(uint8_t pid) const {
        return bits[pid >> 3] & (uint8_t)(0x80 >> (pid & 7));
    }
};

// -----------------------------------------------------------------------------
//  GPS / GNSS fix (u-blox SAM-M10Q over UART, parsed from NMEA RMC + GGA).
//  `valid` means the module is talking; `has_fix` means lat/lon are a real
//  position. Coordinates are decimal degrees; speed is km/h; altitude is meters.
// -----------------------------------------------------------------------------
struct GpsFix {
    bool    valid = false;       // module producing parseable sentences
    bool    has_fix = false;     // positional fix (lat/lon valid)
    double  lat = 0.0;
    double  lon = 0.0;
    float   speed_kph = 0.0f;
    float   course_deg = 0.0f;   // true heading
    float   alt_m = 0.0f;        // altitude above MSL
    float   hdop = 0.0f;         // horizontal dilution of precision
    uint8_t sats = 0;            // satellites in use
    uint8_t utc_h = 0, utc_m = 0, utc_s = 0;
    uint16_t utc_year = 0;       // 4-digit (0 if date not yet decoded)
    uint8_t utc_mon = 0, utc_day = 0;
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
    // Logging starts ON; the SD logger turns it off automatically if no card
    // is detected (see sd_logger ensureCardMounted failure path).
    std::atomic<bool>      logging_enabled{true};
    std::atomic<bool>      sniffer_frozen{false};
    std::atomic<bool>      sniffer_filter_active{false};
    std::atomic<uint32_t>  current_baud{0};

    // --- DTC table (guarded by dtc_mtx_) ------------------------------------
    void  setDtcs(const DtcRecord* recs, size_t count);
    size_t getDtcs(DtcRecord* out, size_t max);

    // --- Vehicle info (guarded by veh_mtx_) ---------------------------------
    void        setVehicleInfo(const VehicleInfo& info);
    VehicleInfo getVehicleInfo();

    // --- I/M readiness (guarded by veh_mtx_) --------------------------------
    void          setReadiness(const ReadinessInfo& info);
    ReadinessInfo getReadiness();

    // --- Freeze frame (guarded by veh_mtx_) ---------------------------------
    void        setFreezeFrame(const FreezeFrame& ff);
    FreezeFrame getFreezeFrame();
    std::atomic<bool> freeze_read_active{false};   // true while a Mode 02 read runs

    // --- Module scan results (guarded by veh_mtx_) --------------------------
    void   setModuleResults(const ModuleResult* mods, size_t count);
    size_t getModuleResults(ModuleResult* out, size_t max);
    std::atomic<bool> module_scan_active{false};   // true while a module scan runs
    std::atomic<bool> dtc_read_active{false};      // true while a DTC read runs

    // --- Supported standard PIDs (guarded by veh_mtx_) ----------------------
    void          setSupportedPids(const SupportedPids& sp);
    SupportedPids getSupportedPids();

    // --- GPS fix (guarded by gps_mtx_) --------------------------------------
    void   setGps(const GpsFix& g);
    GpsFix getGps();
    std::atomic<bool> gps_present{false};   // true once the module sends data

    // --- IMU + drag-tree launch (all lock-free) -----------------------------
    std::atomic<float>    imu_ax{0.0f};      // accelerometer, G (raw, body axes)
    std::atomic<float>    imu_ay{0.0f};
    std::atomic<float>    imu_az{0.0f};
    std::atomic<bool>     imu_present{false};
    std::atomic<bool>     perf_armed{false};   // run armed; IMU watches for launch
    std::atomic<uint64_t> perf_green_us{0};    // tree green-light timestamp (0=none)
    std::atomic<uint64_t> perf_launch_us{0};   // detected launch timestamp (0=none)

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
    ReadinessInfo       readiness_{};
    FreezeFrame         freeze_{};
    ModuleResult        modules_[MAX_MODULES]{};
    size_t              module_count_ = 0;
    SupportedPids       supported_{};

    SemaphoreHandle_t   gps_mtx_    = nullptr;
    GpsFix              gps_{};
};
