// =============================================================================
//  stn_commands.h - ELM327 (AT) and OBDSolutions (STN) command vocabulary.
// -----------------------------------------------------------------------------
//  Reference: STN1100/STN2120 Family Reference & ELM327 datasheet. The OBDLink
//  EX is built on the STN2120, so it accepts both the legacy AT set and the
//  faster ST extensions. We lean on the ST commands for high-rate work.
//
//  Commands are sent without the trailing carriage return; obd_link adds it.
// =============================================================================
#pragma once

// --- ELM327 "AT" configuration ----------------------------------------------
#define STN_CMD_RESET            "ATZ"      // full reset
#define STN_CMD_WARM_RESET       "ATWS"     // warm start (faster than ATZ)
#define STN_CMD_ECHO_OFF         "ATE0"     // disable command echo
#define STN_CMD_ECHO_ON          "ATE1"
#define STN_CMD_LINEFEED_OFF     "ATL0"     // no LF after CR
#define STN_CMD_SPACES_OFF       "ATS0"     // strip spaces from responses
#define STN_CMD_HEADERS_ON       "ATH1"     // include CAN headers (sniffing)
#define STN_CMD_HEADERS_OFF      "ATH0"
#define STN_CMD_ADAPTIVE_T2      "ATAT2"    // aggressive adaptive timing
#define STN_CMD_PROTO_AUTO       "ATSP0"    // auto-detect OBD protocol
#define STN_CMD_DESCRIBE_PROTO   "ATDPN"    // describe protocol number
#define STN_CMD_READ_VOLTAGE     "ATRV"     // battery voltage at the connector

// --- STN device / link ------------------------------------------------------
#define STN_CMD_DEVICE_ID        "STI"      // firmware version string
#define STN_CMD_SERIAL           "STSN"     // device serial number

// --- STN high-speed PID polling ---------------------------------------------
//  STPX lets us issue a request with an explicit header, data, and expected
//  response count in one shot, skipping per-request protocol overhead. Format:
//     STPX h:<hdr>, d:<bytes>, r:<numresponses>
//  We build these dynamically in the parser/poller.
#define STN_CMD_POLL_PREFIX      "STPX"

// --- STN CAN monitor (sniffer) ----------------------------------------------
#define STN_CMD_MONITOR_ALL      "STMA"     // monitor all traffic, no filter
#define STN_CMD_MONITOR_FILTERED "STM"      // STM with active filters applied
//  Filter management:
//     STFAP <pattern>, <mask>   - add pass filter (ID & mask == pattern & mask)
//     STFAB <pattern>, <mask>   - add block filter
//     STFCP / STFCA             - clear pass / clear all filters
#define STN_CMD_FILTER_PASS_ADD  "STFAP"
#define STN_CMD_FILTER_BLOCK_ADD "STFAB"
#define STN_CMD_FILTER_CLEAR_ALL "STFCA"

// --- Standard OBD-II service requests (Mode 0x01 / 0x03 / 0x04) --------------
#define OBD_MODE_CURRENT_DATA    "01"       // live data
#define OBD_MODE_READ_DTC        "03"       // stored DTCs
#define OBD_MODE_CLEAR_DTC       "04"       // clear DTCs + MIL
#define OBD_MODE_PENDING_DTC     "07"       // pending DTCs
#define OBD_MODE_VIN             "0902"     // Mode 09 PID 02: vehicle VIN
