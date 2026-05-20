// =============================================================================
//  obd_task.h - The OBD/USB I/O task (runs on APP_CORE_IO).
// -----------------------------------------------------------------------------
//  This is the orchestrator for everything on the I/O core:
//    * Owns the UsbHostCdc transport and the ObdLink session.
//    * Drives a connection state machine with auto-reconnect.
//    * Mode A (Polling): round-robin STPX PID requests -> TelemetryState.
//    * Mode B (Sniffing): STMA/STM monitor -> frame rings (sniffer + logger).
//    * Services UI commands (mode switch, filters, clear/read DTC, baud, bus).
//    * Runs the performance-timer capture (0-60, 1/4 mile) off VSS.
//
//  Spawn once from app_main with obd_task_start(). The task never returns.
// =============================================================================
#pragma once

void obd_task_start();
