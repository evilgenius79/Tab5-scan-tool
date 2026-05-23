// =============================================================================
//  tab5_power.h - M5Stack Tab5 battery charger control / status.
// -----------------------------------------------------------------------------
//  The Tab5's IP2326 charger is gated by IO-expander (0x44) pin 7 (enable) and
//  reports state on pin 6 (HIGH = charging, LOW = charged/discharging). The
//  tablet only charges once powered on and the enable pin is driven high.
// =============================================================================
#pragma once

namespace tab5pwr {

// Enable the battery charger (call once at boot, after the BSP I2C bus is up).
void enable_charging();

// True if the charge-status pin reports charging (trickle/charging in progress).
bool is_charging();

// Battery pack voltage from the INA226 monitor (0 if unavailable). 2S Li-ion.
float battery_voltage();

// Estimated state of charge [0..100], or -1 if unavailable. Voltage-based, so
// it reads high while charging and sags under load - it's an estimate.
int battery_percent();

} // namespace tab5pwr
