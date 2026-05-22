// =============================================================================
//  vin_decode.h - offline VIN interpretation (no network required).
// -----------------------------------------------------------------------------
//  Decodes the two pieces of a VIN that are self-describing:
//    * Manufacturer - from the World Manufacturer Identifier (chars 1-3).
//    * Model year   - from char 10 (the standardized year code).
//  Full make/model/engine detail needs an online decoder (e.g. NHTSA vPIC) and
//  is intentionally out of scope here.
// =============================================================================
#pragma once

namespace vin {

// Look up a manufacturer name from the VIN's WMI (first 3 chars). Returns
// nullptr if the WMI is not in the built-in table.
const char* manufacturer(const char* vin);

// Decode the model year from char 10. Returns 0 if it can't be determined.
// Char 7 disambiguates the 30-year cycle (alphabetic => 2010+).
int model_year(const char* vin);

} // namespace vin
