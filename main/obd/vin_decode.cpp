// =============================================================================
//  vin_decode.cpp - offline WMI + model-year decode.
// =============================================================================
#include "obd/vin_decode.h"

#include <cstring>
#include <cctype>

namespace {

struct Wmi { const char* prefix; const char* name; };

// Common World Manufacturer Identifiers (VIN chars 1-3). Not exhaustive - the
// long tail is what the online decoder is for - but covers the marques most
// likely to be plugged into an OBD tool.
constexpr Wmi kWmi[] = {
    // North America
    { "1FA", "Ford" }, { "1FB", "Ford" }, { "1FC", "Ford" }, { "1FD", "Ford" },
    { "1FM", "Ford" }, { "1FT", "Ford" }, { "2FA", "Ford" }, { "3FA", "Ford" },
    { "1G1", "Chevrolet" }, { "1GC", "Chevrolet" }, { "1GB", "Chevrolet" },
    { "1GT", "GMC" }, { "1GK", "GMC" }, { "3GN", "Chevrolet" }, { "1GN", "Chevrolet" },
    { "1G6", "Cadillac" }, { "1G4", "Buick" },
    { "1C3", "Chrysler" }, { "1C4", "Jeep/RAM" }, { "1C6", "RAM" }, { "2C3", "Chrysler" },
    { "1B3", "Dodge" }, { "1D4", "Dodge" }, { "3C4", "Chrysler" },
    { "1HG", "Honda" }, { "2HG", "Honda" }, { "3HG", "Honda" }, { "19X", "Honda" },
    { "5FN", "Honda" }, { "JHM", "Honda" }, { "JH4", "Acura" }, { "19U", "Acura" },
    { "1N4", "Nissan" }, { "3N1", "Nissan" }, { "JN1", "Nissan" }, { "JN8", "Nissan" },
    { "5N1", "Nissan" },
    { "4T1", "Toyota" }, { "5TD", "Toyota" }, { "5TF", "Toyota" }, { "2T1", "Toyota" },
    { "JTD", "Toyota" }, { "JT2", "Toyota" }, { "JT3", "Toyota" }, { "JTH", "Lexus" },
    // Japan / Korea
    { "JM1", "Mazda" }, { "JM3", "Mazda" }, { "4F2", "Mazda" }, { "4F4", "Mazda" },
    { "JF1", "Subaru" }, { "JF2", "Subaru" }, { "4S3", "Subaru" }, { "4S4", "Subaru" },
    { "KMH", "Hyundai" }, { "KM8", "Hyundai" }, { "5NP", "Hyundai" },
    { "KNA", "Kia" }, { "KND", "Kia" }, { "5XX", "Kia" }, { "3KP", "Kia" },
    { "JTE", "Toyota" }, { "JS1", "Suzuki" }, { "JS2", "Suzuki" },
    { "JA3", "Mitsubishi" }, { "JA4", "Mitsubishi" }, { "4A3", "Mitsubishi" },
    // Germany
    { "WBA", "BMW" }, { "WBS", "BMW M" }, { "WBY", "BMW i" }, { "4US", "BMW" }, { "5UX", "BMW" },
    { "WDB", "Mercedes-Benz" }, { "WDD", "Mercedes-Benz" }, { "WDC", "Mercedes-Benz" },
    { "W1K", "Mercedes-Benz" }, { "W1N", "Mercedes-Benz" }, { "4JG", "Mercedes-Benz" },
    { "WVW", "Volkswagen" }, { "WV1", "Volkswagen" }, { "WV2", "Volkswagen" },
    { "3VW", "Volkswagen" }, { "1VW", "Volkswagen" },
    { "WAU", "Audi" }, { "WA1", "Audi" }, { "TRU", "Audi" },
    { "WP0", "Porsche" }, { "WP1", "Porsche" },
    { "WMW", "MINI" }, { "WME", "Smart" },
    // Rest of Europe
    { "YV1", "Volvo" }, { "YV4", "Volvo" }, { "LYV", "Volvo" },
    { "SAJ", "Jaguar" }, { "SAL", "Land Rover" }, { "SCC", "Lotus" }, { "SCB", "Bentley" },
    { "SCA", "Rolls-Royce" }, { "SCF", "Aston Martin" },
    { "ZFF", "Ferrari" }, { "ZHW", "Lamborghini" }, { "ZAM", "Maserati" },
    { "ZFA", "Fiat" }, { "ZAR", "Alfa Romeo" },
    { "VF1", "Renault" }, { "VF3", "Peugeot" }, { "VF7", "Citroen" }, { "VF6", "Renault" },
    { "VSS", "SEAT" }, { "TMB", "Skoda" },
    // EV / other
    { "5YJ", "Tesla" }, { "7SA", "Tesla" }, { "LRW", "Tesla" },
};
constexpr size_t kWmiCount = sizeof(kWmi) / sizeof(kWmi[0]);

} // namespace

namespace vin {

const char* manufacturer(const char* vin) {
    if (!vin || std::strlen(vin) < 3) return nullptr;
    char wmi[4] = { (char)std::toupper((unsigned char)vin[0]),
                    (char)std::toupper((unsigned char)vin[1]),
                    (char)std::toupper((unsigned char)vin[2]), '\0' };
    for (const auto& w : kWmi) {
        if (std::strcmp(w.prefix, wmi) == 0) return w.name;
    }
    return nullptr;
}

int model_year(const char* vin) {
    if (!vin || std::strlen(vin) < 10) return 0;

    // Year codes in order: 1980..2009 (then the cycle repeats from 2010).
    static const char* kCodes = "ABCDEFGHJKLMNPRSTVWXY123456789";
    char y = (char)std::toupper((unsigned char)vin[9]);
    const char* p = std::strchr(kCodes, y);
    if (!p || y == '\0') return 0;

    int idx = (int)(p - kCodes);                 // 0 => 1980 / 2010
    // Char 7 (index 6) disambiguates the 30-year cycle: alphabetic => 2010+.
    bool newer = std::isalpha((unsigned char)vin[6]) != 0;
    return (newer ? 2010 : 1980) + idx;
}

} // namespace vin
