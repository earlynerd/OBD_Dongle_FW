#ifndef OBD2_PIDS_H
#define OBD2_PIDS_H

#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════
// PID Definition Structure
// ═══════════════════════════════════════════════════════════════════════════

enum obd2_formula_t {
    FORMULA_DIRECT,         // A (no conversion)
    FORMULA_PERCENT_255,    // A * 100 / 255
    FORMULA_TEMP_40,        // A - 40
    FORMULA_TEMP_50,        // A - 50 (Nissan)
    FORMULA_RPM,            // ((A * 256) + B) / 4
    FORMULA_SPEED_KPH,      // A
    FORMULA_TIMING,         // (A - 128) / 2
    FORMULA_MAF,            // ((A * 256) + B) / 100
    FORMULA_FUEL_TRIM,      // (A - 128) * 100 / 128
    FORMULA_FUEL_PRESSURE,  // A * 3
    FORMULA_VOLTAGE_8,      // A / 12.5 (Nissan battery)
    FORMULA_O2_VOLTAGE,     // A / 200
    FORMULA_O2_CURRENT,     // ((A * 256) + B) / 256 - 128
    FORMULA_EVAP_PRESSURE,  // ((A * 256) + B) / 4 (signed)
    FORMULA_ABS_PRESSURE,   // (A * 256) + B
    FORMULA_RUNTIME,        // (A * 256) + B
};

struct obd2_pid_def_t {
    uint8_t pid;            // PID number
    const char* name;       // Short name
    const char* desc;       // Description
    const char* unit;       // Unit string
    uint8_t bytes;          // Number of data bytes (1-4)
    obd2_formula_t formula; // Conversion formula
    float min_val;          // Minimum value (for display)
    float max_val;          // Maximum value (for display)
};

// ═══════════════════════════════════════════════════════════════════════════
// Standard OBD2 PIDs (Service 0x01)
// ═══════════════════════════════════════════════════════════════════════════

static const obd2_pid_def_t obd2_pids[] = {
    // PID, Name, Description, Unit, Bytes, Formula, Min, Max
    {0x00, "PIDS_A",    "PIDs supported 01-20",         "",     4, FORMULA_DIRECT,        0,    0},
    {0x01, "STATUS",    "Monitor status since clear",   "",     4, FORMULA_DIRECT,        0,    0},
    {0x03, "FUEL_SYS",  "Fuel system status",           "",     2, FORMULA_DIRECT,        0,    0},
    {0x04, "LOAD",      "Engine load",                  "%",    1, FORMULA_PERCENT_255,   0,  100},
    {0x05, "ECT",       "Coolant temperature",          "C",    1, FORMULA_TEMP_40,     -40,  215},
    {0x06, "STFT1",     "Short term fuel trim bank 1",  "%",    1, FORMULA_FUEL_TRIM,  -100,  100},
    {0x07, "LTFT1",     "Long term fuel trim bank 1",   "%",    1, FORMULA_FUEL_TRIM,  -100,  100},
    {0x08, "STFT2",     "Short term fuel trim bank 2",  "%",    1, FORMULA_FUEL_TRIM,  -100,  100},
    {0x09, "LTFT2",     "Long term fuel trim bank 2",   "%",    1, FORMULA_FUEL_TRIM,  -100,  100},
    {0x0A, "FRP",       "Fuel pressure",                "kPa",  1, FORMULA_FUEL_PRESSURE, 0,  765},
    {0x0B, "MAP",       "Intake manifold pressure",     "kPa",  1, FORMULA_DIRECT,        0,  255},
    {0x0C, "RPM",       "Engine RPM",                   "rpm",  2, FORMULA_RPM,           0, 16384},
    {0x0D, "SPEED",     "Vehicle speed",                "km/h", 1, FORMULA_SPEED_KPH,     0,  255},
    {0x0E, "TIMING",    "Timing advance",               "deg",  1, FORMULA_TIMING,      -64,   64},
    {0x0F, "IAT",       "Intake air temperature",       "C",    1, FORMULA_TEMP_40,     -40,  215},
    {0x10, "MAF",       "MAF air flow rate",            "g/s",  2, FORMULA_MAF,           0,  656},
    {0x11, "TP",        "Throttle position",            "%",    1, FORMULA_PERCENT_255,   0,  100},
    {0x12, "SEC_AIR",   "Secondary air status",         "",     1, FORMULA_DIRECT,        0,    0},
    {0x13, "O2_LOC",    "O2 sensors present",           "",     1, FORMULA_DIRECT,        0,    0},
    {0x14, "O2B1S1",    "O2 sensor B1S1 voltage",       "V",    2, FORMULA_O2_VOLTAGE,    0,    1},
    {0x15, "O2B1S2",    "O2 sensor B1S2 voltage",       "V",    2, FORMULA_O2_VOLTAGE,    0,    1},
    {0x1C, "OBD_STD",   "OBD standards compliance",     "",     1, FORMULA_DIRECT,        0,    0},
    {0x1F, "RUN_TIME",  "Run time since start",         "sec",  2, FORMULA_RUNTIME,       0, 65535},
    {0x20, "PIDS_B",    "PIDs supported 21-40",         "",     4, FORMULA_DIRECT,        0,    0},
    {0x21, "DIST_MIL",  "Distance with MIL on",         "km",   2, FORMULA_ABS_PRESSURE,  0, 65535},
    {0x2C, "EGR_CMD",   "Commanded EGR",                "%",    1, FORMULA_PERCENT_255,   0,  100},
    {0x2E, "EVAP_CMD",  "Commanded evap purge",         "%",    1, FORMULA_PERCENT_255,   0,  100},
    {0x2F, "FUEL_LVL",  "Fuel level input",             "%",    1, FORMULA_PERCENT_255,   0,  100},
    {0x30, "WARMUPS",   "Warmups since codes cleared",  "",     1, FORMULA_DIRECT,        0,  255},
    {0x31, "DIST_CLR",  "Distance since codes cleared", "km",   2, FORMULA_ABS_PRESSURE,  0, 65535},
    {0x33, "BARO",      "Barometric pressure",          "kPa",  1, FORMULA_DIRECT,        0,  255},
    {0x40, "PIDS_C",    "PIDs supported 41-60",         "",     4, FORMULA_DIRECT,        0,    0},
    {0x42, "VPWR",      "Control module voltage",       "V",    2, FORMULA_MAF,           0,   66},
    {0x43, "LOAD_ABS",  "Absolute load value",          "%",    2, FORMULA_PERCENT_255,   0,  256},
    {0x44, "LAMBDA",    "Commanded equiv ratio",        "",     2, FORMULA_MAF,           0,    2},
    {0x45, "TP_REL",    "Relative throttle position",   "%",    1, FORMULA_PERCENT_255,   0,  100},
    {0x46, "AAT",       "Ambient air temperature",      "C",    1, FORMULA_TEMP_40,     -40,  215},
    {0x47, "TP_B",      "Absolute throttle position B", "%",    1, FORMULA_PERCENT_255,   0,  100},
    {0x49, "TP_D",      "Accelerator pedal position D", "%",    1, FORMULA_PERCENT_255,   0,  100},
    {0x4A, "TP_E",      "Accelerator pedal position E", "%",    1, FORMULA_PERCENT_255,   0,  100},
    {0x4C, "TAC_CMD",   "Commanded throttle actuator",  "%",    1, FORMULA_PERCENT_255,   0,  100},
    {0x4D, "RUN_MIL",   "Time run with MIL on",         "min",  2, FORMULA_ABS_PRESSURE,  0, 65535},
    {0x4E, "CLR_TIME",  "Time since codes cleared",     "min",  2, FORMULA_ABS_PRESSURE,  0, 65535},
    {0x5C, "OIL_TEMP",  "Engine oil temperature",       "C",    1, FORMULA_TEMP_40,     -40,  210},
};

static const uint8_t OBD2_PID_COUNT = sizeof(obd2_pids) / sizeof(obd2_pids[0]);

// ═══════════════════════════════════════════════════════════════════════════
// Nissan Extended PIDs (Service 0x21)
// Format: First byte is group, second byte is PID within group
// ═══════════════════════════════════════════════════════════════════════════

struct nissan_pid_def_t {
    uint16_t pid;           // 2-byte PID (e.g., 0x1103)
    const char* name;       // Short name
    const char* desc;       // Description
    const char* unit;       // Unit string
    uint8_t bytes;          // Number of data bytes
    obd2_formula_t formula; // Conversion formula
};

static const nissan_pid_def_t nissan_pids[] = {
    // PID, Name, Description, Unit, Bytes, Formula
    {0x1101, "ECT_N",    "Coolant temp (Nissan)",     "C",   1, FORMULA_TEMP_50},
    {0x1103, "VBAT",     "Battery voltage",            "V",   1, FORMULA_VOLTAGE_8},
    {0x1104, "FUEL_T",   "Fuel temperature",           "C",   1, FORMULA_TEMP_50},
    {0x1105, "IAT_N",    "Intake air temp (Nissan)",   "C",   1, FORMULA_TEMP_50},
    {0x110C, "IGN_TIM",  "Ignition timing",            "deg", 1, FORMULA_TIMING},
    {0x1114, "FUEL_N",   "Fuel level (Nissan)",        "",    1, FORMULA_DIRECT},
    {0x1116, "SPEED_N",  "Vehicle speed (Nissan)",     "km/h",1, FORMULA_DIRECT},
    {0x111F, "AAC_VLV",  "AAC valve",                  "%",   1, FORMULA_PERCENT_255},
    {0x1121, "O2_LR",    "O2 sensor left rear",        "mV",  1, FORMULA_DIRECT},
    {0x1122, "O2_RR",    "O2 sensor right rear",       "mV",  1, FORMULA_DIRECT},
    {0x1140, "AF_A1",    "A/F alpha (left)",           "%",   1, FORMULA_DIRECT},
    {0x1141, "AF_A2",    "A/F alpha (right)",          "%",   1, FORMULA_DIRECT},
    {0x1147, "FUEL_INJ", "Fuel injector time",         "ms",  2, FORMULA_DIRECT},
};

static const uint8_t NISSAN_PID_COUNT = sizeof(nissan_pids) / sizeof(nissan_pids[0]);

// ═══════════════════════════════════════════════════════════════════════════
// PID Lookup Functions
// ═══════════════════════════════════════════════════════════════════════════

inline const obd2_pid_def_t* find_obd2_pid(uint8_t pid) {
    for (uint8_t i = 0; i < OBD2_PID_COUNT; i++) {
        if (obd2_pids[i].pid == pid) {
            return &obd2_pids[i];
        }
    }
    return nullptr;
}

inline const nissan_pid_def_t* find_nissan_pid(uint16_t pid) {
    for (uint8_t i = 0; i < NISSAN_PID_COUNT; i++) {
        if (nissan_pids[i].pid == pid) {
            return &nissan_pids[i];
        }
    }
    return nullptr;
}

// ═══════════════════════════════════════════════════════════════════════════
// Value Conversion Function
// ═══════════════════════════════════════════════════════════════════════════

inline float convert_pid_value(const uint8_t* data, uint8_t len, obd2_formula_t formula) {
    uint8_t A = (len >= 1) ? data[0] : 0;
    uint8_t B = (len >= 2) ? data[1] : 0;

    switch (formula) {
        case FORMULA_DIRECT:
            return (float)A;
        case FORMULA_PERCENT_255:
            return A * 100.0f / 255.0f;
        case FORMULA_TEMP_40:
            return (float)A - 40.0f;
        case FORMULA_TEMP_50:
            return (float)A - 50.0f;
        case FORMULA_RPM:
            return ((A * 256.0f) + B) / 4.0f;
        case FORMULA_SPEED_KPH:
            return (float)A;
        case FORMULA_TIMING:
            return (A - 128.0f) / 2.0f;
        case FORMULA_MAF:
            return ((A * 256.0f) + B) / 100.0f;
        case FORMULA_FUEL_TRIM:
            return (A - 128.0f) * 100.0f / 128.0f;
        case FORMULA_FUEL_PRESSURE:
            return A * 3.0f;
        case FORMULA_VOLTAGE_8:
            return A / 12.5f;
        case FORMULA_O2_VOLTAGE:
            return A / 200.0f;
        case FORMULA_O2_CURRENT:
            return ((A * 256.0f) + B) / 256.0f - 128.0f;
        case FORMULA_EVAP_PRESSURE:
            return ((int16_t)((A << 8) | B)) / 4.0f;
        case FORMULA_ABS_PRESSURE:
            return (A * 256.0f) + B;
        case FORMULA_RUNTIME:
            return (A * 256.0f) + B;
        default:
            return (float)A;
    }
}

#endif // OBD2_PIDS_H
