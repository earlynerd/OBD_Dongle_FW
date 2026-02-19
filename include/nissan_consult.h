#ifndef NISSAN_CONSULT_H
#define NISSAN_CONSULT_H

#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════
// Nissan Consult Module Definitions (370Z / VQ37VHR)
// ═══════════════════════════════════════════════════════════════════════════

// Module definition structure
struct consult_module_t {
    const char* name;           // Short name (3-4 chars)
    const char* description;    // Full name
    uint32_t request_id;        // CAN ID for requests
    uint32_t response_id;       // CAN ID for responses
};

// Known modules on Nissan 370Z CAN bus
static const consult_module_t consult_modules[] = {
    {"ECM",  "Engine Control Module",           0x7E0, 0x7E8},
    {"TCM",  "Transmission Control Module",     0x7E1, 0x7E9},
    {"ABS",  "ABS/VDC Module",                  0x740, 0x748},
    {"BCM",  "Body Control Module",             0x797, 0x79A},
    {"IPDM", "Intelligent Power Distribution",  0x747, 0x74F},
    {"EPS",  "Electric Power Steering",         0x742, 0x74A},
    {"METER","Combination Meter",               0x743, 0x74B},
    {"AC",   "Auto A/C Amplifier",              0x744, 0x74C},
    {"SRS",  "Airbag Module",                   0x772, 0x77A},
    {"TPMS", "Tire Pressure Monitor",           0x74D, 0x755},
};

static const uint8_t CONSULT_MODULE_COUNT = sizeof(consult_modules) / sizeof(consult_modules[0]);

// ═══════════════════════════════════════════════════════════════════════════
// Consult Service IDs
// ═══════════════════════════════════════════════════════════════════════════

// Standard UDS/KWP2000 services used by Nissan
#define CONSULT_SVC_START_DIAG          0x10    // Start diagnostic session
#define CONSULT_SVC_ECU_RESET           0x11    // ECU reset
#define CONSULT_SVC_READ_FREEZE         0x12    // Read freeze frame
#define CONSULT_SVC_READ_DTC            0x13    // Read DTCs (Nissan specific)
#define CONSULT_SVC_CLEAR_DTC           0x14    // Clear DTCs
#define CONSULT_SVC_READ_STATUS         0x17    // Read DTC status
#define CONSULT_SVC_READ_DATA_ID        0x21    // Read data by local ID
#define CONSULT_SVC_READ_DATA_PID       0x22    // Read data by PID (2-byte)
#define CONSULT_SVC_READ_MEM_ADDR       0x23    // Read memory by address
#define CONSULT_SVC_SECURITY_ACCESS     0x27    // Security access
#define CONSULT_SVC_WRITE_DATA_ID       0x30    // Write data by local ID
#define CONSULT_SVC_IO_CONTROL          0x30    // I/O control (same as write)
#define CONSULT_SVC_START_ROUTINE       0x31    // Start routine (active test)
#define CONSULT_SVC_STOP_ROUTINE        0x32    // Stop routine
#define CONSULT_SVC_REQUEST_RESULTS     0x33    // Request routine results
#define CONSULT_SVC_DOWNLOAD            0x34    // Request download
#define CONSULT_SVC_UPLOAD              0x35    // Request upload
#define CONSULT_SVC_TRANSFER_DATA       0x36    // Transfer data
#define CONSULT_SVC_TRANSFER_EXIT       0x37    // Transfer exit
#define CONSULT_SVC_TESTER_PRESENT      0x3E    // Tester present (keep-alive)

// Diagnostic session types
#define CONSULT_SESSION_DEFAULT         0x81    // Default session
#define CONSULT_SESSION_PROGRAMMING     0x85    // Programming session
#define CONSULT_SESSION_EXTENDED        0xC0    // Extended session

// ═══════════════════════════════════════════════════════════════════════════
// ECM PIDs (0x7E0/0x7E8)
// ═══════════════════════════════════════════════════════════════════════════

struct consult_pid_t {
    uint16_t pid;           // PID (1 or 2 bytes)
    const char* name;       // Short name
    const char* unit;       // Unit
    uint8_t bytes;          // Response data bytes
    float scale;            // Multiply by this
    float offset;           // Add this
};

// ECM PIDs (Service 0x21)
static const consult_pid_t ecm_pids[] = {
    {0x01, "ECM_ID",     "",     20, 1.0f,   0.0f},   // ECM identification
    {0x06, "IGN_TIM",    "deg",   1, 1.0f, -64.0f},   // Ignition timing
    {0x08, "RPM",        "rpm",   2, 12.5f,  0.0f},   // Engine RPM
    {0x0A, "MAF_V",      "V",     2, 0.005f, 0.0f},   // MAF sensor voltage
    {0x0C, "TPS_V",      "V",     2, 0.005f, 0.0f},   // Throttle sensor voltage
    {0x0E, "ECT",        "C",     1, 1.0f, -50.0f},   // Coolant temp
    {0x10, "O2_L",       "mV",    1, 10.0f,  0.0f},   // Left O2 sensor
    {0x11, "O2_R",       "mV",    1, 10.0f,  0.0f},   // Right O2 sensor
    {0x12, "VSS",        "km/h",  1, 2.0f,   0.0f},   // Vehicle speed
    {0x14, "BAT",        "V",     1, 0.08f,  0.0f},   // Battery voltage
    {0x16, "IAT",        "C",     1, 1.0f, -50.0f},   // Intake air temp
    {0x1A, "FUEL_INJ",   "ms",    2, 0.01f,  0.0f},   // Injector pulse width
    {0x1C, "AAC_VLV",    "%",     1, 0.5f,   0.0f},   // Idle air valve
    {0x1E, "AF_ALPHA",   "%",     1, 1.0f,   0.0f},   // A/F alpha
    {0x20, "AF_ALPHA_L", "%",     1, 1.0f,   0.0f},   // A/F alpha left
    {0x22, "AF_ALPHA_R", "%",     1, 1.0f,   0.0f},   // A/F alpha right
    {0x28, "LOAD",       "%",     1, 1.0f,   0.0f},   // Engine load
    {0x2A, "MAF",        "g/s",   2, 0.01f,  0.0f},   // MAF flow rate
};

static const uint8_t ECM_PID_COUNT = sizeof(ecm_pids) / sizeof(ecm_pids[0]);

// ═══════════════════════════════════════════════════════════════════════════
// BCM PIDs (0x797/0x79A)
// ═══════════════════════════════════════════════════════════════════════════

static const consult_pid_t bcm_pids[] = {
    {0x01, "BCM_ID",     "",      20, 1.0f, 0.0f},    // BCM identification
    {0x10, "DOOR_SW",    "",       1, 1.0f, 0.0f},    // Door switches
    {0x11, "LOCK_SW",    "",       1, 1.0f, 0.0f},    // Lock switches
    {0x12, "LIGHT_SW",   "",       1, 1.0f, 0.0f},    // Light switches
    {0x14, "KEY_ID",     "",       4, 1.0f, 0.0f},    // Key ID status
    {0x20, "LAMP_OUT",   "",       2, 1.0f, 0.0f},    // Lamp output status
    {0x22, "WIPER_ST",   "",       1, 1.0f, 0.0f},    // Wiper status
    {0x24, "WINDOW_ST",  "",       1, 1.0f, 0.0f},    // Window status
};

static const uint8_t BCM_PID_COUNT = sizeof(bcm_pids) / sizeof(bcm_pids[0]);

// ═══════════════════════════════════════════════════════════════════════════
// IPDM PIDs (0x747/0x74F)
// ═══════════════════════════════════════════════════════════════════════════

static const consult_pid_t ipdm_pids[] = {
    {0x01, "IPDM_ID",    "",       20, 1.0f, 0.0f},   // IPDM identification
    {0x10, "RELAY_ST",   "",        2, 1.0f, 0.0f},   // Relay status bitmap
    {0x11, "FUSE_ST",    "",        2, 1.0f, 0.0f},   // Fuse status bitmap
    {0x12, "INPUT_ST",   "",        2, 1.0f, 0.0f},   // Input status
    {0x14, "OUTPUT_ST",  "",        2, 1.0f, 0.0f},   // Output status
    {0x20, "IGN_ST",     "",        1, 1.0f, 0.0f},   // Ignition status
    {0x22, "START_ST",   "",        1, 1.0f, 0.0f},   // Starter status
    {0x24, "FUEL_PMP",   "",        1, 1.0f, 0.0f},   // Fuel pump relay
};

static const uint8_t IPDM_PID_COUNT = sizeof(ipdm_pids) / sizeof(ipdm_pids[0]);

// ═══════════════════════════════════════════════════════════════════════════
// ABS/VDC PIDs (0x740/0x748)
// ═══════════════════════════════════════════════════════════════════════════

static const consult_pid_t abs_pids[] = {
    {0x01, "ABS_ID",     "",      20, 1.0f,   0.0f},  // ABS identification
    {0x10, "WHL_SPD_FL", "km/h",   2, 0.01f,  0.0f},  // Wheel speed FL
    {0x11, "WHL_SPD_FR", "km/h",   2, 0.01f,  0.0f},  // Wheel speed FR
    {0x12, "WHL_SPD_RL", "km/h",   2, 0.01f,  0.0f},  // Wheel speed RL
    {0x13, "WHL_SPD_RR", "km/h",   2, 0.01f,  0.0f},  // Wheel speed RR
    {0x20, "BRAKE_SW",   "",       1, 1.0f,   0.0f},  // Brake switch
    {0x22, "VDC_ST",     "",       1, 1.0f,   0.0f},  // VDC status
    {0x24, "ABS_ST",     "",       1, 1.0f,   0.0f},  // ABS status
    {0x30, "STEER_ANG",  "deg",    2, 0.1f,   0.0f},  // Steering angle
    {0x32, "YAW_RATE",   "deg/s",  2, 0.01f,  0.0f},  // Yaw rate
    {0x34, "LAT_G",      "g",      2, 0.001f, 0.0f},  // Lateral G
};

static const uint8_t ABS_PID_COUNT = sizeof(abs_pids) / sizeof(abs_pids[0]);

// ═══════════════════════════════════════════════════════════════════════════
// Module Lookup Functions
// ═══════════════════════════════════════════════════════════════════════════

inline const consult_module_t* find_module_by_name(const char* name) {
    for (uint8_t i = 0; i < CONSULT_MODULE_COUNT; i++) {
        // Case-insensitive compare
        const char* a = name;
        const char* b = consult_modules[i].name;
        bool match = true;
        while (*a && *b) {
            char ca = (*a >= 'a' && *a <= 'z') ? (*a - 32) : *a;
            char cb = (*b >= 'a' && *b <= 'z') ? (*b - 32) : *b;
            if (ca != cb) {
                match = false;
                break;
            }
            a++;
            b++;
        }
        if (match && *a == '\0' && *b == '\0') {
            return &consult_modules[i];
        }
    }
    return nullptr;
}

inline const consult_module_t* find_module_by_response_id(uint32_t id) {
    for (uint8_t i = 0; i < CONSULT_MODULE_COUNT; i++) {
        if (consult_modules[i].response_id == id) {
            return &consult_modules[i];
        }
    }
    return nullptr;
}

inline const consult_pid_t* find_module_pid(const char* module_name, uint16_t pid) {
    // Determine which PID table to use based on module
    const consult_pid_t* pid_table = nullptr;
    uint8_t pid_count = 0;

    if (strcasecmp(module_name, "ECM") == 0) {
        pid_table = ecm_pids;
        pid_count = ECM_PID_COUNT;
    } else if (strcasecmp(module_name, "BCM") == 0) {
        pid_table = bcm_pids;
        pid_count = BCM_PID_COUNT;
    } else if (strcasecmp(module_name, "IPDM") == 0) {
        pid_table = ipdm_pids;
        pid_count = IPDM_PID_COUNT;
    } else if (strcasecmp(module_name, "ABS") == 0) {
        pid_table = abs_pids;
        pid_count = ABS_PID_COUNT;
    }

    if (pid_table) {
        for (uint8_t i = 0; i < pid_count; i++) {
            if (pid_table[i].pid == pid) {
                return &pid_table[i];
            }
        }
    }
    return nullptr;
}

// ═══════════════════════════════════════════════════════════════════════════
// Active Test Definitions (for future use)
// ═══════════════════════════════════════════════════════════════════════════

struct consult_active_test_t {
    uint16_t routine_id;
    const char* name;
    const char* description;
    const char* module;         // Which module supports this
};

// Known active tests (primarily for reference, use with extreme caution)
static const consult_active_test_t active_tests[] = {
    {0x0001, "INJ_1",      "Injector 1 test",          "ECM"},
    {0x0002, "INJ_2",      "Injector 2 test",          "ECM"},
    {0x0003, "INJ_3",      "Injector 3 test",          "ECM"},
    {0x0004, "INJ_4",      "Injector 4 test",          "ECM"},
    {0x0005, "INJ_5",      "Injector 5 test",          "ECM"},
    {0x0006, "INJ_6",      "Injector 6 test",          "ECM"},
    {0x0010, "FUEL_PMP",   "Fuel pump relay test",     "IPDM"},
    {0x0020, "FAN_LO",     "Cooling fan low test",     "IPDM"},
    {0x0021, "FAN_HI",     "Cooling fan high test",    "IPDM"},
    {0x0030, "HORN",       "Horn test",                "BCM"},
    {0x0040, "HEADLIGHT",  "Headlight test",           "BCM"},
    {0x0041, "TAIL_LIGHT", "Tail light test",          "BCM"},
    {0x0050, "DOOR_LOCK",  "Door lock test",           "BCM"},
    {0x0051, "DOOR_UNLOCK","Door unlock test",         "BCM"},
};

static const uint8_t ACTIVE_TEST_COUNT = sizeof(active_tests) / sizeof(active_tests[0]);

#endif // NISSAN_CONSULT_H
