#ifndef OBD2_H
#define OBD2_H

#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════
// OBD2 CAN IDs
// ═══════════════════════════════════════════════════════════════════════════

// Standard OBD2 (ISO 15765-4)
#define OBD2_REQUEST_BROADCAST  0x7DF   // Broadcast request to all ECUs
#define OBD2_ECM_REQUEST        0x7E0   // ECM specific request
#define OBD2_ECM_RESPONSE       0x7E8   // ECM response

// Nissan BCM (Body Control Module)
#define OBD2_BCM_REQUEST        0x797
#define OBD2_BCM_RESPONSE       0x79A

// ═══════════════════════════════════════════════════════════════════════════
// OBD2 Service IDs (Modes)
// ═══════════════════════════════════════════════════════════════════════════

#define OBD2_SVC_CURRENT_DATA       0x01    // Show current data
#define OBD2_SVC_FREEZE_FRAME       0x02    // Show freeze frame data
#define OBD2_SVC_READ_DTC           0x03    // Show stored DTCs
#define OBD2_SVC_CLEAR_DTC          0x04    // Clear DTCs and freeze frame
#define OBD2_SVC_O2_TEST            0x05    // O2 sensor monitoring
#define OBD2_SVC_OTHER_TEST         0x06    // Other monitoring test results
#define OBD2_SVC_PENDING_DTC        0x07    // Show pending DTCs
#define OBD2_SVC_CONTROL            0x08    // Control on-board system
#define OBD2_SVC_VEHICLE_INFO       0x09    // Request vehicle information
#define OBD2_SVC_PERMANENT_DTC      0x0A    // Permanent DTCs

// Nissan Extended Services
#define OBD2_SVC_NISSAN_READ        0x21    // Nissan read data by local ID
#define OBD2_SVC_NISSAN_WRITE       0x30    // Nissan write data

// ═══════════════════════════════════════════════════════════════════════════
// OBD2 Response Codes
// ═══════════════════════════════════════════════════════════════════════════

#define OBD2_RESPONSE_OFFSET        0x40    // Response = Service + 0x40
#define OBD2_NEGATIVE_RESPONSE      0x7F    // Negative response service ID

// Negative response codes
#define OBD2_NRC_GENERAL_REJECT     0x10
#define OBD2_NRC_SERVICE_NOT_SUPPORTED 0x11
#define OBD2_NRC_SUBFUNCTION_NOT_SUPPORTED 0x12
#define OBD2_NRC_BUSY_REPEAT        0x21
#define OBD2_NRC_CONDITIONS_NOT_CORRECT 0x22
#define OBD2_NRC_REQUEST_OUT_OF_RANGE 0x31
#define OBD2_NRC_SECURITY_ACCESS_DENIED 0x33
#define OBD2_NRC_INVALID_KEY        0x35
#define OBD2_NRC_EXCEEDED_ATTEMPTS  0x36
#define OBD2_NRC_RESPONSE_PENDING   0x78

// ═══════════════════════════════════════════════════════════════════════════
// Vehicle Info PIDs (Service 0x09)
// ═══════════════════════════════════════════════════════════════════════════

#define OBD2_VIN_MSG_COUNT          0x01    // VIN message count
#define OBD2_VIN                    0x02    // Vehicle Identification Number
#define OBD2_CALIBRATION_ID         0x04    // Calibration ID
#define OBD2_CVN                    0x06    // Calibration Verification Number
#define OBD2_ECU_NAME               0x0A    // ECU name

// ═══════════════════════════════════════════════════════════════════════════
// DTC Format
// ═══════════════════════════════════════════════════════════════════════════

// DTC first character based on upper 2 bits
static const char DTC_SYSTEM_CHAR[] = {'P', 'C', 'B', 'U'};
// P = Powertrain, C = Chassis, B = Body, U = Network

// Convert 2-byte DTC to string (e.g., 0x0133 -> "P0133")
inline void dtc_to_string(uint16_t dtc, char* buf) {
    uint8_t hi = (dtc >> 8) & 0xFF;
    uint8_t lo = dtc & 0xFF;

    buf[0] = DTC_SYSTEM_CHAR[(hi >> 6) & 0x03];
    buf[1] = '0' + ((hi >> 4) & 0x03);
    buf[2] = '0' + (hi & 0x0F);
    buf[3] = '0' + ((lo >> 4) & 0x0F);
    buf[4] = '0' + (lo & 0x0F);
    buf[5] = '\0';
}

// ═══════════════════════════════════════════════════════════════════════════
// OBD2 Request/Response Structures
// ═══════════════════════════════════════════════════════════════════════════

// Maximum response data bytes (single frame)
// Mode 3 DTC can have up to 3 DTCs per frame (6 bytes)
#define OBD2_MAX_DATA_BYTES     6

// Request structure
struct obd2_request_t {
    uint32_t can_id;        // Target CAN ID (0x7DF, 0x7E0, 0x797)
    uint8_t service;        // Service ID
    uint8_t pid;            // PID (for services that use it)
    uint8_t sub_pid;        // Sub-PID (for extended PIDs like 0x21 0x11 0x03)
    bool has_sub_pid;       // Whether sub_pid is used
};

// Response state machine
enum obd2_state_t {
    OBD2_IDLE,
    OBD2_WAITING_RESPONSE,
    OBD2_RESPONSE_RECEIVED,
    OBD2_TIMEOUT,
    OBD2_ERROR
};

// Response structure
struct obd2_response_t {
    uint32_t can_id;        // Source CAN ID
    uint8_t service;        // Service ID (should be request + 0x40)
    uint8_t pid;            // PID echoed back
    uint8_t data[OBD2_MAX_DATA_BYTES];  // Response data bytes
    uint8_t data_len;       // Number of valid data bytes
    bool negative;          // True if negative response
    uint8_t nrc;            // Negative response code (if negative)
};

// ═══════════════════════════════════════════════════════════════════════════
// Constants
// ═══════════════════════════════════════════════════════════════════════════

#define OBD2_RESPONSE_TIMEOUT_MS    250     // Timeout for response
#define OBD2_PADDING_BYTE           0x55    // ISO 15765-4 padding

#endif // OBD2_H
