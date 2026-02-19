#ifndef OBD2_TX_H
#define OBD2_TX_H

#include <stdint.h>
#include <ESP32CAN.h>
#include "nissan_consult.h"

// ═══════════════════════════════════════════════════════════════════════════
// OBD2 TX Safety
// ═══════════════════════════════════════════════════════════════════════════

// Check if it's safe to transmit on CAN bus
// Returns true only if TX is enabled AND engine is off AND vehicle is stopped
bool is_safe_to_transmit();

// Get reason why TX is blocked, or nullptr if safe
const char* get_tx_block_reason();

// ═══════════════════════════════════════════════════════════════════════════
// OBD2 Request/Response
// ═══════════════════════════════════════════════════════════════════════════

// Send an OBD2 request frame
// Returns true if frame was sent, false if blocked by safety
bool obd2_send_request(uint32_t can_id, uint8_t service, uint8_t pid,
                       uint8_t sub_pid = 0, bool has_sub = false);

// Send a DTC read request (Service 0x03)
bool obd2_read_dtc();

// Parse OBD2 response frame and update state
void obd2_parse_response(const CAN_frame_t &frame);

// Print decoded OBD2 response for standard PIDs
void obd2_print_response();

// Wait for OBD2 response with timeout
// Returns true if response received, false if timeout
bool obd2_wait_response(uint32_t timeout_ms = 250);

// ═══════════════════════════════════════════════════════════════════════════
// Nissan Consult Functions
// ═══════════════════════════════════════════════════════════════════════════

// Send a Consult query to a specific module
bool consult_query_module(const consult_module_t* module, uint8_t service,
                          uint8_t pid, uint8_t sub_pid = 0, bool has_sub = false);

// Wait for Consult response with timeout
bool consult_wait_response(uint32_t timeout_ms = 250);

// Print Consult response with module context
void consult_print_response(const consult_module_t* module, uint8_t pid);

// Scan for responding modules on the bus
void consult_scan_modules();

#endif // OBD2_TX_H
