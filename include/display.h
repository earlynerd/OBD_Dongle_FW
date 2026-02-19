#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include "can_types.h"
#include "can_signals_370z.h"

// ═══════════════════════════════════════════════════════════════════════════
// Display / Print Functions
// ═══════════════════════════════════════════════════════════════════════════

// Check if serial TX buffer has enough space (non-blocking print guard)
bool serial_can_print(size_t bytes_needed = 128);

// Print raw CAN frame to serial
void print_frame(const can_msg_t &msg);

// Print CAN frame with delta highlighting (changed bytes marked with *)
void print_delta(const can_msg_t &msg, const uint8_t *old_data, uint8_t old_dlc);

// Extract a signal value from CAN data
float extract_signal(const uint8_t *data, uint8_t dlc, const can_signal_t &sig);

// Check if a signal value changed between old and new data
bool signal_changed(const uint8_t *old_data, const uint8_t *new_data,
                    uint8_t old_dlc, uint8_t new_dlc, const can_signal_t &sig);

// Print decoded message (only changed signals if has_old_data)
void print_decoded(const can_msg_t &msg, const uint8_t *old_data,
                   uint8_t old_dlc, bool has_old_data);

#endif // DISPLAY_H
