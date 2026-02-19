#include <Arduino.h>
#include "display.h"
#include "can_signals_370z.h"

// ═══════════════════════════════════════════════════════════════════════════
// Serial Utility
// ═══════════════════════════════════════════════════════════════════════════

bool serial_can_print(size_t bytes_needed)
{
    return Serial.availableForWrite() >= (int)bytes_needed;
}

// ═══════════════════════════════════════════════════════════════════════════
// Frame Printing
// ═══════════════════════════════════════════════════════════════════════════

void print_frame(const can_msg_t &msg)
{
    const CAN_frame_t &f = msg.frame;
    char line[128];
    int pos = snprintf(line, sizeof(line), "[%010lu] %s 0x%03X [%d] ",
                       msg.timestamp_ms,
                       (f.FIR.B.FF == CAN_frame_std) ? "STD" : "EXT",
                       f.MsgID,
                       f.FIR.B.DLC);

    if (f.FIR.B.RTR == CAN_RTR)
    {
        snprintf(line + pos, sizeof(line) - pos, "RTR");
    }
    else
    {
        for (int i = 0; i < f.FIR.B.DLC && i < 8; i++)
        {
            pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", f.data.u8[i]);
        }
    }
    Serial.println(line);
}

void print_delta(const can_msg_t &msg, const uint8_t *old_data, uint8_t old_dlc)
{
    const CAN_frame_t &f = msg.frame;
    char line[192];
    int pos = snprintf(line, sizeof(line), "[%010lu] DELTA 0x%03X [%d] ",
                       msg.timestamp_ms, f.MsgID, f.FIR.B.DLC);

    for (int i = 0; i < f.FIR.B.DLC && i < 8; i++)
    {
        bool changed = (i >= old_dlc) || (f.data.u8[i] != old_data[i]);
        if (changed)
        {
            pos += snprintf(line + pos, sizeof(line) - pos, "*%02X ", f.data.u8[i]);
        }
        else
        {
            pos += snprintf(line + pos, sizeof(line) - pos, " %02X ", f.data.u8[i]);
        }
    }
    Serial.println(line);
}

// ═══════════════════════════════════════════════════════════════════════════
// Signal Extraction
// ═══════════════════════════════════════════════════════════════════════════

float extract_signal(const uint8_t *data, uint8_t dlc, const can_signal_t &sig)
{
    float raw = 0;

    if (sig.start_bit == 0xFF)
    {
        // Whole byte or word extraction
        if (sig.bit_length == 8)
        {
            if (sig.start_byte < dlc)
            {
                raw = data[sig.start_byte];
            }
        }
        else if (sig.bit_length == 16)
        {
            if (sig.start_byte + 1 < dlc)
            {
                if (sig.type == SIG_UINT16_LE || sig.type == SIG_INT16_LE)
                {
                    // Little-endian (low byte first)
                    raw = data[sig.start_byte] | (data[sig.start_byte + 1] << 8);
                }
                else
                {
                    // Big-endian (high byte first)
                    raw = (data[sig.start_byte] << 8) | data[sig.start_byte + 1];
                }
                // Sign extend for signed types
                if (sig.type == SIG_INT16 || sig.type == SIG_INT16_LE)
                {
                    if (raw > 32767)
                        raw -= 65536;
                }
            }
        }
        else if (sig.bit_length == 24)
        {
            // 24-bit big-endian (high byte first)
            if (sig.start_byte + 2 < dlc)
            {
                raw = ((uint32_t)data[sig.start_byte] << 16) |
                      ((uint32_t)data[sig.start_byte + 1] << 8) |
                      data[sig.start_byte + 2];
            }
        }
        else if (sig.bit_length == 10)
        {
            // Special case: 10-bit value spanning byte + upper 2 bits of next byte
            // Used for pedal position: (D << 2) | (E >> 6)
            if (sig.start_byte + 1 < dlc)
            {
                raw = (data[sig.start_byte] << 2) | (data[sig.start_byte + 1] >> 6);
            }
        }
    }
    else
    {
        // Bit field extraction
        if (sig.start_byte < dlc)
        {
            if (sig.bit_length == 1)
            {
                raw = (data[sig.start_byte] >> sig.start_bit) & 0x01;
            }
            else
            {
                uint8_t mask = (1 << sig.bit_length) - 1;
                raw = (data[sig.start_byte] >> sig.start_bit) & mask;
            }
        }
    }

    return raw * sig.scale + sig.offset;
}

bool signal_changed(const uint8_t *old_data, const uint8_t *new_data,
                    uint8_t old_dlc, uint8_t new_dlc, const can_signal_t &sig)
{
    float old_val = extract_signal(old_data, old_dlc, sig);
    float new_val = extract_signal(new_data, new_dlc, sig);

    // For booleans, exact comparison
    if (sig.type == SIG_BOOL)
    {
        return (old_val != 0) != (new_val != 0);
    }

    // For numeric values, check for meaningful change
    float diff = new_val - old_val;
    if (diff < 0)
        diff = -diff;
    return diff > 0.01f; // Ignore tiny floating point differences
}

// ═══════════════════════════════════════════════════════════════════════════
// Decoded Message Printing
// ═══════════════════════════════════════════════════════════════════════════

void print_decoded(const can_msg_t &msg, const uint8_t *old_data,
                   uint8_t old_dlc, bool has_old_data)
{
    const CAN_frame_t &f = msg.frame;
    const can_message_t *def = find_message(f.MsgID);

    if (!def)
    {
        // Unknown message - fall back to raw hex
        print_frame(msg);
        return;
    }

    static const size_t LINE_SIZE = 384;
    char line[LINE_SIZE];
    size_t pos = snprintf(line, LINE_SIZE, "[%010lu] %s:", msg.timestamp_ms, def->name);

    bool any_changed = false;

    for (uint8_t i = 0; i < def->signal_count && pos < LINE_SIZE - 40; i++)
    {
        const can_signal_t &sig = def->signals[i];

        // Skip unchanged signals if we have old data
        if (has_old_data && !signal_changed(old_data, f.data.u8, old_dlc, f.FIR.B.DLC, sig))
        {
            continue;
        }

        float val = extract_signal(f.data.u8, f.FIR.B.DLC, sig);
        any_changed = true;

        if (sig.type == SIG_BOOL)
        {
            pos += snprintf(line + pos, LINE_SIZE - pos, " %s=%s",
                            sig.name, val != 0 ? "ON" : "OFF");
        }
        else if (sig.scale == 1.0f && sig.offset == 0.0f)
        {
            // Integer display
            pos += snprintf(line + pos, LINE_SIZE - pos, " %s=%d%s",
                            sig.name, (int)val, sig.unit);
        }
        else
        {
            // Float display
            pos += snprintf(line + pos, LINE_SIZE - pos, " %s=%.1f%s",
                            sig.name, val, sig.unit);
        }

        pos += snprintf(line + pos, LINE_SIZE - pos, " |");
    }

    // Only print if something changed (or new message)
    if (any_changed)
    {
        // Remove trailing " |"
        if (pos > 2 && pos < LINE_SIZE && line[pos - 1] == '|')
        {
            line[pos - 2] = '\0';
        }
        Serial.println(line);
    }
}
