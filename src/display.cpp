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

void print_hunt_change(uint32_t can_id, int byte_idx, int bit_idx,
                       uint8_t old_bit, uint8_t new_bit, bool sticky,
                       const char* mark)
{
    // Format: [HUNT] 0x354 [3].5  0->1  "ButtonName"
    // Where [3] is byte index, .5 is bit index (0-7, LSB=0)
    if (sticky)
    {
        // In sticky mode, prefix with + to indicate accumulated change
        if (mark && mark[0])
        {
            Serial.printf("[HUNT+] 0x%03X [%d].%d  %d->%d  \"%s\"\r\n",
                          can_id, byte_idx, bit_idx, old_bit, new_bit, mark);
        }
        else
        {
            Serial.printf("[HUNT+] 0x%03X [%d].%d  %d->%d\r\n",
                          can_id, byte_idx, bit_idx, old_bit, new_bit);
        }
    }
    else
    {
        if (mark && mark[0])
        {
            Serial.printf("[HUNT] 0x%03X [%d].%d  %d->%d  \"%s\"\r\n",
                          can_id, byte_idx, bit_idx, old_bit, new_bit, mark);
        }
        else
        {
            Serial.printf("[HUNT] 0x%03X [%d].%d  %d->%d\r\n",
                          can_id, byte_idx, bit_idx, old_bit, new_bit);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Signal Extraction
// ═══════════════════════════════════════════════════════════════════════════

float extract_signal(const uint8_t *data, uint8_t dlc, const can_signal_t &sig)
{
    float raw = 0;

    if (sig.start_bit == 0xFF)
    {
        // Whole byte or multi-byte extraction based on type
        switch (sig.type)
        {
        case SIG_UINT8:
        case SIG_BOOL:
            if (sig.start_byte < dlc)
            {
                raw = data[sig.start_byte];
            }
            break;

        case SIG_UINT7:
            // 7-bit value (mask off high bit)
            if (sig.start_byte < dlc)
            {
                raw = data[sig.start_byte] & 0x7F;
            }
            break;

        case SIG_UINT3:
            // 3-bit value (low 3 bits)
            if (sig.start_byte < dlc)
            {
                raw = data[sig.start_byte] & 0x07;
            }
            break;

        case SIG_UINT4:
            // 4-bit value (low nibble)
            if (sig.start_byte < dlc)
            {
                raw = data[sig.start_byte] & 0x0F;
            }
            break;

        case SIG_UINT16:
        case SIG_INT16:
            // Big-endian 16-bit
            if (sig.start_byte + 1 < dlc)
            {
                raw = (data[sig.start_byte] << 8) | data[sig.start_byte + 1];
                if (sig.type == SIG_INT16 && raw > 32767)
                    raw -= 65536;
            }
            break;

        case SIG_UINT16_LE:
        case SIG_INT16_LE:
            // Little-endian 16-bit
            if (sig.start_byte + 1 < dlc)
            {
                raw = data[sig.start_byte] | (data[sig.start_byte + 1] << 8);
                if (sig.type == SIG_INT16_LE && raw > 32767)
                    raw -= 65536;
            }
            break;

        case SIG_UINT24:
            // 24-bit big-endian
            if (sig.start_byte + 2 < dlc)
            {
                raw = ((uint32_t)data[sig.start_byte] << 16) |
                      ((uint32_t)data[sig.start_byte + 1] << 8) |
                      data[sig.start_byte + 2];
            }
            break;

        case SIG_UINT12_BE:
        case SIG_INT12_BE:
            // 12-bit big-endian: high byte provides upper 8 bits, low byte upper 4 bits
            // Format: (byte[n] << 4) | (byte[n+1] >> 4)
            if (sig.start_byte + 1 < dlc)
            {
                raw = ((uint16_t)data[sig.start_byte] << 4) | (data[sig.start_byte + 1] >> 4);
                if (sig.type == SIG_INT12_BE && raw > 2047)
                    raw -= 4096;  // Sign extend from 12-bit
            }
            break;

        case SIG_UINT12_BE_LOW:
            // 12-bit: low nibble of first byte + full second byte
            // Format: (byte[n] & 0x0F) << 8 | byte[n+1]
            if (sig.start_byte + 1 < dlc)
            {
                raw = ((uint16_t)(data[sig.start_byte] & 0x0F) << 8) | data[sig.start_byte + 1];
            }
            break;

        case SIG_UINT11_BE:
            // 11-bit big-endian: (byte[n] << 3) | (byte[n+1] >> 5)
            if (sig.start_byte + 1 < dlc)
            {
                raw = ((uint16_t)data[sig.start_byte] << 3) | (data[sig.start_byte + 1] >> 5);
            }
            break;

        case SIG_UINT10_BE:
            // 10-bit big-endian: (byte[n] << 2) | (byte[n+1] >> 6)
            // Used for pedal position
            if (sig.start_byte + 1 < dlc)
            {
                raw = ((uint16_t)data[sig.start_byte] << 2) | (data[sig.start_byte + 1] >> 6);
            }
            break;

        default:
            // Legacy bit_length based extraction for compatibility
            if (sig.bit_length == 8 && sig.start_byte < dlc)
            {
                raw = data[sig.start_byte];
            }
            else if (sig.bit_length == 10 && sig.start_byte + 1 < dlc)
            {
                raw = (data[sig.start_byte] << 2) | (data[sig.start_byte + 1] >> 6);
            }
            break;
        }
    }
    else
    {
        // Bit field extraction (within a single byte or spanning bytes)
        if (sig.start_byte < dlc)
        {
            if (sig.bit_length == 1)
            {
                raw = (data[sig.start_byte] >> sig.start_bit) & 0x01;
            }
            else if (sig.bit_length <= 8 - sig.start_bit)
            {
                // Fits within single byte
                uint8_t mask = (1 << sig.bit_length) - 1;
                raw = (data[sig.start_byte] >> sig.start_bit) & mask;
            }
            else if (sig.start_byte + 1 < dlc)
            {
                // Spans two bytes - extract from start_bit in first byte
                uint8_t bits_in_first = 8 - sig.start_bit;
                uint8_t bits_in_second = sig.bit_length - bits_in_first;
                uint16_t mask_first = (1 << bits_in_first) - 1;
                uint16_t mask_second = (1 << bits_in_second) - 1;
                raw = ((data[sig.start_byte] >> sig.start_bit) & mask_first) |
                      ((data[sig.start_byte + 1] & mask_second) << bits_in_first);
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
