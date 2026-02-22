#ifndef CAN_TYPES_H
#define CAN_TYPES_H

#include <stdint.h>
#include <ESP32CAN.h>
#include <CAN_config.h>

// ═══════════════════════════════════════════════════════════════════════════
// Configuration Constants
// ═══════════════════════════════════════════════════════════════════════════

static constexpr int CAN_RX_QUEUE_SIZE = 64;
static constexpr int DISPATCH_QUEUE_SIZE = 64;
static constexpr int LOG_QUEUE_SIZE = 512;          // ~500ms buffer at 1000 msg/sec
static constexpr int CMD_BUF_SIZE = 128;
static constexpr uint32_t STATS_PRINT_INTERVAL = 5000;  // ms
static constexpr int MAX_TRACKED_IDS = 256;
static constexpr uint32_t LOG_FLUSH_INTERVAL = 2000;    // ms
static constexpr uint32_t VBAT_READ_INTERVAL = 10000;   // ms
static const char *SETTINGS_FILE = "/settings.json";

// ═══════════════════════════════════════════════════════════════════════════
// Types
// ═══════════════════════════════════════════════════════════════════════════

// Timestamped CAN frame for internal use
struct can_msg_t
{
    CAN_frame_t frame;
    uint32_t timestamp_ms;
};

// Per-ID tracking for delta detection and statistics
struct id_tracker_t
{
    uint8_t last_data[8];
    uint8_t last_dlc;
    uint32_t msg_count;
    uint32_t last_seen_ms;
    uint32_t first_seen_ms;
    bool seen;
};

// Operating modes
enum op_mode_t
{
    MODE_MONITOR,   // Print all (or filtered) frames to serial
    MODE_DELTA,     // Only print when payload changes for a given ID
    MODE_SILENT,    // Log to SD only, no serial output of frames
    MODE_STATS,     // Periodic statistics summary
    MODE_DECODE,    // Decoded signal values (delta, only show changes)
    MODE_HUNT       // Bit-level change detection from snapshot baseline
};

// Hunt mode snapshot for baseline comparison
struct hunt_snapshot_t
{
    uint8_t data[8];           // Baseline payload
    uint8_t dlc;               // Baseline DLC
    uint8_t change_counts[8];  // Per-byte: how many times any bit changed
    bool captured;             // Whether this ID was in the snapshot
};

// Hunt mode bit identification key (for associating marks with bits)
struct hunt_bit_key_t
{
    uint32_t can_id;
    uint8_t byte_idx;
    uint8_t bit_idx;

    bool operator<(const hunt_bit_key_t &other) const
    {
        if (can_id != other.can_id) return can_id < other.can_id;
        if (byte_idx != other.byte_idx) return byte_idx < other.byte_idx;
        return bit_idx < other.bit_idx;
    }
};

// Noise filter entry - masks bits that should be ignored in delta detection
// When (data[byte_idx] ^ old_data[byte_idx]) & ~mask[byte_idx] == 0, frame is suppressed
struct noise_filter_t
{
    uint8_t mask[8];    // Bits set to 1 are IGNORED (noisy), 0 means monitor this bit
    bool active;        // Whether this filter is in use
};

// Maximum noise filter entries
static constexpr int MAX_NOISE_FILTERS = 64;

#endif // CAN_TYPES_H
