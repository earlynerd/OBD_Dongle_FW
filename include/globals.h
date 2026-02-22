#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <map>
#include <atomic>
#include <CAN_config.h>
#include "can_types.h"
#include "obd2.h"

// ═══════════════════════════════════════════════════════════════════════════
// CAN Configuration
// ═══════════════════════════════════════════════════════════════════════════

extern CAN_device_t CAN_cfg;

// ═══════════════════════════════════════════════════════════════════════════
// FreeRTOS Primitives
// ═══════════════════════════════════════════════════════════════════════════

extern QueueHandle_t dispatch_queue;
extern QueueHandle_t log_queue;
extern TaskHandle_t can_rx_task_h;
extern TaskHandle_t dispatch_task_h;
extern TaskHandle_t logger_task_h;
extern TaskHandle_t cmd_task_h;
extern TaskHandle_t housekeeping_task_h;
extern SemaphoreHandle_t state_mutex;
extern SemaphoreHandle_t tracker_mutex;

// ═══════════════════════════════════════════════════════════════════════════
// Shared State (protected by state_mutex)
// ═══════════════════════════════════════════════════════════════════════════

extern op_mode_t current_mode;
extern bool logging_enabled;
extern bool sd_available;
extern uint32_t filter_id;
extern bool filter_active;
extern bool paused;

// ═══════════════════════════════════════════════════════════════════════════
// Delta Tracking (protected by tracker_mutex)
// ═══════════════════════════════════════════════════════════════════════════

extern std::map<uint32_t, id_tracker_t> id_trackers;

// ═══════════════════════════════════════════════════════════════════════════
// Statistics Counters (Atomic)
// ═══════════════════════════════════════════════════════════════════════════

extern std::atomic<uint32_t> total_frames;
extern std::atomic<uint32_t> dropped_frames;

// ═══════════════════════════════════════════════════════════════════════════
// OBD2 TX State (protected by state_mutex where noted)
// ═══════════════════════════════════════════════════════════════════════════

extern bool tx_enabled;                 // TX mode enabled (mutex protected)
extern obd2_state_t obd2_state;         // Response state machine
extern obd2_response_t obd2_response;   // Last response received
extern uint32_t obd2_request_time;      // When request was sent (for timeout)

// ═══════════════════════════════════════════════════════════════════════════
// Hunt Mode State (only accessed from dispatch task, except where noted)
// ═══════════════════════════════════════════════════════════════════════════

extern std::map<uint32_t, hunt_snapshot_t> hunt_snapshots;  // Baseline state
extern bool hunt_sticky;                // Keep changes on screen (mutex protected)
extern bool hunt_quiet;                 // Suppress noisy bits (mutex protected)
extern uint32_t hunt_change_threshold;  // Changes before suppression (default: 5)

// Hunt marking system
extern char hunt_pending_mark[32];      // Current pending mark label (mutex protected)
extern uint32_t hunt_mark_time;         // When mark was set (millis)
extern std::map<hunt_bit_key_t, String> hunt_bit_marks;  // Bit -> label associations

// ═══════════════════════════════════════════════════════════════════════════
// Noise Filter (protected by tracker_mutex)
// ═══════════════════════════════════════════════════════════════════════════

extern std::map<uint32_t, noise_filter_t> noise_filters;  // CAN ID -> filter mask
extern bool noise_filter_enabled;  // Master enable (mutex protected)

#endif // GLOBALS_H
