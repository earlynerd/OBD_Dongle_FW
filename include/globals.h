#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <map>
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
// Delta Tracking (only accessed from dispatch task)
// ═══════════════════════════════════════════════════════════════════════════

extern std::map<uint32_t, id_tracker_t> id_trackers;

// ═══════════════════════════════════════════════════════════════════════════
// Statistics Counters (atomic-ish, only written from dispatch task)
// ═══════════════════════════════════════════════════════════════════════════

extern volatile uint32_t total_frames;
extern volatile uint32_t dropped_frames;

// ═══════════════════════════════════════════════════════════════════════════
// OBD2 TX State (protected by state_mutex where noted)
// ═══════════════════════════════════════════════════════════════════════════

extern bool tx_enabled;                 // TX mode enabled (mutex protected)
extern obd2_state_t obd2_state;         // Response state machine
extern obd2_response_t obd2_response;   // Last response received
extern uint32_t obd2_request_time;      // When request was sent (for timeout)

#endif // GLOBALS_H
