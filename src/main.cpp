#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <SPI.h>
#include <SD.h>
#include <map>
#include "pin_config.h"
#include "can_types.h"
#include "globals.h"
#include "display.h"
#include "obd2_tx.h"
#include "logger.h"
#include "command_handler.h"
#include "obd2.h"
#include "nissan_consult.h"

// ── Global State ─────────────────────────────────────────────────────────────

CAN_device_t CAN_cfg;

// FreeRTOS primitives
QueueHandle_t dispatch_queue;
QueueHandle_t log_queue;
TaskHandle_t can_rx_task_h;
TaskHandle_t dispatch_task_h;
TaskHandle_t logger_task_h;
TaskHandle_t cmd_task_h;
TaskHandle_t housekeeping_task_h;
SemaphoreHandle_t state_mutex;
SemaphoreHandle_t tracker_mutex;

// Shared state (protected by state_mutex)
op_mode_t current_mode = MODE_DELTA;
bool logging_enabled = false;
bool sd_available = false;
uint32_t filter_id = 0;
bool filter_active = false;
bool paused = false;

// Delta tracking (protected by tracker_mutex)
std::map<uint32_t, id_tracker_t> id_trackers;

// Stats counters (Atomic)
std::atomic<uint32_t> total_frames(0);
std::atomic<uint32_t> dropped_frames(0);

// OBD2 TX state (protected by state_mutex where noted)
bool tx_enabled = false;                    // TX mode enabled (mutex protected)
obd2_state_t obd2_state = OBD2_IDLE;        // Response state machine
obd2_response_t obd2_response;              // Last response received
uint32_t obd2_request_time = 0;             // When request was sent (for timeout)

// Hunt mode state
std::map<uint32_t, hunt_snapshot_t> hunt_snapshots;  // Baseline state
bool hunt_sticky = false;                   // Keep changes on screen (mutex protected)
bool hunt_quiet = false;                    // Suppress noisy bits (mutex protected)
uint32_t hunt_change_threshold = 5;         // Changes before suppression

// Hunt marking system
char hunt_pending_mark[32] = {0};           // Current pending mark label
uint32_t hunt_mark_time = 0;                // When mark was set
std::map<hunt_bit_key_t, String> hunt_bit_marks;  // Bit -> label associations

// Noise filter state
std::map<uint32_t, noise_filter_t> noise_filters;  // CAN ID -> filter mask
bool noise_filter_enabled = false;          // Master enable (mutex protected)

// ── CAN Receive Task ─────────────────────────────────────────────────────────
// Highest priority: pull frames from hardware FIFO ASAP and timestamp them.

static void can_rx_task(void *pv)
{
    CAN_frame_t rx_frame;
    can_msg_t msg;

    while (true)
    {
        if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, portMAX_DELAY) == pdTRUE)
        {
            msg.frame = rx_frame;
            msg.timestamp_ms = millis();
            if (xQueueSend(dispatch_queue, &msg, 0) != pdTRUE)
            {
                dropped_frames++;
            }
        }
    }
}

// ── Dispatch Task ────────────────────────────────────────────────────────────
// Applies filtering, delta detection, mode logic, and forwards to log queue.

static void dispatch_task(void *pv)
{
    can_msg_t msg;

    while (true)
    {
        if (xQueueReceive(dispatch_queue, &msg, portMAX_DELAY) != pdTRUE)
            continue;

        total_frames++;
        uint32_t id = msg.frame.MsgID;

        // Take a snapshot of current state
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        op_mode_t mode = current_mode;
        bool do_log = logging_enabled && sd_available;
        bool filt = filter_active;
        uint32_t fid = filter_id;
        bool is_paused = paused;
        bool h_sticky = hunt_sticky;
        bool h_quiet = hunt_quiet;
        bool nf_enabled = noise_filter_enabled;
        char pending_mark[32];
        strncpy(pending_mark, hunt_pending_mark, sizeof(pending_mark));
        uint32_t mark_time = hunt_mark_time;
        xSemaphoreGive(state_mutex);

        // Apply ID filter
        if (filt && id != fid)
            continue;

        // Forward to SD logger if enabled
        if (do_log)
        {
            xQueueSend(log_queue, &msg, 0);
        }

        if (is_paused)
            continue;

        // Update tracker (protected by tracker_mutex)
        xSemaphoreTake(tracker_mutex, portMAX_DELAY);
        id_tracker_t &trk = id_trackers[id];
        bool is_new = !trk.seen;
        bool is_change = false;

        if (!is_new && msg.frame.FIR.B.RTR != CAN_RTR)
        {
            if (trk.last_dlc != msg.frame.FIR.B.DLC ||
                memcmp(trk.last_data, msg.frame.data.u8, msg.frame.FIR.B.DLC) != 0)
            {
                is_change = true;
            }
        }

        uint8_t old_data[8];
        uint8_t old_dlc = trk.last_dlc;
        if (!is_new)
        {
            memcpy(old_data, trk.last_data, 8);
        }

        // Update tracker state
        if (!trk.seen)
        {
            trk.first_seen_ms = msg.timestamp_ms;
            trk.seen = true;
        }
        trk.last_seen_ms = msg.timestamp_ms;
        trk.msg_count++;
        trk.last_dlc = msg.frame.FIR.B.DLC;
        if (msg.frame.FIR.B.RTR != CAN_RTR)
        {
            memcpy(trk.last_data, msg.frame.data.u8, msg.frame.FIR.B.DLC);
        }

        // Check for Hunt mode baseline (if in hunt mode)
        bool has_snapshot = (mode == MODE_HUNT && hunt_snapshots.count(id) && hunt_snapshots[id].captured);
        hunt_snapshot_t* snap_ptr = has_snapshot ? &hunt_snapshots[id] : nullptr;
        
        xSemaphoreGive(tracker_mutex);

        // Check for OBD2/Consult response frames
        if (id == OBD2_ECM_RESPONSE || id == OBD2_BCM_RESPONSE ||
            find_module_by_response_id(id) != nullptr)
        {
            obd2_parse_response(msg.frame);
        }

        // Mode-dependent serial output
        // Skip if serial TX buffer is too full (prevents blocking/crash)
        if (!serial_can_print())
        {
            continue;  // Drop this frame's output, keep processing
        }

        switch (mode)
        {
        case MODE_MONITOR:
            print_frame(msg);
            break;

        case MODE_DELTA:
            if (is_new)
            {
                Serial.print("  NEW  ");
                print_frame(msg);
            }
            else if (is_change)
            {
                // Check noise filter - suppress if change only affects masked bits
                bool suppress = false;
                if (nf_enabled)
                {
                    xSemaphoreTake(tracker_mutex, portMAX_DELAY);
                    auto nf_it = noise_filters.find(id);
                    if (nf_it != noise_filters.end() && nf_it->second.active)
                    {
                        // Check if any unmasked bits changed
                        bool unmasked_change = false;
                        uint8_t dlc = msg.frame.FIR.B.DLC;
                        for (int i = 0; i < dlc && i < 8; i++)
                        {
                            uint8_t diff = msg.frame.data.u8[i] ^ old_data[i];
                            uint8_t unmasked_diff = diff & ~nf_it->second.mask[i];
                            if (unmasked_diff)
                            {
                                unmasked_change = true;
                                break;
                            }
                        }
                        suppress = !unmasked_change;
                    }
                    xSemaphoreGive(tracker_mutex);
                }

                if (!suppress)
                {
                    print_delta(msg, old_data, old_dlc);
                }
            }
            break;

        case MODE_DECODE:
            if (is_new)
            {
                print_decoded(msg, nullptr, 0, false);
            }
            else if (is_change)
            {
                // Check noise filter - suppress if change only affects masked bits
                bool suppress = false;
                if (nf_enabled)
                {
                    xSemaphoreTake(tracker_mutex, portMAX_DELAY);
                    auto nf_it = noise_filters.find(id);
                    if (nf_it != noise_filters.end() && nf_it->second.active)
                    {
                        bool unmasked_change = false;
                        uint8_t dlc = msg.frame.FIR.B.DLC;
                        for (int i = 0; i < dlc && i < 8; i++)
                        {
                            uint8_t diff = msg.frame.data.u8[i] ^ old_data[i];
                            uint8_t unmasked_diff = diff & ~nf_it->second.mask[i];
                            if (unmasked_diff)
                            {
                                unmasked_change = true;
                                break;
                            }
                        }
                        suppress = !unmasked_change;
                    }
                    xSemaphoreGive(tracker_mutex);
                }

                if (!suppress)
                {
                    print_decoded(msg, old_data, old_dlc, true);
                }
            }
            break;

        case MODE_STATS:
            // Stats are printed periodically by housekeeping, not per-frame
            break;

        case MODE_SILENT:
            break;

        case MODE_HUNT:
            // Bit-level change detection from snapshot baseline
            if (snap_ptr)
            {
                uint8_t dlc = msg.frame.FIR.B.DLC;

                // Check if pending mark is still valid (within 10 seconds)
                bool mark_valid = (pending_mark[0] != '\0') &&
                                  (msg.timestamp_ms - mark_time < 10000);

                for (int b = 0; b < dlc && b < 8; b++)
                {
                    uint8_t diff = snap_ptr->data[b] ^ msg.frame.data.u8[b];
                    if (diff)
                    {
                        // Update change counter (needs tracker_mutex again)
                        xSemaphoreTake(tracker_mutex, portMAX_DELAY);
                        snap_ptr->change_counts[b]++;
                        uint32_t current_change_count = snap_ptr->change_counts[b];
                        xSemaphoreGive(tracker_mutex);

                        // Skip if quiet mode and too many changes
                        if (h_quiet && current_change_count > hunt_change_threshold)
                        {
                            // Show suppression notice once (at threshold + 1)
                            if (current_change_count == hunt_change_threshold + 1)
                            {
                                Serial.printf("[HUNT] 0x%03X [%d] (suppressed - noisy)\r\n", id, b);
                            }
                            continue;
                        }

                        // Print each changed bit
                        for (int bit = 0; bit < 8; bit++)
                        {
                            if (diff & (1 << bit))
                            {
                                uint8_t old_bit = (snap_ptr->data[b] >> bit) & 1;
                                uint8_t new_bit = (msg.frame.data.u8[b] >> bit) & 1;

                                // Associate pending mark with this bit
                                const char* mark_to_use = nullptr;
                                if (mark_valid)
                                {
                                    hunt_bit_key_t key = {id, (uint8_t)b, (uint8_t)bit};
                                    
                                    xSemaphoreTake(tracker_mutex, portMAX_DELAY);
                                    hunt_bit_marks[key] = String(pending_mark);
                                    xSemaphoreGive(tracker_mutex);
                                    
                                    mark_to_use = pending_mark;

                                    // Clear pending mark (with mutex)
                                    xSemaphoreTake(state_mutex, portMAX_DELAY);
                                    hunt_pending_mark[0] = '\0';
                                    xSemaphoreGive(state_mutex);
                                    mark_valid = false;  // Only use mark once per mark command
                                }

                                print_hunt_change(id, b, bit, old_bit, new_bit, h_sticky, mark_to_use);
                            }
                        }
                    }
                }
            }
            break;
        }
    }
}

// ── Housekeeping Task ────────────────────────────────────────────────────────
// Periodic stats printing (in STATS mode), battery monitoring, card detect.

static void housekeeping_task(void *pv)
{
    uint32_t last_stats = 0;
    uint32_t last_vbat = 0;

    while (true)
    {
        uint32_t now = millis();

        // Periodic stats output
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        op_mode_t mode = current_mode;
        xSemaphoreGive(state_mutex);

        if (mode == MODE_STATS && (now - last_stats >= STATS_PRINT_INTERVAL))
        {
            last_stats = now;
            xSemaphoreTake(tracker_mutex, portMAX_DELAY);
            uint32_t unique_ids = id_trackers.size();
            Serial.printf("\r\n── Stats @ %lus ── Frames: %lu | Dropped: %lu | IDs: %lu ──\r\n",
                          now / 1000, (uint32_t)total_frames, (uint32_t)dropped_frames, unique_ids);
            print_ids();
            xSemaphoreGive(tracker_mutex);
        }

        // Periodic battery voltage check
        if (now - last_vbat >= VBAT_READ_INTERVAL)
        {
            last_vbat = now;
            float vbat = read_vbat();
            if (vbat < 11.5f && vbat > 2.0f)
            { // Sanity check: >2V means connected
                Serial.printf("[WARN] Battery low: %.2fV\r\n", vbat);
            }
        }

        // Check for power fault
        if (!digitalRead(PWR_FAULT))
        {
            Serial.println("[WARN] Power fault detected!");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ── Setup & Loop ─────────────────────────────────────────────────────────────

void setup()
{
    delay(500);
    Serial.begin(921600);  // Higher baud for heavy CAN traffic
    // Wait up to 2 seconds for serial, then continue without it (headless mode)
    uint32_t serial_wait_start = millis();
    while (!Serial && (millis() - serial_wait_start < 2000))
        ;
    // GPIO init
    pinMode(CAN_SLNT, OUTPUT);
    digitalWrite(CAN_SLNT, HIGH); // HIGH = Silent mode
    pinMode(PWR_FAULT, INPUT_PULLUP);
    pinMode(VBAT_DIV, INPUT);
    pinMode(SD_CD, INPUT_PULLUP);

    // CAN init
    CAN_cfg.speed = CAN_SPEED_500KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_5;
    CAN_cfg.rx_pin_id = GPIO_NUM_4;
    CAN_cfg.rx_queue = xQueueCreate(CAN_RX_QUEUE_SIZE, sizeof(CAN_frame_t));

    if (ESP32Can.CANInit() != 0)
    {
        Serial.println("[ERR] CAN init failed!");
    }

    // Queues
    state_mutex = xSemaphoreCreateMutex();
    tracker_mutex = xSemaphoreCreateMutex();
    dispatch_queue = xQueueCreate(DISPATCH_QUEUE_SIZE, sizeof(can_msg_t));
    log_queue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(can_msg_t));

    // Tasks — priorities: CAN RX (highest) > dispatch > logger > housekeeping > cmd
    xTaskCreatePinnedToCore(can_rx_task, "can_rx", 4096, NULL, 6, &can_rx_task_h, 1);
    xTaskCreatePinnedToCore(dispatch_task, "dispatch", 16384, NULL, 5, &dispatch_task_h, 1);
    xTaskCreatePinnedToCore(logger_task, "sd_logger", 8192, NULL, 3, &logger_task_h, 0);
    xTaskCreatePinnedToCore(cmd_task, "cmd", 8192, NULL, 2, &cmd_task_h, 0);
    xTaskCreatePinnedToCore(housekeeping_task, "housekeeping", 4096, NULL, 1, &housekeeping_task_h, 0);
}

void loop()
{
    // All work is in FreeRTOS tasks. Yield the Arduino loop task.
    vTaskDelay(portMAX_DELAY);
}
