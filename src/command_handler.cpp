// ═══════════════════════════════════════════════════════════════════════════
// Command Handler Module
// Serial CLI command parsing and execution
// ═══════════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <map>
#include "pin_config.h"
#include "can_types.h"
#include "globals.h"
#include "settings.h"
#include "obd2_tx.h"
#include "obd2.h"
#include "obd2_pids.h"
#include "nissan_consult.h"
#include "command_handler.h"

// External reference to id_trackers (owned by main.cpp, accessed from dispatch task)
extern std::map<uint32_t, id_tracker_t> id_trackers;

// ── Utility ──────────────────────────────────────────────────────────────────

float read_vbat()
{
    int raw = analogRead(VBAT_DIV);
    // 12-bit ADC, 3.3V ref, voltage divider ratio: (40.2k + 10k) / 10k = 5.02
    return ((float)raw / 4095.0f) * 3.3f * 5.02f;
}

// ── Help Display ─────────────────────────────────────────────────────────────

static void print_help()
{
    Serial.println();
    Serial.println("══════════════════════════════════════════════════");
    Serial.println("     CAN Bus Sniffer — Command Reference");
    Serial.println("══════════════════════════════════════════════════");
    Serial.println(" MODES:");
    Serial.println("  monitor      All frames to serial");
    Serial.println("  delta        Only payload changes (default)");
    Serial.println("  decode       Decoded signal values (370Z)");
    Serial.println("  stats        Periodic ID statistics");
    Serial.println("  silent       No serial output (SD only)");
    Serial.println();
    Serial.println(" FILTERS:");
    Serial.println("  filter <ID>  Show only hex ID (e.g. filter 2A0)");
    Serial.println("  nofilter     Remove ID filter");
    Serial.println();
    Serial.println(" LOGGING:");
    Serial.println("  log          Toggle SD card logging");
    Serial.println("  pause        Pause/resume serial output");
    Serial.println();
    Serial.println(" OBD2 DIAGNOSTICS:");
    Serial.println("  tx on/off    Enable/disable CAN transmission");
    Serial.println("  obd <PID>    Query standard OBD2 PID (hex)");
    Serial.println("  ext <PID>    Query Nissan extended PID (hex)");
    Serial.println("  dtc          Read diagnostic trouble codes");
    Serial.println("  dtc clear    Clear DTCs (requires confirm)");
    Serial.println("  vin          Read vehicle VIN");
    Serial.println("  scan         Scan for supported PIDs");
    Serial.println("  pids         List known PID definitions");
    Serial.println();
    Serial.println(" NISSAN CONSULT:");
    Serial.println("  modules      List known Consult modules");
    Serial.println("  scan-modules Scan for responding modules");
    Serial.println("  query <M> <P> Query module M for PID P (hex)");
    Serial.println("  id <module>  Get module identification");
    Serial.println("  <module>     List PIDs for module (ecm,bcm,etc)");
    Serial.println();
    Serial.println(" UTILITY:");
    Serial.println("  ids          List all seen CAN IDs");
    Serial.println("  clear        Reset ID trackers");
    Serial.println("  vbat         Read battery voltage");
    Serial.println("  status       Show current settings");
    Serial.println("  help         This message");
    Serial.println("══════════════════════════════════════════════════");
    Serial.println(" NOTE: TX is blocked if engine running or moving");
    Serial.println("══════════════════════════════════════════════════");
    Serial.println();
}

// ── Status Display ───────────────────────────────────────────────────────────

static void print_status()
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    op_mode_t mode = current_mode;
    bool log_en = logging_enabled;
    bool sd_ok = sd_available;
    bool filt = filter_active;
    uint32_t fid = filter_id;
    bool p = paused;
    bool tx_en = tx_enabled;
    xSemaphoreGive(state_mutex);

    const char *mode_str;
    switch (mode)
    {
    case MODE_MONITOR:
        mode_str = "MONITOR";
        break;
    case MODE_DELTA:
        mode_str = "DELTA";
        break;
    case MODE_DECODE:
        mode_str = "DECODE";
        break;
    case MODE_STATS:
        mode_str = "STATS";
        break;
    case MODE_SILENT:
        mode_str = "SILENT";
        break;
    default:
        mode_str = "UNKNOWN";
        break;
    }

    Serial.println();
    Serial.printf("  Mode:      %s%s\r\n", mode_str, p ? " (PAUSED)" : "");
    Serial.printf("  Filter:    %s", filt ? "" : "OFF\r\n");
    if (filt)
        Serial.printf("0x%03X\r\n", fid);
    Serial.printf("  SD card:   %s\r\n", sd_ok ? "OK" : "NOT AVAILABLE");
    Serial.printf("  Logging:   %s\r\n", log_en ? "ON" : "OFF");
    Serial.printf("  TX Mode:   %s\r\n", tx_en ? "ENABLED" : "OFF (silent)");
    if (tx_en)
    {
        const char* reason = get_tx_block_reason();
        Serial.printf("  TX Safe:   %s\r\n", reason ? "BLOCKED" : "OK");
        if (reason)
            Serial.printf("             (%s)\r\n", reason);
    }
    Serial.printf("  Frames:    %lu total, %lu dropped\r\n", total_frames, dropped_frames);
    Serial.printf("  Unique IDs: %d\r\n", id_trackers.size());
    Serial.printf("  Uptime:    %lus\r\n", millis() / 1000);
    Serial.printf("  Vbat:      %.2fV\r\n", read_vbat());
    Serial.println();
}

// ── ID List Display ──────────────────────────────────────────────────────────

void print_ids()
{
    if (id_trackers.empty())
    {
        Serial.println("  No CAN IDs seen yet.");
        return;
    }

    Serial.println();
    Serial.println("  ID        Count     Rate(Hz)  Last Data");
    Serial.println("  ────────  ────────  ────────  ─────────────────────────");

    for (auto &pair : id_trackers)
    {
        uint32_t id = pair.first;
        id_tracker_t &t = pair.second;

        uint32_t elapsed = t.last_seen_ms - t.first_seen_ms;
        float rate = (elapsed > 0) ? (t.msg_count * 1000.0f / elapsed) : 0;

        char data_str[32] = {0};
        int pos = 0;
        for (int i = 0; i < t.last_dlc && i < 8; i++)
        {
            pos += snprintf(data_str + pos, sizeof(data_str) - pos, "%02X ", t.last_data[i]);
        }

        Serial.printf("  0x%03X     %-8lu  %-8.1f  %s\r\n", id, t.msg_count, rate, data_str);
    }
    Serial.println();
}

// ── Command Task ─────────────────────────────────────────────────────────────
// Serial command interface for interactive use.

void cmd_task(void *pv)
{
    char buf[CMD_BUF_SIZE];
    int buf_pos = 0;

    vTaskDelay(pdMS_TO_TICKS(200));
    Serial.println();
    Serial.println("╔══════════════════════════════════════╗");
    Serial.println("║     CAN Bus Sniffer v2.0 Ready      ║");
    Serial.println("║     Type 'help' for commands         ║");
    Serial.println("╚══════════════════════════════════════╝");
    print_status();
    bool lastSerial = false;
    while (true)
    {
        if (Serial)
        {
            if (!lastSerial)
            {
                lastSerial = true;
                Serial.println();
                Serial.println("╔══════════════════════════════════════╗");
                Serial.println("║     CAN Bus Sniffer v2.0 Ready      ║");
                Serial.println("║     Type 'help' for commands         ║");
                Serial.println("╚══════════════════════════════════════╝");
                print_status();
            }
            if (Serial.available())
            {
                char c = Serial.read();
                if (c == '\n' || c == '\r')
                {
                    if (buf_pos == 0)
                        continue;
                    buf[buf_pos] = '\0';
                    buf_pos = 0;

                    // Parse command
                    String cmd = String(buf);
                    cmd.trim();
                    cmd.toLowerCase();

                    if (cmd == "help" || cmd == "?")
                    {
                        print_help();
                    }
                    else if (cmd == "monitor")
                    {
                        xSemaphoreTake(state_mutex, portMAX_DELAY);
                        current_mode = MODE_MONITOR;
                        xSemaphoreGive(state_mutex);
                        Serial.println("  → Mode: MONITOR (all frames)");
                        save_settings();
                    }
                    else if (cmd == "delta")
                    {
                        xSemaphoreTake(state_mutex, portMAX_DELAY);
                        current_mode = MODE_DELTA;
                        xSemaphoreGive(state_mutex);
                        Serial.println("  → Mode: DELTA (changes only)");
                        save_settings();
                    }
                    else if (cmd == "decode")
                    {
                        xSemaphoreTake(state_mutex, portMAX_DELAY);
                        current_mode = MODE_DECODE;
                        xSemaphoreGive(state_mutex);
                        Serial.println("  → Mode: DECODE (370Z signals)");
                        save_settings();
                    }
                    else if (cmd == "stats")
                    {
                        xSemaphoreTake(state_mutex, portMAX_DELAY);
                        current_mode = MODE_STATS;
                        xSemaphoreGive(state_mutex);
                        Serial.println("  → Mode: STATS (periodic summary)");
                        save_settings();
                    }
                    else if (cmd == "silent")
                    {
                        xSemaphoreTake(state_mutex, portMAX_DELAY);
                        current_mode = MODE_SILENT;
                        xSemaphoreGive(state_mutex);
                        Serial.println("  → Mode: SILENT (SD logging only)");
                        save_settings();
                    }
                    else if (cmd.startsWith("filter "))
                    {
                        String id_str = cmd.substring(7);
                        id_str.trim();
                        uint32_t id = strtoul(id_str.c_str(), NULL, 16);
                        xSemaphoreTake(state_mutex, portMAX_DELAY);
                        filter_id = id;
                        filter_active = true;
                        xSemaphoreGive(state_mutex);
                        Serial.printf("  → Filter: 0x%03X\r\n", id);
                        save_settings();
                    }
                    else if (cmd == "nofilter")
                    {
                        xSemaphoreTake(state_mutex, portMAX_DELAY);
                        filter_active = false;
                        xSemaphoreGive(state_mutex);
                        Serial.println("  → Filter: OFF");
                        save_settings();
                    }
                    else if (cmd == "log")
                    {
                        xSemaphoreTake(state_mutex, portMAX_DELAY);
                        logging_enabled = !logging_enabled;
                        bool state = logging_enabled;
                        xSemaphoreGive(state_mutex);
                        Serial.printf("  → Logging: %s\r\n", state ? "ON" : "OFF");
                        save_settings();
                    }
                    else if (cmd == "pause")
                    {
                        xSemaphoreTake(state_mutex, portMAX_DELAY);
                        paused = !paused;
                        bool state = paused;
                        xSemaphoreGive(state_mutex);
                        Serial.printf("  → Serial output: %s\r\n", state ? "PAUSED" : "RESUMED");
                    }
                    else if (cmd == "ids")
                    {
                        print_ids();
                    }
                    else if (cmd == "clear")
                    {
                        id_trackers.clear();
                        total_frames = 0;
                        dropped_frames = 0;
                        Serial.println("  → Trackers and counters cleared.");
                    }
                    else if (cmd == "vbat")
                    {
                        Serial.printf("  → Vbat: %.2fV\r\n", read_vbat());
                    }
                    else if (cmd == "status")
                    {
                        print_status();
                    }
                    // ── OBD2 Commands ──────────────────────────────────────
                    else if (cmd == "tx on")
                    {
                        xSemaphoreTake(state_mutex, portMAX_DELAY);
                        tx_enabled = true;
                        xSemaphoreGive(state_mutex);
                        Serial.println();
                        Serial.println("[WARN] TX mode ENABLED. Commands will transmit on CAN bus.");
                        Serial.println("[WARN] TX is blocked while engine running or vehicle moving.");
                        Serial.println();
                    }
                    else if (cmd == "tx off")
                    {
                        xSemaphoreTake(state_mutex, portMAX_DELAY);
                        tx_enabled = false;
                        xSemaphoreGive(state_mutex);
                        Serial.println("  → TX mode disabled (silent/receive only)");
                    }
                    else if (cmd == "tx")
                    {
                        xSemaphoreTake(state_mutex, portMAX_DELAY);
                        bool tx_en = tx_enabled;
                        xSemaphoreGive(state_mutex);
                        Serial.printf("  → TX mode: %s\r\n", tx_en ? "ENABLED" : "OFF");
                        Serial.println("    Use 'tx on' or 'tx off' to change");
                    }
                    else if (cmd.startsWith("obd "))
                    {
                        String pid_str = cmd.substring(4);
                        pid_str.trim();
                        uint8_t pid = strtoul(pid_str.c_str(), NULL, 16);

                        const obd2_pid_def_t *pid_def = find_obd2_pid(pid);
                        if (pid_def)
                        {
                            Serial.printf("[OBD2] Querying PID 0x%02X (%s)...\r\n", pid, pid_def->name);
                        }
                        else
                        {
                            Serial.printf("[OBD2] Querying PID 0x%02X...\r\n", pid);
                        }

                        if (obd2_send_request(OBD2_REQUEST_BROADCAST, OBD2_SVC_CURRENT_DATA, pid))
                        {
                            // Wait for response with timeout
                            uint32_t start = millis();
                            while (obd2_state == OBD2_WAITING_RESPONSE &&
                                   (millis() - start < OBD2_RESPONSE_TIMEOUT_MS))
                            {
                                vTaskDelay(pdMS_TO_TICKS(10));
                            }

                            if (obd2_state == OBD2_RESPONSE_RECEIVED)
                            {
                                obd2_print_response();
                            }
                            else if (obd2_state == OBD2_ERROR)
                            {
                                obd2_print_response(); // Will print negative response
                            }
                            else
                            {
                                Serial.println("[OBD2] Timeout - no response");
                            }
                            obd2_state = OBD2_IDLE;
                        }
                    }
                    else if (cmd.startsWith("ext "))
                    {
                        String pid_str = cmd.substring(4);
                        pid_str.trim();
                        uint16_t pid = strtoul(pid_str.c_str(), NULL, 16);
                        uint8_t pid_hi = (pid >> 8) & 0xFF;
                        uint8_t pid_lo = pid & 0xFF;

                        const nissan_pid_def_t *pid_def = find_nissan_pid(pid);
                        if (pid_def)
                        {
                            Serial.printf("[OBD2] Querying Nissan PID 0x%04X (%s)...\r\n", pid, pid_def->name);
                        }
                        else
                        {
                            Serial.printf("[OBD2] Querying Nissan PID 0x%04X...\r\n", pid);
                        }

                        if (obd2_send_request(OBD2_ECM_REQUEST, OBD2_SVC_NISSAN_READ, pid_hi, pid_lo, true))
                        {
                            uint32_t start = millis();
                            while (obd2_state == OBD2_WAITING_RESPONSE &&
                                   (millis() - start < OBD2_RESPONSE_TIMEOUT_MS))
                            {
                                vTaskDelay(pdMS_TO_TICKS(10));
                            }

                            if (obd2_state == OBD2_RESPONSE_RECEIVED)
                            {
                                // For Nissan extended, show raw response if no definition
                                if (pid_def)
                                {
                                    float value = convert_pid_value(obd2_response.data,
                                                                     obd2_response.data_len,
                                                                     pid_def->formula);
                                    Serial.printf("[OBD2] %s: %.1f %s\r\n",
                                                  pid_def->desc, value, pid_def->unit);
                                }
                                else
                                {
                                    Serial.print("[OBD2] Response: ");
                                    for (uint8_t i = 0; i < obd2_response.data_len; i++)
                                    {
                                        Serial.printf("%02X ", obd2_response.data[i]);
                                    }
                                    Serial.println();
                                }
                            }
                            else if (obd2_state == OBD2_ERROR)
                            {
                                obd2_print_response();
                            }
                            else
                            {
                                Serial.println("[OBD2] Timeout - no response");
                            }
                            obd2_state = OBD2_IDLE;
                        }
                    }
                    else if (cmd == "dtc")
                    {
                        Serial.println("[OBD2] Reading DTCs...");
                        if (obd2_read_dtc())
                        {
                            uint32_t start = millis();
                            while (obd2_state == OBD2_WAITING_RESPONSE &&
                                   (millis() - start < OBD2_RESPONSE_TIMEOUT_MS))
                            {
                                vTaskDelay(pdMS_TO_TICKS(10));
                            }

                            if (obd2_state == OBD2_RESPONSE_RECEIVED)
                            {
                                // Service 0x43 response: DTC pairs (2 bytes each)
                                // data[] contains raw DTC bytes, data_len = total bytes
                                uint8_t num_dtcs = obd2_response.data_len / 2;
                                if (num_dtcs == 0)
                                {
                                    Serial.println("[OBD2] No DTCs stored");
                                }
                                else
                                {
                                    Serial.printf("[OBD2] %d DTC(s) found:\r\n", num_dtcs);
                                    for (uint8_t i = 0; i < num_dtcs; i++)
                                    {
                                        uint16_t dtc = (obd2_response.data[i * 2] << 8) |
                                                        obd2_response.data[i * 2 + 1];
                                        char dtc_str[8];
                                        dtc_to_string(dtc, dtc_str);
                                        Serial.printf("  - %s\r\n", dtc_str);
                                    }
                                }
                            }
                            else if (obd2_state == OBD2_ERROR)
                            {
                                obd2_print_response();
                            }
                            else
                            {
                                Serial.println("[OBD2] Timeout - no response");
                            }
                            obd2_state = OBD2_IDLE;
                        }
                    }
                    else if (cmd == "dtc clear")
                    {
                        Serial.println("[OBD2] WARNING: This will clear all DTCs and freeze frame data!");
                        Serial.println("[OBD2] Type 'dtc clear confirm' to proceed.");
                    }
                    else if (cmd == "dtc clear confirm")
                    {
                        Serial.println("[OBD2] Clearing DTCs...");
                        if (obd2_send_request(OBD2_REQUEST_BROADCAST, OBD2_SVC_CLEAR_DTC, 0))
                        {
                            uint32_t start = millis();
                            while (obd2_state == OBD2_WAITING_RESPONSE &&
                                   (millis() - start < OBD2_RESPONSE_TIMEOUT_MS))
                            {
                                vTaskDelay(pdMS_TO_TICKS(10));
                            }

                            if (obd2_state == OBD2_RESPONSE_RECEIVED)
                            {
                                Serial.println("[OBD2] DTCs cleared successfully");
                            }
                            else if (obd2_state == OBD2_ERROR)
                            {
                                obd2_print_response();
                            }
                            else
                            {
                                Serial.println("[OBD2] Timeout - no response");
                            }
                            obd2_state = OBD2_IDLE;
                        }
                    }
                    else if (cmd == "vin")
                    {
                        Serial.println("[OBD2] Reading VIN...");
                        if (obd2_send_request(OBD2_REQUEST_BROADCAST, OBD2_SVC_VEHICLE_INFO, OBD2_VIN))
                        {
                            // VIN requires multi-frame response, simplified single-frame attempt
                            uint32_t start = millis();
                            while (obd2_state == OBD2_WAITING_RESPONSE &&
                                   (millis() - start < OBD2_RESPONSE_TIMEOUT_MS * 2))
                            {
                                vTaskDelay(pdMS_TO_TICKS(10));
                            }

                            if (obd2_state == OBD2_RESPONSE_RECEIVED)
                            {
                                Serial.print("[OBD2] VIN (partial): ");
                                for (uint8_t i = 0; i < obd2_response.data_len; i++)
                                {
                                    char c = obd2_response.data[i];
                                    if (c >= 0x20 && c <= 0x7E)
                                        Serial.print(c);
                                    else
                                        Serial.print('.');
                                }
                                Serial.println();
                                Serial.println("[OBD2] Note: Full VIN requires multi-frame support");
                            }
                            else if (obd2_state == OBD2_ERROR)
                            {
                                obd2_print_response();
                            }
                            else
                            {
                                Serial.println("[OBD2] Timeout - no response");
                            }
                            obd2_state = OBD2_IDLE;
                        }
                    }
                    else if (cmd == "scan")
                    {
                        Serial.println("[OBD2] Scanning supported PIDs...");
                        if (obd2_send_request(OBD2_REQUEST_BROADCAST, OBD2_SVC_CURRENT_DATA, 0x00))
                        {
                            uint32_t start = millis();
                            while (obd2_state == OBD2_WAITING_RESPONSE &&
                                   (millis() - start < OBD2_RESPONSE_TIMEOUT_MS))
                            {
                                vTaskDelay(pdMS_TO_TICKS(10));
                            }

                            if (obd2_state == OBD2_RESPONSE_RECEIVED)
                            {
                                Serial.println("[OBD2] Supported PIDs 01-20:");
                                uint32_t supported = 0;
                                if (obd2_response.data_len >= 4)
                                {
                                    supported = (obd2_response.data[0] << 24) |
                                                (obd2_response.data[1] << 16) |
                                                (obd2_response.data[2] << 8) |
                                                 obd2_response.data[3];
                                }
                                Serial.print("  ");
                                for (uint8_t i = 0; i < 32; i++)
                                {
                                    if (supported & (1UL << (31 - i)))
                                    {
                                        Serial.printf("0x%02X ", i + 1);
                                    }
                                }
                                Serial.println();
                            }
                            else if (obd2_state == OBD2_ERROR)
                            {
                                obd2_print_response();
                            }
                            else
                            {
                                Serial.println("[OBD2] Timeout - no response");
                            }
                            obd2_state = OBD2_IDLE;
                        }
                    }
                    else if (cmd == "pids")
                    {
                        Serial.println();
                        Serial.println("Standard OBD2 PIDs (service 0x01):");
                        for (uint8_t i = 0; i < OBD2_PID_COUNT; i++)
                        {
                            Serial.printf("  0x%02X %-10s %s\r\n",
                                          obd2_pids[i].pid,
                                          obd2_pids[i].name,
                                          obd2_pids[i].desc);
                        }
                        Serial.println();
                        Serial.println("Nissan Extended PIDs (service 0x21):");
                        for (uint8_t i = 0; i < NISSAN_PID_COUNT; i++)
                        {
                            Serial.printf("  0x%04X %-10s %s\r\n",
                                          nissan_pids[i].pid,
                                          nissan_pids[i].name,
                                          nissan_pids[i].desc);
                        }
                        Serial.println();
                    }
                    // ── Nissan Consult Commands ────────────────────────────
                    else if (cmd == "modules")
                    {
                        Serial.println();
                        Serial.println("Known Nissan Consult Modules:");
                        Serial.println("  Name   Request  Response  Description");
                        Serial.println("  ─────  ───────  ────────  ────────────────────────────");
                        for (uint8_t i = 0; i < CONSULT_MODULE_COUNT; i++)
                        {
                            Serial.printf("  %-5s  0x%03X    0x%03X     %s\r\n",
                                          consult_modules[i].name,
                                          consult_modules[i].request_id,
                                          consult_modules[i].response_id,
                                          consult_modules[i].description);
                        }
                        Serial.println();
                    }
                    else if (cmd == "scan-modules")
                    {
                        consult_scan_modules();
                    }
                    else if (cmd.startsWith("query "))
                    {
                        // Parse "query <module> <pid>"
                        String args = cmd.substring(6);
                        args.trim();
                        int space = args.indexOf(' ');
                        if (space < 0)
                        {
                            Serial.println("  Usage: query <module> <pid>");
                            Serial.println("  Example: query ecm 08");
                        }
                        else
                        {
                            String mod_name = args.substring(0, space);
                            String pid_str = args.substring(space + 1);
                            mod_name.trim();
                            pid_str.trim();

                            const consult_module_t* mod = find_module_by_name(mod_name.c_str());
                            if (!mod)
                            {
                                Serial.printf("  Unknown module: %s\r\n", mod_name.c_str());
                                Serial.println("  Type 'modules' to see available modules");
                            }
                            else
                            {
                                uint8_t pid = strtoul(pid_str.c_str(), NULL, 16);
                                Serial.printf("[%s] Querying PID 0x%02X...\r\n", mod->name, pid);

                                if (consult_query_module(mod, CONSULT_SVC_READ_DATA_ID, pid))
                                {
                                    if (consult_wait_response())
                                    {
                                        consult_print_response(mod, pid);
                                    }
                                    else if (obd2_state == OBD2_ERROR)
                                    {
                                        consult_print_response(mod, pid);
                                    }
                                    else
                                    {
                                        Serial.printf("[%s] Timeout - no response\r\n", mod->name);
                                    }
                                    obd2_state = OBD2_IDLE;
                                }
                            }
                        }
                    }
                    else if (cmd.startsWith("id "))
                    {
                        // Get module identification
                        String mod_name = cmd.substring(3);
                        mod_name.trim();

                        const consult_module_t* mod = find_module_by_name(mod_name.c_str());
                        if (!mod)
                        {
                            Serial.printf("  Unknown module: %s\r\n", mod_name.c_str());
                        }
                        else
                        {
                            Serial.printf("[%s] Reading identification...\r\n", mod->name);

                            if (consult_query_module(mod, CONSULT_SVC_READ_DATA_ID, 0x01))
                            {
                                if (consult_wait_response(500))  // Longer timeout for ID
                                {
                                    Serial.printf("[%s] ID: ", mod->name);
                                    for (uint8_t i = 0; i < obd2_response.data_len; i++)
                                    {
                                        char c = obd2_response.data[i];
                                        if (c >= 0x20 && c <= 0x7E)
                                            Serial.print(c);
                                        else if (c != 0x00)
                                            Serial.print('.');
                                    }
                                    Serial.println();
                                }
                                else if (obd2_state == OBD2_ERROR)
                                {
                                    consult_print_response(mod, 0x01);
                                }
                                else
                                {
                                    Serial.printf("[%s] Timeout - no response\r\n", mod->name);
                                }
                                obd2_state = OBD2_IDLE;
                            }
                        }
                    }
                    // Module-specific PID listings
                    else if (cmd == "ecm")
                    {
                        Serial.println();
                        Serial.println("ECM PIDs (query ecm <pid>):");
                        for (uint8_t i = 0; i < ECM_PID_COUNT; i++)
                        {
                            Serial.printf("  0x%02X %-10s %s\r\n",
                                          ecm_pids[i].pid, ecm_pids[i].name, ecm_pids[i].unit);
                        }
                        Serial.println();
                    }
                    else if (cmd == "bcm")
                    {
                        Serial.println();
                        Serial.println("BCM PIDs (query bcm <pid>):");
                        for (uint8_t i = 0; i < BCM_PID_COUNT; i++)
                        {
                            Serial.printf("  0x%02X %-10s\r\n",
                                          bcm_pids[i].pid, bcm_pids[i].name);
                        }
                        Serial.println();
                    }
                    else if (cmd == "ipdm")
                    {
                        Serial.println();
                        Serial.println("IPDM PIDs (query ipdm <pid>):");
                        for (uint8_t i = 0; i < IPDM_PID_COUNT; i++)
                        {
                            Serial.printf("  0x%02X %-10s\r\n",
                                          ipdm_pids[i].pid, ipdm_pids[i].name);
                        }
                        Serial.println();
                    }
                    else if (cmd == "abs")
                    {
                        Serial.println();
                        Serial.println("ABS/VDC PIDs (query abs <pid>):");
                        for (uint8_t i = 0; i < ABS_PID_COUNT; i++)
                        {
                            Serial.printf("  0x%02X %-10s %s\r\n",
                                          abs_pids[i].pid, abs_pids[i].name, abs_pids[i].unit);
                        }
                        Serial.println();
                    }
                    else
                    {
                        Serial.printf("  Unknown command: '%s'. Type 'help'.\r\n", buf);
                    }
                }
                else if (buf_pos < CMD_BUF_SIZE - 1)
                {
                    buf[buf_pos++] = c;
                }
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
        else
        {
            lastSerial = false;
        }
    }
}
