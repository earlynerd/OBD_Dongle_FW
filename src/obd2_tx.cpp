#include <Arduino.h>
#include <ESP32CAN.h>
#include "obd2_tx.h"
#include "pin_config.h"
#include "can_types.h"
#include "globals.h"
#include "obd2.h"
#include "obd2_pids.h"
#include "nissan_consult.h"

// ═══════════════════════════════════════════════════════════════════════════
// OBD2 TX Safety
// ═══════════════════════════════════════════════════════════════════════════

bool is_safe_to_transmit()
{
    // Must have TX enabled
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool tx_en = tx_enabled;
    xSemaphoreGive(state_mutex);

    if (!tx_en)
    {
        return false;
    }

    // Check passive CAN data for RPM (0x180 ECM1)
    auto rpm_tracker = id_trackers.find(0x180);
    if (rpm_tracker != id_trackers.end() && rpm_tracker->second.seen)
    {
        // RPM is in bytes 0,1 as big-endian, scaled by 0.1
        uint16_t raw_rpm = (rpm_tracker->second.last_data[0] << 8) |
                            rpm_tracker->second.last_data[1];
        if (raw_rpm > 0)
        {
            return false; // Engine running
        }
    }

    // Check passive CAN data for speed (0x280 or 0x354)
    auto speed_tracker = id_trackers.find(0x280);
    if (speed_tracker != id_trackers.end() && speed_tracker->second.seen)
    {
        // Speed is in bytes 4,5 as big-endian
        uint16_t raw_speed = (speed_tracker->second.last_data[4] << 8) |
                              speed_tracker->second.last_data[5];
        if (raw_speed > 0)
        {
            return false; // Vehicle moving
        }
    }

    return true;
}

const char* get_tx_block_reason()
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool tx_en = tx_enabled;
    xSemaphoreGive(state_mutex);

    if (!tx_en)
    {
        return "TX mode not enabled (use 'tx on')";
    }

    auto rpm_tracker = id_trackers.find(0x180);
    if (rpm_tracker != id_trackers.end() && rpm_tracker->second.seen)
    {
        uint16_t raw_rpm = (rpm_tracker->second.last_data[0] << 8) |
                            rpm_tracker->second.last_data[1];
        if (raw_rpm > 0)
        {
            return "Engine running (RPM > 0)";
        }
    }

    auto speed_tracker = id_trackers.find(0x280);
    if (speed_tracker != id_trackers.end() && speed_tracker->second.seen)
    {
        uint16_t raw_speed = (speed_tracker->second.last_data[4] << 8) |
                              speed_tracker->second.last_data[5];
        if (raw_speed > 0)
        {
            return "Vehicle moving (speed > 0)";
        }
    }

    return nullptr; // Safe to transmit
}

// ═══════════════════════════════════════════════════════════════════════════
// OBD2 Request/Response
// ═══════════════════════════════════════════════════════════════════════════

bool obd2_send_request(uint32_t can_id, uint8_t service, uint8_t pid,
                       uint8_t sub_pid, bool has_sub)
{
    const char* reason = get_tx_block_reason();
    if (reason != nullptr)
    {
        Serial.printf("[OBD2] TX blocked: %s\r\n", reason);
        return false;
    }

    // Switch transceiver to normal mode (LOW = normal, HIGH = silent)
    digitalWrite(CAN_SLNT, LOW);
    delayMicroseconds(100); // Allow transceiver to wake

    CAN_frame_t frame;
    frame.FIR.B.FF = CAN_frame_std;
    frame.FIR.B.RTR = CAN_no_RTR;
    frame.MsgID = can_id;
    frame.FIR.B.DLC = 8;

    if (has_sub)
    {
        // Extended PID (e.g., Nissan 0x21 service with 2-byte PID)
        frame.data.u8[0] = 3;                    // Length
        frame.data.u8[1] = service;
        frame.data.u8[2] = pid;
        frame.data.u8[3] = sub_pid;
        frame.data.u8[4] = OBD2_PADDING_BYTE;
        frame.data.u8[5] = OBD2_PADDING_BYTE;
        frame.data.u8[6] = OBD2_PADDING_BYTE;
        frame.data.u8[7] = OBD2_PADDING_BYTE;
    }
    else
    {
        // Standard OBD2 request
        frame.data.u8[0] = 2;                    // Length
        frame.data.u8[1] = service;
        frame.data.u8[2] = pid;
        frame.data.u8[3] = OBD2_PADDING_BYTE;
        frame.data.u8[4] = OBD2_PADDING_BYTE;
        frame.data.u8[5] = OBD2_PADDING_BYTE;
        frame.data.u8[6] = OBD2_PADDING_BYTE;
        frame.data.u8[7] = OBD2_PADDING_BYTE;
    }

    ESP32Can.CANWriteFrame(&frame);

    // Wait for TX to complete (CAN frame at 500kbps takes ~230us max)
    delay(2);

    // Set up response state machine
    obd2_state = OBD2_WAITING_RESPONSE;
    obd2_request_time = millis();
    memset(&obd2_response, 0, sizeof(obd2_response));

    // Return to silent mode
    digitalWrite(CAN_SLNT, HIGH);

    return true;
}

bool obd2_read_dtc()
{
    const char* reason = get_tx_block_reason();
    if (reason != nullptr)
    {
        Serial.printf("[OBD2] TX blocked: %s\r\n", reason);
        return false;
    }

    digitalWrite(CAN_SLNT, LOW);
    delayMicroseconds(100);

    CAN_frame_t frame;
    frame.FIR.B.FF = CAN_frame_std;
    frame.FIR.B.RTR = CAN_no_RTR;
    frame.MsgID = OBD2_REQUEST_BROADCAST;
    frame.FIR.B.DLC = 8;
    frame.data.u8[0] = 1;                    // Length (just service, no PID)
    frame.data.u8[1] = OBD2_SVC_READ_DTC;
    frame.data.u8[2] = OBD2_PADDING_BYTE;
    frame.data.u8[3] = OBD2_PADDING_BYTE;
    frame.data.u8[4] = OBD2_PADDING_BYTE;
    frame.data.u8[5] = OBD2_PADDING_BYTE;
    frame.data.u8[6] = OBD2_PADDING_BYTE;
    frame.data.u8[7] = OBD2_PADDING_BYTE;

    ESP32Can.CANWriteFrame(&frame);

    // Wait for TX to complete
    delay(2);

    obd2_state = OBD2_WAITING_RESPONSE;
    obd2_request_time = millis();
    memset(&obd2_response, 0, sizeof(obd2_response));

    digitalWrite(CAN_SLNT, HIGH);
    return true;
}

void obd2_parse_response(const CAN_frame_t &frame)
{
    if (obd2_state != OBD2_WAITING_RESPONSE)
        return;

    uint8_t len = frame.data.u8[0];
    if (len < 1 || len > 7)
        return;

    uint8_t service = frame.data.u8[1];

    // Check for negative response
    if (service == OBD2_NEGATIVE_RESPONSE)
    {
        obd2_response.negative = true;
        obd2_response.service = frame.data.u8[2];
        obd2_response.nrc = frame.data.u8[3];
        obd2_state = OBD2_ERROR;
        return;
    }

    // Positive response: service should be request + 0x40
    obd2_response.can_id = frame.MsgID;
    obd2_response.service = service;
    obd2_response.negative = false;

    // Mode 3 (DTC) and Mode 4 (Clear DTC) responses have no PID - data starts at byte 2
    // Other modes have PID at byte 2, data at byte 3+
    if (service == 0x43 || service == 0x44 || service == 0x47)
    {
        // DTC responses: [len][service][DTC data...]
        obd2_response.pid = 0;
        obd2_response.data_len = (len > 1) ? (len - 1) : 0;
        for (uint8_t i = 0; i < obd2_response.data_len && i < OBD2_MAX_DATA_BYTES; i++)
        {
            obd2_response.data[i] = frame.data.u8[2 + i];
        }
    }
    else
    {
        // Standard response: [len][service][pid][data...]
        if (len >= 2)
        {
            obd2_response.pid = frame.data.u8[2];
        }
        obd2_response.data_len = (len > 2) ? (len - 2) : 0;
        for (uint8_t i = 0; i < obd2_response.data_len && i < OBD2_MAX_DATA_BYTES; i++)
        {
            obd2_response.data[i] = frame.data.u8[3 + i];
        }
    }

    obd2_state = OBD2_RESPONSE_RECEIVED;
}

void obd2_print_response()
{
    if (obd2_response.negative)
    {
        Serial.printf("[OBD2] Negative response: Service 0x%02X, NRC 0x%02X",
                      obd2_response.service, obd2_response.nrc);
        switch (obd2_response.nrc)
        {
        case OBD2_NRC_SERVICE_NOT_SUPPORTED:
            Serial.println(" (Service not supported)");
            break;
        case OBD2_NRC_SUBFUNCTION_NOT_SUPPORTED:
            Serial.println(" (PID not supported)");
            break;
        case OBD2_NRC_CONDITIONS_NOT_CORRECT:
            Serial.println(" (Conditions not correct)");
            break;
        default:
            Serial.println();
            break;
        }
        return;
    }

    // Look up PID definition
    const obd2_pid_def_t *pid_def = find_obd2_pid(obd2_response.pid);

    if (pid_def)
    {
        float value = convert_pid_value(obd2_response.data, obd2_response.data_len,
                                         pid_def->formula);
        Serial.printf("[OBD2] %s (0x%02X): %.1f %s\r\n",
                      pid_def->desc, obd2_response.pid, value, pid_def->unit);
    }
    else
    {
        // Unknown PID - show raw data
        Serial.printf("[OBD2] PID 0x%02X: ", obd2_response.pid);
        for (uint8_t i = 0; i < obd2_response.data_len; i++)
        {
            Serial.printf("%02X ", obd2_response.data[i]);
        }
        Serial.println();
    }
}

bool obd2_wait_response(uint32_t timeout_ms)
{
    uint32_t start = millis();
    while (obd2_state == OBD2_WAITING_RESPONSE && (millis() - start < timeout_ms))
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return (obd2_state == OBD2_RESPONSE_RECEIVED);
}

// ═══════════════════════════════════════════════════════════════════════════
// Nissan Consult Functions
// ═══════════════════════════════════════════════════════════════════════════

bool consult_query_module(const consult_module_t* module, uint8_t service,
                          uint8_t pid, uint8_t sub_pid, bool has_sub)
{
    const char* reason = get_tx_block_reason();
    if (reason != nullptr)
    {
        Serial.printf("[CONSULT] TX blocked: %s\r\n", reason);
        return false;
    }

    digitalWrite(CAN_SLNT, LOW);
    delayMicroseconds(100);

    CAN_frame_t frame;
    frame.FIR.B.FF = CAN_frame_std;
    frame.FIR.B.RTR = CAN_no_RTR;
    frame.MsgID = module->request_id;
    frame.FIR.B.DLC = 8;

    if (has_sub)
    {
        frame.data.u8[0] = 3;
        frame.data.u8[1] = service;
        frame.data.u8[2] = pid;
        frame.data.u8[3] = sub_pid;
    }
    else
    {
        frame.data.u8[0] = 2;
        frame.data.u8[1] = service;
        frame.data.u8[2] = pid;
        frame.data.u8[3] = OBD2_PADDING_BYTE;
    }
    frame.data.u8[4] = OBD2_PADDING_BYTE;
    frame.data.u8[5] = OBD2_PADDING_BYTE;
    frame.data.u8[6] = OBD2_PADDING_BYTE;
    frame.data.u8[7] = OBD2_PADDING_BYTE;

    ESP32Can.CANWriteFrame(&frame);

    // Wait for TX to complete
    delay(2);

    obd2_state = OBD2_WAITING_RESPONSE;
    obd2_request_time = millis();
    memset(&obd2_response, 0, sizeof(obd2_response));

    digitalWrite(CAN_SLNT, HIGH);
    return true;
}

bool consult_wait_response(uint32_t timeout_ms)
{
    uint32_t start = millis();
    while (obd2_state == OBD2_WAITING_RESPONSE && (millis() - start < timeout_ms))
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return (obd2_state == OBD2_RESPONSE_RECEIVED);
}

void consult_print_response(const consult_module_t* module, uint8_t pid)
{
    if (obd2_response.negative)
    {
        Serial.printf("[%s] Negative response: NRC 0x%02X", module->name, obd2_response.nrc);
        switch (obd2_response.nrc)
        {
        case OBD2_NRC_SERVICE_NOT_SUPPORTED:
            Serial.println(" (Service not supported)");
            break;
        case OBD2_NRC_SUBFUNCTION_NOT_SUPPORTED:
            Serial.println(" (PID not supported)");
            break;
        case OBD2_NRC_CONDITIONS_NOT_CORRECT:
            Serial.println(" (Conditions not correct)");
            break;
        case OBD2_NRC_SECURITY_ACCESS_DENIED:
            Serial.println(" (Security access denied)");
            break;
        default:
            Serial.println();
            break;
        }
        return;
    }

    // Look up PID in module-specific table
    const consult_pid_t* pid_def = find_module_pid(module->name, pid);

    if (pid_def && pid_def->scale != 1.0f)
    {
        // Calculate scaled value
        float raw = 0;
        if (pid_def->bytes == 1 && obd2_response.data_len >= 1)
        {
            raw = obd2_response.data[0];
        }
        else if (pid_def->bytes == 2 && obd2_response.data_len >= 2)
        {
            raw = (obd2_response.data[0] << 8) | obd2_response.data[1];
        }
        float value = raw * pid_def->scale + pid_def->offset;
        Serial.printf("[%s] %s: %.1f %s\r\n", module->name, pid_def->name, value, pid_def->unit);
    }
    else if (pid_def)
    {
        // Show raw data with name
        Serial.printf("[%s] %s: ", module->name, pid_def->name);
        for (uint8_t i = 0; i < obd2_response.data_len; i++)
        {
            Serial.printf("%02X ", obd2_response.data[i]);
        }
        Serial.println();
    }
    else
    {
        // Unknown PID - show raw data
        Serial.printf("[%s] PID 0x%02X: ", module->name, pid);
        for (uint8_t i = 0; i < obd2_response.data_len; i++)
        {
            Serial.printf("%02X ", obd2_response.data[i]);
        }
        Serial.println();
    }
}

void consult_scan_modules()
{
    Serial.println("[CONSULT] Scanning for modules...");
    Serial.println();

    for (uint8_t i = 0; i < CONSULT_MODULE_COUNT; i++)
    {
        const consult_module_t* mod = &consult_modules[i];

        // Send tester present to check if module responds
        if (consult_query_module(mod, CONSULT_SVC_TESTER_PRESENT, 0x00))
        {
            if (consult_wait_response(100))  // Short timeout for scan
            {
                Serial.printf("  [OK]  %-5s (0x%03X/0x%03X) - %s\r\n",
                              mod->name, mod->request_id, mod->response_id, mod->description);
            }
            else
            {
                Serial.printf("  [--]  %-5s (0x%03X/0x%03X) - No response\r\n",
                              mod->name, mod->request_id, mod->response_id);
            }
            obd2_state = OBD2_IDLE;
        }
        vTaskDelay(pdMS_TO_TICKS(50));  // Small delay between queries
    }
    Serial.println();
}
