#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "logger.h"
#include "pin_config.h"
#include "can_types.h"
#include "globals.h"
#include "settings.h"

// ═══════════════════════════════════════════════════════════════════════════
// SD Logger Task
// ═══════════════════════════════════════════════════════════════════════════

void logger_task(void *pv)
{
    can_msg_t msg;
    uint32_t msg_counter = 0;
    uint32_t last_flush = 0;

    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

    // Wait for card insertion
    Serial.println("[LOG] Waiting for SD card...");
    while (digitalRead(SD_CD))
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelay(pdMS_TO_TICKS(500)); // Debounce

    if (!SD.begin(SD_CS, SPI, 16000000))
    {
        Serial.println("[LOG] SD mount failed. Logger disabled.");
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        sd_available = false;
        xSemaphoreGive(state_mutex);
        vTaskDelete(NULL);
        return;
    }

    Serial.printf("[LOG] SD ready. Size: %lluMB, Used: %lluMB\r\n",
                  SD.cardSize() / (1024 * 1024), SD.usedBytes() / (1024 * 1024));

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    sd_available = true;
    xSemaphoreGive(state_mutex);

    // Load saved settings from SD
    load_settings();

    // Open log file - create with header if new, keep open for performance
    bool need_header = false;
    {
        File check = SD.open("/canlog.csv", FILE_READ);
        need_header = (!check || check.size() == 0);
        if (check) check.close();
    }

    File logfile = SD.open("/canlog.csv", FILE_APPEND, true);
    if (!logfile)
    {
        Serial.println("[LOG] Failed to open log file for writing.");
        vTaskDelete(NULL);
        return;
    }

    if (need_header)
    {
        logfile.println("seq,timestamp_ms,format,type,id,dlc,data");
        logfile.flush();
    }

    // RAM buffer for batching writes (reduces SD write calls)
    static const size_t BUF_SIZE = 4096;
    char* write_buf = (char*)malloc(BUF_SIZE);
    if (!write_buf)
    {
        Serial.println("[LOG] Failed to allocate write buffer.");
        logfile.close();
        vTaskDelete(NULL);
        return;
    }
    size_t buf_pos = 0;
    uint32_t last_msg_time = millis();
    bool file_open = true;
    static const uint32_t BUS_QUIET_TIMEOUT = 5000;  // Close file after 5s of no messages

    while (true)
    {
        // Block with timeout so we periodically flush even during lulls
        bool got_msg = xQueueReceive(log_queue, &msg, pdMS_TO_TICKS(LOG_FLUSH_INTERVAL)) == pdTRUE;
        uint32_t now = millis();

        if (got_msg)
        {
            last_msg_time = now;

            // Reopen file if it was closed due to bus going quiet
            if (!file_open)
            {
                logfile = SD.open("/canlog.csv", FILE_APPEND);
                if (logfile)
                {
                    file_open = true;
                    Serial.println("[LOG] Bus active - file reopened");
                }
            }

            if (!file_open)
                continue;

            // Drain the queue into RAM buffer
            do
            {
                msg_counter++;
                const CAN_frame_t &fr = msg.frame;

                // Format into buffer
                int written = snprintf(write_buf + buf_pos, BUF_SIZE - buf_pos,
                    "%lu,%lu,%s,%s,0x%03X,%d",
                    msg_counter, msg.timestamp_ms,
                    (fr.FIR.B.FF == CAN_frame_std) ? "STD" : "EXT",
                    (fr.FIR.B.RTR == CAN_RTR) ? "RTR" : "DATA",
                    fr.MsgID, fr.FIR.B.DLC);

                if (written > 0 && buf_pos + written < BUF_SIZE - 20)
                {
                    buf_pos += written;

                    if (fr.FIR.B.RTR != CAN_RTR)
                    {
                        buf_pos += snprintf(write_buf + buf_pos, BUF_SIZE - buf_pos, ",0x");
                        for (int i = 0; i < fr.FIR.B.DLC && i < 8; i++)
                        {
                            buf_pos += snprintf(write_buf + buf_pos, BUF_SIZE - buf_pos, "%02X", fr.data.u8[i]);
                        }
                    }
                    write_buf[buf_pos++] = '\n';
                }

                // If buffer is getting full, write to SD
                if (buf_pos > BUF_SIZE - 100)
                {
                    logfile.write((uint8_t*)write_buf, buf_pos);
                    buf_pos = 0;
                }
            } while (xQueueReceive(log_queue, &msg, 0) == pdTRUE);
        }

        // Flush to SD periodically or when we have data after timeout
        if (file_open && buf_pos > 0 && (now - last_flush >= LOG_FLUSH_INTERVAL || !got_msg))
        {
            logfile.write((uint8_t*)write_buf, buf_pos);
            buf_pos = 0;
            logfile.flush();
            last_flush = now;
        }

        // Close file if bus has been quiet for a while (safe to remove device)
        if (file_open && (now - last_msg_time >= BUS_QUIET_TIMEOUT))
        {
            if (buf_pos > 0)
            {
                logfile.write((uint8_t*)write_buf, buf_pos);
                buf_pos = 0;
            }
            logfile.flush();
            logfile.close();
            file_open = false;
            Serial.println("[LOG] Bus quiet - file closed (safe to remove)");
        }
    }
}
