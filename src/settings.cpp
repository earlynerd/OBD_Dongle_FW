#include <Arduino.h>
#include <SD.h>
#include "settings.h"
#include "can_types.h"
#include "globals.h"

// ═══════════════════════════════════════════════════════════════════════════
// Settings Persistence
// ═══════════════════════════════════════════════════════════════════════════

void save_settings()
{
    if (!sd_available)
        return;

    File f = SD.open(SETTINGS_FILE, FILE_WRITE);
    if (!f)
    {
        Serial.println("[SETTINGS] Failed to save settings");
        return;
    }

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    op_mode_t mode = current_mode;
    bool log_en = logging_enabled;
    bool filt = filter_active;
    uint32_t fid = filter_id;
    xSemaphoreGive(state_mutex);

    f.println("{");
    f.printf("  \"mode\": %d,\n", (int)mode);
    f.printf("  \"logging\": %s,\n", log_en ? "true" : "false");
    f.printf("  \"filter_active\": %s,\n", filt ? "true" : "false");
    f.printf("  \"filter_id\": %lu\n", fid);
    f.println("}");
    f.close();

    Serial.println("[SETTINGS] Saved");
}

void load_settings()
{
    if (!sd_available)
        return;

    File f = SD.open(SETTINGS_FILE, FILE_READ);
    if (!f)
    {
        Serial.println("[SETTINGS] No settings file, using defaults");
        return;
    }

    // Simple JSON parsing (no library needed for this basic format)
    String content = f.readString();
    f.close();

    // Parse mode
    int mode_idx = content.indexOf("\"mode\":");
    if (mode_idx >= 0)
    {
        int mode_val = content.substring(mode_idx + 7).toInt();
        if (mode_val >= 0 && mode_val <= 4)
        {
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            current_mode = (op_mode_t)mode_val;
            xSemaphoreGive(state_mutex);
        }
    }

    // Parse logging - search for "true" within next 10 chars (handles whitespace)
    int log_idx = content.indexOf("\"logging\":");
    if (log_idx >= 0)
    {
        String log_str = content.substring(log_idx + 10, log_idx + 20);
        bool log_val = log_str.indexOf("true") >= 0;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        logging_enabled = log_val;
        xSemaphoreGive(state_mutex);
    }

    // Parse filter_active - search for "true" within next 10 chars
    int filt_idx = content.indexOf("\"filter_active\":");
    if (filt_idx >= 0)
    {
        String filt_str = content.substring(filt_idx + 16, filt_idx + 26);
        bool filt_val = filt_str.indexOf("true") >= 0;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        filter_active = filt_val;
        xSemaphoreGive(state_mutex);
    }

    // Parse filter_id
    int fid_idx = content.indexOf("\"filter_id\":");
    if (fid_idx >= 0)
    {
        uint32_t fid_val = content.substring(fid_idx + 12).toInt();
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        filter_id = fid_val;
        xSemaphoreGive(state_mutex);
    }

    Serial.println("[SETTINGS] Loaded from SD");
}
