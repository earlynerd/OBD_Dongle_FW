#ifndef SETTINGS_H
#define SETTINGS_H

// ═══════════════════════════════════════════════════════════════════════════
// Settings Persistence
// ═══════════════════════════════════════════════════════════════════════════

// Save current settings (mode, logging, filter) to SD card
void save_settings();

// Load settings from SD card, applying defaults if not found
void load_settings();

#endif // SETTINGS_H
