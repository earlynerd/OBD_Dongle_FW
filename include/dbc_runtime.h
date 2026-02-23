#ifndef DBC_RUNTIME_H
#define DBC_RUNTIME_H

#include <stdint.h>

// Forward declaration from can_signals_370z.h
struct can_message_t;

// Load first .dbc file found in SD root (case-insensitive).
// Returns true if a DBC was loaded and parsed successfully.
bool dbc_runtime_load_from_sd_root();

// Load a specific DBC path from SD (e.g. "/Nissan_370Z_Z34.dbc").
bool dbc_runtime_load_from_path(const char *path);

// Runtime DB query helpers.
bool dbc_runtime_is_loaded();
const char *dbc_runtime_loaded_path();
const can_message_t *dbc_runtime_find_message(uint32_t id);

#endif // DBC_RUNTIME_H
