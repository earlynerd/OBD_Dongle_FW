#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

// ═══════════════════════════════════════════════════════════════════════════
// Command Handler Task
// ═══════════════════════════════════════════════════════════════════════════

// FreeRTOS task that handles serial CLI commands
void cmd_task(void *pv);

// Helper function to read battery voltage (shared with command handler)
float read_vbat();

// Print list of seen CAN IDs (used by housekeeping stats mode)
void print_ids();

#endif // COMMAND_HANDLER_H
