#ifndef LOGGER_H
#define LOGGER_H

// ═══════════════════════════════════════════════════════════════════════════
// SD Card Logger Task
// ═══════════════════════════════════════════════════════════════════════════

// FreeRTOS task that handles SD card logging
// Batch-writes frames to SD card with periodic flush
void logger_task(void *pv);

#endif // LOGGER_H
