#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// ═══════════════════════════════════════════════════════════════════════════
// Pin Definitions for Adafruit Feather ESP32 + CAN + SD
// ═══════════════════════════════════════════════════════════════════════════

// SD Card SPI
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK  18
#define SD_CS   16
#define SD_CD   17      // Card detect

// CAN Bus (MCP2551 transceiver)
#define CAN_RX   4
#define CAN_TX   5
#define CAN_SLNT 22     // Silent mode control (LOW = TX enabled, HIGH = silent)

// Power Management
#define PWR_FAULT 21
#define VBAT_DIV  27    // Battery voltage divider: 40.2k upper, 10k lower

#endif // PIN_CONFIG_H
