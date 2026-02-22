# ESP32 CAN Bus Monitor

ESP32-based CAN bus sniffer and monitor for automotive applications. Features real-time CAN monitoring, SD card logging, delta detection, OBD2 diagnostics, and an interactive serial command interface.

Tested with Nissan 370Z but adaptable to other vehicles.

## Features

- **Real-time CAN monitoring** at 500kbps with microsecond timestamps
- **Multiple display modes**: raw frames, delta detection, decoded signals, statistics
- **SD card logging** to CSV for offline analysis
- **OBD2 diagnostics**: query PIDs, read/clear DTCs, VIN retrieval
- **Nissan Consult protocol** support for extended diagnostics
- **370Z signal decoding**: steering angle, RPM, wheel speeds, temperatures, TPMS
- **Safety interlocks**: TX blocked when engine running or vehicle moving
- **Headless operation**: configure once, then collect data without a computer
- **Battery monitoring** for portable use

## Hardware
- **Custom ESP32 dongle**
- **MCU**: ESP32
- **CAN Transceiver**: SN65HVD256D or similar (with silent mode support)
- **Storage**: MicroSD card module (SPI)
- **Power**: OBD2 port powered


## Building

Requires [PlatformIO](https://platformio.org/).

```bash
# Build
pio run -e featheresp32

# Upload to ESP32
pio run -t upload -e featheresp32

# Serial monitor (921600 baud)
pio device monitor -b 921600
```

## Usage

Connect to the serial port at 921600 baud. Type `help` for available commands.

### Operating Modes

| Command | Description |
|---------|-------------|
| `monitor` | Print all CAN frames |
| `delta` | Only print frames when payload changes (default) |
| `decode` | Human-readable decoded signals (370Z-specific) |
| `stats` | Periodic ID statistics summary |
| `silent` | Log to SD card only, no serial output |

### Filtering

| Command | Description |
|---------|-------------|
| `filter <ID>` | Show only frames with specified CAN ID (hex) |
| `nofilter` | Show all CAN IDs |

### Logging

| Command | Description |
|---------|-------------|
| `log` | Start logging to SD card |
| `pause` | Stop logging |

Logs are saved as CSV files in `/logs/` on the SD card.

### OBD2 Diagnostics

| Command | Description |
|---------|-------------|
| `tx on` | Enable CAN transmission (required for queries) |
| `tx off` | Disable CAN transmission (silent mode) |
| `obd <PID>` | Query standard OBD2 PID (e.g., `obd 0c` for RPM) |
| `ext <PID>` | Query Nissan extended PID |
| `dtc` | Read diagnostic trouble codes |
| `dtc clear` | Clear DTCs (requires confirmation) |
| `vin` | Read vehicle VIN |
| `scan` | Scan for supported PIDs |
| `pids` | List known PID definitions |

### Nissan Consult

| Command | Description |
|---------|-------------|
| `modules` | List all known ECU modules |
| `scan-modules` | Scan bus for responding modules |
| `query <module> <pid>` | Query specific module PID |
| `id <module>` | Get module identification string |
| `ecm`, `bcm`, `ipdm`, `abs` | List PIDs for each module |

### Hunt Mode (Button/Switch Identification)

Hunt mode helps identify 1-bit CAN signals like buttons, switches, and toggles by showing bit-level changes from a captured baseline.

| Command | Description |
|---------|-------------|
| `snapshot` | Capture baseline of all CAN IDs |
| `hunt` | Enter hunt mode, show bit changes from snapshot |
| `hunt sticky` | Hunt mode with persistent change display |
| `hunt -q` | Hunt mode, suppress noisy bits (>5 changes) |
| `mark <name>` | Label the next bit change (e.g., `mark HazardSwitch`) |
| `hunt clear` | Clear change counters |
| `hunt save` | Export findings to `/hunt_results.csv` on SD card |

**Workflow:**
1. With car on (engine off), run `snapshot` to capture baseline
2. Run `hunt` to enter hunt mode
3. Type `mark HazardSwitch` (or whatever you're about to test)
4. Press the button or activate the switch
5. See the labeled change: `[HUNT] 0x354 [3].5  0->1  "HazardSwitch"`
6. Repeat steps 3-5 for other controls
7. Run `hunt save` to export all findings with labels

**Output format:** `[HUNT] 0x<CAN_ID> [<byte>].<bit>  <old>-><new>`

### Utility

| Command | Description |
|---------|-------------|
| `ids` | Show all detected CAN IDs with frame counts |
| `clear` | Clear ID tracking data |
| `vbat` | Show battery voltage |
| `status` | Show current settings and status |
| `help` | Show command help |

## Safety Features

TX mode is **disabled by default** (receive-only). When enabled:

- TX is blocked if engine is running (detected via RPM on CAN bus)
- TX is blocked if vehicle is moving (detected via speed on CAN bus)
- CAN transceiver automatically switches between silent and normal mode

This prevents accidental interference with vehicle systems while driving.

## Settings Persistence

Settings are automatically saved to `/settings.json` on the SD card:

- Operating mode
- Logging enabled/disabled
- Filter settings

Settings are restored on boot, enabling headless data collection.

## Architecture

FreeRTOS multi-tasking with dedicated cores:

```
CAN Hardware FIFO
    |
can_rx_task (Core 1, Priority 6)
    |
dispatch_queue
    |
dispatch_task (Core 1, Priority 5)
    |
    +---> Serial output
    +---> log_queue ---> logger_task (Core 0) ---> SD card
```

## 370Z CAN Signals

Decoded signals include:

| CAN ID | Signals |
|--------|---------|
| 0x002 | Steering angle, angular velocity |
| 0x180 | RPM, pedal position, clutch |
| 0x182 | Throttle position, brake light |
| 0x284/285 | Wheel speeds (all 4 corners) |
| 0x354 | Vehicle speed, VDC status |
| 0x35D | Wipers, brake pedal |
| 0x385 | Tire pressures (TPMS) |
| 0x551/5C5 | Coolant/oil temperature |

See `include/can_signals_370z.h` for the complete signal database.

## Dependencies

- Platform: espressif32
- Framework: Arduino
- Library: [ESP32CAN](https://github.com/miwagner/ESP32-Arduino-CAN) v0.0.1

## License

MIT License - See LICENSE file for details.
