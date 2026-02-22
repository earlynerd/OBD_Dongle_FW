#ifndef CAN_SIGNALS_370Z_H
#define CAN_SIGNALS_370Z_H

#include <stdint.h>

// Signal value types
enum signal_type_t {
    SIG_BOOL,       // Single bit, displayed as ON/OFF
    SIG_UINT8,      // Unsigned 8-bit
    SIG_UINT16,     // Unsigned 16-bit (big-endian: high byte first)
    SIG_INT16,      // Signed 16-bit (big-endian)
    SIG_UINT16_LE,  // Unsigned 16-bit little-endian (low byte first)
    SIG_INT16_LE,   // Signed 16-bit little-endian
    SIG_UINT24,     // Unsigned 24-bit (big-endian)
    // Non-byte-aligned types (for DBC compatibility)
    SIG_UINT12_BE,  // 12-bit big-endian: (byte[n] << 4) | (byte[n+1] >> 4)
    SIG_INT12_BE,   // 12-bit signed big-endian (sign extend from bit 11)
    SIG_UINT12_BE_LOW, // 12-bit: low nibble of byte[n] << 8 | byte[n+1]
    SIG_UINT7,      // 7-bit unsigned (masked from 8-bit byte)
    SIG_UINT3,      // 3-bit unsigned (in low bits of byte)
    SIG_UINT4,      // 4-bit unsigned (nibble, in low bits of byte)
    SIG_UINT11_BE,  // 11-bit big-endian: (byte[n] << 3) | (byte[n+1] >> 5)
    SIG_UINT10_BE   // 10-bit big-endian: (byte[n] << 2) | (byte[n+1] >> 6) - existing 10-bit
};

// Signal definition
struct can_signal_t {
    const char* name;       // Short name for display
    uint8_t start_byte;     // Byte index (0=A, 1=B, etc.)
    uint8_t start_bit;      // Bit position (0-7), or 0xFF for whole byte/word
    uint8_t bit_length;     // Number of bits (1 for bool, 8 for byte, 16 for word)
    signal_type_t type;
    float scale;            // Multiply raw value by this
    float offset;           // Add this after scaling
    const char* unit;       // Unit string ("RPM", "mph", "%", etc.)
};

// Message definition
struct can_message_t {
    uint32_t id;
    const char* name;
    const char* source;
    uint8_t dlc;
    const can_signal_t* signals;
    uint8_t signal_count;
};

// ═══════════════════════════════════════════════════════════════════════════
// Signal definitions per CAN ID
// ═══════════════════════════════════════════════════════════════════════════

// 0x002 - Steering Angle Sensor
static const can_signal_t signals_002[] = {
    {"SteerAngle",    0, 0xFF, 16, SIG_INT16_LE, 0.1f,    0.0f, "deg"},  // A,B: signed LE, /10
    {"SteerVel",      2, 0xFF,  8, SIG_UINT8,    4.0f,    0.0f, "d/s"},  // C: angular velocity (REF: *4, 0-1020°/s)
};

// 0x160 - Throttle/Pedal
static const can_signal_t signals_160[] = {
    {"Pedal1",        3, 0xFF, 10, SIG_UINT10_BE, 0.125f,  0.0f, "%"},    // D[7:0]+E[7:6] = 10 bits, /8
    {"WOT",           4,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // E bit 5
    {"ThrottleClosed",4,    3,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // E bit 3
    {"Unk160_B0_0",   0,    0,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 0 (hunt: 2 changes)
    {"Unk160_B2_5",   2,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // C bit 5 (hunt: 8 changes)
    {"Unk160_B2_6",   2,    6,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // C bit 6 (hunt: 10 changes)
    {"Unk160_B6_5",   6,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // G bit 5 (hunt: 4 changes, press/release)
};

// 0x180 - ECM Primary
static const can_signal_t signals_180[] = {
    {"RPM",           0, 0xFF, 16, SIG_UINT16,   0.125f,  0.0f, "rpm"},  // A,B: /8
    {"Pedal",         5, 0xFF, 10, SIG_UINT10_BE, 0.125f,  0.0f, "%"},    // F[7:0]+G[7:6] = 10 bits, /8
    {"Clutch",        7,    2,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // H bit 2
    {"ClutchFull",    7,    0,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // H bit 0
};

// 0x182 - ECM Secondary
static const can_signal_t signals_182[] = {
    {"ThrottleClosed",1,    7,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 7
    {"Throttle",      4, 0xFF,  8, SIG_UINT8,    0.392f,  0.0f, "%"},    // E: *100/255
    {"BrakeLight",    6,    6,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // G bit 6
};

// 0x1F9 - ECM AC/RPM
static const can_signal_t signals_1F9[] = {
    {"ACReq",         0,    3,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 3 (hunt: 4 changes)
    {"Unk1F9_B0_4",   0,    4,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 4 (hunt: 1 change)
    {"RPM",           2, 0xFF, 16, SIG_UINT16,   0.125f,  0.0f, "rpm"},  // C,D: /8
    {"IgnRun",        0,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 5: ignition run (REF bit 61)
    {"IgnStart",      0,    4,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 4: ignition starter (REF bit 60)
};

// 0x215 - HVAC
static const can_signal_t signals_215[] = {
    {"AC",            1,    3,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 3
    {"ACComp",        1,    2,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 2
};

// 0x216 - IPDM (Power Distribution)
static const can_signal_t signals_216[] = {
    {"IgnBtn",        0,    1,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 1
    {"IgnON",         0,    0,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 0 (hunt: 8 changes)
    {"Unk216_B0_3",   0,    3,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 3 (hunt: 20 changes)
    {"Hood",          1,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 5
    {"ClutchFull",    1,    3,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 3 (hunt: 12 changes)
    {"Unk216_B1_0",   1,    0,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 0 (hunt: 4 changes, ~150ms pairs)
    {"Unk216_B1_2",   1,    2,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 2 (hunt: 4 changes)
    {"Unk216_B1_6",   1,    6,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 6 (hunt: 4 changes)
};

// 0x280 - Seatbelt/Speed
static const can_signal_t signals_280[] = {
    {"Seatbelt",      0,    0,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 0 (inverted: 0=buckled)
    {"Speed",         4, 0xFF, 16, SIG_UINT16,   0.00621f,0.0f, "mph"},  // E,F: /160.934
};

// 0x284 - ABS Wheel Speeds (Front)
static const can_signal_t signals_284[] = {
    {"WheelFR",       0, 0xFF, 16, SIG_UINT16,   0.00311f,0.0f, "mph"},  // A,B: /(2*160.934)
    {"WheelFL",       2, 0xFF, 16, SIG_UINT16,   0.00311f,0.0f, "mph"},  // C,D
    {"Speed",         4, 0xFF, 16, SIG_UINT16,   0.00621f,0.0f, "mph"},  // E,F
};

// 0x285 - ABS Wheel Speeds (Rear)
static const can_signal_t signals_285[] = {
    {"WheelRR",       0, 0xFF, 16, SIG_UINT16,   0.00311f,0.0f, "mph"},  // A,B
    {"WheelRL",       2, 0xFF, 16, SIG_UINT16,   0.00311f,0.0f, "mph"},  // C,D
};

// 0x292 - IMU and Brake (from 370Z CSV + empirical validation)
// Byte layout: [FF] [Fx LL] [HH Hx] [FE] [BB] [00]
//   Byte 0: 0xFF constant
//   Bytes 1-2: 12-bit LatAccel (low nibble of byte 1 + byte 2), offset -2048
//   Bytes 3-4: 12-bit Yaw Rate (byte 3 + high nibble of byte 4), offset -2048
//   Byte 4 low nibble: 0xF constant
//   Byte 5: 0xFE constant
//   Byte 6: Brake pressure (0-29 observed, increases during braking)
//   Byte 7: 0x00 constant
//   Invalid sentinel frame seen in logs: FF FF FF FF FF FE FF 00 (discard)
static const can_signal_t signals_292[] = {
    {"LatAccel",      1, 0xFF, 12, SIG_UINT12_BE_LOW, 1.0f, -2048.0f, ""},  // B,C: lateral accel (low nibble B + full C)
    {"Yaw",           3, 0xFF, 12, SIG_UINT12_BE,     1.0f, -2048.0f, ""},  // D,E: yaw rate (full D + high nibble E)
    {"BrakePres",     6, 0xFF,  8, SIG_UINT8,         1.0f,    0.0f,  ""},  // G: brake pressure (0 when released)
};

// 0x351 - Key/Clutch/Buttons
static const can_signal_t signals_351[] = {
    {"KeyPresent",    5,    2,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // F bit 2
    {"Unk351_B5_0",   5,    0,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // F bit 0 (hunt: 3 changes)
    {"Unk351_B5_6",   5,    6,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // F bit 6 (hunt: 6 changes)
    {"Unk351_B6_0",   6,    0,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // G bit 0 (hunt: 4 changes, ~260ms pairs)
    {"Unk351_B6_6",   6,    6,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // G bit 6 (hunt: 2 changes)
    {"ClutchFull",    7,    2,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // H bit 2 (hunt: 12 changes)
    {"Unk351_B1_6",   1,    6,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 6 (hunt: 2 changes, 300ms pair)
    {"Unk351_B1_7",   1,    7,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 7 (hunt: 2 changes, 300ms pair)
    {"Unk351_B7_0",   7,    0,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // H bit 0 (hunt: 4 changes, ~1400ms pairs)
    {"Unk351_B7_1",   7,    1,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // H bit 1 (hunt: 4 changes)
    {"Unk351_B7_3",   7,    3,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // H bit 3 (hunt: 8 changes)
    {"Unk351_B7_4",   7,    4,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // H bit 4 (hunt: 4 changes, ~1050ms pairs)
};

// 0x354 - ABS/VDC
static const can_signal_t signals_354[] = {
    {"Speed",         0, 0xFF, 16, SIG_UINT16,   0.00621f,0.0f, "mph"},  // A,B: /160.934
    {"VDC",           4,    6,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // E bit 6
    {"VDCOff",        5,    7,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // F bit 7
    {"Brake",         6,    4,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // G bit 4
    {"VDCWarn",       6,    0,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // G bit 0
};

// 0x355 - Speed/Units
static const can_signal_t signals_355[] = {
    {"Speed1",        0, 0xFF, 16, SIG_UINT16,   0.00621f,0.0f, "mph"},  // A,B
    {"Speed2",        2, 0xFF, 16, SIG_UINT16,   0.00621f,0.0f, "mph"},  // C,D (0xFFFF observed as invalid sentinel)
    {"Units",         4,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // E bit 5: 0=Metric, 1=English
};

// 0x358 - BCM Status
static const can_signal_t signals_358[] = {
    {"KeySlot",       0,    0,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 0
    {"Unk358_B0_1",   0,    1,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 1 (hunt: 9 changes)
    {"Unk358_B0_4",   0,    4,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 4 (hunt: 20 changes)
    {"Stopped",       1,    3,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 3
    {"Headlights",    1,    1,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 1
    {"Unk358_B1_7",   1,    7,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 7 (hunt: 5 changes)
    {"Unk358_B3_5",   3,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // D bit 5 (hunt: 4 changes, ~1850ms pairs)
    {"Unk358_B3_7",   3,    7,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // D bit 7 (hunt: 1 change)
    {"Unk358_B4_7",   4,    7,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // E bit 7 (hunt: 4 changes, ~1010ms pairs)
    {"Trunk",         6,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // G bit 5 (release)
};

// 0x35D - BCM Wipers/Brake
static const can_signal_t signals_35D[] = {
    {"Unk35D_B0_6",   0,    6,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 6 (hunt: 4 changes, ~1600ms pairs)
    {"Unk35D_B0_7",   0,    7,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 7 (hunt: 4 changes)
    {"WiperCont",     2,    7,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // C bit 7
    {"WiperOn",       2,    6,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // C bit 6
    {"WiperFast",     2,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // C bit 5
    {"Forward",       4,    6,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // E bit 6
    {"Brake",         4,    4,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // E bit 4
};

// 0x385 - Tire Pressure
static const can_signal_t signals_385[] = {
    {"Tire1",         2, 0xFF,  8, SIG_UINT8,    0.25f,   0.0f, "psi"},  // C: /4
    {"Tire2",         3, 0xFF,  8, SIG_UINT8,    0.25f,   0.0f, "psi"},  // D
    {"Tire3",         4, 0xFF,  8, SIG_UINT8,    0.25f,   0.0f, "psi"},  // E
    {"Tire4",         5, 0xFF,  8, SIG_UINT8,    0.25f,   0.0f, "psi"},  // F
    {"Tire1Valid",    6,    7,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // G bit 7
    {"Tire2Valid",    6,    6,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // G bit 6
    {"Tire3Valid",    6,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // G bit 5
    {"Tire4Valid",    6,    4,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // G bit 4
};

// 0x551 - Coolant Temperature and Cruise Control
// Cruise speed: 255=off, 254=on but no speed set, else km/h
static const can_signal_t signals_551[] = {
    {"Coolant",       0, 0xFF,  8, SIG_UINT8,    1.0f,  -40.0f, "C"},    // A: -40
    {"FuelCons",      1, 0xFF,  8, SIG_UINT8,    1.0f,    0.0f, "mm3"},  // B: fuel consumption (from 370Z CSV)
    {"CruiseSpd",     4, 0xFF,  8, SIG_UINT8,    0.621f,  0.0f, "mph"},  // E: km/h to mph
    {"CruiseMaster",  5,    6,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // F bit 6: master on
    {"CruiseActive",  5,    4,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // F bit 4: active (0=active when master on)
};

// 0x580 - Oil Temperature (alternate source)
static const can_signal_t signals_580[] = {
    {"OilTemp",       4, 0xFF,  8, SIG_UINT8,    1.0f,  -40.0f, "C"},    // E: -40
};

// 0x5C5 - Odometer, Hazards, Parking Brake, S-Mode Button
// Odometer is in bytes 1-3 (24-bit big-endian), units are miles
static const can_signal_t signals_5C5[] = {
    {"Hazard",        0,    6,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 6: hazard lights
    {"ParkBrake",     0,    2,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 2: parking brake
    {"Odometer",      1, 0xFF, 24, SIG_UINT24,   1.0f,    0.0f, "mi"},   // B,C,D: 24-bit BE
    {"SModeBtn",      4,    0,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // E bit 0: S-Mode button
};

// 0x174 - TCM Gear Position (7AT automatic only)
// Byte 3 low nibble: 1-7 = gears 1-7, 9 = Reverse, 10 = Park
static const can_signal_t signals_174[] = {
    {"ATGear",        3,    0,  4, SIG_UINT8,    1.0f,    0.0f, ""},     // D low nibble: AT gear
};

// 0x421 - Shifter Position (both 6MT and 7AT)
// 6MT: 16=R, 24=N, 128=1, 136=2, 144=3, 152=4, 160=5, 168=6
// 7AT: 8=P, 16=R, 24=N, 32=D
static const can_signal_t signals_421[] = {
    {"Shifter",       0, 0xFF,  8, SIG_UINT8,    1.0f,    0.0f, ""},     // A: shifter position
    {"Unk421_B0_2",   0,    2,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 2 (hunt: 4 changes, ~2000ms pairs)
    {"Unk421_B0_3",   0,    3,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 3 (hunt: 9 changes)
    {"Unk421_B0_4",   0,    4,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 4 (hunt: 9 changes)
    {"Unk421_B0_5",   0,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 5 (hunt: 2 changes)
    {"SMode",         1,    6,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 6: S-Mode status
};

// 0x54C - HVAC Status (nav-equipped cars)
static const can_signal_t signals_54C[] = {
    {"EvapTemp",      0, 0xFF,  8, SIG_UINT8,    1.0f,    0.0f, ""},     // A: AC evaporator temp
    {"EvapTarget",    1, 0xFF,  8, SIG_UINT8,    1.0f,    0.0f, ""},     // B: Target evap temp
    {"ACStatus",      2,    7,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // C bit 7: AC on
    {"Blower",        2,    6,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // C bit 6: Blower on
    {"Recirc",        7,    0,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // H bit 0: Recirculate
};

// 0x60D - BCM Doors/Lights/Locks
static const can_signal_t signals_60D[] = {
    {"Trunk",         0,    7,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 7: trunk open (hunt: 4 changes)
    {"PassDoor",      0,    4,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 4: passenger door
    {"DrvDoor",       0,    3,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 3: driver door (hunt: 6 changes)
    {"Running",       0,    2,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 2: running lights
    {"Headlights",    0,    1,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 1: headlights
    {"TurnR",         1,    6,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 6: right turn
    {"TurnL",         1,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 5: left turn
    {"HighBeam",      1,    3,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 3: high beams
    {"IgnON",         1,    2,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 2: ignition ON (hunt: 3 changes)
    {"IgnACC",        1,    1,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 1: ignition ACC (hunt: 8 changes)
    {"DrvLock",       2,    4,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // C bit 4: driver lock (hunt: 1 change)
    {"Unk60D_B2_3",   2,    3,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // C bit 3 (hunt: 1 change)
};

// 0x625 - IPDM Feedback (lights, wipers, defogger status)
static const can_signal_t signals_625[] = {
    {"Defogger",      0,    0,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 0: rear defogger
    {"ACCompFB",      1,    7,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 7: AC compressor (hunt: 4 changes)
    {"RunningFB",     1,    6,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 6: running lights
    {"LowBeamFB",     1,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 5: low beam
    {"HighBeamFB",    1,    4,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 4: high beam
    {"WiperFast",     1,    3,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 3: wipers fast
    {"WiperSlow",     1,    2,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 2: wipers slow
    {"WiperHome",     1,    1,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 1: wipers home
    {"Unk625_B3_4",   3,    4,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // D bit 4 (hunt: 2 changes)
    {"Unk625_B3_7",   3,    7,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // D bit 7 (hunt: 2 changes)
    {"Unk625_B4_5",   4,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // E bit 5 (hunt: 1 change)
};

// ═══════════════════════════════════════════════════════════════════════════
// G37 DBC signals - need verification on 370Z
// ═══════════════════════════════════════════════════════════════════════════

// 0x15C - Gas Pedal (from G37 DBC, verify on 370Z)
// Alternative source to 0x160/0x180 pedal signals
static const can_signal_t signals_15C[] = {
    {"GasPedal",      5, 0xFF, 10, SIG_UINT10_BE, 0.1f,   0.0f, "%"},    // F,G: gas pedal (10-bit, 0-100%)
};

// 0x20B - Cruise Stalk Buttons (from G37 DBC, verify on 370Z)
// Note: ProPilot button renamed to CruiseMain for 370Z context
static const can_signal_t signals_20B[] = {
    {"CruiseMain",    1,    0,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 0: cruise main button
    {"CruiseCancel",  1,    1,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 1: cancel button
    {"CruiseDist",    1,    2,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 2: follow distance button
    {"CruiseSet",     1,    3,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 3: set button
    {"CruiseRes",     1,    4,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 4: resume button
    {"NoButton",      1,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 5: no button pressed
    {"GasOff",        2,    4,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // C bit 4: gas pedal released (inverted)
    {"BrakeOn",       2,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // C bit 5: brake pressed
};

// 0x300 - Steering Torque Sensor (from G37 DBC, verify on 370Z)
static const can_signal_t signals_300[] = {
    {"SteerTorque",   0, 0xFF,  7, SIG_UINT7,    1.0f,    0.0f, ""},     // A: steering torque (7-bit, 0-127)
    {"SteerPressed",  1,    7,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 7: steering input detected
};

// 0x41F - Gearbox (from G37 DBC, verify on 370Z)
// Gear values: TBD (may differ from 0x421 shifter position)
static const can_signal_t signals_41F[] = {
    {"GearPos",       0,    3,  3, SIG_UINT3,    1.0f,    0.0f, ""},     // A bits 3-5: gear position (3-bit)
    {"SportsMode",    1,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 5: sports mode
};

// 0x453 - Lights/Blinkers (from G37 DBC, verify on 370Z)
static const can_signal_t signals_453[] = {
    {"Headlights",    0,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 5: headlights
    {"TurnL",         1,    3,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 3: left blinker
    {"TurnR",         1,    4,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // B bit 4: right blinker
};

// 0x4F9 - HUD/Seatbelt (from G37 DBC, verify on 370Z)
static const can_signal_t signals_4F9[] = {
    {"SpeedMPH",      0,    5,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 5: speed display in MPH
    {"SeatbeltDrv",   3,    1,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // D bit 1: driver seatbelt latched
};

// 0x542 - HVAC Temperature Settings (from G37 DBC, verify on 370Z)
// Note: Temps are 7-bit values, using 8-bit read
static const can_signal_t signals_542[] = {
    {"TempDrv",       1, 0xFF,  8, SIG_UINT8,    1.0f,    0.0f, ""},     // B: driver temp setting (7-bit, masked)
    {"TempPass",      2, 0xFF,  8, SIG_UINT8,    1.0f,    0.0f, ""},     // C: passenger temp setting (7-bit, masked)
};

// 0x54B - HVAC Controls (from G37 DBC, verify on 370Z)
// Mode: 0=off, 1=face, 2=face+feet, 3=feet, 4=feet+defrost, 5=defrost
// Fan: 0=off, 1-7=speed levels
static const can_signal_t signals_54B[] = {
    {"HVACOff",       0,    0,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // A bit 0: HVAC off
    {"HVACMode",      2,    3,  3, SIG_UINT3,    1.0f,    0.0f, ""},     // C bits 3-5: HVAC mode (3-bit)
    {"RecircOn",      3,    0,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // D bit 0: recirculate on
    {"RecircOff",     3,    1,  1, SIG_BOOL,     1.0f,    0.0f, ""},     // D bit 1: recirculate off
    {"FanLevel",      4,    3,  3, SIG_UINT3,    1.0f,    0.0f, ""},     // E bits 3-5: fan level (3-bit, 0-7)
};

// ═══════════════════════════════════════════════════════════════════════════
// Message Database
// ═══════════════════════════════════════════════════════════════════════════

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

static const can_message_t can_messages[] = {
    {0x002, "STRG",   "STRG",    5, signals_002, ARRAY_SIZE(signals_002)},
    {0x160, "THROT",  "ECM",     7, signals_160, ARRAY_SIZE(signals_160)},
    {0x174, "TCM",    "TCM",     8, signals_174, ARRAY_SIZE(signals_174)},
    {0x180, "ECM1",   "ECM",     8, signals_180, ARRAY_SIZE(signals_180)},
    {0x182, "ECM2",   "ECM",     8, signals_182, ARRAY_SIZE(signals_182)},
    {0x15C, "GAS",    "ECM",     8, signals_15C, ARRAY_SIZE(signals_15C)},  // G37 DBC
    {0x1F9, "ECM_AC", "ECM",     8, signals_1F9, ARRAY_SIZE(signals_1F9)},
    {0x20B, "CRUIS",  "STALK",   6, signals_20B, ARRAY_SIZE(signals_20B)},  // G37 DBC
    {0x215, "HVAC",   "HVAC",    6, signals_215, ARRAY_SIZE(signals_215)},
    {0x216, "IPDM",   "IPDM",    2, signals_216, ARRAY_SIZE(signals_216)},
    {0x280, "SEAT",   "M&A",     8, signals_280, ARRAY_SIZE(signals_280)},
    {0x284, "ABS_F",  "ABS",     8, signals_284, ARRAY_SIZE(signals_284)},
    {0x285, "ABS_R",  "ABS",     8, signals_285, ARRAY_SIZE(signals_285)},
    {0x292, "IMU",    "ABS",     8, signals_292, ARRAY_SIZE(signals_292)},
    {0x300, "STRTQ",  "STRG",    2, signals_300, ARRAY_SIZE(signals_300)},  // G37 DBC
    {0x351, "KEY",    "BCM",     8, signals_351, ARRAY_SIZE(signals_351)},
    {0x354, "VDC",    "ABS",     8, signals_354, ARRAY_SIZE(signals_354)},
    {0x355, "SPEED",  "ABS",     7, signals_355, ARRAY_SIZE(signals_355)},
    {0x358, "BCM1",   "BCM",     8, signals_358, ARRAY_SIZE(signals_358)},
    {0x35D, "BCM2",   "BCM",     8, signals_35D, ARRAY_SIZE(signals_35D)},
    {0x385, "TPMS",   "BCM",     7, signals_385, ARRAY_SIZE(signals_385)},
    {0x41F, "GBOX",   "TCM",     2, signals_41F, ARRAY_SIZE(signals_41F)},  // G37 DBC
    {0x421, "GEAR",   "ECM",     3, signals_421, ARRAY_SIZE(signals_421)},
    {0x453, "LIGHT",  "BCM",     8, signals_453, ARRAY_SIZE(signals_453)},  // G37 DBC
    {0x4F9, "HUD",    "M&A",     7, signals_4F9, ARRAY_SIZE(signals_4F9)},  // G37 DBC
    {0x542, "HVTMP",  "HVAC",    8, signals_542, ARRAY_SIZE(signals_542)},  // G37 DBC
    {0x54B, "HVCTL",  "HVAC",    8, signals_54B, ARRAY_SIZE(signals_54B)},  // G37 DBC
    {0x54C, "HVAC2",  "HVAC",    8, signals_54C, ARRAY_SIZE(signals_54C)},
    {0x551, "COOL",   "ECM",     8, signals_551, ARRAY_SIZE(signals_551)},
    {0x580, "OIL2",   "ECM",     8, signals_580, ARRAY_SIZE(signals_580)},
    {0x5C5, "M&A",    "M&A",     8, signals_5C5, ARRAY_SIZE(signals_5C5)},
    {0x60D, "BCM3",   "BCM",     8, signals_60D, ARRAY_SIZE(signals_60D)},
    {0x625, "IPDM2",  "IPDM",    8, signals_625, ARRAY_SIZE(signals_625)},
};

static const uint8_t CAN_MESSAGE_COUNT = ARRAY_SIZE(can_messages);

// ═══════════════════════════════════════════════════════════════════════════
// Lookup function
// ═══════════════════════════════════════════════════════════════════════════

inline const can_message_t* find_message(uint32_t id) {
    for (uint8_t i = 0; i < CAN_MESSAGE_COUNT; i++) {
        if (can_messages[i].id == id) {
            return &can_messages[i];
        }
    }
    return nullptr;
}

#endif // CAN_SIGNALS_370Z_H
