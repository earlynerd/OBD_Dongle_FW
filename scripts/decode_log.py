#!/usr/bin/env python3
"""
370Z CAN Log Decoder
Decodes canlog.csv files using the 370Z signal database.

Usage:
    python decode_log.py canlog.csv                    # Print decoded output
    python decode_log.py canlog.csv -o decoded.csv    # Save to CSV
    python decode_log.py canlog.csv --filter 0x180    # Filter to specific ID
    python decode_log.py canlog.csv --delta           # Only show changes
    python decode_log.py canlog.csv --plot RPM,Speed  # Plot signals over time
    python decode_log.py canlog.csv --plot ECM1.RPM,Throttle --subplots  # Separate subplots
"""

import argparse
import csv
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Set

# Try to import matplotlib for plotting
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

# Signal types
SIG_BOOL = 0
SIG_UINT8 = 1
SIG_UINT16 = 2      # Big-endian unsigned
SIG_INT16 = 3       # Big-endian signed
SIG_UINT16_LE = 4   # Little-endian unsigned
SIG_INT16_LE = 5    # Little-endian signed
SIG_UINT24 = 6      # Big-endian 24-bit unsigned
# Non-byte-aligned types (for DBC compatibility)
SIG_UINT12_BE = 7   # 12-bit big-endian: (byte[n] << 4) | (byte[n+1] >> 4)
SIG_INT12_BE = 8    # 12-bit signed big-endian (sign extend from bit 11)
SIG_UINT7 = 9       # 7-bit unsigned (masked from 8-bit byte)
SIG_UINT3 = 10      # 3-bit unsigned (in low bits of byte)
SIG_UINT4 = 11      # 4-bit unsigned (nibble, in low bits of byte)
SIG_UINT11_BE = 12  # 11-bit big-endian: (byte[n] << 3) | (byte[n+1] >> 5)
SIG_UINT10_BE = 13  # 10-bit big-endian: (byte[n] << 2) | (byte[n+1] >> 6)
SIG_UINT12_BE_LOW = 14  # 12-bit: (byte[n] & 0x0F) << 8 | byte[n+1]

@dataclass
class Signal:
    name: str
    start_byte: int
    start_bit: int  # 0xFF = whole byte/word
    bit_length: int
    sig_type: int
    scale: float
    offset: float
    unit: str

@dataclass
class Message:
    id: int
    name: str
    source: str
    dlc: int
    signals: List[Signal]

# Signal database (matches can_signals_370z.h)
MESSAGES: Dict[int, Message] = {
    0x002: Message(0x002, "STRG", "STRG", 5, [
        Signal("SteerAngle", 0, 0xFF, 16, SIG_INT16_LE, 0.1, 0.0, "deg"),  # Little-endian signed
        Signal("SteerVel", 2, 0xFF, 8, SIG_UINT8, 4.0, 0.0, "d/s"),
    ]),
    0x160: Message(0x160, "THROT", "ECM", 7, [
        Signal("Pedal1", 3, 0xFF, 10, SIG_UINT16, 0.125, 0.0, "%"),
        Signal("WOT", 4, 5, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("ThrottleClosed", 4, 3, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x174: Message(0x174, "TCM", "TCM", 8, [
        Signal("ATGear", 3, 0, 4, SIG_UINT8, 1.0, 0.0, ""),  # Low nibble of byte 3
    ]),
    0x180: Message(0x180, "ECM1", "ECM", 8, [
        Signal("RPM", 0, 0xFF, 16, SIG_UINT16, 0.125, 0.0, "rpm"),
        Signal("Pedal", 5, 0xFF, 10, SIG_UINT16, 0.125, 0.0, "%"),
        Signal("Clutch", 7, 2, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("ClutchFull", 7, 0, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x182: Message(0x182, "ECM2", "ECM", 8, [
        Signal("ThrottleClosed", 1, 7, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("Throttle", 4, 0xFF, 8, SIG_UINT8, 0.392, 0.0, "%"),
        Signal("BrakeLight", 6, 6, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x15C: Message(0x15C, "GAS", "ECM", 8, [  # G37 DBC
        Signal("GasPedal", 5, 0xFF, 10, SIG_UINT10_BE, 0.1, 0.0, "%"),
    ]),
    0x1F9: Message(0x1F9, "ECM_AC", "ECM", 8, [
        Signal("ACReq", 0, 3, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("RPM", 2, 0xFF, 16, SIG_UINT16, 0.125, 0.0, "rpm"),
        Signal("IgnRun", 0, 5, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("IgnStart", 0, 4, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x20B: Message(0x20B, "CRUIS", "STALK", 6, [  # G37 DBC
        Signal("CruiseMain", 1, 0, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("CruiseCancel", 1, 1, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("CruiseDist", 1, 2, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("CruiseSet", 1, 3, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("CruiseRes", 1, 4, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("NoButton", 1, 5, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("GasOff", 2, 4, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("BrakeOn", 2, 5, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x215: Message(0x215, "HVAC", "HVAC", 6, [
        Signal("AC", 1, 3, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("ACComp", 1, 2, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x216: Message(0x216, "IPDM", "IPDM", 2, [
        Signal("IgnBtn", 0, 1, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("IgnON", 0, 0, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("Hood", 1, 5, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("ClutchFull", 1, 3, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x280: Message(0x280, "SEAT", "M&A", 8, [
        Signal("Seatbelt", 0, 0, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("Speed", 4, 0xFF, 16, SIG_UINT16, 0.00621, 0.0, "mph"),
    ]),
    0x284: Message(0x284, "ABS_F", "ABS", 8, [
        Signal("WheelFR", 0, 0xFF, 16, SIG_UINT16, 0.00311, 0.0, "mph"),
        Signal("WheelFL", 2, 0xFF, 16, SIG_UINT16, 0.00311, 0.0, "mph"),
        Signal("Speed", 4, 0xFF, 16, SIG_UINT16, 0.00621, 0.0, "mph"),
    ]),
    0x285: Message(0x285, "ABS_R", "ABS", 8, [
        Signal("WheelRR", 0, 0xFF, 16, SIG_UINT16, 0.00311, 0.0, "mph"),
        Signal("WheelRL", 2, 0xFF, 16, SIG_UINT16, 0.00311, 0.0, "mph"),
    ]),
    0x292: Message(0x292, "IMU", "ABS", 8, [
        Signal("LatAccel", 1, 0xFF, 12, SIG_UINT12_BE_LOW, 1.0, -2048.0, ""),
        Signal("Yaw", 3, 0xFF, 12, SIG_UINT12_BE, 1.0, -2048.0, ""),
        Signal("BrakePres", 6, 0xFF, 8, SIG_UINT8, 1.0, 0.0, ""),
    ]),
    0x300: Message(0x300, "STRTQ", "STRG", 2, [  # G37 DBC
        Signal("SteerTorque", 0, 0xFF, 7, SIG_UINT7, 1.0, 0.0, ""),
        Signal("SteerPressed", 1, 7, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x351: Message(0x351, "KEY", "BCM", 8, [
        Signal("KeyPresent", 5, 2, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("ClutchFull", 7, 2, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x354: Message(0x354, "VDC", "ABS", 8, [
        Signal("Speed", 0, 0xFF, 16, SIG_UINT16, 0.00621, 0.0, "mph"),
        Signal("VDC", 4, 6, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("VDCOff", 5, 7, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("Brake", 6, 4, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("VDCWarn", 6, 0, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x355: Message(0x355, "SPEED", "ABS", 7, [
        Signal("Speed1", 0, 0xFF, 16, SIG_UINT16, 0.00621, 0.0, "mph"),
        Signal("Speed2", 2, 0xFF, 16, SIG_UINT16, 0.00621, 0.0, "mph"),
        Signal("Units", 4, 5, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x358: Message(0x358, "BCM1", "BCM", 8, [
        Signal("KeySlot", 0, 0, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("Stopped", 1, 3, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("Headlights", 1, 1, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("Trunk", 6, 5, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x35D: Message(0x35D, "BCM2", "BCM", 8, [
        Signal("WiperCont", 2, 7, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("WiperOn", 2, 6, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("WiperFast", 2, 5, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("Forward", 4, 6, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("Brake", 4, 4, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x385: Message(0x385, "TPMS", "BCM", 7, [
        Signal("Tire1", 2, 0xFF, 8, SIG_UINT8, 0.25, 0.0, "psi"),
        Signal("Tire2", 3, 0xFF, 8, SIG_UINT8, 0.25, 0.0, "psi"),
        Signal("Tire3", 4, 0xFF, 8, SIG_UINT8, 0.25, 0.0, "psi"),
        Signal("Tire4", 5, 0xFF, 8, SIG_UINT8, 0.25, 0.0, "psi"),
        Signal("Tire1Valid", 6, 7, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("Tire2Valid", 6, 6, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("Tire3Valid", 6, 5, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("Tire4Valid", 6, 4, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x551: Message(0x551, "COOL", "ECM", 8, [
        Signal("Coolant", 0, 0xFF, 8, SIG_UINT8, 1.0, -40.0, "C"),
        Signal("FuelCons", 1, 0xFF, 8, SIG_UINT8, 1.0, 0.0, "mm3"),
        Signal("CruiseSpd", 4, 0xFF, 8, SIG_UINT8, 0.621, 0.0, "mph"),
        Signal("CruiseMaster", 5, 6, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("CruiseActive", 5, 4, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x5C5: Message(0x5C5, "M&A", "M&A", 8, [
        Signal("Hazard", 0, 6, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("ParkBrake", 0, 2, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("Odometer", 1, 0xFF, 24, SIG_UINT24, 1.0, 0.0, "mi"),
        Signal("SModeBtn", 4, 0, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x41F: Message(0x41F, "GBOX", "TCM", 2, [  # G37 DBC
        Signal("GearPos", 0, 3, 3, SIG_UINT3, 1.0, 0.0, ""),
        Signal("SportsMode", 1, 5, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x421: Message(0x421, "GEAR", "ECM", 3, [
        Signal("Shifter", 0, 0xFF, 8, SIG_UINT8, 1.0, 0.0, ""),
        Signal("SMode", 1, 6, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x453: Message(0x453, "LIGHT", "BCM", 8, [  # G37 DBC
        Signal("Headlights", 0, 5, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("TurnL", 1, 3, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("TurnR", 1, 4, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x4F9: Message(0x4F9, "HUD", "M&A", 7, [  # G37 DBC
        Signal("SpeedMPH", 0, 5, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("SeatbeltDrv", 3, 1, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x542: Message(0x542, "HVTMP", "HVAC", 8, [  # G37 DBC
        Signal("TempDrv", 1, 0xFF, 8, SIG_UINT8, 1.0, 0.0, ""),
        Signal("TempPass", 2, 0xFF, 8, SIG_UINT8, 1.0, 0.0, ""),
    ]),
    0x54B: Message(0x54B, "HVCTL", "HVAC", 8, [  # G37 DBC
        Signal("HVACOff", 0, 0, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("HVACMode", 2, 3, 3, SIG_UINT3, 1.0, 0.0, ""),
        Signal("RecircOn", 3, 0, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("RecircOff", 3, 1, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("FanLevel", 4, 3, 3, SIG_UINT3, 1.0, 0.0, ""),
    ]),
    0x54C: Message(0x54C, "HVAC2", "HVAC", 8, [
        Signal("EvapTemp", 0, 0xFF, 8, SIG_UINT8, 1.0, 0.0, ""),
        Signal("EvapTarget", 1, 0xFF, 8, SIG_UINT8, 1.0, 0.0, ""),
        Signal("ACStatus", 2, 7, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("Blower", 2, 6, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("Recirc", 7, 0, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x580: Message(0x580, "OIL2", "ECM", 8, [
        Signal("OilTemp", 4, 0xFF, 8, SIG_UINT8, 1.0, -40.0, "C"),
    ]),
    0x60D: Message(0x60D, "BCM3", "BCM", 8, [
        Signal("Trunk", 0, 7, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("PassDoor", 0, 4, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("DrvDoor", 0, 3, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("Running", 0, 2, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("Headlights", 0, 1, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("TurnR", 1, 6, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("TurnL", 1, 5, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("HighBeam", 1, 3, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("IgnON", 1, 2, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("IgnACC", 1, 1, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("DrvLock", 2, 4, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
    0x625: Message(0x625, "IPDM2", "IPDM", 8, [
        Signal("Defogger", 0, 0, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("ACCompFB", 1, 7, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("RunningFB", 1, 6, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("LowBeamFB", 1, 5, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("HighBeamFB", 1, 4, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("WiperFast", 1, 3, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("WiperSlow", 1, 2, 1, SIG_BOOL, 1.0, 0.0, ""),
        Signal("WiperHome", 1, 1, 1, SIG_BOOL, 1.0, 0.0, ""),
    ]),
}


def parse_data_bytes(data_str: str) -> bytes:
    """Parse hex data string like '0x1234ABCD' to bytes."""
    if data_str.startswith('0x'):
        data_str = data_str[2:]
    return bytes.fromhex(data_str)


def extract_signal(data: bytes, sig: Signal) -> float:
    """Extract a signal value from CAN data bytes."""
    if len(data) <= sig.start_byte:
        return 0.0

    raw = 0.0

    if sig.start_bit == 0xFF:
        # Whole byte or multi-byte extraction based on type
        if sig.sig_type == SIG_UINT8 or sig.sig_type == SIG_BOOL:
            raw = data[sig.start_byte]

        elif sig.sig_type == SIG_UINT7:
            # 7-bit value (mask off high bit)
            raw = data[sig.start_byte] & 0x7F

        elif sig.sig_type == SIG_UINT3:
            # 3-bit value (low 3 bits)
            raw = data[sig.start_byte] & 0x07

        elif sig.sig_type == SIG_UINT4:
            # 4-bit value (low nibble)
            raw = data[sig.start_byte] & 0x0F

        elif sig.sig_type in (SIG_UINT16, SIG_INT16):
            # Big-endian 16-bit
            if sig.start_byte + 1 < len(data):
                raw = (data[sig.start_byte] << 8) | data[sig.start_byte + 1]
                if sig.sig_type == SIG_INT16 and raw > 32767:
                    raw -= 65536

        elif sig.sig_type in (SIG_UINT16_LE, SIG_INT16_LE):
            # Little-endian 16-bit
            if sig.start_byte + 1 < len(data):
                raw = data[sig.start_byte] | (data[sig.start_byte + 1] << 8)
                if sig.sig_type == SIG_INT16_LE and raw > 32767:
                    raw -= 65536

        elif sig.sig_type == SIG_UINT24:
            # 24-bit big-endian
            if sig.start_byte + 2 < len(data):
                raw = (data[sig.start_byte] << 16) | (data[sig.start_byte + 1] << 8) | data[sig.start_byte + 2]

        elif sig.sig_type in (SIG_UINT12_BE, SIG_INT12_BE):
            # 12-bit big-endian: (byte[n] << 4) | (byte[n+1] >> 4)
            if sig.start_byte + 1 < len(data):
                raw = (data[sig.start_byte] << 4) | (data[sig.start_byte + 1] >> 4)
                if sig.sig_type == SIG_INT12_BE and raw > 2047:
                    raw -= 4096  # Sign extend from 12-bit

        elif sig.sig_type == SIG_UINT12_BE_LOW:
            # 12-bit: low nibble of byte[n] << 8 | byte[n+1]
            if sig.start_byte + 1 < len(data):
                raw = ((data[sig.start_byte] & 0x0F) << 8) | data[sig.start_byte + 1]

        elif sig.sig_type == SIG_UINT11_BE:
            # 11-bit big-endian: (byte[n] << 3) | (byte[n+1] >> 5)
            if sig.start_byte + 1 < len(data):
                raw = (data[sig.start_byte] << 3) | (data[sig.start_byte + 1] >> 5)

        elif sig.sig_type == SIG_UINT10_BE:
            # 10-bit big-endian: (byte[n] << 2) | (byte[n+1] >> 6)
            if sig.start_byte + 1 < len(data):
                raw = (data[sig.start_byte] << 2) | (data[sig.start_byte + 1] >> 6)

        else:
            # Legacy bit_length based extraction for compatibility
            if sig.bit_length == 8:
                raw = data[sig.start_byte]
            elif sig.bit_length == 16 and sig.start_byte + 1 < len(data):
                raw = (data[sig.start_byte] << 8) | data[sig.start_byte + 1]
            elif sig.bit_length == 10 and sig.start_byte + 1 < len(data):
                raw = (data[sig.start_byte] << 2) | (data[sig.start_byte + 1] >> 6)
    else:
        # Bit field extraction (within a single byte or spanning bytes)
        if sig.bit_length == 1:
            raw = (data[sig.start_byte] >> sig.start_bit) & 0x01
        elif sig.bit_length <= 8 - sig.start_bit:
            # Fits within single byte
            mask = (1 << sig.bit_length) - 1
            raw = (data[sig.start_byte] >> sig.start_bit) & mask
        elif sig.start_byte + 1 < len(data):
            # Spans two bytes - extract from start_bit in first byte
            bits_in_first = 8 - sig.start_bit
            bits_in_second = sig.bit_length - bits_in_first
            mask_first = (1 << bits_in_first) - 1
            mask_second = (1 << bits_in_second) - 1
            raw = ((data[sig.start_byte] >> sig.start_bit) & mask_first) | \
                  ((data[sig.start_byte + 1] & mask_second) << bits_in_first)

    return raw * sig.scale + sig.offset


def signal_changed(old_val: float, new_val: float, sig: Signal) -> bool:
    """Check if a signal value has changed."""
    if sig.sig_type == SIG_BOOL:
        return (old_val != 0) != (new_val != 0)
    else:
        return abs(old_val - new_val) > 0.01


def format_value(val: float, sig: Signal) -> str:
    """Format a signal value for display."""
    if sig.sig_type == SIG_BOOL:
        return "ON" if val != 0 else "OFF"
    elif sig.name == "ATGear":
        # 7AT current gear (from 0x174 low nibble)
        # Values 1-7 = gears 1-7, 9 = Reverse, 10 = Park
        gear_map = {1: "1", 2: "2", 3: "3", 4: "4", 5: "5", 6: "6", 7: "7", 9: "R", 10: "P"}
        return gear_map.get(int(val), f"?{int(val)}")
    elif sig.name == "Shifter":
        # Shifter position (0x421) - works for both 6MT and 7AT
        # 7AT: 8=P, 16=R, 24=N, 32=D
        # 6MT: 16=R, 24=N, 128=1, 136=2, 144=3, 152=4, 160=5, 168=6
        shifter_map = {
            8: "P", 16: "R", 24: "N", 32: "D",  # 7AT
            128: "1", 136: "2", 144: "3", 152: "4", 160: "5", 168: "6"  # 6MT
        }
        return shifter_map.get(int(val), f"?{int(val)}")
    elif sig.name == "HVACMode":
        # HVAC mode: 0=off, 1=face, 2=face+feet, 3=feet, 4=feet+defrost, 5=defrost
        mode_map = {0: "OFF", 1: "FACE", 2: "F+FT", 3: "FEET", 4: "FT+DF", 5: "DEF"}
        return mode_map.get(int(val), f"?{int(val)}")
    elif sig.name == "GearPos":
        # G37 gearbox position (3-bit)
        gear_map = {0: "P", 1: "R", 2: "N", 3: "D", 4: "S"}
        return gear_map.get(int(val), f"?{int(val)}")
    elif sig.scale == 1.0 and sig.offset == 0.0:
        return f"{int(val)}{sig.unit}"
    else:
        return f"{val:.1f}{sig.unit}"


def decode_frame(can_id: int, data: bytes) -> Optional[Dict[str, float]]:
    """Decode a CAN frame into signal values."""
    msg = MESSAGES.get(can_id)
    if not msg:
        return None

    result = {}
    for sig in msg.signals:
        result[sig.name] = extract_signal(data, sig)
    return result


def decode_frame_str(can_id: int, data: bytes, changed_signals: Optional[Set[str]] = None,
                     prev_data: Optional[bytes] = None, show_raw_changes: bool = False) -> str:
    """Decode a CAN frame to a formatted string.

    If changed_signals is provided, prefix changed signals with '*'.
    If show_raw_changes is True and prev_data is provided, append hex delta for undecoded byte changes.
    """
    msg = MESSAGES.get(can_id)
    if not msg:
        return f"0x{can_id:03X}: [unknown] {data.hex().upper()}"

    parts = [f"{msg.name}:"]
    for sig in msg.signals:
        val = extract_signal(data, sig)
        formatted = format_value(val, sig)

        # Mark changed signals with asterisk
        if changed_signals is not None and sig.name in changed_signals:
            parts.append(f"*{sig.name}={formatted}")
        else:
            parts.append(f"{sig.name}={formatted}")

    # If raw data changed but no decoded signals changed, show the hex delta
    if show_raw_changes and prev_data is not None and (changed_signals is None or len(changed_signals) == 0):
        parts.append(f"| raw: {format_hex_delta(prev_data, data)}")

    return " ".join(parts)


def format_hex_delta(old_data: bytes, new_data: bytes) -> str:
    """Format hex data with changed bytes marked with *."""
    parts = []
    max_len = max(len(old_data), len(new_data))
    for i in range(max_len):
        old_byte = old_data[i] if i < len(old_data) else None
        new_byte = new_data[i] if i < len(new_data) else 0

        if old_byte is None or old_byte != new_byte:
            parts.append(f"*{new_byte:02X}")
        else:
            parts.append(f"{new_byte:02X}")

    return " ".join(parts)


def find_signal_sources(signal_name: str) -> List[Tuple[int, Message, Signal]]:
    """Find all messages containing a signal with the given name.

    Returns list of (can_id, message, signal) tuples.
    """
    results = []
    for can_id, msg in MESSAGES.items():
        for sig in msg.signals:
            if sig.name == signal_name:
                results.append((can_id, msg, sig))
    return results


def parse_signal_spec(spec: str) -> List[Tuple[str, int, Signal]]:
    """Parse a signal specification like 'RPM', 'ECM1.RPM', or '0x180.RPM'.

    Returns list of (display_name, can_id, signal) tuples.
    """
    results = []

    if '.' in spec:
        # Qualified name: MessageName.SignalName or 0xID.SignalName
        qualifier, sig_name = spec.split('.', 1)

        # Try as hex CAN ID first
        if qualifier.startswith('0x') or qualifier.startswith('0X'):
            try:
                can_id = int(qualifier, 16)
                if can_id in MESSAGES:
                    msg = MESSAGES[can_id]
                    for sig in msg.signals:
                        if sig.name == sig_name:
                            results.append((spec, can_id, sig))
                            break
                return results
            except ValueError:
                pass

        # Try as message name
        for can_id, msg in MESSAGES.items():
            if msg.name.upper() == qualifier.upper():
                for sig in msg.signals:
                    if sig.name == sig_name:
                        results.append((spec, can_id, sig))
                        break
    else:
        # Unqualified name: find all matching signals
        sources = find_signal_sources(spec)
        if len(sources) == 1:
            can_id, msg, sig = sources[0]
            results.append((spec, can_id, sig))
        else:
            # Multiple sources - use qualified names
            for can_id, msg, sig in sources:
                display_name = f"{msg.name}.{sig.name}"
                results.append((display_name, can_id, sig))

    return results


def collect_plot_data(input_file: str, signal_specs: List[str]) -> Dict[str, Tuple[List[float], List[float], str]]:
    """Collect time-series data for specified signals.

    Returns dict: display_name -> (timestamps, values, unit)
    """
    # Parse signal specs and build lookup table
    signal_lookup: Dict[Tuple[int, str], Tuple[str, Signal]] = {}  # (can_id, sig_name) -> (display_name, signal)

    for spec in signal_specs:
        parsed = parse_signal_spec(spec)
        if not parsed:
            print(f"Warning: Signal '{spec}' not found", file=sys.stderr)
            continue
        for display_name, can_id, sig in parsed:
            signal_lookup[(can_id, sig.name)] = (display_name, sig)

    if not signal_lookup:
        return {}

    # Collect data
    plot_data: Dict[str, Tuple[List[float], List[float], str]] = {}
    for display_name, sig in signal_lookup.values():
        plot_data[display_name] = ([], [], sig.unit)

    with open(input_file, 'r') as f:
        reader = csv.DictReader(f)

        for row in reader:
            try:
                timestamp = int(row['timestamp_ms'])
                can_id = int(row['id'], 16)
                data_str = row.get('data', '')
            except (KeyError, ValueError):
                continue

            if data_str:
                data = parse_data_bytes(data_str)
            else:
                continue

            # Check if this message has any signals we care about
            msg = MESSAGES.get(can_id)
            if not msg:
                continue

            for sig in msg.signals:
                key = (can_id, sig.name)
                if key in signal_lookup:
                    display_name, _ = signal_lookup[key]
                    value = extract_signal(data, sig)

                    # Convert timestamp to seconds
                    time_sec = timestamp / 1000.0

                    plot_data[display_name][0].append(time_sec)
                    plot_data[display_name][1].append(value)

    return plot_data


def plot_signals(plot_data: Dict[str, Tuple[List[float], List[float], str]],
                 subplots: bool = False, output_file: Optional[str] = None):
    """Plot signal data using matplotlib."""
    if not HAS_MATPLOTLIB:
        print("Error: matplotlib is required for plotting. Install with: pip install matplotlib",
              file=sys.stderr)
        return

    if not plot_data:
        print("No data to plot", file=sys.stderr)
        return

    # Normalize timestamps to start at 0
    all_times = []
    for times, values, unit in plot_data.values():
        all_times.extend(times)

    if not all_times:
        print("No data points collected", file=sys.stderr)
        return

    t_min = min(all_times)

    signal_names = list(plot_data.keys())
    n_signals = len(signal_names)

    if subplots:
        # Separate subplot for each signal
        fig, axes = plt.subplots(n_signals, 1, figsize=(12, 3 * n_signals), sharex=True)
        if n_signals == 1:
            axes = [axes]

        for ax, name in zip(axes, signal_names):
            times, values, unit = plot_data[name]
            # Normalize time
            times_normalized = [t - t_min for t in times]

            ax.plot(times_normalized, values, linewidth=0.5)
            ax.set_ylabel(f"{name}\n({unit})" if unit else name)
            ax.grid(True, alpha=0.3)
            ax.set_xlim(left=0)

        axes[-1].set_xlabel("Time (seconds)")
        plt.tight_layout()
    else:
        # All signals on one plot (with secondary y-axis if units differ)
        fig, ax1 = plt.subplots(figsize=(12, 6))

        # Group signals by unit
        unit_groups: Dict[str, List[str]] = {}
        for name in signal_names:
            _, _, unit = plot_data[name]
            unit_key = unit if unit else "value"
            if unit_key not in unit_groups:
                unit_groups[unit_key] = []
            unit_groups[unit_key].append(name)

        colors = plt.cm.tab10.colors
        color_idx = 0

        # Plot first unit group on primary axis
        units = list(unit_groups.keys())
        for name in unit_groups[units[0]]:
            times, values, unit = plot_data[name]
            times_normalized = [t - t_min for t in times]
            ax1.plot(times_normalized, values, linewidth=0.5,
                    color=colors[color_idx % len(colors)], label=name)
            color_idx += 1

        ax1.set_xlabel("Time (seconds)")
        ax1.set_ylabel(units[0])
        ax1.grid(True, alpha=0.3)
        ax1.set_xlim(left=0)

        # If there are multiple unit groups, use secondary y-axis
        if len(units) > 1:
            ax2 = ax1.twinx()
            for unit_key in units[1:]:
                for name in unit_groups[unit_key]:
                    times, values, unit = plot_data[name]
                    times_normalized = [t - t_min for t in times]
                    ax2.plot(times_normalized, values, linewidth=0.5,
                            color=colors[color_idx % len(colors)], label=name, linestyle='--')
                    color_idx += 1
            ax2.set_ylabel(" / ".join(units[1:]))

        # Combine legends
        lines1, labels1 = ax1.get_legend_handles_labels()
        if len(units) > 1:
            lines2, labels2 = ax2.get_legend_handles_labels()
            ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
        else:
            ax1.legend(loc='upper right')

        plt.tight_layout()

    plt.suptitle("CAN Signal Plot", y=1.02)

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved plot to {output_file}")
    else:
        plt.show()


def process_log(input_file: str, output_file: Optional[str] = None,
                filter_id: Optional[int] = None, delta_mode: bool = False,
                raw_mode: bool = False):
    """Process a CAN log file."""

    # Track both decoded values and raw data for delta comparison
    last_decoded: Dict[int, Dict[str, float]] = {}
    last_raw: Dict[int, bytes] = {}

    output_rows = []

    with open(input_file, 'r') as f:
        reader = csv.DictReader(f)

        for row in reader:
            # Parse row
            try:
                seq = int(row['seq'])
                timestamp = int(row['timestamp_ms'])
                can_id = int(row['id'], 16)
                dlc = int(row['dlc'])
                data_str = row.get('data', '')
            except (KeyError, ValueError) as e:
                continue

            # Filter
            if filter_id is not None and can_id != filter_id:
                continue

            # Parse data
            if data_str:
                data = parse_data_bytes(data_str)
            else:
                data = bytes()

            # Decode
            decoded = decode_frame(can_id, data)
            changed_signals: Set[str] = set()

            # Get previous raw data for comparison (before updating)
            prev_raw = last_raw.get(can_id)

            # Delta mode - check for changes
            if delta_mode:
                raw_changed = False
                decoded_changed = False

                # Check raw data change
                if prev_raw is not None:
                    if data != prev_raw:
                        raw_changed = True
                else:
                    raw_changed = True  # First time seeing this ID

                # Check decoded signal changes
                if decoded and can_id in last_decoded:
                    msg = MESSAGES.get(can_id)
                    if msg:
                        for sig in msg.signals:
                            old_val = last_decoded[can_id].get(sig.name)
                            new_val = decoded.get(sig.name)
                            if old_val is not None and new_val is not None:
                                if signal_changed(old_val, new_val, sig):
                                    decoded_changed = True
                                    changed_signals.add(sig.name)
                            elif new_val is not None:
                                decoded_changed = True
                                changed_signals.add(sig.name)
                elif decoded:
                    # First time seeing this ID with decoded data
                    decoded_changed = True
                    msg = MESSAGES.get(can_id)
                    if msg:
                        changed_signals = {sig.name for sig in msg.signals}

                # Skip if nothing changed
                if not raw_changed:
                    continue

                # If raw changed but no decoded signals changed, we still want to show it
                # (for unknown messages or messages where only un-decoded bytes changed)

            # Store for next delta comparison
            last_raw[can_id] = data
            if decoded:
                last_decoded[can_id] = decoded

            # Format output
            if raw_mode:
                decoded_str = f"0x{can_id:03X} [{dlc}] {data.hex().upper()}"
            elif decoded is None:
                # Unknown message - show hex, with delta markers if applicable
                if delta_mode and prev_raw is not None:
                    decoded_str = f"0x{can_id:03X}: [unknown] {format_hex_delta(prev_raw, data)}"
                else:
                    decoded_str = f"0x{can_id:03X}: [unknown] {data.hex().upper()}"
            else:
                decoded_str = decode_frame_str(
                    can_id, data,
                    changed_signals if delta_mode else None,
                    prev_raw if delta_mode else None,
                    show_raw_changes=delta_mode
                )

            # Output
            if output_file:
                output_rows.append({
                    'seq': seq,
                    'timestamp_ms': timestamp,
                    'id': f"0x{can_id:03X}",
                    'decoded': decoded_str,
                    **{k: v for k, v in (decoded or {}).items()}
                })
            else:
                print(f"[{timestamp:010d}] {decoded_str}")

    # Write CSV output
    if output_file and output_rows:
        # Collect all signal names for header
        all_signals = set()
        for row in output_rows:
            all_signals.update(k for k in row.keys()
                             if k not in ('seq', 'timestamp_ms', 'id', 'decoded'))

        fieldnames = ['seq', 'timestamp_ms', 'id', 'decoded'] + sorted(all_signals)

        with open(output_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(output_rows)

        print(f"Wrote {len(output_rows)} rows to {output_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Decode 370Z CAN log files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s canlog.csv                         Print decoded output
  %(prog)s canlog.csv --delta                 Only show changes
  %(prog)s canlog.csv --plot RPM,Speed        Plot RPM and Speed over time
  %(prog)s canlog.csv --plot ECM1.RPM         Plot specific signal source
  %(prog)s canlog.csv --plot RPM --subplots   Separate subplot per signal
  %(prog)s canlog.csv --plot RPM -o plot.png  Save plot to file
        """
    )
    parser.add_argument('input', nargs='?', help='Input CSV log file')
    parser.add_argument('-o', '--output', help='Output file (CSV for decode, PNG/PDF for plot)')
    parser.add_argument('--filter', type=lambda x: int(x, 0),
                       help='Filter to specific CAN ID (e.g., 0x180)')
    parser.add_argument('--delta', action='store_true',
                       help='Only show frames where values changed')
    parser.add_argument('--raw', action='store_true',
                       help='Show raw hex instead of decoded values')
    parser.add_argument('--list-ids', action='store_true',
                       help='List all known CAN IDs and exit')
    parser.add_argument('--list-signals', action='store_true',
                       help='List all known signals and exit')
    parser.add_argument('--plot', type=str, metavar='SIGNALS',
                       help='Plot signals over time (comma-separated, e.g., RPM,Speed,Throttle)')
    parser.add_argument('--subplots', action='store_true',
                       help='Use separate subplot for each signal (with --plot)')

    args = parser.parse_args()

    if args.list_ids:
        print("Known CAN IDs:")
        print("-" * 60)
        for msg_id, msg in sorted(MESSAGES.items()):
            signals = ", ".join(s.name for s in msg.signals)
            print(f"  0x{msg_id:03X} {msg.name:8s} ({msg.source}): {signals}")
        return

    if args.list_signals:
        print("Available signals:")
        print("-" * 70)
        # Build signal list with sources
        signal_sources: Dict[str, List[Tuple[int, str]]] = {}
        for can_id, msg in MESSAGES.items():
            for sig in msg.signals:
                if sig.name not in signal_sources:
                    signal_sources[sig.name] = []
                signal_sources[sig.name].append((can_id, msg.name))

        for sig_name in sorted(signal_sources.keys()):
            sources = signal_sources[sig_name]
            if len(sources) == 1:
                can_id, msg_name = sources[0]
                # Find signal to get unit
                sig = next(s for s in MESSAGES[can_id].signals if s.name == sig_name)
                unit = f"[{sig.unit}]" if sig.unit else ""
                print(f"  {sig_name:20s} {unit:8s}  (0x{can_id:03X} {msg_name})")
            else:
                # Multiple sources - list them all
                print(f"  {sig_name}:")
                for can_id, msg_name in sources:
                    sig = next(s for s in MESSAGES[can_id].signals if s.name == sig_name)
                    unit = f"[{sig.unit}]" if sig.unit else ""
                    print(f"    {msg_name}.{sig_name:16s} {unit:8s}  (0x{can_id:03X})")
        return

    # Commands that require input file
    if not args.input:
        parser.error("Input file is required for this command")

    if args.plot:
        if not HAS_MATPLOTLIB:
            print("Error: matplotlib is required for plotting.", file=sys.stderr)
            print("Install with: pip install matplotlib", file=sys.stderr)
            sys.exit(1)

        signal_specs = [s.strip() for s in args.plot.split(',')]
        plot_data = collect_plot_data(args.input, signal_specs)
        plot_signals(plot_data, subplots=args.subplots, output_file=args.output)
        return

    process_log(args.input, args.output, args.filter, args.delta, args.raw)


if __name__ == '__main__':
    main()
