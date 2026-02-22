#!/usr/bin/env python3
"""Generate a standard Vector DBC file from the 370Z CAN signal database."""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))
from decode_log import MESSAGES, Signal, SIG_BOOL, SIG_UINT8, SIG_UINT16, SIG_INT16
from decode_log import SIG_UINT16_LE, SIG_INT16_LE, SIG_UINT24, SIG_UINT12_BE, SIG_INT12_BE
from decode_log import SIG_UINT7, SIG_UINT3, SIG_UINT4, SIG_UINT11_BE, SIG_UINT10_BE
from decode_log import SIG_UINT12_BE_LOW


def sig_to_dbc(sig: Signal) -> tuple:
    """Convert our signal definition to DBC (start_bit, length, byte_order, is_signed).

    Returns (start_bit, bit_length, byte_order_char, sign_char).
    byte_order_char: '0' = Motorola/BE, '1' = Intel/LE
    sign_char: '+' = unsigned, '-' = signed
    """
    is_signed = sig.sig_type in (SIG_INT16, SIG_INT16_LE, SIG_INT12_BE)

    if sig.start_bit != 0xFF:
        # Bit-field extraction: (data[byte] >> bit) & mask
        # DBC Intel: start_bit = byte*8 + lsb_bit
        dbc_start = sig.start_byte * 8 + sig.start_bit
        return (dbc_start, sig.bit_length, '1', '-' if is_signed else '+')

    # Whole-byte / multi-byte extraction based on type
    if sig.sig_type in (SIG_UINT16_LE, SIG_INT16_LE):
        # Intel (little-endian): start_bit = byte*8 + 0 (LSB)
        dbc_start = sig.start_byte * 8
        sign = '-' if sig.sig_type == SIG_INT16_LE else '+'
        return (dbc_start, 16, '1', sign)

    if sig.sig_type in (SIG_UINT8, SIG_BOOL):
        # Full byte, Motorola: MSB at byte*8 + 7
        dbc_start = sig.start_byte * 8 + 7
        return (dbc_start, 8, '0', '+')

    if sig.sig_type == SIG_UINT7:
        # 7-bit (mask off MSB): MSB at bit 6
        dbc_start = sig.start_byte * 8 + 6
        return (dbc_start, 7, '0', '+')

    if sig.sig_type == SIG_UINT3:
        # Low 3 bits: MSB at bit 2
        dbc_start = sig.start_byte * 8 + 2
        return (dbc_start, 3, '0', '+')

    if sig.sig_type == SIG_UINT4:
        # Low nibble: MSB at bit 3
        dbc_start = sig.start_byte * 8 + 3
        return (dbc_start, 4, '0', '+')

    if sig.sig_type in (SIG_UINT16, SIG_INT16):
        # Big-endian: MSB at byte*8 + 7
        # Use bit_length if < 16 (e.g., 10-bit pedal signals) to avoid overlap
        length = sig.bit_length if sig.bit_length < 16 else 16
        dbc_start = sig.start_byte * 8 + 7
        sign = '-' if sig.sig_type == SIG_INT16 else '+'
        return (dbc_start, length, '0', sign)

    if sig.sig_type == SIG_UINT24:
        # Big-endian 24-bit: MSB at byte*8 + 7
        dbc_start = sig.start_byte * 8 + 7
        return (dbc_start, 24, '0', '+')

    if sig.sig_type in (SIG_UINT12_BE, SIG_INT12_BE):
        # 12-bit std BE: (byte[n]<<4)|(byte[n+1]>>4), MSB at byte*8+7
        dbc_start = sig.start_byte * 8 + 7
        sign = '-' if sig.sig_type == SIG_INT12_BE else '+'
        return (dbc_start, 12, '0', sign)

    if sig.sig_type == SIG_UINT12_BE_LOW:
        # 12-bit low-nibble+full: (byte[n]&0xF)<<8|byte[n+1], MSB at byte*8+3
        dbc_start = sig.start_byte * 8 + 3
        return (dbc_start, 12, '0', '+')

    if sig.sig_type == SIG_UINT10_BE:
        # 10-bit BE: (byte[n]<<2)|(byte[n+1]>>6), MSB at byte*8+7
        dbc_start = sig.start_byte * 8 + 7
        return (dbc_start, 10, '0', '+')

    if sig.sig_type == SIG_UINT11_BE:
        # 11-bit BE: (byte[n]<<3)|(byte[n+1]>>5), MSB at byte*8+7
        dbc_start = sig.start_byte * 8 + 7
        return (dbc_start, 11, '0', '+')

    # Fallback: treat as full-byte Motorola
    dbc_start = sig.start_byte * 8 + 7
    return (dbc_start, sig.bit_length, '0', '+')


def compute_range(sig: Signal) -> tuple:
    """Compute physical min/max from signal parameters."""
    dbc_start, bit_length, bo, sign = sig_to_dbc(sig)

    if sign == '-':
        raw_min = -(1 << (bit_length - 1))
        raw_max = (1 << (bit_length - 1)) - 1
    else:
        raw_min = 0
        raw_max = (1 << bit_length) - 1

    phys_min = raw_min * sig.scale + sig.offset
    phys_max = raw_max * sig.scale + sig.offset

    if phys_min > phys_max:
        phys_min, phys_max = phys_max, phys_min

    return (phys_min, phys_max)


def generate_dbc():
    lines = []

    lines.append('VERSION ""')
    lines.append('')
    lines.append('NS_ :')
    lines.append('')
    lines.append('BS_:')
    lines.append('')

    # Node definitions
    nodes = set()
    for msg in MESSAGES.values():
        src = msg.source.replace('&', '_').replace(' ', '_')
        nodes.add(src)
    lines.append('BU_: ' + ' '.join(sorted(nodes)))
    lines.append('')
    lines.append('')

    # Messages and signals, sorted by CAN ID
    for can_id in sorted(MESSAGES.keys()):
        msg = MESSAGES[can_id]
        src = msg.source.replace('&', '_')
        # DBC identifiers: alphanumeric + underscore only
        dbc_name = msg.name.replace('&', '_')

        lines.append(f'BO_ {can_id} {dbc_name}: {msg.dlc} {src}')

        # Track signal names to avoid duplicates within a message
        seen_names = set()
        for sig in msg.signals:
            # Skip duplicates (e.g., Unk signals that overlap with named signals)
            if sig.name in seen_names:
                continue
            seen_names.add(sig.name)

            dbc_start, bit_length, bo, sign = sig_to_dbc(sig)
            phys_min, phys_max = compute_range(sig)

            factor = sig.scale
            offset = sig.offset
            if factor == int(factor):
                factor_s = str(int(factor))
            else:
                factor_s = f'{factor:g}'
            if offset == int(offset):
                offset_s = str(int(offset))
            else:
                offset_s = f'{offset:g}'

            # Format range
            if phys_min == int(phys_min) and phys_max == int(phys_max):
                range_s = f'[{int(phys_min)}|{int(phys_max)}]'
            else:
                range_s = f'[{phys_min:g}|{phys_max:g}]'

            unit = sig.unit
            lines.append(
                f' SG_ {sig.name} : {dbc_start}|{bit_length}@{bo}{sign}'
                f' ({factor_s},{offset_s}) {range_s} "{unit}" Vector__XXX'
            )

        lines.append('')

    # Comments
    lines.append('')
    comments = {
        0x002: 'Steering angle sensor - angle and angular velocity',
        0x15C: 'Gas pedal position (G37 DBC, verify on 370Z)',
        0x160: 'Throttle/pedal - accelerator and WOT detection',
        0x174: 'TCM gear position (7AT automatic transmission only)',
        0x180: 'ECM primary - RPM, pedal position, clutch',
        0x182: 'ECM secondary - throttle position, brake light',
        0x1F9: 'ECM AC request and ignition status',
        0x20B: 'Cruise control stalk buttons (G37 DBC)',
        0x215: 'HVAC - AC request',
        0x216: 'IPDM - ignition, hood, clutch',
        0x280: 'Seatbelt and vehicle speed',
        0x284: 'ABS front wheel speeds and vehicle speed',
        0x285: 'ABS rear wheel speeds',
        0x292: 'IMU - yaw rate, lateral acceleration, brake pressure',
        0x300: 'Steering torque sensor (G37 DBC)',
        0x351: 'Key/clutch/button status',
        0x354: 'ABS/VDC - vehicle speed, VDC status, brake',
        0x355: 'Vehicle speed (dual) and unit setting',
        0x358: 'BCM - key slot, headlights, stopped, trunk',
        0x35D: 'BCM - wipers, forward gear, brake pedal',
        0x385: 'TPMS - tire pressures and validity',
        0x41F: 'Gearbox position and sports mode (G37 DBC)',
        0x421: 'Shifter position and S-Mode (6MT and 7AT)',
        0x453: 'Lights and blinkers (G37 DBC)',
        0x4F9: 'HUD speed display and seatbelt (G37 DBC)',
        0x542: 'HVAC temperature settings (G37 DBC)',
        0x54B: 'HVAC controls - mode, fan, recirculate (G37 DBC)',
        0x54C: 'HVAC evaporator and AC status',
        0x551: 'Coolant temp, fuel consumption, cruise control',
        0x580: 'Oil temperature (alternate source)',
        0x5C5: 'Odometer, hazards, parking brake, S-Mode button',
        0x60D: 'BCM doors, lights, locks, ignition',
        0x625: 'IPDM feedback - lights, wipers, AC compressor',
    }

    for can_id, comment in sorted(comments.items()):
        if can_id in MESSAGES:
            lines.append(f'CM_ BO_ {can_id} "{comment}";')

    # Signal comments for non-obvious signals
    lines.append('')
    sig_comments = [
        (0x174, 'ATGear', '7AT: 1-7=gears, 9=Reverse, 10=Park'),
        (0x280, 'Seatbelt', 'Inverted: 0=buckled, 1=unbuckled'),
        (0x292, 'LatAccel', 'Unsigned 12-bit with -2048 offset; near zero when driving straight'),
        (0x292, 'Yaw', 'Unsigned 12-bit with -2048 offset; negative=turning right'),
        (0x292, 'BrakePres', '0 when released, ~25 when stopped, 6-8 during light braking'),
        (0x354, 'VDCOff', 'VDC system disabled by driver'),
        (0x421, 'Shifter', '6MT: 16=R,24=N,128=1,136=2,144=3,152=4,160=5,168=6; 7AT: 8=P,16=R,24=N,32=D'),
        (0x54B, 'HVACMode', '0=off, 1=face, 2=face+feet, 3=feet, 4=feet+defrost, 5=defrost'),
        (0x551, 'CruiseSpd', '255=off, 254=on but no speed set, else speed in km/h'),
        (0x551, 'Coolant', 'Raw value minus 40 = temperature in Celsius'),
    ]
    for can_id, sig_name, comment in sig_comments:
        if can_id in MESSAGES:
            lines.append(f'CM_ SG_ {can_id} {sig_name} "{comment}";')

    # Value descriptions for enums
    lines.append('')
    lines.append('VAL_ {} ATGear 1 "1st" 2 "2nd" 3 "3rd" 4 "4th" 5 "5th" 6 "6th" 7 "7th" 9 "Reverse" 10 "Park" ;'.format(0x174))
    lines.append('VAL_ {} Shifter 8 "Park" 16 "Reverse" 24 "Neutral" 32 "Drive" 128 "1st" 136 "2nd" 144 "3rd" 152 "4th" 160 "5th" 168 "6th" ;'.format(0x421))
    lines.append('VAL_ {} HVACMode 0 "Off" 1 "Face" 2 "Face+Feet" 3 "Feet" 4 "Feet+Defrost" 5 "Defrost" ;'.format(0x54B))
    lines.append('VAL_ {} GearPos 0 "Park" 1 "Reverse" 2 "Neutral" 3 "Drive" 4 "Sport" ;'.format(0x41F))
    lines.append('')

    return '\n'.join(lines)


if __name__ == '__main__':
    dbc_content = generate_dbc()
    out_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Nissan_370Z_Z34.dbc')
    with open(out_path, 'w', newline='\n') as f:
        f.write(dbc_content)
    print(f'Wrote {out_path}')
    print(f'{len(MESSAGES)} messages, {sum(len(m.signals) for m in MESSAGES.values())} signals')
