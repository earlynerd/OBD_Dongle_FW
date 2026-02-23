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
import pathlib
import re
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Set

# Try to import matplotlib for plotting
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

@dataclass
class Signal:
    name: str
    start_bit: int
    bit_length: int
    byte_order: int  # 0=Motorola(big-endian), 1=Intel(little-endian)
    is_signed: bool
    scale: float
    offset: float
    unit: str
    value_map: Dict[int, str] = field(default_factory=dict)

@dataclass
class Message:
    id: int
    name: str
    source: str
    dlc: int
    signals: List[Signal]

_RE_BO = re.compile(r"^BO_\s+(\d+)\s+(\w+)\s*:\s*(\d+)\s+(\w+)")
_RE_VAL = re.compile(r"^VAL_\s+(\d+)\s+(\w+)\s+(.+)\s*;")
_RE_VAL_PAIR = re.compile(r"(-?\d+)\s+\"([^\"]*)\"")


def _parse_sg_line(line: str) -> Optional[Tuple[str, int, int, int, bool, float, float, str]]:
    # Example:
    # SG_ Speed : 7|16@0+ (0.00621,0) [0|406.972] "mph" Vector__XXX
    s = line.strip()
    if not s.startswith("SG_"):
        return None
    if ":" not in s:
        return None
    left, right = s.split(":", 1)
    name = left.replace("SG_", "", 1).strip().split()[0]
    right = right.strip()

    # bit token
    toks = right.split(None, 1)
    if not toks:
        return None
    bit_tok = toks[0]
    if "|" not in bit_tok or "@" not in bit_tok:
        return None
    try:
        start_s, rest = bit_tok.split("|", 1)
        length_s, ord_sign = rest.split("@", 1)
        start_bit = int(start_s)
        bit_length = int(length_s)
        byte_order = int(ord_sign[0])
        is_signed = ord_sign[1] == "-"
    except Exception:
        return None

    # scale/offset
    lp = right.find("(")
    rp = right.find(")", lp + 1)
    if lp < 0 or rp < 0:
        return None
    try:
        scale_s, offset_s = right[lp + 1 : rp].split(",", 1)
        scale = float(scale_s.strip())
        offset = float(offset_s.strip())
    except Exception:
        return None

    # unit (optional)
    unit = ""
    q1 = right.find('"')
    if q1 >= 0:
        q2 = right.find('"', q1 + 1)
        if q2 > q1:
            unit = right[q1 + 1 : q2]

    return name, start_bit, bit_length, byte_order, is_signed, scale, offset, unit


def load_dbc_messages(dbc_path: str) -> Dict[int, Message]:
    messages: Dict[int, Message] = {}
    current_id: Optional[int] = None
    value_maps: Dict[Tuple[int, str], Dict[int, str]] = {}

    with open(dbc_path, "r", encoding="utf-8", errors="ignore") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line:
                continue

            m_bo = _RE_BO.match(line)
            if m_bo:
                msg_id = int(m_bo.group(1))
                name = m_bo.group(2)
                dlc = int(m_bo.group(3))
                src = m_bo.group(4)
                messages[msg_id] = Message(msg_id, name, src, dlc, [])
                current_id = msg_id
                continue

            sg = _parse_sg_line(line)
            if sg is not None and current_id is not None:
                sig = Signal(
                    name=sg[0],
                    start_bit=sg[1],
                    bit_length=sg[2],
                    byte_order=sg[3],
                    is_signed=sg[4],
                    scale=sg[5],
                    offset=sg[6],
                    unit=sg[7],
                )
                messages[current_id].signals.append(sig)
                continue

            m_val = _RE_VAL.match(line)
            if m_val:
                msg_id = int(m_val.group(1))
                sig_name = m_val.group(2)
                rest = m_val.group(3)
                vmap: Dict[int, str] = {}
                for vm in _RE_VAL_PAIR.finditer(rest):
                    vmap[int(vm.group(1))] = vm.group(2)
                value_maps[(msg_id, sig_name)] = vmap

    for msg_id, msg in messages.items():
        for sig in msg.signals:
            sig.value_map = value_maps.get((msg_id, sig.name), {})

    return messages


DEFAULT_DBC_PATH = (pathlib.Path(__file__).resolve().parent.parent / "Nissan_370Z_Z34.dbc")
MESSAGES: Dict[int, Message] = {}
if DEFAULT_DBC_PATH.exists():
    try:
        MESSAGES = load_dbc_messages(str(DEFAULT_DBC_PATH))
    except Exception:
        MESSAGES = {}


def parse_data_bytes(data_str: str) -> bytes:
    """Parse hex data string like '0x1234ABCD' to bytes."""
    if data_str.startswith('0x'):
        data_str = data_str[2:]
    return bytes.fromhex(data_str)


def _bit_at(data: bytes, bit_index: int) -> int:
    byte_idx = bit_index // 8
    if byte_idx < 0 or byte_idx >= len(data):
        return 0
    bit_in_byte = bit_index % 8
    return (data[byte_idx] >> bit_in_byte) & 0x01


def _extract_raw_intel(data: bytes, start_bit: int, bit_length: int) -> int:
    raw = 0
    for i in range(bit_length):
        raw |= (_bit_at(data, start_bit + i) << i)
    return raw


def _extract_raw_motorola(data: bytes, start_bit: int, bit_length: int) -> int:
    raw = 0
    pos = start_bit
    for _ in range(bit_length):
        raw = (raw << 1) | _bit_at(data, pos)
        if pos % 8 == 0:
            pos += 15
        else:
            pos -= 1
    return raw


def extract_signal(data: bytes, sig: Signal, can_id: Optional[int] = None) -> Optional[float]:
    """Extract a signal value from CAN data bytes.

    Returns None for known invalid/sentinel encodings.
    """
    if sig.bit_length <= 0:
        return 0.0

    if sig.byte_order == 1:
        raw = _extract_raw_intel(data, sig.start_bit, sig.bit_length)
    else:
        raw = _extract_raw_motorola(data, sig.start_bit, sig.bit_length)

    if sig.is_signed and sig.bit_length > 0:
        sign_bit = 1 << (sig.bit_length - 1)
        if raw & sign_bit:
            raw -= (1 << sig.bit_length)

    # Known invalid/sentinel values from field logs:
    # - 0x355 Speed2 sometimes transmits 0xFFFF (invalid)
    if can_id == 0x355 and sig.name == "Speed2" and len(data) >= 4:
        if data[2] == 0xFF and data[3] == 0xFF:
            return None

    # - 0x292 occasionally carries an all-ones invalid IMU frame:
    #   FF FF FF FF FF FE FF 00
    if can_id == 0x292 and len(data) >= 8:
        if data[0] == 0xFF and data[1] == 0xFF and data[2] == 0xFF and \
           data[3] == 0xFF and data[4] == 0xFF and data[5] == 0xFE and \
           data[6] == 0xFF and data[7] == 0x00:
            if sig.name in ("LatAccel", "Yaw", "BrakePres"):
                return None

    return raw * sig.scale + sig.offset


def signal_changed(old_val: Optional[float], new_val: Optional[float], sig: Signal) -> bool:
    """Check if a signal value has changed."""
    # Treat missing/invalid new values as "no decoded update"
    if new_val is None:
        return False
    if old_val is None:
        return True

    if sig.bit_length == 1 and sig.scale == 1.0 and sig.offset == 0.0:
        return (old_val != 0) != (new_val != 0)
    else:
        return abs(old_val - new_val) > 0.01


def format_value(val: float, sig: Signal) -> str:
    """Format a signal value for display."""
    if sig.value_map:
        return sig.value_map.get(int(round(val)), f"?{int(round(val))}")

    if sig.bit_length == 1 and sig.scale == 1.0 and sig.offset == 0.0:
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
        val = extract_signal(data, sig, can_id)
        if val is not None:
            result[sig.name] = val
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
        val = extract_signal(data, sig, can_id)
        if val is None:
            continue
        formatted = format_value(val, sig)

        # Mark changed signals with asterisk
        if changed_signals is not None and sig.name in changed_signals:
            parts.append(f"*{sig.name}={formatted}")
        else:
            parts.append(f"{sig.name}={formatted}")

    if len(parts) == 1:
        return f"{msg.name}: [invalid/sentinel]"

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
        prev_seq: Optional[int] = None
        session_id = 0

        for row in reader:
            try:
                seq = int(row['seq'])
                timestamp = int(row['timestamp_ms'])
                can_id = int(row['id'], 16)
                data_str = row.get('data', '')
            except (KeyError, ValueError):
                continue

            # Detect session boundaries from seq counter resets.
            if prev_seq is not None and seq <= prev_seq:
                session_id += 1
            prev_seq = seq

            if COLLECT_SESSION is not None and session_id != COLLECT_SESSION:
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
                    value = extract_signal(data, sig, can_id)
                    if value is None:
                        continue

                    # Convert timestamp to seconds
                    time_sec = timestamp / 1000.0

                    plot_data[display_name][0].append(time_sec)
                    plot_data[display_name][1].append(value)

    return plot_data


# Optional session filter used by collect_plot_data/plot path.
COLLECT_SESSION: Optional[int] = None


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
                raw_mode: bool = False, session_filter: Optional[int] = None):
    """Process a CAN log file."""

    # Track both decoded values and raw data for delta comparison
    last_decoded: Dict[int, Dict[str, float]] = {}
    last_raw: Dict[int, bytes] = {}

    output_rows = []

    with open(input_file, 'r') as f:
        reader = csv.DictReader(f)
        prev_seq: Optional[int] = None
        session_id = 0

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

            # Detect new capture session when seq counter restarts.
            if prev_seq is not None and seq <= prev_seq:
                session_id += 1
            prev_seq = seq

            if session_filter is not None and session_id != session_filter:
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
                    'session': session_id,
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
                             if k not in ('seq', 'session', 'timestamp_ms', 'id', 'decoded'))

        fieldnames = ['seq', 'session', 'timestamp_ms', 'id', 'decoded'] + sorted(all_signals)

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
  %(prog)s canlog.csv --list-sessions         Show detected trip/session boundaries
  %(prog)s canlog.csv --session 1 --plot RPM,Speed
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
    parser.add_argument('--dbc', type=str, default=str(DEFAULT_DBC_PATH),
                       help='Path to DBC file (default: Nissan_370Z_Z34.dbc)')
    parser.add_argument('--session', type=int,
                       help='Only process one session index (0-based), detected from seq resets')
    parser.add_argument('--list-sessions', action='store_true',
                       help='List detected sessions from seq counter resets and exit')

    args = parser.parse_args()

    global MESSAGES
    try:
        MESSAGES = load_dbc_messages(args.dbc)
    except Exception as e:
        print(f"Error: failed to load DBC '{args.dbc}': {e}", file=sys.stderr)
        sys.exit(1)

    if not MESSAGES:
        print(f"Error: no messages loaded from DBC '{args.dbc}'", file=sys.stderr)
        sys.exit(1)

    if args.list_sessions:
        if not args.input:
            parser.error("Input file is required for --list-sessions")
        sessions: Dict[int, Dict[str, int]] = {}
        with open(args.input, 'r') as f:
            reader = csv.DictReader(f)
            prev_seq: Optional[int] = None
            session_id = 0
            for row in reader:
                try:
                    seq = int(row['seq'])
                    ts = int(row['timestamp_ms'])
                except (KeyError, ValueError):
                    continue
                if prev_seq is not None and seq <= prev_seq:
                    session_id += 1
                prev_seq = seq
                s = sessions.setdefault(session_id, {"rows": 0, "seq_start": seq, "seq_end": seq,
                                                     "ts_start": ts, "ts_end": ts})
                s["rows"] += 1
                s["seq_end"] = seq
                s["ts_end"] = ts

        print("Detected sessions (0-based):")
        print("-" * 72)
        for sid in sorted(sessions.keys()):
            s = sessions[sid]
            dur_s = (s["ts_end"] - s["ts_start"]) / 1000.0
            print(f"  session={sid:2d} rows={s['rows']:8d} "
                  f"seq={s['seq_start']}..{s['seq_end']} "
                  f"time_ms={s['ts_start']}..{s['ts_end']} ({dur_s:.1f}s)")
        return

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

        global COLLECT_SESSION
        COLLECT_SESSION = args.session
        signal_specs = [s.strip() for s in args.plot.split(',')]
        plot_data = collect_plot_data(args.input, signal_specs)
        plot_signals(plot_data, subplots=args.subplots, output_file=args.output)
        return

    process_log(args.input, args.output, args.filter, args.delta, args.raw, args.session)


if __name__ == '__main__':
    main()
