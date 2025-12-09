#!/usr/bin/env python3
"""
GPS Dump Analyzer for PPK RTCM Data Verification

Parses ULog files containing gps_dump messages with RTCM3 data,
validates message integrity, and verifies message timing.

Usage:
    python3 gps_dump_analyzer.py <ulog_file> [--plot out.png] [--no-plot]

Requirements:
    pip install pyulog numpy matplotlib
"""

import argparse
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

try:
    from pyulog import ULog
except ImportError:
    print("Error: pyulog is required. Install with: pip install pyulog")
    sys.exit(1)

try:
    import numpy as np
except ImportError:
    print("Error: numpy is required. Install with: pip install numpy")
    sys.exit(1)


# RTCM3 Constants
RTCM3_PREAMBLE = 0xD3
RTCM3_MAX_PAYLOAD_LEN = 1023
RTCM3_CRC_LEN = 3
RTCM3_HEADER_LEN = 3  # Preamble + 2 bytes for reserved/length

# CRC-24Q polynomial and lookup table
CRC24Q_POLY = 0x1864CFB


def init_crc24q_table() -> List[int]:
    """Initialize CRC-24Q lookup table."""
    table = []
    for i in range(256):
        crc = i << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= CRC24Q_POLY
        table.append(crc & 0xFFFFFF)
    return table


CRC24Q_TABLE = init_crc24q_table()


def crc24q(data: bytes) -> int:
    """Calculate CRC-24Q checksum."""
    crc = 0
    for byte in data:
        crc = ((crc << 8) & 0xFFFFFF) ^ CRC24Q_TABLE[(crc >> 16) ^ byte]
    return crc


# RTCM3 Message Type Names (commonly encountered)
RTCM_MSG_NAMES = {
    1005: "Base ARP",
    1074: "GPS MSM4",
    1077: "GPS MSM7",
    1084: "GLO MSM4",
    1087: "GLO MSM7",
    1094: "GAL MSM4",
    1097: "GAL MSM7",
    1124: "BDS MSM4",
    1127: "BDS MSM7",
    1230: "GLO Bias",
    4072: "u-blox",
}


@dataclass
class ParserState:
    """State for reassembling RTCM stream from gps_dump chunks."""
    buffer: bytearray = field(default_factory=bytearray)
    timestamps: List[int] = field(default_factory=list)  # Timestamps for each byte
    bytes_discarded: int = 0
    partial_messages: int = 0


@dataclass
class AnalysisResult:
    """Results of RTCM data analysis."""
    total_bytes: int = 0
    bytes_discarded: int = 0
    messages_parsed: int = 0
    messages_valid: int = 0
    messages_invalid_crc: int = 0
    partial_messages: int = 0
    message_counts: Dict[int, int] = field(default_factory=lambda: defaultdict(int))
    message_timestamps: Dict[int, List[int]] = field(default_factory=lambda: defaultdict(list))
    timing_gaps: Dict[int, List[float]] = field(default_factory=lambda: defaultdict(list))
    log_duration_s: float = 0.0
    first_timestamp_us: int = 0
    last_timestamp_us: int = 0


def parse_rtcm_stream(state: ParserState, result: AnalysisResult):
    """Parse RTCM3 messages from the reassembled byte stream."""
    buf = state.buffer
    timestamps = state.timestamps

    pos = 0
    while pos < len(buf):
        # Find preamble
        if buf[pos] != RTCM3_PREAMBLE:
            state.bytes_discarded += 1
            pos += 1
            continue

        # Need at least header to check length
        if pos + RTCM3_HEADER_LEN > len(buf):
            break  # Wait for more data

        # Extract length (lower 10 bits of bytes 1-2)
        length_bytes = (buf[pos + 1] << 8) | buf[pos + 2]
        payload_len = length_bytes & 0x3FF  # 10 bits

        # Validate length
        if payload_len > RTCM3_MAX_PAYLOAD_LEN:
            state.bytes_discarded += 1
            pos += 1
            continue

        # Check if we have the complete message
        total_len = RTCM3_HEADER_LEN + payload_len + RTCM3_CRC_LEN
        if pos + total_len > len(buf):
            break  # Incomplete message, wait for more data

        # Extract message components
        msg_data = bytes(buf[pos:pos + total_len])
        header_and_payload = msg_data[:-RTCM3_CRC_LEN]
        crc_bytes = msg_data[-RTCM3_CRC_LEN:]

        # Validate CRC
        received_crc = (crc_bytes[0] << 16) | (crc_bytes[1] << 8) | crc_bytes[2]
        calculated_crc = crc24q(header_and_payload)
        crc_valid = received_crc == calculated_crc

        # Extract message type (first 12 bits of payload)
        if payload_len >= 2:
            payload = msg_data[RTCM3_HEADER_LEN:RTCM3_HEADER_LEN + payload_len]
            msg_type = (payload[0] << 4) | (payload[1] >> 4)
        else:
            msg_type = 0

        # Update statistics
        result.messages_parsed += 1
        if crc_valid:
            result.messages_valid += 1
            result.message_counts[msg_type] += 1
            start_ts = timestamps[pos] if pos < len(timestamps) else 0
            result.message_timestamps[msg_type].append(start_ts)
        else:
            result.messages_invalid_crc += 1

        pos += total_len

    # Keep unparsed bytes for next iteration
    state.buffer = bytearray(buf[pos:])
    state.timestamps = timestamps[pos:] if pos < len(timestamps) else []


def analyze_timing(result: AnalysisResult):
    """Compute inter-message intervals for each message type."""
    for msg_type, timestamps in result.message_timestamps.items():
        if len(timestamps) < 2:
            continue
        timestamps_sorted = sorted(timestamps)
        for i in range(1, len(timestamps_sorted)):
            interval_s = (timestamps_sorted[i] - timestamps_sorted[i-1]) / 1_000_000
            result.timing_gaps[msg_type].append(interval_s)


def load_gps_dump(ulog_file: str) -> Tuple[Optional[AnalysisResult], Optional[str]]:
    """
    Load and parse gps_dump messages from a ULog file.

    Returns (AnalysisResult, error_message) tuple.
    """
    try:
        ulog = ULog(ulog_file, message_name_filter_list=['gps_dump'])
    except Exception as e:
        return None, f"Failed to load ULog file: {e}"

    # Find gps_dump dataset
    gps_dump_data = None
    for dataset in ulog.data_list:
        if dataset.name == 'gps_dump':
            gps_dump_data = dataset
            break

    if gps_dump_data is None:
        return None, "No gps_dump messages found in log file"

    result = AnalysisResult()
    state = ParserState()

    # Extract data arrays
    timestamps = gps_dump_data.data['timestamp']
    lengths = gps_dump_data.data['len']

    # The data field is stored as individual columns data[0], data[1], ... data[78]
    data_columns = []
    for i in range(79):
        col_name = f'data[{i}]'
        if col_name in gps_dump_data.data:
            data_columns.append(gps_dump_data.data[col_name])
        else:
            break

    if not data_columns:
        return None, "No data columns found in gps_dump"

    num_entries = len(timestamps)
    result.first_timestamp_us = int(timestamps[0]) if num_entries > 0 else 0
    result.last_timestamp_us = int(timestamps[-1]) if num_entries > 0 else 0
    result.log_duration_s = (result.last_timestamp_us - result.first_timestamp_us) / 1_000_000

    # Process each gps_dump entry
    for idx in range(num_entries):
        ts = int(timestamps[idx])
        length = int(lengths[idx]) & 0x7F  # Mask off direction bit

        # Extract data bytes
        for i in range(min(length, len(data_columns))):
            byte_val = int(data_columns[i][idx]) & 0xFF
            state.buffer.append(byte_val)
            state.timestamps.append(ts)
            result.total_bytes += 1

    # Parse all accumulated data
    parse_rtcm_stream(state, result)

    # Check for remaining unparsed data (partial message at end)
    if len(state.buffer) > 0:
        result.partial_messages = 1

    result.bytes_discarded = state.bytes_discarded

    # Analyze timing
    analyze_timing(result)

    return result, None


def print_report(result: AnalysisResult):
    """Print analysis report to console."""
    print("\n" + "=" * 70)
    print("GPS DUMP RTCM ANALYSIS REPORT")
    print("=" * 70)

    print(f"\n{'LOG SUMMARY':=^70}")
    print(f"  Duration:       {result.log_duration_s:.2f} seconds")
    print(f"  Total Bytes:    {result.total_bytes:,}")
    print(f"  Discarded:      {result.bytes_discarded:,} ({100*result.bytes_discarded/max(1, result.total_bytes):.2f}%)")

    print(f"\n{'MESSAGE PARSING':=^70}")
    print(f"  Parsed:         {result.messages_parsed:,}")
    print(f"  Valid:          {result.messages_valid:,}")
    print(f"  Invalid CRC:    {result.messages_invalid_crc:,}")
    print(f"  Partial (EOF):  {result.partial_messages:,}")

    if result.messages_invalid_crc > 0:
        print(f"\n  WARNING: {result.messages_invalid_crc} messages failed CRC validation!")

    if result.bytes_discarded > 0:
        print(f"\n  WARNING: {result.bytes_discarded} bytes were not part of any valid message!")

    def print_msg_row(msg_type, count, duration_s, timing_gaps):
        name = RTCM_MSG_NAMES.get(msg_type, str(msg_type))[:12]
        rate = count / max(0.001, duration_s)

        if msg_type in timing_gaps and len(timing_gaps[msg_type]) >= 2:
            gaps_np = np.array(timing_gaps[msg_type])
            mean_ms = np.mean(gaps_np) * 1000
            std_ms = np.std(gaps_np) * 1000
            min_ms = np.min(gaps_np) * 1000
            max_ms = np.max(gaps_np) * 1000
            print(f"  {msg_type:<5} {name:<12} {count:>7} {rate:>6.1f}Hz {mean_ms:>7.0f} {std_ms:>7.0f} {min_ms:>7.0f} {max_ms:>7.0f}")
        else:
            print(f"  {msg_type:<5} {name:<12} {count:>7} {rate:>6.1f}Hz {'--':>7} {'--':>7} {'--':>7} {'--':>7}")

    def print_table_header():
        print(f"  {'Type':<5} {'Name':<12} {'Count':>7} {'Rate':>7} {'Mean':>7} {'StdDev':>7} {'Min':>7} {'Max':>7}")
        print(f"  {'-'*5} {'-'*12} {'-'*7} {'-'*7} {'(ms)':>7} {'(ms)':>7} {'(ms)':>7} {'(ms)':>7}")

    # Define MSM message groups by constellation
    MSM4_MESSAGES = {1074, 1084, 1094, 1124}  # GPS, GLONASS, Galileo, BeiDou
    MSM7_MESSAGES = {1077, 1087, 1097, 1127}  # GPS, GLONASS, Galileo, BeiDou
    BIAS_MESSAGES = {1230}  # GLONASS biases
    OTHER_MESSAGES = set(result.message_counts.keys()) - MSM4_MESSAGES - MSM7_MESSAGES - BIAS_MESSAGES

    # MSM7 Messages Table
    msm7_found = MSM7_MESSAGES & set(result.message_counts.keys())
    if msm7_found:
        print(f"\n{'MSM7 MESSAGES (PPK)':=^70}")
        print_table_header()
        for msg_type in sorted(msm7_found):
            print_msg_row(msg_type, result.message_counts[msg_type], result.log_duration_s, result.timing_gaps)

    # MSM4 Messages Table
    msm4_found = MSM4_MESSAGES & set(result.message_counts.keys())
    if msm4_found:
        print(f"\n{'MSM4 MESSAGES (RTK)':=^70}")
        print_table_header()
        for msg_type in sorted(msm4_found):
            print_msg_row(msg_type, result.message_counts[msg_type], result.log_duration_s, result.timing_gaps)

    # Bias Messages Table
    bias_found = BIAS_MESSAGES & set(result.message_counts.keys())
    if bias_found:
        print(f"\n{'GLONASS BIAS':=^70}")
        print_table_header()
        for msg_type in sorted(bias_found):
            print_msg_row(msg_type, result.message_counts[msg_type], result.log_duration_s, result.timing_gaps)

    # Other Messages Table
    if OTHER_MESSAGES:
        print(f"\n{'OTHER MESSAGES':=^70}")
        print_table_header()
        for msg_type in sorted(OTHER_MESSAGES):
            print_msg_row(msg_type, result.message_counts[msg_type], result.log_duration_s, result.timing_gaps)

    # Overall assessment
    print(f"\n{'ASSESSMENT':=^70}")
    if result.messages_invalid_crc == 0 and result.bytes_discarded == 0:
        print("  PASS - All messages valid, no data corruption detected")
    else:
        print("  ISSUES DETECTED:")
        if result.messages_invalid_crc > 0:
            print(f"    - {result.messages_invalid_crc} CRC errors")
        if result.bytes_discarded > 0:
            print(f"    - {result.bytes_discarded} orphan bytes")

    print("=" * 70)


def generate_plots(result: AnalysisResult, output_file: Optional[str] = None):
    """Generate visualization plots."""
    try:
        import matplotlib
        if output_file:
            matplotlib.use('Agg')  # Non-interactive backend for file output
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    except Exception as e:
        # Catch any error during matplotlib import or initialization
        print(f"Warning: matplotlib not available ({type(e).__name__}: {e}), skipping plots")
        return

    # Define message groupings for ordered display
    MSM4_TYPES = [1074, 1084, 1094, 1124]  # GPS, GLONASS, Galileo, BeiDou
    MSM7_TYPES = [1077, 1087, 1097, 1127]  # GPS, GLONASS, Galileo, BeiDou
    BIAS_TYPES = [1230]

    # Order: MSM4 first, then MSM7, then bias, then others
    ordered_types = []
    for t in MSM4_TYPES:
        if t in result.message_counts:
            ordered_types.append(t)
    for t in MSM7_TYPES:
        if t in result.message_counts:
            ordered_types.append(t)
    for t in BIAS_TYPES:
        if t in result.message_counts:
            ordered_types.append(t)
    for t in sorted(result.message_counts.keys()):
        if t not in ordered_types:
            ordered_types.append(t)

    # Plot 1: Message counts bar chart with Hz rates
    ax1 = axes[0]
    counts = [result.message_counts[t] for t in ordered_types]
    rates = [result.message_counts[t] / max(0.001, result.log_duration_s) for t in ordered_types]

    # Color by type: blue for MSM4, green for MSM7, orange for bias, gray for others
    colors = []
    for t in ordered_types:
        if t in MSM4_TYPES:
            colors.append('#2196F3')  # Blue for MSM4
        elif t in MSM7_TYPES:
            colors.append('#4CAF50')  # Green for MSM7
        elif t in BIAS_TYPES:
            colors.append('#FF9800')  # Orange for bias
        else:
            colors.append('#9E9E9E')  # Gray for others

    bars = ax1.bar(range(len(ordered_types)), counts, color=colors, edgecolor='black', linewidth=0.5)
    ax1.set_xticks(range(len(ordered_types)))

    # Labels with message type on first line, name below
    labels = [f"{t}\n{RTCM_MSG_NAMES.get(t, '')}" for t in ordered_types]
    ax1.set_xticklabels(labels, fontsize=8)
    ax1.set_ylabel('Message Count')
    ax1.set_title('RTCM Message Counts by Type')
    ax1.grid(axis='y', alpha=0.3)

    # Add legend for color coding
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='#2196F3', label='MSM4'),
        Patch(facecolor='#4CAF50', label='MSM7'),
        Patch(facecolor='#FF9800', label='Bias'),
    ]
    ax1.legend(handles=legend_elements, loc='upper right', fontsize=8)

    # Add count and rate labels on bars
    max_count = max(counts) if counts else 1
    for bar, count, rate in zip(bars, counts, rates):
        # Count on top
        ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max_count*0.01,
                f"{count}", ha='center', va='bottom', fontsize=7, fontweight='bold')
        # Rate inside bar
        ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height()/2,
                f"{rate:.1f}Hz", ha='center', va='center', fontsize=6, color='white', fontweight='bold')

    # Plot 2: Message timing timeline
    ax2 = axes[1]
    base_time = result.first_timestamp_us

    # Plot message reception points and gap markers
    y_positions = {t: i for i, t in enumerate(ordered_types)}
    for msg_type in ordered_types:
        if msg_type not in result.message_timestamps:
            continue
        timestamps = sorted(result.message_timestamps[msg_type])
        times_s = [(t - base_time) / 1_000_000 for t in timestamps]
        y_val = y_positions[msg_type]

        # Color by type
        if msg_type in MSM4_TYPES:
            color = '#2196F3'
        elif msg_type in MSM7_TYPES:
            color = '#4CAF50'
        elif msg_type in BIAS_TYPES:
            color = '#FF9800'
        else:
            color = '#9E9E9E'

        ax2.scatter(times_s, [y_val] * len(times_s), s=3, color=color, alpha=0.7)

        # Mark gaps with red markers (gaps > 1.5x mean)
        if msg_type in result.timing_gaps and len(result.timing_gaps[msg_type]) >= 2:
            gaps_np = np.array(result.timing_gaps[msg_type])
            gap_threshold = np.mean(gaps_np) * 1.5

            for i in range(1, len(timestamps)):
                interval_s = (timestamps[i] - timestamps[i-1]) / 1_000_000
                if interval_s > gap_threshold:
                    gap_center = (times_s[i] + times_s[i-1]) / 2
                    ax2.scatter([gap_center], [y_val], s=50, color='red', marker='x', linewidths=1.5, zorder=5)

    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Message Type')
    ax2.set_yticks(range(len(ordered_types)))
    ax2.set_yticklabels([str(t) for t in ordered_types], fontsize=8)
    ax2.set_title('Message Arrival Timeline (red X = jitter, > 1.5x mean interval)')
    ax2.grid(alpha=0.3, axis='x')
    ax2.set_xlim(0, result.log_duration_s)

    # Plot 3: Inter-message interval histogram (all message types)
    ax3 = axes[2]

    for msg_type in ordered_types:
        if msg_type not in result.timing_gaps:
            continue

        gaps = result.timing_gaps[msg_type]
        if len(gaps) < 2:
            continue

        gaps_ms = [g * 1000 for g in gaps]

        if msg_type in MSM4_TYPES:
            color = '#2196F3'
        elif msg_type in MSM7_TYPES:
            color = '#4CAF50'
        elif msg_type in BIAS_TYPES:
            color = '#FF9800'
        else:
            color = '#9E9E9E'

        ax3.hist(gaps_ms, bins=50, alpha=0.4, label=f"{msg_type}", range=(0, 500), color=color)

    ax3.set_xlabel('Interval (ms)')
    ax3.set_ylabel('Count')
    ax3.set_title('Timing Jitter')
    ax3.legend(loc='upper right', fontsize=7, ncol=3)
    ax3.grid(alpha=0.3)

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {output_file}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Analyze GPS dump RTCM data from PX4 ULog files for PPK verification.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s flight.ulg                 # Analyze and show plot
  %(prog)s flight.ulg --plot out.png  # Save plot to file
  %(prog)s flight.ulg --no-plot       # Console output only
        """
    )
    parser.add_argument('ulog_file', help='Path to ULog file')
    parser.add_argument('--plot', '-p', help='Save plot to file (PNG)')
    parser.add_argument('--no-plot', action='store_true', help='Skip plotting')

    args = parser.parse_args()

    print(f"Loading ULog file: {args.ulog_file}")
    result, error = load_gps_dump(args.ulog_file)

    if error:
        print(f"Error: {error}")
        sys.exit(1)

    print_report(result)

    if not args.no_plot:
        generate_plots(result, args.plot)

    # Return exit code based on data integrity
    if result.messages_invalid_crc > 0 or result.bytes_discarded > 0:
        sys.exit(1)


if __name__ == '__main__':
    main()
