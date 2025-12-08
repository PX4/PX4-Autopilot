#!/usr/bin/env python3
"""
GPS Dump Analyzer for PPK RTCM Data Verification

This tool parses ULog files containing gps_dump messages with RTCM3 data,
validates message integrity, and verifies message timing for PPK workflows.

Usage:
    python3 gps_dump_analyzer.py <ulog_file> [--output report.html] [--no-plot]

Requirements:
    pip install pyulog numpy matplotlib

Author: PX4 Development Team
"""

import argparse
import struct
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


# RTCM3 Message Type Names (comprehensive list)
RTCM_MSG_NAMES = {
    # Station coordinates
    1005: "Stationary RTK Reference Station ARP",
    1006: "Stationary RTK Reference Station ARP with Height",
    1007: "Antenna Descriptor",
    1008: "Antenna Descriptor & Serial Number",
    # GPS
    1001: "GPS L1 Observations",
    1002: "GPS Extended L1 Observations",
    1003: "GPS L1/L2 Observations",
    1004: "GPS Extended L1/L2 Observations",
    1071: "GPS MSM1",
    1072: "GPS MSM2",
    1073: "GPS MSM3",
    1074: "GPS MSM4",
    1075: "GPS MSM5",
    1076: "GPS MSM6",
    1077: "GPS MSM7",
    # GLONASS
    1009: "GLONASS L1 Observations",
    1010: "GLONASS Extended L1 Observations",
    1011: "GLONASS L1/L2 Observations",
    1012: "GLONASS Extended L1/L2 Observations",
    1081: "GLONASS MSM1",
    1082: "GLONASS MSM2",
    1083: "GLONASS MSM3",
    1084: "GLONASS MSM4",
    1085: "GLONASS MSM5",
    1086: "GLONASS MSM6",
    1087: "GLONASS MSM7",
    1230: "GLONASS Code-Phase Biases",
    # Galileo
    1091: "Galileo MSM1",
    1092: "Galileo MSM2",
    1093: "Galileo MSM3",
    1094: "Galileo MSM4",
    1095: "Galileo MSM5",
    1096: "Galileo MSM6",
    1097: "Galileo MSM7",
    # SBAS
    1101: "SBAS MSM1",
    1102: "SBAS MSM2",
    1103: "SBAS MSM3",
    1104: "SBAS MSM4",
    1105: "SBAS MSM5",
    1106: "SBAS MSM6",
    1107: "SBAS MSM7",
    # QZSS
    1111: "QZSS MSM1",
    1112: "QZSS MSM2",
    1113: "QZSS MSM3",
    1114: "QZSS MSM4",
    1115: "QZSS MSM5",
    1116: "QZSS MSM6",
    1117: "QZSS MSM7",
    # BeiDou
    1121: "BeiDou MSM1",
    1122: "BeiDou MSM2",
    1123: "BeiDou MSM3",
    1124: "BeiDou MSM4",
    1125: "BeiDou MSM5",
    1126: "BeiDou MSM6",
    1127: "BeiDou MSM7",
    # NavIC/IRNSS
    1131: "NavIC MSM1",
    1132: "NavIC MSM2",
    1133: "NavIC MSM3",
    1134: "NavIC MSM4",
    1135: "NavIC MSM5",
    1136: "NavIC MSM6",
    1137: "NavIC MSM7",
    # Proprietary
    4072: "u-blox Proprietary",
    4094: "Trimble Proprietary",
}

# Target MSM7 messages for PPK
PPK_TARGET_MESSAGES = {1077, 1087, 1097, 1127, 1230}


@dataclass
class RTCMMessage:
    """Represents a parsed RTCM3 message."""
    msg_type: int
    payload_len: int
    timestamp_us: int  # Timestamp when first byte was received
    end_timestamp_us: int  # Timestamp when last byte was received
    crc_valid: bool
    raw_data: bytes


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
    messages: List[RTCMMessage] = field(default_factory=list)
    log_duration_s: float = 0.0
    first_timestamp_us: int = 0
    last_timestamp_us: int = 0


def parse_rtcm_stream(state: ParserState, result: AnalysisResult) -> List[RTCMMessage]:
    """
    Parse RTCM3 messages from the reassembled byte stream.

    Returns list of parsed messages.
    """
    messages = []
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

        # Get timestamps for this message
        start_ts = timestamps[pos] if pos < len(timestamps) else 0
        end_idx = min(pos + total_len - 1, len(timestamps) - 1)
        end_ts = timestamps[end_idx] if end_idx >= 0 else start_ts

        msg = RTCMMessage(
            msg_type=msg_type,
            payload_len=payload_len,
            timestamp_us=start_ts,
            end_timestamp_us=end_ts,
            crc_valid=crc_valid,
            raw_data=msg_data
        )
        messages.append(msg)

        # Update statistics
        result.messages_parsed += 1
        if crc_valid:
            result.messages_valid += 1
            result.message_counts[msg_type] += 1
            result.message_timestamps[msg_type].append(start_ts)
        else:
            result.messages_invalid_crc += 1

        pos += total_len

    # Keep unparsed bytes for next iteration
    remaining = buf[pos:]
    remaining_ts = timestamps[pos:] if pos < len(timestamps) else []
    state.buffer = bytearray(remaining)
    state.timestamps = list(remaining_ts)

    return messages


def analyze_timing(result: AnalysisResult, expected_rate_hz: float = 5.0):
    """Analyze message timing and detect gaps."""
    expected_interval_us = 1_000_000 / expected_rate_hz
    tolerance = 0.5  # 50% tolerance

    for msg_type, timestamps in result.message_timestamps.items():
        if len(timestamps) < 2:
            continue

        timestamps_sorted = sorted(timestamps)
        for i in range(1, len(timestamps_sorted)):
            interval_us = timestamps_sorted[i] - timestamps_sorted[i-1]
            interval_s = interval_us / 1_000_000
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
    messages = parse_rtcm_stream(state, result)
    result.messages.extend(messages)

    # Check for remaining unparsed data (partial message at end)
    if len(state.buffer) > 0:
        result.partial_messages = 1

    result.bytes_discarded = state.bytes_discarded

    # Analyze timing
    analyze_timing(result)

    return result, None


def print_report(result: AnalysisResult, verbose: bool = False):
    """Print analysis report to console."""
    print("\n" + "=" * 70)
    print("GPS DUMP RTCM ANALYSIS REPORT")
    print("=" * 70)

    print(f"\n{'LOG SUMMARY':=^70}")
    print(f"  Log Duration:           {result.log_duration_s:.2f} seconds")
    print(f"  Total Bytes Received:   {result.total_bytes:,}")
    print(f"  Bytes Discarded:        {result.bytes_discarded:,} ({100*result.bytes_discarded/max(1, result.total_bytes):.2f}%)")

    print(f"\n{'MESSAGE PARSING':=^70}")
    print(f"  Messages Parsed:        {result.messages_parsed:,}")
    print(f"  Valid Messages:         {result.messages_valid:,}")
    print(f"  Invalid CRC:            {result.messages_invalid_crc:,}")
    print(f"  Partial (end of log):   {result.partial_messages:,}")

    if result.messages_invalid_crc > 0:
        print(f"\n  ⚠ WARNING: {result.messages_invalid_crc} messages failed CRC validation!")

    if result.bytes_discarded > 0:
        print(f"\n  ⚠ WARNING: {result.bytes_discarded} bytes were not part of any valid message!")

    # Helper to print a message row with timing stats
    def print_msg_row(msg_type, count, duration_s, timing_gaps, verbose_gaps=False):
        name = RTCM_MSG_NAMES.get(msg_type, "Unknown")[:12]
        rate = count / max(0.001, duration_s)

        if msg_type in timing_gaps and len(timing_gaps[msg_type]) >= 2:
            gaps_np = np.array(timing_gaps[msg_type])
            mean_ms = np.mean(gaps_np) * 1000
            std_ms = np.std(gaps_np) * 1000
            min_ms = np.min(gaps_np) * 1000
            max_ms = np.max(gaps_np) * 1000
            # Large gap threshold: 1.5x mean or 300ms minimum
            large_gap_threshold = max(np.mean(gaps_np) * 1.5, 0.3)
            large_gaps = gaps_np[gaps_np > large_gap_threshold]
            num_gaps = len(large_gaps)
            gap_str = f"{num_gaps:>4}!" if num_gaps > 0 else f"{num_gaps:>5}"
            print(f"  {msg_type:<5} {name:<12} {count:>7} {rate:>7.2f}Hz {mean_ms:>8.1f} {std_ms:>8.1f} {min_ms:>8.1f} {max_ms:>8.1f} {gap_str}")
            if verbose_gaps and num_gaps > 0 and num_gaps <= 5:
                for gap in large_gaps[:5]:
                    print(f"        └─ gap: {gap*1000:.1f} ms")
        else:
            print(f"  {msg_type:<5} {name:<12} {count:>7} {rate:>7.2f}Hz {'--':>8} {'--':>8} {'--':>8} {'--':>8} {'--':>5}")

    def print_table_header():
        print(f"  {'Type':<5} {'Name':<12} {'Count':>7} {'Rate':>8} {'Mean':>8} {'StdDev':>8} {'Min':>8} {'Max':>8} {'Gaps':>5}")
        print(f"  {'-'*5} {'-'*12} {'-'*7} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*5}")

    # Define MSM message groups by constellation
    MSM4_MESSAGES = {1074, 1084, 1094, 1124}  # GPS, GLONASS, Galileo, BeiDou
    MSM7_MESSAGES = {1077, 1087, 1097, 1127}  # GPS, GLONASS, Galileo, BeiDou
    BIAS_MESSAGES = {1230}  # GLONASS biases
    OTHER_MESSAGES = set(result.message_counts.keys()) - MSM4_MESSAGES - MSM7_MESSAGES - BIAS_MESSAGES

    # MSM7 Messages Table
    msm7_found = MSM7_MESSAGES & set(result.message_counts.keys())
    if msm7_found:
        print(f"\n{'MSM7 MESSAGES (Full Pseudorange/Carrier-Phase)':=^90}")
        print_table_header()
        for msg_type in sorted(msm7_found):
            print_msg_row(msg_type, result.message_counts[msg_type], result.log_duration_s, result.timing_gaps, verbose)

    # MSM4 Messages Table
    msm4_found = MSM4_MESSAGES & set(result.message_counts.keys())
    if msm4_found:
        print(f"\n{'MSM4 MESSAGES (Pseudorange/Carrier-Phase)':=^90}")
        print_table_header()
        for msg_type in sorted(msm4_found):
            print_msg_row(msg_type, result.message_counts[msg_type], result.log_duration_s, result.timing_gaps, verbose)

    # Bias Messages Table
    bias_found = BIAS_MESSAGES & set(result.message_counts.keys())
    if bias_found:
        print(f"\n{'GLONASS BIAS MESSAGES':=^90}")
        print_table_header()
        for msg_type in sorted(bias_found):
            print_msg_row(msg_type, result.message_counts[msg_type], result.log_duration_s, result.timing_gaps, verbose)

    # Other Messages Table
    if OTHER_MESSAGES:
        print(f"\n{'OTHER RTCM MESSAGES':=^90}")
        print_table_header()
        for msg_type in sorted(OTHER_MESSAGES):
            print_msg_row(msg_type, result.message_counts[msg_type], result.log_duration_s, result.timing_gaps, verbose)

    # Overall assessment
    print(f"\n{'OVERALL ASSESSMENT':=^90}")
    issues = []
    if result.messages_invalid_crc > 0:
        issues.append(f"{result.messages_invalid_crc} CRC errors")
    if result.bytes_discarded > 0:
        issues.append(f"{result.bytes_discarded} orphan bytes")

    if not issues:
        print("  ✓ PASS - All messages have valid CRC, no data corruption detected")
    else:
        print("  ✗ ISSUES DETECTED:")
        for issue in issues:
            print(f"    - {issue}")

    print("\n" + "=" * 90)


def generate_plots(result: AnalysisResult, output_file: Optional[str] = None):
    """Generate visualization plots."""
    try:
        import matplotlib
        if output_file:
            matplotlib.use('Agg')  # Non-interactive backend for file output
        import matplotlib.pyplot as plt
        from datetime import datetime, timedelta
        # Test that matplotlib actually works
        fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    except Exception as e:
        # Catch any error during matplotlib import or initialization
        print(f"Warning: matplotlib not available ({type(e).__name__}: {e}), skipping plots")
        return

    # fig, axes already created above
    fig.suptitle('RTCM Message Analysis', fontsize=14, fontweight='bold')

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

    # Labels with message type and short name
    labels = [f"{t}\n{RTCM_MSG_NAMES.get(t, '')[:10]}" for t in ordered_types]
    ax1.set_xticklabels(labels, fontsize=7)
    ax1.set_ylabel('Message Count')
    ax1.set_title('RTCM Message Counts by Type (MSM4=blue, MSM7=green, Bias=orange)')
    ax1.grid(axis='y', alpha=0.3)

    # Add count and rate labels on bars
    max_count = max(counts) if counts else 1
    for bar, count, rate in zip(bars, counts, rates):
        # Count on top
        ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max_count*0.01,
                f"{count}", ha='center', va='bottom', fontsize=7, fontweight='bold')
        # Rate inside bar
        ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height()/2,
                f"{rate:.1f}Hz", ha='center', va='center', fontsize=6, color='white', fontweight='bold')

    # Plot 2: Message timing timeline with gap highlighting
    ax2 = axes[1]
    base_time = result.first_timestamp_us

    # First, find and highlight gaps (regions where data is missing)
    # A gap is where interval > 1.5x mean or > 300ms
    gap_regions = []  # List of (start_time_s, end_time_s) tuples

    for msg_type in ordered_types:
        if msg_type not in result.message_timestamps:
            continue
        timestamps = sorted(result.message_timestamps[msg_type])
        if len(timestamps) < 2:
            continue

        gaps_np = np.array(result.timing_gaps.get(msg_type, []))
        if len(gaps_np) == 0:
            continue
        mean_interval = np.mean(gaps_np)
        gap_threshold = max(mean_interval * 1.5, 0.3)  # 1.5x mean or 300ms

        for i in range(1, len(timestamps)):
            interval_s = (timestamps[i] - timestamps[i-1]) / 1_000_000
            if interval_s > gap_threshold:
                start_s = (timestamps[i-1] - base_time) / 1_000_000
                end_s = (timestamps[i] - base_time) / 1_000_000
                gap_regions.append((start_s, end_s))

    # Draw gap regions as light red vertical spans
    for start_s, end_s in gap_regions:
        ax2.axvspan(start_s, end_s, alpha=0.15, color='red', zorder=0)

    # Plot message reception points
    y_positions = {t: i for i, t in enumerate(ordered_types)}
    for msg_type in ordered_types:
        if msg_type not in result.message_timestamps:
            continue
        timestamps = result.message_timestamps[msg_type]
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

    ax2.set_xlabel('Time (seconds from start)')
    ax2.set_ylabel('Message Type')
    ax2.set_yticks(range(len(ordered_types)))
    ax2.set_yticklabels([str(t) for t in ordered_types], fontsize=8)
    ax2.set_title('RTCM Message Reception Timeline (red = data gaps)')
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

    ax3.axvline(x=200, color='red', linestyle='--', linewidth=2, label='5Hz (200ms)')
    ax3.set_xlabel('Inter-message Interval (ms)')
    ax3.set_ylabel('Frequency')
    ax3.set_title('Message Timing Distribution')
    ax3.legend(loc='upper right', fontsize=7, ncol=3)
    ax3.grid(alpha=0.3)

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {output_file}")
    else:
        plt.show()


def generate_html_report(result: AnalysisResult, output_file: str):
    """Generate an HTML report with embedded plots."""
    import io
    import base64

    plot_data = None
    try:
        import matplotlib
        matplotlib.use('Agg')  # Non-interactive backend
        import matplotlib.pyplot as plt
    except (ImportError, AttributeError, Exception) as e:
        print(f"Warning: matplotlib not available ({type(e).__name__}), generating text-only HTML report")
    else:
        # Generate plot to buffer
        fig, axes = plt.subplots(3, 1, figsize=(12, 9))

        # Message counts
        ax1 = axes[0]
        msg_types = sorted(result.message_counts.keys())
        counts = [result.message_counts[t] for t in msg_types]
        labels = [f"{t}" for t in msg_types]
        colors = ['#28a745' if t in PPK_TARGET_MESSAGES else '#6c757d' for t in msg_types]
        ax1.bar(range(len(msg_types)), counts, color=colors)
        ax1.set_xticks(range(len(msg_types)))
        ax1.set_xticklabels(labels)
        ax1.set_ylabel('Count')
        ax1.set_title('Message Counts by Type')
        ax1.grid(axis='y', alpha=0.3)

        # Timeline (all message types)
        ax2 = axes[1]
        base_time = result.first_timestamp_us
        for msg_type in sorted(result.message_timestamps.keys()):
            if msg_type not in result.message_timestamps:
                continue
            timestamps = result.message_timestamps[msg_type]
            times_s = [(t - base_time) / 1_000_000 for t in timestamps]
            ax2.scatter(times_s, [msg_type] * len(times_s), s=1, alpha=0.5, label=str(msg_type))
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Message Type')
        ax2.set_title('All RTCM Message Timeline')
        ax2.legend(loc='upper right', fontsize=7, ncol=2)
        ax2.grid(alpha=0.3)

        # Histogram (all message types)
        ax3 = axes[2]
        for msg_type in sorted(result.timing_gaps.keys()):
            if msg_type not in result.timing_gaps or len(result.timing_gaps[msg_type]) < 2:
                continue
            gaps_ms = [g * 1000 for g in result.timing_gaps[msg_type]]
            color = '#28a745' if msg_type in PPK_TARGET_MESSAGES else '#6c757d'
            ax3.hist(gaps_ms, bins=50, alpha=0.4, label=str(msg_type), range=(0, 1100), color=color)
        ax3.axvline(x=200, color='red', linestyle='--', linewidth=2, label='5Hz')
        ax3.axvline(x=1000, color='orange', linestyle='--', linewidth=2, label='1Hz')
        ax3.set_xlabel('Interval (ms)')
        ax3.set_ylabel('Frequency')
        ax3.set_title('Timing Distribution (all types)')
        ax3.legend(fontsize=7, ncol=2)
        ax3.grid(alpha=0.3)

        plt.tight_layout()

        buf = io.BytesIO()
        plt.savefig(buf, format='png', dpi=120, bbox_inches='tight')
        buf.seek(0)
        plot_data = base64.b64encode(buf.read()).decode('utf-8')
        plt.close()

    # Build HTML
    issues = []
    if result.messages_invalid_crc > 0:
        issues.append(f"{result.messages_invalid_crc} CRC errors")
    if result.bytes_discarded > 0:
        issues.append(f"{result.bytes_discarded} orphan bytes")
    missing = PPK_TARGET_MESSAGES - set(result.message_counts.keys())
    if missing:
        issues.append(f"missing: {sorted(missing)}")

    status_class = "pass" if not issues else "fail"
    status_text = "PASS" if not issues else "ISSUES DETECTED"

    html = f"""<!DOCTYPE html>
<html>
<head>
    <title>GPS Dump RTCM Analysis Report</title>
    <style>
        body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; margin: 40px; background: #f5f5f5; }}
        .container {{ max-width: 1200px; margin: 0 auto; background: white; padding: 30px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }}
        h1 {{ color: #333; border-bottom: 2px solid #007bff; padding-bottom: 10px; }}
        h2 {{ color: #555; margin-top: 30px; }}
        table {{ border-collapse: collapse; width: 100%; margin: 15px 0; }}
        th, td {{ border: 1px solid #ddd; padding: 10px; text-align: left; }}
        th {{ background: #f8f9fa; font-weight: 600; }}
        tr:nth-child(even) {{ background: #f8f9fa; }}
        .stat-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin: 20px 0; }}
        .stat-box {{ background: #f8f9fa; padding: 15px; border-radius: 6px; text-align: center; }}
        .stat-value {{ font-size: 24px; font-weight: bold; color: #333; }}
        .stat-label {{ font-size: 12px; color: #666; text-transform: uppercase; }}
        .pass {{ color: #28a745; }}
        .fail {{ color: #dc3545; }}
        .warning {{ color: #ffc107; }}
        .status-banner {{ padding: 15px; border-radius: 6px; margin: 20px 0; font-weight: bold; font-size: 18px; }}
        .status-banner.pass {{ background: #d4edda; color: #155724; }}
        .status-banner.fail {{ background: #f8d7da; color: #721c24; }}
        .plot {{ text-align: center; margin: 20px 0; }}
        .plot img {{ max-width: 100%; height: auto; border: 1px solid #ddd; border-radius: 4px; }}
        .ppk {{ background: #d4edda !important; }}
        code {{ background: #e9ecef; padding: 2px 6px; border-radius: 3px; font-family: monospace; }}
    </style>
</head>
<body>
    <div class="container">
        <h1>GPS Dump RTCM Analysis Report</h1>

        <div class="status-banner {status_class}">
            Overall Status: {status_text}
            {' - ' + ', '.join(issues) if issues else ''}
        </div>

        <h2>Log Summary</h2>
        <div class="stat-grid">
            <div class="stat-box">
                <div class="stat-value">{result.log_duration_s:.1f}s</div>
                <div class="stat-label">Duration</div>
            </div>
            <div class="stat-box">
                <div class="stat-value">{result.total_bytes:,}</div>
                <div class="stat-label">Total Bytes</div>
            </div>
            <div class="stat-box">
                <div class="stat-value">{result.messages_valid:,}</div>
                <div class="stat-label">Valid Messages</div>
            </div>
            <div class="stat-box">
                <div class="stat-value {'' if result.messages_invalid_crc == 0 else 'fail'}">{result.messages_invalid_crc}</div>
                <div class="stat-label">CRC Errors</div>
            </div>
            <div class="stat-box">
                <div class="stat-value {'' if result.bytes_discarded == 0 else 'warning'}">{result.bytes_discarded}</div>
                <div class="stat-label">Bytes Discarded</div>
            </div>
        </div>

"""

    # Helper function to generate table rows
    def gen_table_rows(msg_types_set):
        rows = ""
        found = msg_types_set & set(result.message_counts.keys())
        for msg_type in sorted(found):
            count = result.message_counts[msg_type]
            name = RTCM_MSG_NAMES.get(msg_type, "Unknown")[:25]
            rate = count / max(0.001, result.log_duration_s)

            if msg_type in result.timing_gaps and len(result.timing_gaps[msg_type]) >= 2:
                gaps = np.array(result.timing_gaps[msg_type])
                mean_ms = np.mean(gaps) * 1000
                std_ms = np.std(gaps) * 1000
                min_ms = np.min(gaps) * 1000
                max_ms = np.max(gaps) * 1000
                large_gap_threshold = max(np.mean(gaps) * 1.5, 0.3)
                large_gaps_count = len(gaps[gaps > large_gap_threshold])
                gap_class = "fail" if large_gaps_count > 0 else ""
                rows += f'<tr><td>{msg_type}</td><td>{name}</td><td>{count:,}</td><td>{rate:.2f}</td><td>{mean_ms:.1f}</td><td>{std_ms:.1f}</td><td>{min_ms:.1f}</td><td>{max_ms:.1f}</td><td class="{gap_class}">{large_gaps_count}</td></tr>\n'
            else:
                rows += f'<tr><td>{msg_type}</td><td>{name}</td><td>{count:,}</td><td>{rate:.2f}</td><td>--</td><td>--</td><td>--</td><td>--</td><td>--</td></tr>\n'
        return rows, len(found) > 0

    table_header = """<table>
            <tr><th>Type</th><th>Name</th><th>Count</th><th>Rate (Hz)</th><th>Mean (ms)</th><th>StdDev (ms)</th><th>Min (ms)</th><th>Max (ms)</th><th>Gaps</th></tr>
"""

    # Define message groups
    MSM7_MESSAGES = {1077, 1087, 1097, 1127}
    MSM4_MESSAGES = {1074, 1084, 1094, 1124}
    BIAS_MESSAGES = {1230}
    OTHER_MESSAGES = set(result.message_counts.keys()) - MSM7_MESSAGES - MSM4_MESSAGES - BIAS_MESSAGES

    # MSM7 Table
    msm7_rows, has_msm7 = gen_table_rows(MSM7_MESSAGES)
    if has_msm7:
        html += f"""
        <h2>MSM7 Messages (Full Pseudorange/Carrier-Phase - PPK)</h2>
        {table_header}
{msm7_rows}        </table>
"""

    # MSM4 Table
    msm4_rows, has_msm4 = gen_table_rows(MSM4_MESSAGES)
    if has_msm4:
        html += f"""
        <h2>MSM4 Messages (Pseudorange/Carrier-Phase)</h2>
        {table_header}
{msm4_rows}        </table>
"""

    # Bias Table
    bias_rows, has_bias = gen_table_rows(BIAS_MESSAGES)
    if has_bias:
        html += f"""
        <h2>GLONASS Bias Messages</h2>
        {table_header}
{bias_rows}        </table>
"""

    # Other Messages Table
    other_rows, has_other = gen_table_rows(OTHER_MESSAGES)
    if has_other:
        html += f"""
        <h2>Other RTCM Messages</h2>
        {table_header}
{other_rows}        </table>
"""

    if plot_data:
        html += f"""
        <h2>Visualizations</h2>
        <div class="plot">
            <img src="data:image/png;base64,{plot_data}" alt="Analysis Plots">
        </div>
"""

    html += """
        <h2>Notes</h2>
        <ul>
            <li><b>MSM7</b>: Full pseudorange and carrier-phase observations - required for PPK processing</li>
            <li><b>MSM4</b>: Pseudorange and carrier-phase observations - used for RTK</li>
            <li><b>Large gaps</b>: Intervals exceeding 1.5x the mean interval or 300ms minimum</li>
            <li><b>CRC errors</b>: Indicate data corruption in the logging chain (F9P → driver → uORB → UAVCAN → logger)</li>
            <li><b>Discarded bytes</b>: Bytes not part of any valid RTCM message frame (sync errors)</li>
        </ul>

        <p style="color: #666; font-size: 12px; margin-top: 30px;">
            Generated by PX4 GPS Dump Analyzer
        </p>
    </div>
</body>
</html>
"""

    with open(output_file, 'w') as f:
        f.write(html)

    print(f"HTML report saved to: {output_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Analyze GPS dump RTCM data from PX4 ULog files for PPK verification.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s flight.ulg                    # Print analysis to console
  %(prog)s flight.ulg --output report.html  # Generate HTML report
  %(prog)s flight.ulg --no-plot          # Skip plotting (faster)
  %(prog)s flight.ulg -v                 # Verbose output
        """
    )
    parser.add_argument('ulog_file', help='Path to ULog file')
    parser.add_argument('--output', '-o', help='Output HTML report file')
    parser.add_argument('--plot', '-p', help='Save plot to file (PNG)')
    parser.add_argument('--no-plot', action='store_true', help='Skip plotting')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')

    args = parser.parse_args()

    print(f"Loading ULog file: {args.ulog_file}")
    result, error = load_gps_dump(args.ulog_file)

    if error:
        print(f"Error: {error}")
        sys.exit(1)

    # Print console report
    print_report(result, verbose=args.verbose)

    # Generate outputs
    if args.output:
        generate_html_report(result, args.output)

    if not args.no_plot:
        if args.plot:
            generate_plots(result, args.plot)
        elif not args.output:
            # Show interactive plot if no file output specified
            generate_plots(result)

    # Return exit code based on data integrity
    if result.messages_invalid_crc > 0 or result.bytes_discarded > 0:
        sys.exit(1)

    sys.exit(0)


if __name__ == '__main__':
    main()
