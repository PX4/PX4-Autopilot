#!/usr/bin/env python3
"""
RTCM MSM Burst Analyzer

Analyzes RTCM message timing to understand burst patterns from the moving base.
Parses gps_dump messages from ULog files and examines when MSM messages arrive,
their sizes, and whether they come in bursts.

Usage:
    python3 rtcm_burst_analyzer.py <ulog_file> [--plot out.png] [--no-plot]

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
RTCM3_HEADER_LEN = 3

# CRC-24Q lookup table
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


# RTCM3 Message Type Names
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

# MSM message type groups
MSM4_TYPES = {1074, 1084, 1094, 1124}  # GPS, GLONASS, Galileo, BeiDou
MSM7_TYPES = {1077, 1087, 1097, 1127}


@dataclass
class RtcmMessage:
    """A single RTCM message with timing info."""
    msg_type: int
    size: int  # Total message size including header and CRC
    timestamp_us: int  # Timestamp when first byte was received
    crc_valid: bool


@dataclass
class Burst:
    """A group of messages that arrive close together."""
    start_timestamp_us: int
    end_timestamp_us: int
    messages: List[RtcmMessage]
    total_bytes: int

    @property
    def duration_ms(self) -> float:
        return (self.end_timestamp_us - self.start_timestamp_us) / 1000.0

    @property
    def msg_types(self) -> List[int]:
        return [m.msg_type for m in self.messages]


@dataclass
class AnalysisResult:
    """Results of burst analysis."""
    messages: List[RtcmMessage] = field(default_factory=list)
    bursts: List[Burst] = field(default_factory=list)
    log_duration_s: float = 0.0
    first_timestamp_us: int = 0
    last_timestamp_us: int = 0
    bytes_discarded: int = 0


def parse_rtcm_from_gps_dump(ulog_file: str) -> Tuple[Optional[AnalysisResult], Optional[str]]:
    """
    Parse gps_dump messages and extract RTCM messages with precise timing.
    """
    try:
        ulog = ULog(ulog_file, message_name_filter_list=['gps_dump'])
    except Exception as e:
        return None, f"Failed to load ULog file: {e}"

    gps_dump_data = None
    for dataset in ulog.data_list:
        if dataset.name == 'gps_dump':
            gps_dump_data = dataset
            break

    if gps_dump_data is None:
        return None, "No gps_dump messages found in log file"

    result = AnalysisResult()

    timestamps = gps_dump_data.data['timestamp']
    lengths = gps_dump_data.data['len']

    # Extract data columns
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

    # Build a stream with per-byte timestamps
    stream_bytes = bytearray()
    byte_timestamps = []  # Timestamp for each byte

    for idx in range(num_entries):
        ts = int(timestamps[idx])
        length = int(lengths[idx]) & 0x7F

        for i in range(min(length, len(data_columns))):
            byte_val = int(data_columns[i][idx]) & 0xFF
            stream_bytes.append(byte_val)
            byte_timestamps.append(ts)

    # Parse RTCM messages from stream
    pos = 0
    while pos < len(stream_bytes):
        if stream_bytes[pos] != RTCM3_PREAMBLE:
            result.bytes_discarded += 1
            pos += 1
            continue

        if pos + RTCM3_HEADER_LEN > len(stream_bytes):
            break

        length_bytes = (stream_bytes[pos + 1] << 8) | stream_bytes[pos + 2]
        payload_len = length_bytes & 0x3FF

        if payload_len > RTCM3_MAX_PAYLOAD_LEN:
            result.bytes_discarded += 1
            pos += 1
            continue

        total_len = RTCM3_HEADER_LEN + payload_len + RTCM3_CRC_LEN
        if pos + total_len > len(stream_bytes):
            break

        msg_data = bytes(stream_bytes[pos:pos + total_len])
        header_and_payload = msg_data[:-RTCM3_CRC_LEN]
        crc_bytes = msg_data[-RTCM3_CRC_LEN:]

        received_crc = (crc_bytes[0] << 16) | (crc_bytes[1] << 8) | crc_bytes[2]
        calculated_crc = crc24q(header_and_payload)
        crc_valid = received_crc == calculated_crc

        # Extract message type
        if payload_len >= 2:
            payload = msg_data[RTCM3_HEADER_LEN:RTCM3_HEADER_LEN + payload_len]
            msg_type = (payload[0] << 4) | (payload[1] >> 4)
        else:
            msg_type = 0

        # Use timestamp of first byte as message timestamp
        msg_timestamp = byte_timestamps[pos]

        result.messages.append(RtcmMessage(
            msg_type=msg_type,
            size=total_len,
            timestamp_us=msg_timestamp,
            crc_valid=crc_valid
        ))

        pos += total_len

    return result, None


def detect_bursts(result: AnalysisResult, gap_threshold_ms: float = 50.0):
    """
    Group messages into bursts based on timing gaps.

    Messages separated by less than gap_threshold_ms are considered part of the same burst.
    """
    if not result.messages:
        return

    # Sort messages by timestamp
    sorted_msgs = sorted(result.messages, key=lambda m: m.timestamp_us)

    current_burst_msgs = [sorted_msgs[0]]

    for i in range(1, len(sorted_msgs)):
        gap_ms = (sorted_msgs[i].timestamp_us - sorted_msgs[i-1].timestamp_us) / 1000.0

        if gap_ms > gap_threshold_ms:
            # End current burst, start new one
            burst = Burst(
                start_timestamp_us=current_burst_msgs[0].timestamp_us,
                end_timestamp_us=current_burst_msgs[-1].timestamp_us,
                messages=current_burst_msgs,
                total_bytes=sum(m.size for m in current_burst_msgs)
            )
            result.bursts.append(burst)
            current_burst_msgs = [sorted_msgs[i]]
        else:
            current_burst_msgs.append(sorted_msgs[i])

    # Don't forget the last burst
    if current_burst_msgs:
        burst = Burst(
            start_timestamp_us=current_burst_msgs[0].timestamp_us,
            end_timestamp_us=current_burst_msgs[-1].timestamp_us,
            messages=current_burst_msgs,
            total_bytes=sum(m.size for m in current_burst_msgs)
        )
        result.bursts.append(burst)


def print_report(result: AnalysisResult):
    """Print burst analysis report."""
    print("\n" + "=" * 80)
    print("RTCM BURST ANALYSIS REPORT")
    print("=" * 80)

    print(f"\n{'LOG SUMMARY':=^80}")
    print(f"  Duration:           {result.log_duration_s:.2f} seconds")
    print(f"  Total messages:     {len(result.messages):,}")
    print(f"  Bytes discarded:    {result.bytes_discarded:,}")

    # Message type breakdown
    msg_by_type: Dict[int, List[RtcmMessage]] = defaultdict(list)
    for msg in result.messages:
        msg_by_type[msg.msg_type].append(msg)

    print(f"\n{'MESSAGE TYPE BREAKDOWN':=^80}")
    print(f"  {'Type':<6} {'Name':<12} {'Count':>7} {'Rate':>8} {'Avg Size':>10} {'Total':>12}")
    print(f"  {'-'*6} {'-'*12} {'-'*7} {'-'*8} {'-'*10} {'-'*12}")

    for msg_type in sorted(msg_by_type.keys()):
        msgs = msg_by_type[msg_type]
        name = RTCM_MSG_NAMES.get(msg_type, str(msg_type))[:12]
        count = len(msgs)
        rate = count / max(0.001, result.log_duration_s)
        avg_size = np.mean([m.size for m in msgs])
        total_bytes = sum(m.size for m in msgs)
        print(f"  {msg_type:<6} {name:<12} {count:>7} {rate:>7.1f}Hz {avg_size:>9.1f}B {total_bytes:>11,}B")

    # Burst analysis
    print(f"\n{'BURST ANALYSIS':=^80}")
    print(f"  Total bursts:       {len(result.bursts):,}")

    if result.bursts:
        burst_sizes = [b.total_bytes for b in result.bursts]
        burst_msg_counts = [len(b.messages) for b in result.bursts]
        burst_durations = [b.duration_ms for b in result.bursts]

        # Inter-burst intervals
        burst_intervals_ms = []
        for i in range(1, len(result.bursts)):
            interval = (result.bursts[i].start_timestamp_us - result.bursts[i-1].start_timestamp_us) / 1000.0
            burst_intervals_ms.append(interval)

        print(f"\n  Burst Size (bytes):")
        print(f"    Min:              {min(burst_sizes):,}")
        print(f"    Max:              {max(burst_sizes):,}")
        print(f"    Mean:             {np.mean(burst_sizes):,.1f}")
        print(f"    Median:           {np.median(burst_sizes):,.1f}")
        print(f"    StdDev:           {np.std(burst_sizes):,.1f}")

        print(f"\n  Messages per Burst:")
        print(f"    Min:              {min(burst_msg_counts)}")
        print(f"    Max:              {max(burst_msg_counts)}")
        print(f"    Mean:             {np.mean(burst_msg_counts):.1f}")
        print(f"    Median:           {np.median(burst_msg_counts):.1f}")

        print(f"\n  Burst Duration (ms):")
        print(f"    Min:              {min(burst_durations):.2f}")
        print(f"    Max:              {max(burst_durations):.2f}")
        print(f"    Mean:             {np.mean(burst_durations):.2f}")
        print(f"    Median:           {np.median(burst_durations):.2f}")

        if burst_intervals_ms:
            print(f"\n  Inter-Burst Interval (ms):")
            print(f"    Min:              {min(burst_intervals_ms):.1f}")
            print(f"    Max:              {max(burst_intervals_ms):.1f}")
            print(f"    Mean:             {np.mean(burst_intervals_ms):.1f}")
            print(f"    Median:           {np.median(burst_intervals_ms):.1f}")
            print(f"    StdDev:           {np.std(burst_intervals_ms):.1f}")

        # Show burst composition for MSM messages
        print(f"\n{'BURST COMPOSITION (MSM Messages)':=^80}")

        # Analyze which message types appear together in bursts
        msm_types = MSM4_TYPES | MSM7_TYPES
        msm_bursts = [b for b in result.bursts if any(m.msg_type in msm_types for m in b.messages)]

        if msm_bursts:
            print(f"  Bursts containing MSM: {len(msm_bursts)}")

            # Count co-occurrence
            type_cooccurrence: Dict[frozenset, int] = defaultdict(int)
            for burst in msm_bursts:
                msm_in_burst = frozenset(m.msg_type for m in burst.messages if m.msg_type in msm_types)
                if msm_in_burst:
                    type_cooccurrence[msm_in_burst] += 1

            print(f"\n  Common burst patterns (MSM types appearing together):")
            for types, count in sorted(type_cooccurrence.items(), key=lambda x: -x[1])[:10]:
                type_names = [f"{t}({RTCM_MSG_NAMES.get(t, '?')[:3]})" for t in sorted(types)]
                pct = 100.0 * count / len(msm_bursts)
                print(f"    {' + '.join(type_names)}: {count} bursts ({pct:.1f}%)")

            # Typical MSM burst size
            msm_burst_sizes = []
            msm_burst_bytes = []
            for burst in msm_bursts:
                msm_msgs = [m for m in burst.messages if m.msg_type in msm_types]
                msm_burst_sizes.append(len(msm_msgs))
                msm_burst_bytes.append(sum(m.size for m in msm_msgs))

            print(f"\n  MSM messages per burst:")
            print(f"    Mean:             {np.mean(msm_burst_sizes):.1f}")
            print(f"    Typical burst:    {int(np.median(msm_burst_sizes))} MSM messages")

            print(f"\n  MSM bytes per burst:")
            print(f"    Mean:             {np.mean(msm_burst_bytes):.1f}")
            print(f"    Median:           {np.median(msm_burst_bytes):.1f}")

    # Intra-message timing (time between consecutive messages within bursts)
    print(f"\n{'INTRA-BURST TIMING':=^80}")

    intra_burst_gaps_ms = []
    for burst in result.bursts:
        if len(burst.messages) < 2:
            continue
        sorted_msgs = sorted(burst.messages, key=lambda m: m.timestamp_us)
        for i in range(1, len(sorted_msgs)):
            gap = (sorted_msgs[i].timestamp_us - sorted_msgs[i-1].timestamp_us) / 1000.0
            intra_burst_gaps_ms.append(gap)

    if intra_burst_gaps_ms:
        print(f"  Gap between messages within bursts (ms):")
        print(f"    Min:              {min(intra_burst_gaps_ms):.3f}")
        print(f"    Max:              {max(intra_burst_gaps_ms):.3f}")
        print(f"    Mean:             {np.mean(intra_burst_gaps_ms):.3f}")
        print(f"    Median:           {np.median(intra_burst_gaps_ms):.3f}")

    # Per-message-type timing
    print(f"\n{'PER-TYPE INTER-MESSAGE INTERVALS':=^80}")
    print(f"  {'Type':<6} {'Name':<12} {'Min':>8} {'Mean':>8} {'Median':>8} {'Max':>8} {'StdDev':>8}")
    print(f"  {'-'*6} {'-'*12} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*8}")

    for msg_type in sorted(msg_by_type.keys()):
        msgs = sorted(msg_by_type[msg_type], key=lambda m: m.timestamp_us)
        if len(msgs) < 2:
            continue

        intervals_ms = []
        for i in range(1, len(msgs)):
            interval = (msgs[i].timestamp_us - msgs[i-1].timestamp_us) / 1000.0
            intervals_ms.append(interval)

        name = RTCM_MSG_NAMES.get(msg_type, str(msg_type))[:12]
        print(f"  {msg_type:<6} {name:<12} "
              f"{min(intervals_ms):>7.1f} "
              f"{np.mean(intervals_ms):>7.1f} "
              f"{np.median(intervals_ms):>7.1f} "
              f"{max(intervals_ms):>7.1f} "
              f"{np.std(intervals_ms):>7.1f}")

    print("\n" + "=" * 80)


def generate_plots(result: AnalysisResult, output_file: Optional[str] = None):
    """Generate visualization plots."""
    try:
        import matplotlib
        if output_file:
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"Warning: matplotlib not available ({e}), skipping plots")
        return

    fig, axes = plt.subplots(4, 1, figsize=(14, 14))

    base_time_us = result.first_timestamp_us

    # Plot 1: Message timeline with message types color-coded
    ax1 = axes[0]

    # Group by message type
    msg_by_type: Dict[int, List[RtcmMessage]] = defaultdict(list)
    for msg in result.messages:
        msg_by_type[msg.msg_type].append(msg)

    # Define colors
    type_colors = {}
    msm4_colors = ['#1f77b4', '#2ca02c', '#9467bd', '#8c564b']  # Blues/greens
    msm7_colors = ['#ff7f0e', '#d62728', '#e377c2', '#bcbd22']  # Oranges/reds
    other_color = '#7f7f7f'

    for i, t in enumerate(sorted(MSM4_TYPES)):
        type_colors[t] = msm4_colors[i % len(msm4_colors)]
    for i, t in enumerate(sorted(MSM7_TYPES)):
        type_colors[t] = msm7_colors[i % len(msm7_colors)]

    # Assign y-positions to message types
    all_types = sorted(msg_by_type.keys())
    y_pos = {t: i for i, t in enumerate(all_types)}

    for msg_type, msgs in msg_by_type.items():
        times_s = [(m.timestamp_us - base_time_us) / 1e6 for m in msgs]
        y = y_pos[msg_type]
        color = type_colors.get(msg_type, other_color)
        ax1.scatter(times_s, [y] * len(times_s), s=5, color=color, alpha=0.7,
                   label=f"{msg_type} ({RTCM_MSG_NAMES.get(msg_type, '?')})")

    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Message Type')
    ax1.set_yticks(range(len(all_types)))
    ax1.set_yticklabels([str(t) for t in all_types], fontsize=8)
    ax1.set_title('RTCM Message Timeline by Type')
    ax1.legend(loc='upper right', fontsize=7, ncol=2)
    ax1.grid(alpha=0.3, axis='x')

    # Plot 2: Burst size distribution
    ax2 = axes[1]

    if result.bursts:
        burst_sizes = [b.total_bytes for b in result.bursts]
        ax2.hist(burst_sizes, bins=50, color='#2196F3', edgecolor='black', alpha=0.7)
        ax2.axvline(np.mean(burst_sizes), color='red', linestyle='--', linewidth=2,
                   label=f'Mean: {np.mean(burst_sizes):.0f}B')
        ax2.axvline(np.median(burst_sizes), color='green', linestyle='--', linewidth=2,
                   label=f'Median: {np.median(burst_sizes):.0f}B')
        ax2.set_xlabel('Burst Size (bytes)')
        ax2.set_ylabel('Count')
        ax2.set_title('Burst Size Distribution')
        ax2.legend()
        ax2.grid(alpha=0.3)

    # Plot 3: Inter-burst interval distribution
    ax3 = axes[2]

    if len(result.bursts) > 1:
        burst_intervals_ms = []
        for i in range(1, len(result.bursts)):
            interval = (result.bursts[i].start_timestamp_us - result.bursts[i-1].start_timestamp_us) / 1000.0
            burst_intervals_ms.append(interval)

        ax3.hist(burst_intervals_ms, bins=50, color='#4CAF50', edgecolor='black', alpha=0.7)
        ax3.axvline(np.mean(burst_intervals_ms), color='red', linestyle='--', linewidth=2,
                   label=f'Mean: {np.mean(burst_intervals_ms):.0f}ms')
        ax3.axvline(np.median(burst_intervals_ms), color='blue', linestyle='--', linewidth=2,
                   label=f'Median: {np.median(burst_intervals_ms):.0f}ms')
        ax3.set_xlabel('Inter-Burst Interval (ms)')
        ax3.set_ylabel('Count')
        ax3.set_title('Time Between Bursts')
        ax3.legend()
        ax3.grid(alpha=0.3)

    # Plot 4: Burst timeline showing size over time
    ax4 = axes[3]

    if result.bursts:
        burst_times_s = [(b.start_timestamp_us - base_time_us) / 1e6 for b in result.bursts]
        burst_sizes = [b.total_bytes for b in result.bursts]
        burst_msg_counts = [len(b.messages) for b in result.bursts]

        # Color by whether burst contains MSM messages
        msm_types = MSM4_TYPES | MSM7_TYPES
        colors = []
        for b in result.bursts:
            has_msm = any(m.msg_type in msm_types for m in b.messages)
            colors.append('#2196F3' if has_msm else '#9E9E9E')

        scatter = ax4.scatter(burst_times_s, burst_sizes, c=colors, s=30, alpha=0.7)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Burst Size (bytes)')
        ax4.set_title('Burst Size Over Time (blue=contains MSM, gray=other)')
        ax4.grid(alpha=0.3)

        # Add secondary y-axis for message count
        ax4b = ax4.twinx()
        ax4b.plot(burst_times_s, burst_msg_counts, 'r-', alpha=0.3, linewidth=1)
        ax4b.set_ylabel('Messages per Burst', color='red')
        ax4b.tick_params(axis='y', labelcolor='red')

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {output_file}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Analyze RTCM message burst patterns from PX4 gps_dump logs.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s flight.ulg                 # Analyze and show plot
  %(prog)s flight.ulg --plot out.png  # Save plot to file
  %(prog)s flight.ulg --no-plot       # Console output only
  %(prog)s flight.ulg --gap 100       # Use 100ms gap threshold for burst detection
        """
    )
    parser.add_argument('ulog_file', help='Path to ULog file')
    parser.add_argument('--plot', '-p', help='Save plot to file (PNG)')
    parser.add_argument('--no-plot', action='store_true', help='Skip plotting')
    parser.add_argument('--gap', '-g', type=float, default=50.0,
                       help='Gap threshold (ms) for burst detection (default: 50)')

    args = parser.parse_args()

    print(f"Loading ULog file: {args.ulog_file}")
    result, error = parse_rtcm_from_gps_dump(args.ulog_file)

    if error:
        print(f"Error: {error}")
        sys.exit(1)

    print(f"Detecting bursts with {args.gap}ms gap threshold...")
    detect_bursts(result, gap_threshold_ms=args.gap)

    print_report(result)

    if not args.no_plot:
        generate_plots(result, args.plot)


if __name__ == '__main__':
    main()
