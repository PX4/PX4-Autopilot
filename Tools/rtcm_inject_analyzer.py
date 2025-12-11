#!/usr/bin/env python3
"""
RTCM Injection Timing Analyzer

Parses ULog files containing PX4_INFO messages from the GPS driver and UAVCAN
modules to analyze RTCM injection timing and detect data loss in the pipeline.

Usage:
    python3 rtcm_inject_analyzer.py <ulog_file> [--plot out.png] [--no-plot]

The tool parses logged_messages from the ULog file looking for:
    - MBD RX/TX stats from uavcan/gnss.cpp and uavcannode
    - RTCM parser stats from gps.cpp
    - TX injection stats from gps.cpp (INJECT_STATS lines)

Requirements:
    pip install pyulog numpy matplotlib
"""

import argparse
import re
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, TYPE_CHECKING

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


@dataclass
class InjectStats:
    """Parsed TX injection statistics from gps.cpp."""
    timestamp_us: int
    msg_count: int
    bytes_total: int
    min_interval_ms: float
    avg_interval_ms: float
    max_interval_ms: float


@dataclass
class RtcmParserStats:
    """RTCM parser stats from gps.cpp (legacy format)."""
    timestamp_us: int
    source: str  # e.g. "uavcan:52:gps"
    rx_hwm: int
    messages_parsed: int
    crc_errors: int
    bytes_discarded: int


@dataclass
class RtcmFrameStats:
    """RTCM frame counts from unified instrumentation."""
    timestamp_us: int
    source: str  # e.g. "uavcan:116:gps"
    stage: str   # 'TX', 'RX', 'LOG', or 'INJ'
    frame_count: int
    frame_rate: float
    bytes_total: int
    byte_rate: float


@dataclass
class LatencyStats:
    """Latency statistics from gps.cpp."""
    timestamp_us: int
    min_ms: float
    avg_ms: float
    max_ms: float


@dataclass
class MbdStats:
    """MovingBaselineData stats from uavcan drivers."""
    timestamp_us: int
    source: str  # e.g. "uavcan:52:uavcannode"
    direction: str  # 'RX' or 'TX'
    msg_count: int
    msg_rate: float
    bytes_total: int


@dataclass
class AnalysisResult:
    """Aggregated analysis results."""
    inject_stats: List[InjectStats] = field(default_factory=list)
    rtcm_stats: List[RtcmParserStats] = field(default_factory=list)
    rtcm_frame_stats: List[RtcmFrameStats] = field(default_factory=list)
    latency_stats: List[LatencyStats] = field(default_factory=list)
    mbd_stats: List[MbdStats] = field(default_factory=list)
    first_timestamp_us: int = 0
    last_timestamp_us: int = 0
    duration_s: float = 0.0


def parse_ulog_file(filepath: str) -> Tuple[Optional[AnalysisResult], Optional[str]]:
    """Parse ULog file and extract statistics from logged_messages."""
    try:
        ulog = ULog(filepath)
    except Exception as e:
        return None, f"Failed to load ULog file: {e}"

    if not hasattr(ulog, 'logged_messages') or not ulog.logged_messages:
        return None, "No logged messages found in ULog file"

    result = AnalysisResult()

    # Regex patterns
    # INJECT_STATS,<timestamp_us>,<count>,<bytes>,<min_ms>,<avg_ms>,<max_ms>
    inject_stats_pattern = re.compile(
        r'INJECT_STATS,(\d+),(\d+),(\d+),([\d.]+),([\d.]+),([\d.]+)'
    )

    # TX inj: N msgs (X/s), B B (Y B/s)
    tx_inj_pattern = re.compile(
        r'TX inj:\s*(\d+)\s*msgs\s*\(([\d.]+)/s\),\s*(\d+)\s*B'
    )

    # TX interval ms: min=X avg=Y max=Z
    tx_interval_pattern = re.compile(
        r'TX interval ms:\s*min=([\d.]+)\s*avg=([\d.]+)\s*max=([\d.]+)'
    )

    # RX hwm: N, RTCM: X ok, Y crc err, Z dropped (legacy format)
    rtcm_pattern = re.compile(
        r'\[([^\]]+)\]\s*RX hwm:\s*(-?\d+),\s*RTCM:\s*(\d+)\s*ok,\s*(\d+)\s*crc err,\s*(\d+)\s*dropped'
    )

    # New unified RTCM frame stats:
    # RTCM TX: N frames (X/s), B B (Y B/s)
    # RTCM RX: N frames (X/s), B B (Y B/s)
    # RTCM LOG: N frames (X/s), B B (Y B/s)
    # RTCM INJ: N frames (X/s), B B (Y B/s)
    rtcm_frame_pattern = re.compile(
        r'(?:\[([^\]]+)\]\s*)?RTCM (TX|RX|LOG|INJ):\s*(\d+)\s*frames\s*\(([\d.]+)/s\),\s*(\d+)\s*B\s*\(([\d.]+)\s*B/s\)'
    )

    # RTCM latency ms: min=X avg=Y max=Z
    latency_pattern = re.compile(
        r'RTCM latency ms:\s*min=([\d.]+)\s*avg=([\d.]+)\s*max=([\d.]+)'
    )

    # MBD RX: N msgs (X/s), B bytes  OR  MBD TX: N msgs (X/s), B bytes
    mbd_pattern = re.compile(
        r'\[([^\]]+)\]\s*MBD (RX|TX):\s*(\d+)\s*msgs\s*\(([\d.]+)/s\),\s*(\d+)\s*bytes'
    )

    # Track pending TX injection data (human readable before INJECT_STATS)
    pending_tx_inj = {}

    for msg in ulog.logged_messages:
        text = msg.message
        timestamp = msg.timestamp

        # Try INJECT_STATS (parseable format with explicit timestamp)
        match = inject_stats_pattern.search(text)
        if match:
            stats = InjectStats(
                timestamp_us=int(match.group(1)),
                msg_count=int(match.group(2)),
                bytes_total=int(match.group(3)),
                min_interval_ms=float(match.group(4)),
                avg_interval_ms=float(match.group(5)),
                max_interval_ms=float(match.group(6)),
            )
            result.inject_stats.append(stats)
            continue

        # Try TX inj human readable (backup if INJECT_STATS not present)
        match = tx_inj_pattern.search(text)
        if match:
            pending_tx_inj['timestamp'] = timestamp
            pending_tx_inj['msg_count'] = int(match.group(1))
            pending_tx_inj['bytes_total'] = int(match.group(3))
            continue

        # Try TX interval (completes pending_tx_inj)
        match = tx_interval_pattern.search(text)
        if match and 'timestamp' in pending_tx_inj:
            stats = InjectStats(
                timestamp_us=pending_tx_inj['timestamp'],
                msg_count=pending_tx_inj.get('msg_count', 0),
                bytes_total=pending_tx_inj.get('bytes_total', 0),
                min_interval_ms=float(match.group(1)),
                avg_interval_ms=float(match.group(2)),
                max_interval_ms=float(match.group(3)),
            )
            result.inject_stats.append(stats)
            pending_tx_inj = {}
            continue

        # Try new unified RTCM frame stats (check first as it's the new format)
        match = rtcm_frame_pattern.search(text)
        if match:
            stats = RtcmFrameStats(
                timestamp_us=timestamp,
                source=match.group(1) or "unknown",
                stage=match.group(2),
                frame_count=int(match.group(3)),
                frame_rate=float(match.group(4)),
                bytes_total=int(match.group(5)),
                byte_rate=float(match.group(6)),
            )
            result.rtcm_frame_stats.append(stats)
            continue

        # Try latency stats
        match = latency_pattern.search(text)
        if match:
            stats = LatencyStats(
                timestamp_us=timestamp,
                min_ms=float(match.group(1)),
                avg_ms=float(match.group(2)),
                max_ms=float(match.group(3)),
            )
            result.latency_stats.append(stats)
            continue

        # Try legacy RTCM parser stats
        match = rtcm_pattern.search(text)
        if match:
            stats = RtcmParserStats(
                timestamp_us=timestamp,
                source=match.group(1),
                rx_hwm=int(match.group(2)),
                messages_parsed=int(match.group(3)),
                crc_errors=int(match.group(4)),
                bytes_discarded=int(match.group(5)),
            )
            result.rtcm_stats.append(stats)
            continue

        # Try MBD stats
        match = mbd_pattern.search(text)
        if match:
            stats = MbdStats(
                timestamp_us=timestamp,
                source=match.group(1),
                direction=match.group(2),
                msg_count=int(match.group(3)),
                msg_rate=float(match.group(4)),
                bytes_total=int(match.group(5)),
            )
            result.mbd_stats.append(stats)
            continue

    if not result.inject_stats and not result.rtcm_stats and not result.rtcm_frame_stats and not result.mbd_stats:
        return None, "No RTCM/MBD statistics found in logged messages"

    # Calculate duration
    all_timestamps = []
    if result.inject_stats:
        all_timestamps.extend([s.timestamp_us for s in result.inject_stats])
    if result.rtcm_stats:
        all_timestamps.extend([s.timestamp_us for s in result.rtcm_stats])
    if result.rtcm_frame_stats:
        all_timestamps.extend([s.timestamp_us for s in result.rtcm_frame_stats])
    if result.latency_stats:
        all_timestamps.extend([s.timestamp_us for s in result.latency_stats])
    if result.mbd_stats:
        all_timestamps.extend([s.timestamp_us for s in result.mbd_stats])

    if all_timestamps:
        result.first_timestamp_us = min(all_timestamps)
        result.last_timestamp_us = max(all_timestamps)
        result.duration_s = (result.last_timestamp_us - result.first_timestamp_us) / 1_000_000

    return result, None


def print_report(result: AnalysisResult):
    """Print analysis report to console."""
    print("\n" + "=" * 70)
    print("RTCM INJECTION PIPELINE ANALYSIS")
    print("=" * 70)
    print(f"  Duration: {result.duration_s:.1f} seconds")

    # Group MBD stats by source and direction
    mbd_by_source: Dict[str, Dict[str, List[MbdStats]]] = {}
    for s in result.mbd_stats:
        if s.source not in mbd_by_source:
            mbd_by_source[s.source] = {'RX': [], 'TX': []}
        mbd_by_source[s.source][s.direction].append(s)

    # Group RTCM stats by source
    rtcm_by_source: Dict[str, List[RtcmParserStats]] = {}
    for s in result.rtcm_stats:
        if s.source not in rtcm_by_source:
            rtcm_by_source[s.source] = []
        rtcm_by_source[s.source].append(s)

    # Print MBD stats
    if result.mbd_stats:
        print(f"\n{'MOVING BASELINE DATA (CAN)':=^70}")
        for source, directions in sorted(mbd_by_source.items()):
            print(f"\n  [{source}]")
            for direction in ['RX', 'TX']:
                stats_list = directions[direction]
                if stats_list:
                    # Get cumulative totals (last entry has running totals)
                    last = stats_list[-1]
                    avg_rate = np.mean([s.msg_rate for s in stats_list])
                    print(f"    {direction}: {last.msg_count:,} msgs, {last.bytes_total:,} bytes, "
                          f"avg {avg_rate:.1f} msg/s")

    # Print RTCM parser stats
    if result.rtcm_stats:
        print(f"\n{'RTCM PARSER (GPS DRIVER)':=^70}")
        for source, stats_list in sorted(rtcm_by_source.items()):
            print(f"\n  [{source}]")
            # Get cumulative totals (values are cumulative in the driver)
            last = stats_list[-1]
            max_hwm = max(s.rx_hwm for s in stats_list)
            total_crc = last.crc_errors
            total_discard = last.bytes_discarded

            print(f"    Messages parsed: {last.messages_parsed:,}")
            print(f"    CRC errors: {total_crc:,}")
            print(f"    Bytes discarded: {total_discard:,}")
            print(f"    Max RX buffer HWM: {max_hwm}")

            if total_crc > 0:
                print(f"    WARNING: {total_crc} CRC errors!")
            if total_discard > 0:
                print(f"    WARNING: {total_discard} bytes discarded!")

    # Print TX injection stats
    if result.inject_stats:
        print(f"\n{'TX SERIAL INJECTION':=^70}")

        total_msgs = sum(s.msg_count for s in result.inject_stats)
        total_bytes = sum(s.bytes_total for s in result.inject_stats)
        all_min = [s.min_interval_ms for s in result.inject_stats if s.min_interval_ms > 0]
        all_avg = [s.avg_interval_ms for s in result.inject_stats if s.avg_interval_ms > 0]
        all_max = [s.max_interval_ms for s in result.inject_stats if s.max_interval_ms > 0]

        print(f"\n  Total messages injected: {total_msgs:,}")
        print(f"  Total bytes injected: {total_bytes:,}")
        if result.duration_s > 0:
            print(f"  Average rate: {total_msgs / result.duration_s:.1f} msg/s, "
                  f"{total_bytes / result.duration_s:.0f} B/s")

        if all_avg:
            print(f"\n  Interval timing (ms):")
            print(f"    Min of mins:    {min(all_min):.1f}")
            print(f"    Avg of avgs:    {np.mean(all_avg):.1f}")
            print(f"    Max of maxs:    {max(all_max):.1f}")
            print(f"    StdDev of avgs: {np.std(all_avg):.1f}")

            if max(all_max) > 2 * np.mean(all_avg):
                print(f"\n    WARNING: High jitter detected!")
                print(f"      Max interval ({max(all_max):.1f}ms) > 2x average ({np.mean(all_avg):.1f}ms)")

    # NEW: RTCM Frame Pipeline (unified instrumentation)
    if result.rtcm_frame_stats:
        print(f"\n{'RTCM FRAME PIPELINE':=^70}")

        # Group by stage
        frames_by_stage: Dict[str, List[RtcmFrameStats]] = {}
        for s in result.rtcm_frame_stats:
            if s.stage not in frames_by_stage:
                frames_by_stage[s.stage] = []
            frames_by_stage[s.stage].append(s)

        # Get last (cumulative) values for each stage
        stage_totals: Dict[str, Tuple[int, int]] = {}  # stage -> (frames, bytes)
        for stage, stats_list in frames_by_stage.items():
            if stats_list:
                last = stats_list[-1]
                stage_totals[stage] = (last.frame_count, last.bytes_total)

        # Print in pipeline order
        stage_order = ['TX', 'LOG', 'RX', 'INJ']
        stage_labels = {
            'TX': 'MovingBase TX',
            'LOG': 'FC Logged',
            'RX': 'Rover RX (CAN)',
            'INJ': 'Rover INJ (serial)',
        }

        print(f"\n  {'Stage':<25} {'Frames':>10}   {'Bytes':>12}   {'Loss':>8}")
        print(f"  {'-'*60}")

        baseline_frames = None
        for stage in stage_order:
            if stage in stage_totals:
                frames, bytes_total = stage_totals[stage]
                if baseline_frames is None:
                    baseline_frames = frames
                    loss_str = '-'
                elif baseline_frames > 0:
                    loss_pct = 100.0 * (baseline_frames - frames) / baseline_frames
                    loss_str = f"{loss_pct:.1f}%"
                else:
                    loss_str = '-'

                label = stage_labels.get(stage, stage)
                print(f"  {label:<25} {frames:>10,}   {bytes_total:>12,}   {loss_str:>8}")

        # Warn if any loss detected
        if baseline_frames is not None:
            for stage in stage_order:
                if stage in stage_totals and stage != 'TX':
                    frames, _ = stage_totals[stage]
                    if frames < baseline_frames * 0.99:  # Allow 1% tolerance
                        print(f"\n  WARNING: Frame loss detected at {stage_labels.get(stage, stage)}!")
                        print(f"    Expected: {baseline_frames}, Got: {frames}, Lost: {baseline_frames - frames}")

    # Latency stats
    if result.latency_stats:
        print(f"\n{'LATENCY (CAN RX → Serial TX)':=^70}")
        all_min = [s.min_ms for s in result.latency_stats]
        all_avg = [s.avg_ms for s in result.latency_stats]
        all_max = [s.max_ms for s in result.latency_stats]

        print(f"\n  Min:  {min(all_min):.1f} ms")
        print(f"  Avg:  {np.mean(all_avg):.1f} ms")
        print(f"  Max:  {max(all_max):.1f} ms")

        if max(all_max) > 100:
            print(f"\n  WARNING: High latency spikes detected (>{max(all_max):.0f}ms)!")

    # Legacy pipeline comparison (MBD messages - keep for secondary validation)
    print(f"\n{'MBD MESSAGE COMPARISON (secondary)':=^70}")

    # Sum up MBD RX from all sources
    total_mbd_rx_msgs = 0
    total_mbd_rx_bytes = 0
    for source, directions in mbd_by_source.items():
        if directions['RX']:
            last = directions['RX'][-1]
            total_mbd_rx_msgs += last.msg_count
            total_mbd_rx_bytes += last.bytes_total

    # Sum up RTCM parsed (legacy)
    total_rtcm_parsed = 0
    for source, stats_list in rtcm_by_source.items():
        if stats_list:
            total_rtcm_parsed += stats_list[-1].messages_parsed

    # TX inject totals
    total_tx_msgs = sum(s.msg_count for s in result.inject_stats) if result.inject_stats else 0
    total_tx_bytes = sum(s.bytes_total for s in result.inject_stats) if result.inject_stats else 0

    print(f"\n  Stage                    Messages        Bytes")
    print(f"  {'-'*50}")
    print(f"  MBD RX (CAN in)          {total_mbd_rx_msgs:>10,}   {total_mbd_rx_bytes:>12,}")
    if total_rtcm_parsed > 0:
        print(f"  RTCM parsed (legacy)     {total_rtcm_parsed:>10,}   {'--':>12}")
    if total_tx_msgs > 0:
        print(f"  TX inject (serial out)   {total_tx_msgs:>10,}   {total_tx_bytes:>12,}")

    print("\n" + "=" * 70)


def generate_plots(result: AnalysisResult, output_file: Optional[str] = None):
    """Generate visualization plots."""
    try:
        import matplotlib
        if output_file:
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("Warning: matplotlib not available, skipping plots")
        return

    # Determine subplots needed
    has_inject = len(result.inject_stats) > 0
    has_rtcm = len(result.rtcm_stats) > 0
    has_mbd = len(result.mbd_stats) > 0

    num_plots = sum([has_mbd, has_rtcm, has_inject * 2])  # inject gets 2 plots
    if num_plots == 0:
        print("No data to plot")
        return

    fig, axes = plt.subplots(num_plots, 1, figsize=(14, 3 * num_plots))
    if num_plots == 1:
        axes = [axes]

    plot_idx = 0
    base_time = result.first_timestamp_us

    # Plot MBD rates
    if has_mbd:
        ax = axes[plot_idx]

        # Group by source and direction
        for s in result.mbd_stats:
            key = f"{s.source} {s.direction}"
            # Collect for this source
            pass

        # Simpler approach: just plot all
        rx_stats = [s for s in result.mbd_stats if s.direction == 'RX']
        tx_stats = [s for s in result.mbd_stats if s.direction == 'TX']

        if rx_stats:
            # Group by source
            sources = set(s.source for s in rx_stats)
            colors = ['#2196F3', '#03A9F4', '#00BCD4']
            for i, src in enumerate(sorted(sources)):
                src_stats = [s for s in rx_stats if s.source == src]
                times = [(s.timestamp_us - base_time) / 1e6 for s in src_stats]
                rates = [s.msg_rate for s in src_stats]
                ax.plot(times, rates, 'o-', markersize=3, color=colors[i % len(colors)],
                        label=f'{src} RX')

        if tx_stats:
            sources = set(s.source for s in tx_stats)
            colors = ['#F44336', '#E91E63', '#9C27B0']
            for i, src in enumerate(sorted(sources)):
                src_stats = [s for s in tx_stats if s.source == src]
                times = [(s.timestamp_us - base_time) / 1e6 for s in src_stats]
                rates = [s.msg_rate for s in src_stats]
                ax.plot(times, rates, 's-', markersize=3, color=colors[i % len(colors)],
                        label=f'{src} TX')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Message Rate (msg/s)')
        ax.set_title('MovingBaselineData Rate over Time')
        ax.grid(alpha=0.3)
        ax.legend(loc='upper right', fontsize=7)
        plot_idx += 1

    # Plot RTCM parser stats
    if has_rtcm:
        ax = axes[plot_idx]

        sources = set(s.source for s in result.rtcm_stats)
        colors = ['#4CAF50', '#8BC34A', '#CDDC39']

        for i, src in enumerate(sorted(sources)):
            src_stats = [s for s in result.rtcm_stats if s.source == src]
            times = [(s.timestamp_us - base_time) / 1e6 for s in src_stats]

            # Calculate per-interval parsed (diff of cumulative)
            parsed_per_interval = []
            for j in range(len(src_stats)):
                if j == 0:
                    parsed_per_interval.append(src_stats[j].messages_parsed)
                else:
                    diff = src_stats[j].messages_parsed - src_stats[j-1].messages_parsed
                    parsed_per_interval.append(max(0, diff))

            ax.plot(times, parsed_per_interval, 'o-', markersize=3,
                    color=colors[i % len(colors)], label=f'{src}')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('RTCM Messages Parsed (per interval)')
        ax.set_title('RTCM Parser Throughput')
        ax.grid(alpha=0.3)
        ax.legend(loc='upper right', fontsize=7)
        plot_idx += 1

    # Plot TX injection stats
    if has_inject:
        times_s = [(s.timestamp_us - base_time) / 1e6 for s in result.inject_stats]

        # Plot 1: Message/byte rate
        ax = axes[plot_idx]
        msg_counts = [s.msg_count for s in result.inject_stats]
        bytes_counts = [s.bytes_total for s in result.inject_stats]

        ax.bar(times_s, msg_counts, width=4, alpha=0.7, label='Messages', color='#2196F3')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Messages per interval')
        ax.set_title('TX Serial Injection Rate (per 5s interval)')
        ax.grid(alpha=0.3)

        ax2 = ax.twinx()
        ax2.plot(times_s, bytes_counts, 'r-', alpha=0.7, linewidth=2, label='Bytes')
        ax2.set_ylabel('Bytes per interval', color='red')
        ax2.tick_params(axis='y', labelcolor='red')

        plot_idx += 1

        # Plot 2: Interval timing
        ax = axes[plot_idx]
        min_intervals = [s.min_interval_ms for s in result.inject_stats]
        avg_intervals = [s.avg_interval_ms for s in result.inject_stats]
        max_intervals = [s.max_interval_ms for s in result.inject_stats]

        ax.fill_between(times_s, min_intervals, max_intervals, alpha=0.3, color='#4CAF50',
                        label='Min-Max range')
        ax.plot(times_s, avg_intervals, 'g-', linewidth=2, label='Average')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Interval (ms)')
        ax.set_title('TX Injection Timing Jitter')
        ax.grid(alpha=0.3)
        ax.legend(loc='upper right')

        # Mark high jitter points
        if avg_intervals:
            mean_avg = np.mean(avg_intervals)
            for i, (t, mx) in enumerate(zip(times_s, max_intervals)):
                if mx > 2 * mean_avg:
                    ax.scatter([t], [mx], color='red', s=50, marker='x', zorder=5)

        plot_idx += 1

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {output_file}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Analyze RTCM injection pipeline from PX4 ULog files.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s flight.ulg                 # Analyze and show plot
  %(prog)s flight.ulg --plot out.png  # Save plot to file
  %(prog)s flight.ulg --no-plot       # Console output only

Parses logged_messages from ULog for:
  - MBD RX/TX stats (CAN MovingBaselineData)
  - RTCM parser stats (gps.cpp)
  - TX injection stats (gps.cpp serial writes)
        """
    )
    parser.add_argument('ulog_file', help='Path to ULog file')
    parser.add_argument('--plot', '-p', help='Save plot to file (PNG)')
    parser.add_argument('--no-plot', action='store_true', help='Skip plotting')

    args = parser.parse_args()

    print(f"Loading ULog file: {args.ulog_file}")
    result, error = parse_ulog_file(args.ulog_file)

    if error:
        print(f"Error: {error}")
        sys.exit(1)

    print_report(result)

    if not args.no_plot:
        generate_plots(result, args.plot)


if __name__ == '__main__':
    main()
