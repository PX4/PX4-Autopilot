#!/usr/bin/env python3
"""
BDShot Timing Jitter Analyzer

Analyzes BDShot (Bidirectional DShot) signal timing from Saleae Logic Analyzer
CSV exports. Measures scheduling consistency and jitter in PX4 flight controller
DShot output timing.

Usage:
    python bdshot_analyzer.py capture.csv
    python bdshot_analyzer.py capture.csv --dshot-rate 300 --loop-rate 800
    python bdshot_analyzer.py capture.csv --channel-a 0 --channel-b 1

Arguments:
    input_file          Path to Saleae CSV export
    --dshot-rate        DShot variant: 150/300/600/1200 (default: 300)
    --loop-rate         Expected loop rate in Hz (default: 800)
    --channel-a         CSV column index for first channel (default: 0)
    --channel-b         CSV column index for second channel (default: 1)
    --no-plots          Disable histogram display
    --verbose           Enable verbose debugging output

Capture Setup:
    1. Connect Saleae to two BDShot signals from different timers
       (e.g., Motor 1 from Timer1, Motor 5 from Timer2)
    2. Sample rate: 24+ MS/s for DShot300
    3. Export as CSV: File -> Export Raw Data -> CSV

Interpreting Results:
    - Frame Intervals: Should cluster tightly around expected loop period
    - Inter-Channel Gap: Measures sequential DMA scheduling overhead
    - Jitter: Small std dev is typical for well-behaved systems

Dependencies: numpy, matplotlib
"""

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

# Try to import matplotlib, but allow running without it
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


# DShot bit periods in microseconds
DSHOT_BIT_PERIODS = {
    150: 6.67,
    300: 3.33,
    600: 1.67,
    1200: 0.83,
}


@dataclass
class BDShotTransaction:
    """Represents a single BDShot transaction (command + response)."""
    start: float   # Timestamp of first falling edge (seconds)
    end: float     # Timestamp of final rising edge before idle (seconds)

    @property
    def duration_us(self) -> float:
        """Transaction duration in microseconds."""
        return (self.end - self.start) * 1_000_000


@dataclass
class ChannelStats:
    """Statistics for a single channel's frame intervals."""
    channel_name: str
    count: int
    min_us: float
    max_us: float
    mean_us: float
    std_us: float
    expected_us: float

    @property
    def min_jitter_us(self) -> float:
        return self.min_us - self.expected_us

    @property
    def max_jitter_us(self) -> float:
        return self.max_us - self.expected_us

    @property
    def mean_jitter_us(self) -> float:
        return self.mean_us - self.expected_us


@dataclass
class GapStats:
    """Statistics for inter-channel gaps."""
    count: int
    min_us: float
    max_us: float
    mean_us: float
    std_us: float


def parse_csv(filepath: Path, channel_a_idx: int, channel_b_idx: int,
              verbose: bool = False) -> Tuple[List[Tuple[float, int]], List[Tuple[float, int]]]:
    """
    Parse Saleae CSV export and extract edges for each channel.

    Returns:
        Tuple of (edges_a, edges_b) where each is a list of (timestamp, new_state) tuples.
    """
    edges_a = []
    edges_b = []

    prev_state_a = None
    prev_state_b = None

    with open(filepath, 'r', newline='') as f:
        reader = csv.reader(f)

        # Read header
        header = next(reader)
        if verbose:
            print(f"CSV Header: {header}")

        # Determine column indices
        # Header format: "Time [s],Channel0,Channel1" or "Time [s],CH1,CH5" etc.
        time_col = 0
        ch_a_col = channel_a_idx + 1  # +1 because time is column 0
        ch_b_col = channel_b_idx + 1

        if ch_a_col >= len(header) or ch_b_col >= len(header):
            raise ValueError(
                f"Channel indices out of range. CSV has {len(header)-1} channels. "
                f"Requested channel_a={channel_a_idx}, channel_b={channel_b_idx}"
            )

        if verbose:
            print(f"Using columns: time={time_col}, ch_a={ch_a_col} ({header[ch_a_col]}), "
                  f"ch_b={ch_b_col} ({header[ch_b_col]})")

        for row_num, row in enumerate(reader, start=2):
            try:
                timestamp = float(row[time_col])
                state_a = int(row[ch_a_col])
                state_b = int(row[ch_b_col])

                # Record edges (state changes)
                if prev_state_a is not None and state_a != prev_state_a:
                    edges_a.append((timestamp, state_a))
                elif prev_state_a is None:
                    # First row - record initial state as an edge if LOW
                    if state_a == 0:
                        edges_a.append((timestamp, state_a))

                if prev_state_b is not None and state_b != prev_state_b:
                    edges_b.append((timestamp, state_b))
                elif prev_state_b is None:
                    if state_b == 0:
                        edges_b.append((timestamp, state_b))

                prev_state_a = state_a
                prev_state_b = state_b

            except (ValueError, IndexError) as e:
                if verbose:
                    print(f"Warning: Skipping malformed row {row_num}: {e}")
                continue

    if verbose:
        print(f"Parsed {len(edges_a)} edges for channel A, {len(edges_b)} edges for channel B")

    return edges_a, edges_b


def detect_transactions(edges: List[Tuple[float, int]], idle_threshold_us: float,
                        verbose: bool = False) -> List[BDShotTransaction]:
    """
    Detect BDShot transactions from edge list.

    A transaction starts with a falling edge after an idle period and ends
    when the line returns HIGH and stays HIGH for > idle_threshold_us.

    Args:
        edges: List of (timestamp, new_state) tuples
        idle_threshold_us: Minimum idle time in microseconds to consider transaction complete
        verbose: Print debug information

    Returns:
        List of BDShotTransaction objects
    """
    if not edges:
        return []

    idle_threshold_s = idle_threshold_us / 1_000_000
    transactions = []
    tx_start = None
    last_rising_edge = None

    for i, (timestamp, state) in enumerate(edges):
        if state == 0:  # Falling edge
            if tx_start is None:
                # Check if this is after an idle period
                if last_rising_edge is None or (timestamp - last_rising_edge) > idle_threshold_s:
                    tx_start = timestamp
        else:  # Rising edge (state == 1)
            last_rising_edge = timestamp

            # Check if next edge is far enough away to consider this transaction complete
            if tx_start is not None:
                # Look ahead to see if there's a gap
                if i + 1 < len(edges):
                    next_timestamp = edges[i + 1][0]
                    gap = next_timestamp - timestamp
                    if gap > idle_threshold_s:
                        # Transaction complete
                        transactions.append(BDShotTransaction(start=tx_start, end=timestamp))
                        tx_start = None
                else:
                    # Last edge in file - complete the transaction
                    transactions.append(BDShotTransaction(start=tx_start, end=timestamp))
                    tx_start = None

    if verbose:
        print(f"Detected {len(transactions)} transactions")
        if transactions:
            durations = [tx.duration_us for tx in transactions]
            print(f"  Duration range: {min(durations):.1f} - {max(durations):.1f} us")

    return transactions


def compute_channel_stats(transactions: List[BDShotTransaction], channel_name: str,
                          expected_interval_us: float) -> Tuple[ChannelStats, np.ndarray]:
    """
    Compute frame interval statistics for a channel.

    Returns:
        Tuple of (ChannelStats, intervals_array_us)
    """
    if len(transactions) < 2:
        raise ValueError(f"Need at least 2 transactions to compute intervals, got {len(transactions)}")

    # Compute intervals between transaction starts
    starts = np.array([tx.start for tx in transactions])
    intervals_s = np.diff(starts)
    intervals_us = intervals_s * 1_000_000

    stats = ChannelStats(
        channel_name=channel_name,
        count=len(intervals_us),
        min_us=float(np.min(intervals_us)),
        max_us=float(np.max(intervals_us)),
        mean_us=float(np.mean(intervals_us)),
        std_us=float(np.std(intervals_us)),
        expected_us=expected_interval_us,
    )

    return stats, intervals_us


def compute_inter_channel_gaps(transactions_a: List[BDShotTransaction],
                               transactions_b: List[BDShotTransaction],
                               expected_interval_us: float,
                               verbose: bool = False) -> Tuple[GapStats, np.ndarray]:
    """
    Compute inter-channel gap statistics (CH_A end -> CH_B start).

    Args:
        transactions_a: Channel A transactions
        transactions_b: Channel B transactions
        expected_interval_us: Expected loop interval (used to filter out wrong matches)
        verbose: Print debug information

    Returns:
        Tuple of (GapStats, gaps_array_us)
    """
    gaps = []

    # Maximum reasonable gap - should be much less than loop period
    max_reasonable_gap_us = expected_interval_us * 0.5

    b_idx = 0
    matched = 0
    skipped = 0

    for tx_a in transactions_a:
        # Find first CH_B transaction that starts after this CH_A ends
        while b_idx < len(transactions_b) and transactions_b[b_idx].start <= tx_a.end:
            b_idx += 1

        if b_idx >= len(transactions_b):
            break

        tx_b = transactions_b[b_idx]
        gap_us = (tx_b.start - tx_a.end) * 1_000_000

        # Sanity check: gap should be positive and less than half the loop period
        if 0 < gap_us < max_reasonable_gap_us:
            gaps.append(gap_us)
            matched += 1
        else:
            skipped += 1

    if verbose:
        print(f"Inter-channel gap matching: {matched} matched, {skipped} skipped")

    if not gaps:
        raise ValueError("No valid inter-channel gaps found")

    gaps_arr = np.array(gaps)

    stats = GapStats(
        count=len(gaps_arr),
        min_us=float(np.min(gaps_arr)),
        max_us=float(np.max(gaps_arr)),
        mean_us=float(np.mean(gaps_arr)),
        std_us=float(np.std(gaps_arr)),
    )

    return stats, gaps_arr


def _plot_histogram(ax, data: np.ndarray, title: str, xlabel: str,
                    expected_value: Optional[float] = None,
                    stats_text: Optional[str] = None):
    """Plot a histogram on the given axes."""
    # Determine bin count based on data range
    data_range = np.max(data) - np.min(data)
    if data_range > 0:
        bin_count = min(100, max(20, int(len(data) / 100)))
    else:
        bin_count = 20

    ax.hist(data, bins=bin_count, edgecolor='black', alpha=0.7)

    ax.set_title(title, fontsize=12, fontweight='bold')
    ax.set_xlabel(xlabel, fontsize=10)
    ax.set_ylabel('Count', fontsize=10)

    # Add vertical line for expected/mean value
    if expected_value is not None:
        ax.axvline(expected_value, color='red', linestyle='--', linewidth=2,
                   label=f'Expected: {expected_value:.2f} us')

    mean_val = np.mean(data)
    ax.axvline(mean_val, color='green', linestyle='-', linewidth=2,
               label=f'Mean: {mean_val:.2f} us')

    # Add statistics text box
    if stats_text:
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        ax.text(0.98, 0.97, stats_text, transform=ax.transAxes, fontsize=9,
                verticalalignment='top', horizontalalignment='right', bbox=props,
                family='monospace')

    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True, alpha=0.3)


def generate_combined_histogram(
    intervals_a: np.ndarray, intervals_b: np.ndarray, gaps: np.ndarray,
    stats_a: 'ChannelStats', stats_b: 'ChannelStats', gap_stats: 'GapStats',
    expected_interval_us: float, dshot_rate: int
):
    """Generate a combined figure with all histograms."""
    if not HAS_MATPLOTLIB:
        return None

    # Create figure with GridSpec: 2 rows, 2 columns
    # Top row: Channel A (left), Channel B (right)
    # Bottom row: Inter-channel gap (spans both columns)
    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1], hspace=0.3, wspace=0.25)

    ax_a = fig.add_subplot(gs[0, 0])
    ax_b = fig.add_subplot(gs[0, 1])
    ax_gap = fig.add_subplot(gs[1, :])  # Span both columns

    # Channel A intervals
    _plot_histogram(
        ax_a, intervals_a,
        f"Channel A Frame Intervals",
        "Interval (us)",
        expected_value=expected_interval_us,
        stats_text=format_stats_text(
            stats_a.min_us, stats_a.max_us, stats_a.mean_us,
            stats_a.std_us, stats_a.count
        )
    )

    # Channel B intervals
    _plot_histogram(
        ax_b, intervals_b,
        f"Channel B Frame Intervals",
        "Interval (us)",
        expected_value=expected_interval_us,
        stats_text=format_stats_text(
            stats_b.min_us, stats_b.max_us, stats_b.mean_us,
            stats_b.std_us, stats_b.count
        )
    )

    # Inter-channel gaps
    if len(gaps) > 0:
        _plot_histogram(
            ax_gap, gaps,
            "Inter-Channel Gap (CH_A End -> CH_B Start)",
            "Gap (us)",
            stats_text=format_stats_text(
                gap_stats.min_us, gap_stats.max_us, gap_stats.mean_us,
                gap_stats.std_us, gap_stats.count
            )
        )
    else:
        ax_gap.text(0.5, 0.5, "No inter-channel gap data available",
                    ha='center', va='center', transform=ax_gap.transAxes)
        ax_gap.set_title("Inter-Channel Gap (CH_A End -> CH_B Start)")

    # Add overall title
    fig.suptitle(f"BDShot Timing Analysis (DShot{dshot_rate})",
                 fontsize=14, fontweight='bold', y=0.98)

    return fig


def format_stats_text(min_val: float, max_val: float, mean_val: float,
                      std_val: float, count: int) -> str:
    """Format statistics as a text block for histogram annotation."""
    return (f"Count: {count}\n"
            f"Min:   {min_val:.2f} us\n"
            f"Max:   {max_val:.2f} us\n"
            f"Mean:  {mean_val:.2f} us\n"
            f"Std:   {std_val:.2f} us")


def print_report(input_file: Path, dshot_rate: int, loop_rate: float,
                 capture_duration: float, tx_count_a: int, tx_count_b: int,
                 stats_a: ChannelStats, stats_b: ChannelStats,
                 gap_stats: GapStats):
    """Print the analysis report to console."""
    bit_period = DSHOT_BIT_PERIODS.get(dshot_rate, 3.33)
    expected_interval = 1_000_000 / loop_rate

    print()
    print("BDShot Timing Analysis")
    print("=" * 60)
    print(f"Input: {input_file}")
    print(f"DShot Rate: {dshot_rate} (bit period: {bit_period:.2f} us)")
    print(f"Expected Loop Rate: {loop_rate:.0f} Hz ({expected_interval:.2f} us interval)")
    print(f"Capture Duration: {capture_duration:.2f} s")
    print(f"Transactions Detected: CH_A={tx_count_a}, CH_B={tx_count_b}")
    print()

    # Channel A stats
    print(f"Channel A Frame Intervals")
    print("-" * 40)
    print(f"  Count:     {stats_a.count}")
    print(f"  Min:       {stats_a.min_us:.2f} us  ({stats_a.min_jitter_us:+.2f} us from nominal)")
    print(f"  Max:       {stats_a.max_us:.2f} us  ({stats_a.max_jitter_us:+.2f} us from nominal)")
    print(f"  Mean:      {stats_a.mean_us:.2f} us  ({stats_a.mean_jitter_us:+.2f} us from nominal)")
    print(f"  Std Dev:   {stats_a.std_us:.2f} us")
    print()

    # Channel B stats
    print(f"Channel B Frame Intervals")
    print("-" * 40)
    print(f"  Count:     {stats_b.count}")
    print(f"  Min:       {stats_b.min_us:.2f} us  ({stats_b.min_jitter_us:+.2f} us from nominal)")
    print(f"  Max:       {stats_b.max_us:.2f} us  ({stats_b.max_jitter_us:+.2f} us from nominal)")
    print(f"  Mean:      {stats_b.mean_us:.2f} us  ({stats_b.mean_jitter_us:+.2f} us from nominal)")
    print(f"  Std Dev:   {stats_b.std_us:.2f} us")
    print()

    # Inter-channel gap stats
    print(f"Inter-Channel Gap (CH_A End -> CH_B Start)")
    print("-" * 40)
    print(f"  Count:     {gap_stats.count}")
    print(f"  Min:       {gap_stats.min_us:.2f} us")
    print(f"  Max:       {gap_stats.max_us:.2f} us")
    print(f"  Mean:      {gap_stats.mean_us:.2f} us")
    print(f"  Std Dev:   {gap_stats.std_us:.2f} us")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Analyze BDShot timing from Saleae Logic Analyzer CSV exports",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s capture.csv
  %(prog)s capture.csv --dshot-rate 300 --loop-rate 800
  %(prog)s capture.csv --channel-a 0 --channel-b 1
        """
    )

    parser.add_argument('input_file', type=Path,
                        help='Path to Saleae CSV export')
    parser.add_argument('--dshot-rate', type=int, default=300,
                        choices=[150, 300, 600, 1200],
                        help='DShot variant (default: 300)')
    parser.add_argument('--loop-rate', type=float, default=800,
                        help='Expected loop rate in Hz (default: 800)')
    parser.add_argument('--channel-a', type=int, default=0,
                        help='CSV column index for first channel (default: 0)')
    parser.add_argument('--channel-b', type=int, default=1,
                        help='CSV column index for second channel (default: 1)')
    parser.add_argument('--no-plots', action='store_true',
                        help='Disable histogram display')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Enable verbose output for debugging')

    args = parser.parse_args()

    # Validate input file
    if not args.input_file.exists():
        print(f"Error: Input file not found: {args.input_file}", file=sys.stderr)
        sys.exit(1)

    # Calculate timing parameters
    bit_period_us = DSHOT_BIT_PERIODS.get(args.dshot_rate, 3.33)
    # Idle threshold must be longer than the turnaround gap (~25-30us) between
    # command and response frames, but shorter than the inter-transaction gap.
    # Use 50us minimum to handle all DShot rates safely.
    idle_threshold_us = max(50.0, bit_period_us * 15)
    expected_interval_us = 1_000_000 / args.loop_rate

    if args.verbose:
        print(f"Bit period: {bit_period_us:.2f} us")
        print(f"Idle threshold: {idle_threshold_us:.2f} us")
        print(f"Expected interval: {expected_interval_us:.2f} us")
        print()

    # Parse CSV
    print(f"Parsing {args.input_file}...")
    try:
        edges_a, edges_b = parse_csv(
            args.input_file, args.channel_a, args.channel_b, args.verbose
        )
    except Exception as e:
        print(f"Error parsing CSV: {e}", file=sys.stderr)
        sys.exit(1)

    if not edges_a or not edges_b:
        print("Error: No edges found in one or both channels", file=sys.stderr)
        sys.exit(1)

    # Detect transactions
    print("Detecting transactions...")
    transactions_a = detect_transactions(edges_a, idle_threshold_us, args.verbose)
    transactions_b = detect_transactions(edges_b, idle_threshold_us, args.verbose)

    if len(transactions_a) < 2:
        print(f"Error: Insufficient transactions in channel A ({len(transactions_a)})",
              file=sys.stderr)
        sys.exit(1)

    if len(transactions_b) < 2:
        print(f"Error: Insufficient transactions in channel B ({len(transactions_b)})",
              file=sys.stderr)
        sys.exit(1)

    # Calculate capture duration
    all_starts = [tx.start for tx in transactions_a] + [tx.start for tx in transactions_b]
    all_ends = [tx.end for tx in transactions_a] + [tx.end for tx in transactions_b]
    capture_duration = max(all_ends) - min(all_starts)

    # Compute statistics
    print("Computing statistics...")
    stats_a, intervals_a = compute_channel_stats(
        transactions_a, "Channel A", expected_interval_us
    )
    stats_b, intervals_b = compute_channel_stats(
        transactions_b, "Channel B", expected_interval_us
    )

    try:
        gap_stats, gaps = compute_inter_channel_gaps(
            transactions_a, transactions_b, expected_interval_us, args.verbose
        )
    except ValueError as e:
        print(f"Warning: {e}", file=sys.stderr)
        gap_stats = GapStats(count=0, min_us=0, max_us=0, mean_us=0, std_us=0)
        gaps = np.array([])

    # Print report
    print_report(
        args.input_file, args.dshot_rate, args.loop_rate,
        capture_duration, len(transactions_a), len(transactions_b),
        stats_a, stats_b, gap_stats
    )

    # Generate and show histogram
    if not args.no_plots and HAS_MATPLOTLIB:
        generate_combined_histogram(
            intervals_a, intervals_b, gaps,
            stats_a, stats_b, gap_stats,
            expected_interval_us, args.dshot_rate
        )
        plt.show()


if __name__ == '__main__':
    main()
