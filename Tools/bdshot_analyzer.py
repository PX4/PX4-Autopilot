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


def parse_csv(filepath: Path, channel_a_idx: int, channel_b_idx: int, verbose: bool = False):
    """
    Parse Saleae CSV export and extract edges for each channel.

    Returns:
        Tuple of (edges_a, edges_b) where each is a list of (timestamp, new_state) tuples.
    """
    edges_a, edges_b = [], []
    prev_a, prev_b = None, None
    ch_a_col, ch_b_col = channel_a_idx + 1, channel_b_idx + 1  # +1 for time column

    with open(filepath, 'r', newline='') as f:
        reader = csv.reader(f)
        header = next(reader)

        if verbose:
            print(f"CSV Header: {header}")
            print(f"Using columns: time=0, ch_a={ch_a_col} ({header[ch_a_col]}), "
                  f"ch_b={ch_b_col} ({header[ch_b_col]})")

        if ch_a_col >= len(header) or ch_b_col >= len(header):
            raise ValueError(
                f"Channel indices out of range. CSV has {len(header)-1} channels. "
                f"Requested channel_a={channel_a_idx}, channel_b={channel_b_idx}"
            )

        for row_num, row in enumerate(reader, start=2):
            try:
                timestamp = float(row[0])
                state_a, state_b = int(row[ch_a_col]), int(row[ch_b_col])

                # Record edges (state changes) or initial LOW state
                if (prev_a is not None and state_a != prev_a) or (prev_a is None and state_a == 0):
                    edges_a.append((timestamp, state_a))
                if (prev_b is not None and state_b != prev_b) or (prev_b is None and state_b == 0):
                    edges_b.append((timestamp, state_b))

                prev_a, prev_b = state_a, state_b

            except (ValueError, IndexError) as e:
                if verbose:
                    print(f"Warning: Skipping malformed row {row_num}: {e}")

    if verbose:
        print(f"Parsed {len(edges_a)} edges for channel A, {len(edges_b)} edges for channel B")

    return edges_a, edges_b


def detect_transactions(edges, idle_threshold_us: float, verbose: bool = False):
    """
    Detect BDShot transactions from edge list.

    A transaction starts with a falling edge after an idle period and ends
    when the line returns HIGH and stays HIGH for > idle_threshold_us.
    """
    if not edges:
        return []

    idle_threshold_s = idle_threshold_us / 1_000_000
    transactions = []
    tx_start = None
    last_rising = None

    for i, (timestamp, state) in enumerate(edges):
        if state == 0:  # Falling edge - start transaction if idle
            if tx_start is None and (last_rising is None or timestamp - last_rising > idle_threshold_s):
                tx_start = timestamp
        elif tx_start is not None:  # Rising edge with active transaction
            last_rising = timestamp
            # Complete transaction if next edge is far away or this is the last edge
            next_gap = edges[i + 1][0] - timestamp if i + 1 < len(edges) else float('inf')
            if next_gap > idle_threshold_s:
                transactions.append(BDShotTransaction(start=tx_start, end=timestamp))
                tx_start = None
        else:
            last_rising = timestamp

    if verbose:
        print(f"Detected {len(transactions)} transactions")
        if transactions:
            durations = [tx.duration_us for tx in transactions]
            print(f"  Duration range: {min(durations):.1f} - {max(durations):.1f} us")

    return transactions


def compute_channel_stats(transactions, channel_name: str, expected_interval_us: float):
    """Compute frame interval statistics for a channel."""
    if len(transactions) < 2:
        raise ValueError(f"Need at least 2 transactions to compute intervals, got {len(transactions)}")

    intervals_us = np.diff([tx.start for tx in transactions]) * 1_000_000

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


def compute_inter_channel_gaps(transactions_a, transactions_b, expected_interval_us: float,
                               verbose: bool = False):
    """Compute inter-channel gap statistics (CH_A end -> CH_B start)."""
    gaps = []
    max_reasonable_gap_us = expected_interval_us * 0.5
    b_idx = 0

    for tx_a in transactions_a:
        # Find first CH_B transaction that starts after this CH_A ends
        while b_idx < len(transactions_b) and transactions_b[b_idx].start <= tx_a.end:
            b_idx += 1
        if b_idx >= len(transactions_b):
            break

        gap_us = (transactions_b[b_idx].start - tx_a.end) * 1_000_000
        if 0 < gap_us < max_reasonable_gap_us:
            gaps.append(gap_us)

    if verbose:
        print(f"Inter-channel gap matching: {len(gaps)} matched")

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


def _plot_histogram(ax, data, title: str, xlabel: str, expected_value=None, stats_text=None):
    """Plot a histogram on the given axes."""
    bin_count = min(100, max(20, len(data) // 100)) if np.ptp(data) > 0 else 20
    ax.hist(data, bins=bin_count, edgecolor='black', alpha=0.7)
    ax.set_title(title, fontsize=12, fontweight='bold')
    ax.set_xlabel(xlabel, fontsize=10)
    ax.set_ylabel('Count', fontsize=10)

    if expected_value is not None:
        ax.axvline(expected_value, color='red', linestyle='--', linewidth=2,
                   label=f'Expected: {expected_value:.2f} us')
    ax.axvline(np.mean(data), color='green', linestyle='-', linewidth=2,
               label=f'Mean: {np.mean(data):.2f} us')

    if stats_text:
        ax.text(0.98, 0.97, stats_text, transform=ax.transAxes, fontsize=9,
                verticalalignment='top', horizontalalignment='right',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8), family='monospace')

    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True, alpha=0.3)


def generate_combined_histogram(intervals_a, intervals_b, gaps, stats_a, stats_b,
                                gap_stats, expected_interval_us: float, dshot_rate: int):
    """Generate a combined figure with all histograms."""
    if not HAS_MATPLOTLIB:
        return None

    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1], hspace=0.3, wspace=0.25)
    ax_a, ax_b, ax_gap = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, :])

    def stats_text(s):
        return format_stats_text(s.min_us, s.max_us, s.mean_us, s.std_us, s.count)

    _plot_histogram(ax_a, intervals_a, "Channel A Frame Intervals", "Interval (us)",
                    expected_value=expected_interval_us, stats_text=stats_text(stats_a))
    _plot_histogram(ax_b, intervals_b, "Channel B Frame Intervals", "Interval (us)",
                    expected_value=expected_interval_us, stats_text=stats_text(stats_b))

    if len(gaps) > 0:
        _plot_histogram(ax_gap, gaps, "Inter-Channel Gap (CH_A End -> CH_B Start)",
                        "Gap (us)", stats_text=stats_text(gap_stats))
    else:
        ax_gap.text(0.5, 0.5, "No inter-channel gap data available",
                    ha='center', va='center', transform=ax_gap.transAxes)
        ax_gap.set_title("Inter-Channel Gap (CH_A End -> CH_B Start)")

    fig.suptitle(f"BDShot Timing Analysis (DShot{dshot_rate})", fontsize=14, fontweight='bold', y=0.98)
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
                 stats_a: ChannelStats, stats_b: ChannelStats, gap_stats: GapStats):
    """Print the analysis report to console."""
    bit_period = DSHOT_BIT_PERIODS.get(dshot_rate, 3.33)
    expected_interval = 1_000_000 / loop_rate

    print(f"""
BDShot Timing Analysis
{"=" * 60}
Input: {input_file}
DShot Rate: {dshot_rate} (bit period: {bit_period:.2f} us)
Expected Loop Rate: {loop_rate:.0f} Hz ({expected_interval:.2f} us interval)
Capture Duration: {capture_duration:.2f} s
Transactions Detected: CH_A={tx_count_a}, CH_B={tx_count_b}
""")

    def print_channel_stats(name: str, s: ChannelStats):
        print(f"{name} Frame Intervals")
        print("-" * 40)
        print(f"  Count:     {s.count}")
        print(f"  Min:       {s.min_us:.2f} us  ({s.min_jitter_us:+.2f} us from nominal)")
        print(f"  Max:       {s.max_us:.2f} us  ({s.max_jitter_us:+.2f} us from nominal)")
        print(f"  Mean:      {s.mean_us:.2f} us  ({s.mean_jitter_us:+.2f} us from nominal)")
        print(f"  Std Dev:   {s.std_us:.2f} us\n")

    print_channel_stats("Channel A", stats_a)
    print_channel_stats("Channel B", stats_b)

    print(f"""Inter-Channel Gap (CH_A End -> CH_B Start)
{"-" * 40}
  Count:     {gap_stats.count}
  Min:       {gap_stats.min_us:.2f} us
  Max:       {gap_stats.max_us:.2f} us
  Mean:      {gap_stats.mean_us:.2f} us
  Std Dev:   {gap_stats.std_us:.2f} us
""")


def fatal(msg: str):
    """Print error and exit."""
    print(f"Error: {msg}", file=sys.stderr)
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="Analyze BDShot timing from Saleae Logic Analyzer CSV exports",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Examples:\n  %(prog)s capture.csv\n  %(prog)s capture.csv --dshot-rate 300 --loop-rate 800"
    )
    parser.add_argument('input_file', type=Path, help='Path to Saleae CSV export')
    parser.add_argument('--dshot-rate', type=int, default=300, choices=[150, 300, 600, 1200],
                        help='DShot variant (default: 300)')
    parser.add_argument('--loop-rate', type=float, default=800, help='Expected loop rate in Hz (default: 800)')
    parser.add_argument('--channel-a', type=int, default=0, help='CSV column index for first channel (default: 0)')
    parser.add_argument('--channel-b', type=int, default=1, help='CSV column index for second channel (default: 1)')
    parser.add_argument('--no-plots', action='store_true', help='Disable histogram display')
    parser.add_argument('--verbose', '-v', action='store_true', help='Enable verbose output for debugging')
    args = parser.parse_args()

    if not args.input_file.exists():
        fatal(f"Input file not found: {args.input_file}")

    # Calculate timing parameters
    bit_period_us = DSHOT_BIT_PERIODS.get(args.dshot_rate, 3.33)
    idle_threshold_us = max(50.0, bit_period_us * 15)  # Must exceed turnaround gap (~25-30us)
    expected_interval_us = 1_000_000 / args.loop_rate

    if args.verbose:
        print(f"Bit period: {bit_period_us:.2f} us\nIdle threshold: {idle_threshold_us:.2f} us\n"
              f"Expected interval: {expected_interval_us:.2f} us\n")

    # Parse CSV
    print(f"Parsing {args.input_file}...")
    try:
        edges_a, edges_b = parse_csv(args.input_file, args.channel_a, args.channel_b, args.verbose)
    except Exception as e:
        fatal(f"parsing CSV: {e}")

    if not edges_a or not edges_b:
        fatal("No edges found in one or both channels")

    # Detect transactions
    print("Detecting transactions...")
    transactions_a = detect_transactions(edges_a, idle_threshold_us, args.verbose)
    transactions_b = detect_transactions(edges_b, idle_threshold_us, args.verbose)

    for name, txns in [("A", transactions_a), ("B", transactions_b)]:
        if len(txns) < 2:
            fatal(f"Insufficient transactions in channel {name} ({len(txns)})")

    # Calculate capture duration
    all_txns = transactions_a + transactions_b
    capture_duration = max(tx.end for tx in all_txns) - min(tx.start for tx in all_txns)

    # Compute statistics
    print("Computing statistics...")
    stats_a, intervals_a = compute_channel_stats(transactions_a, "Channel A", expected_interval_us)
    stats_b, intervals_b = compute_channel_stats(transactions_b, "Channel B", expected_interval_us)

    try:
        gap_stats, gaps = compute_inter_channel_gaps(transactions_a, transactions_b,
                                                     expected_interval_us, args.verbose)
    except ValueError as e:
        print(f"Warning: {e}", file=sys.stderr)
        gap_stats = GapStats(count=0, min_us=0, max_us=0, mean_us=0, std_us=0)
        gaps = np.array([])

    print_report(args.input_file, args.dshot_rate, args.loop_rate, capture_duration,
                 len(transactions_a), len(transactions_b), stats_a, stats_b, gap_stats)

    if not args.no_plots and HAS_MATPLOTLIB:
        generate_combined_histogram(intervals_a, intervals_b, gaps, stats_a, stats_b,
                                    gap_stats, expected_interval_us, args.dshot_rate)
        plt.show()


if __name__ == '__main__':
    main()
