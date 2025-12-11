#!/usr/bin/env python3
"""
RTCM Test Report Generator

Generates a comprehensive test report with analysis and plots for RTCM pipeline testing.
Combines output from gps_dump_analyzer and rtcm_burst_analyzer into a markdown report
with embedded plot images.

Usage:
    python3 rtcm_test_report.py <ulog_file> --name <report_name> --config <config_desc>

Example:
    python3 rtcm_test_report.py flight.ulg --name dual_can_msm4 --config "Dual CAN GPS, MSM4"

Output:
    reports/<name>/
        report.md           - Markdown report
        gps_dump_analysis.png   - RTCM message analysis plots
        burst_analysis.png      - Burst timing analysis plots

Requirements:
    pip install pyulog numpy matplotlib
"""

import argparse
import os
import sys
from datetime import datetime
from pathlib import Path

# Add Tools directory to path for imports
TOOLS_DIR = Path(__file__).parent
sys.path.insert(0, str(TOOLS_DIR))

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

# Import our analysis modules
import gps_dump_analyzer
import rtcm_burst_analyzer


def get_log_info(ulog_file: str) -> dict:
    """Extract basic log information."""
    try:
        ulog = ULog(ulog_file)
        info = {
            'filename': os.path.basename(ulog_file),
            'start_timestamp': ulog.start_timestamp,
            'duration_s': (ulog.last_timestamp - ulog.start_timestamp) / 1e6,
        }

        # Try to get additional info
        if hasattr(ulog, 'msg_info_dict'):
            info['sys_name'] = ulog.msg_info_dict.get('sys_name', 'Unknown')
            info['ver_hw'] = ulog.msg_info_dict.get('ver_hw', 'Unknown')

        return info
    except Exception as e:
        return {'filename': os.path.basename(ulog_file), 'error': str(e)}


def generate_report(ulog_file: str, report_name: str, config_desc: str, output_dir: Path):
    """Generate the complete test report."""

    # Create output directory
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Generating report in: {output_dir}")

    # Get log info
    log_info = get_log_info(ulog_file)

    # Run gps_dump analysis
    print("Running GPS dump analysis...")
    dump_result, dump_error = gps_dump_analyzer.load_gps_dump(ulog_file)

    # Run burst analysis
    print("Running burst analysis...")
    burst_result, burst_error = rtcm_burst_analyzer.parse_rtcm_from_gps_dump(ulog_file)
    if burst_result and not burst_error:
        rtcm_burst_analyzer.detect_bursts(burst_result)

    # Generate plots
    dump_plot_path = output_dir / "gps_dump_analysis.png"
    burst_plot_path = output_dir / "burst_analysis.png"

    if dump_result:
        print(f"Generating GPS dump plot: {dump_plot_path}")
        gps_dump_analyzer.generate_plots(dump_result, str(dump_plot_path))

    if burst_result:
        print(f"Generating burst plot: {burst_plot_path}")
        rtcm_burst_analyzer.generate_plots(burst_result, str(burst_plot_path))

    # Generate markdown report
    report_path = output_dir / "report.md"
    print(f"Writing report: {report_path}")

    with open(report_path, 'w') as f:
        f.write(f"# RTCM Pipeline Test Report: {config_desc}\n\n")
        f.write(f"**Generated:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        f.write(f"**Log File:** `{log_info.get('filename', 'Unknown')}`\n\n")
        f.write(f"**Duration:** {log_info.get('duration_s', 0):.1f} seconds\n\n")

        f.write("---\n\n")

        # Summary section
        f.write("## Summary\n\n")

        if dump_result:
            f.write("| Metric | Value |\n")
            f.write("|--------|-------|\n")
            f.write(f"| Total RTCM Messages | {dump_result.messages_valid:,} |\n")
            f.write(f"| Total Bytes | {dump_result.total_bytes:,} |\n")
            f.write(f"| CRC Errors | {dump_result.messages_invalid_crc} |\n")
            f.write(f"| Bytes Discarded | {dump_result.bytes_discarded} |\n")
            f.write(f"| Data Integrity | {'✅ PASS' if dump_result.messages_invalid_crc == 0 and dump_result.bytes_discarded == 0 else '❌ ISSUES'} |\n")
            f.write("\n")
        else:
            f.write(f"**Error loading GPS dump data:** {dump_error}\n\n")

        # Message breakdown
        if dump_result and dump_result.message_counts:
            f.write("## RTCM Message Breakdown\n\n")
            f.write("| Type | Name | Count | Rate (Hz) | Avg Interval (ms) |\n")
            f.write("|------|------|-------|-----------|------------------|\n")

            for msg_type in sorted(dump_result.message_counts.keys()):
                count = dump_result.message_counts[msg_type]
                name = gps_dump_analyzer.RTCM_MSG_NAMES.get(msg_type, str(msg_type))
                rate = count / max(0.001, dump_result.log_duration_s)

                if msg_type in dump_result.timing_gaps and len(dump_result.timing_gaps[msg_type]) >= 2:
                    avg_interval = np.mean(dump_result.timing_gaps[msg_type]) * 1000
                    f.write(f"| {msg_type} | {name} | {count:,} | {rate:.1f} | {avg_interval:.1f} |\n")
                else:
                    f.write(f"| {msg_type} | {name} | {count:,} | {rate:.1f} | - |\n")

            f.write("\n")

        # Burst analysis
        if burst_result and burst_result.bursts:
            f.write("## Burst Analysis\n\n")

            burst_sizes = [b.total_bytes for b in burst_result.bursts]
            burst_counts = [len(b.messages) for b in burst_result.bursts]

            f.write("| Metric | Value |\n")
            f.write("|--------|-------|\n")
            f.write(f"| Total Bursts | {len(burst_result.bursts):,} |\n")
            f.write(f"| Avg Burst Size | {np.mean(burst_sizes):.0f} bytes |\n")
            f.write(f"| Avg Messages/Burst | {np.mean(burst_counts):.1f} |\n")

            # Inter-burst interval
            if len(burst_result.bursts) > 1:
                intervals = []
                for i in range(1, len(burst_result.bursts)):
                    interval = (burst_result.bursts[i].start_timestamp_us -
                               burst_result.bursts[i-1].start_timestamp_us) / 1000.0
                    intervals.append(interval)
                f.write(f"| Avg Inter-Burst Interval | {np.mean(intervals):.1f} ms |\n")
                f.write(f"| Inter-Burst Jitter (StdDev) | {np.std(intervals):.1f} ms |\n")

            f.write("\n")

            # MSM burst composition
            msm_types = rtcm_burst_analyzer.MSM4_TYPES | rtcm_burst_analyzer.MSM7_TYPES
            msm_bursts = [b for b in burst_result.bursts
                        if any(m.msg_type in msm_types for m in b.messages)]

            if msm_bursts:
                from collections import defaultdict
                type_cooccurrence = defaultdict(int)
                for burst in msm_bursts:
                    msm_in_burst = frozenset(m.msg_type for m in burst.messages if m.msg_type in msm_types)
                    if msm_in_burst:
                        type_cooccurrence[msm_in_burst] += 1

                f.write("### MSM Burst Patterns\n\n")
                f.write("| Pattern | Count | Percentage |\n")
                f.write("|---------|-------|------------|\n")

                for types, count in sorted(type_cooccurrence.items(), key=lambda x: -x[1])[:5]:
                    type_names = [f"{t}" for t in sorted(types)]
                    pct = 100.0 * count / len(msm_bursts)
                    f.write(f"| {' + '.join(type_names)} | {count} | {pct:.1f}% |\n")

                f.write("\n")

        # Plots section
        f.write("## Analysis Plots\n\n")

        if dump_plot_path.exists():
            f.write("### RTCM Message Analysis\n\n")
            f.write(f"![GPS Dump Analysis](gps_dump_analysis.png)\n\n")

        if burst_plot_path.exists():
            f.write("### Burst Timing Analysis\n\n")
            f.write(f"![Burst Analysis](burst_analysis.png)\n\n")

        # Configuration section
        f.write("---\n\n")
        f.write("## Test Configuration\n\n")
        f.write(f"- **Configuration:** {config_desc}\n")
        f.write(f"- **Report Name:** {report_name}\n")

        # Determine MSM type from message types present
        if dump_result and dump_result.message_counts:
            msm7_present = any(t in rtcm_burst_analyzer.MSM7_TYPES for t in dump_result.message_counts.keys())
            msm4_present = any(t in rtcm_burst_analyzer.MSM4_TYPES for t in dump_result.message_counts.keys())
            if msm7_present:
                f.write("- **RTCM Type:** MSM7 (PPK)\n")
            elif msm4_present:
                f.write("- **RTCM Type:** MSM4 (RTK)\n")

    print(f"\nReport generated successfully!")
    print(f"  Report: {report_path}")
    if dump_plot_path.exists():
        print(f"  Plot: {dump_plot_path}")
    if burst_plot_path.exists():
        print(f"  Plot: {burst_plot_path}")

    return report_path


def main():
    parser = argparse.ArgumentParser(
        description='Generate RTCM pipeline test report from ULog file.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s flight.ulg --name dual_can_msm4 --config "Dual CAN GPS, MSM4"
  %(prog)s flight.ulg --name dual_can_msm7 --config "Dual CAN GPS, MSM7"

Output:
  Creates reports/<name>/ directory with:
    - report.md             Markdown report
    - gps_dump_analysis.png RTCM message plots
    - burst_analysis.png    Burst timing plots
        """
    )
    parser.add_argument('ulog_file', help='Path to ULog file')
    parser.add_argument('--name', '-n', required=True,
                       help='Report name (used for output directory)')
    parser.add_argument('--config', '-c', required=True,
                       help='Configuration description (e.g., "Dual CAN GPS, MSM4")')
    parser.add_argument('--output', '-o', default=None,
                       help='Output base directory (default: ~/Downloads/gps_reports/)')
    parser.add_argument('--no-copy-log', action='store_true',
                       help='Do not copy the ULog file to the report directory')

    args = parser.parse_args()

    # Determine output directory
    if args.output:
        base_dir = Path(args.output)
    else:
        # Default to ~/Downloads/gps_reports/
        base_dir = Path.home() / "Downloads" / "gps_reports"

    output_dir = base_dir / args.name

    # Generate report
    generate_report(args.ulog_file, args.name, args.config, output_dir)

    # Copy log file (default behavior, disable with --no-copy-log)
    if not args.no_copy_log:
        import shutil
        log_dest = output_dir / Path(args.ulog_file).name
        print(f"Copying log file to: {log_dest}")
        shutil.copy2(args.ulog_file, log_dest)


if __name__ == '__main__':
    main()
