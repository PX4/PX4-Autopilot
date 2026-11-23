#!/usr/bin/env python3
"""
Optical flow CAN transport latency analysis.

Estimates the pipeline delay from the flow sensor measurement to the FC-side
sensor_optical_flow timestamp by cross-correlating the flow sensor's onboard
gyro (delta_angle) with the FC's own gyro (sensor_combined).

The flow sensor's delta_angle is an integrated gyro over each measurement
interval. Dividing by integration_timespan gives the average angular rate,
which should match the FC's gyro with only a CAN transport delay offset.

Usage:
    python3 flow_sensor_latency.py <log.ulg> [--output-dir <dir>]
"""

import argparse
import os
import sys

import numpy as np
from scipy import signal as sig

try:
    from pyulog import ULog
except ImportError:
    print("Error: pyulog not installed. Run: pip install pyulog", file=sys.stderr)
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
except ImportError:
    print("Error: matplotlib not installed. Run: pip install matplotlib", file=sys.stderr)
    sys.exit(1)


# ---------------------------------------------------------------------------
# ULog helpers
# ---------------------------------------------------------------------------

def get_topic(ulog, name, multi_id=0):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == multi_id:
            return d
    return None


def us_to_s(ts_us, start_us):
    return (ts_us.astype(np.int64) - np.int64(start_us)) / 1e6


# ---------------------------------------------------------------------------
# Signal processing
# ---------------------------------------------------------------------------

def butter_highpass(cutoff_hz, fs, order=2):
    nyq = fs / 2.0
    return sig.butter(order, cutoff_hz / nyq, btype="high")


def parabolic_peak(cc, lags_s):
    """Refine cross-correlation peak with parabolic interpolation."""
    idx = np.argmax(np.abs(cc))
    if idx == 0 or idx == len(cc) - 1:
        return lags_s[idx], cc[idx]
    y0, y1, y2 = np.abs(cc[idx - 1]), np.abs(cc[idx]), np.abs(cc[idx + 1])
    dt = lags_s[1] - lags_s[0]
    denom = y0 - 2 * y1 + y2
    if abs(denom) < 1e-12:
        return lags_s[idx], cc[idx]
    offset = 0.5 * (y0 - y2) / denom
    return lags_s[idx] + offset * dt, cc[idx]


# ---------------------------------------------------------------------------
# Data extraction
# ---------------------------------------------------------------------------

def extract_flow_gyro(ulog):
    """Extract angular rate from the flow sensor's onboard gyro."""
    d = get_topic(ulog, "sensor_optical_flow")
    if d is None:
        return None

    t0 = ulog.start_timestamp
    t = us_to_s(d.data["timestamp_sample"], t0)
    dt_us = d.data["integration_timespan_us"].astype(np.float64)
    quality = d.data["quality"]

    # Convert integrated delta_angle to angular rate (rad/s)
    valid = (quality > 0) & (dt_us > 100)
    t = t[valid]
    dt_s = dt_us[valid] / 1e6

    rate_x = d.data["delta_angle[0]"][valid] / dt_s
    rate_y = d.data["delta_angle[1]"][valid] / dt_s

    fs = 1.0 / np.median(np.diff(t))
    return {"time_s": t, "rate_x": rate_x, "rate_y": rate_y, "fs": fs}


def extract_fc_gyro(ulog):
    """Extract FC gyro rate from sensor_combined (highest rate available)."""
    d = get_topic(ulog, "sensor_combined")
    if d is None:
        return None

    t0 = ulog.start_timestamp
    t = us_to_s(d.data["timestamp"], t0)
    gx = d.data["gyro_rad[0]"]
    gy = d.data["gyro_rad[1]"]

    fs = 1.0 / np.median(np.diff(t))
    return {"time_s": t, "rate_x": gx, "rate_y": gy, "fs": fs}


# ---------------------------------------------------------------------------
# Cross-correlation
# ---------------------------------------------------------------------------

def cross_correlate_delay(ref_time, ref_signal, test_time, test_signal,
                          max_lag_s=0.1, grid_fs=None):
    """
    Cross-correlate two signals on a common time grid.
    Positive lag = test is delayed relative to ref.
    """
    t_start = max(ref_time[0], test_time[0])
    t_end = min(ref_time[-1], test_time[-1])
    if t_end - t_start < 1.0:
        return None

    if grid_fs is None:
        grid_fs = max(1.0 / np.median(np.diff(ref_time)),
                      1.0 / np.median(np.diff(test_time)))
    grid_fs = max(grid_fs, 1000.0)

    t_grid = np.arange(t_start, t_end, 1.0 / grid_fs)
    ref_i = np.interp(t_grid, ref_time, ref_signal)
    test_i = np.interp(t_grid, test_time, test_signal)

    # HP filter to remove bias
    b_hp, a_hp = butter_highpass(0.5, grid_fs, order=2)
    ref_i = sig.filtfilt(b_hp, a_hp, ref_i)
    test_i = sig.filtfilt(b_hp, a_hp, test_i)

    max_lag_samples = int(max_lag_s * grid_fs)
    # np.correlate(a, b)[mid + k] peaks at k = -D when b is delayed by D
    cc_full = np.correlate(ref_i, test_i, mode="full")
    mid = len(ref_i) - 1
    cc = cc_full[mid - max_lag_samples:mid + max_lag_samples + 1]
    # Negate lags so positive = test delayed
    lags = -np.arange(-max_lag_samples, max_lag_samples + 1) / grid_fs

    norm = np.sqrt(np.sum(ref_i**2) * np.sum(test_i**2))
    cc = cc / norm if norm > 0 else cc

    peak_lag, peak_cc = parabolic_peak(cc, lags)
    return {"lags_s": lags, "cc": cc, "peak_lag_s": peak_lag, "peak_cc": peak_cc,
            "grid_fs": grid_fs}


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def topic_info(time_s, name, field, role):
    """Return dict of topic metadata for the methodology table."""
    n = len(time_s)
    dur = time_s[-1] - time_s[0] if n > 1 else 0
    dt = np.median(np.diff(time_s)) if n > 1 else 0
    rate = 1.0 / dt if dt > 0 else 0
    return {"topic": name, "field": field.strip(), "role": role,
            "samples": n, "rate_hz": rate, "duration_s": dur}


def format_topic_table(infos):
    """Format a list of topic_info dicts into a fixed-width table string."""
    hdr = (f"  {'Topic':<25s}  {'Field':<22s}  {'Role':<10s}"
           f"  {'Samples':>7s}  {'Rate':>8s}  {'Duration':>8s}")
    sep = "  " + "-" * len(hdr.strip())
    rows = [hdr, sep]
    for t in infos:
        rows.append(
            f"  {t['topic']:<25s}  {t['field']:<22s}  {t['role']:<10s}"
            f"  {t['samples']:>7d}  {t['rate_hz']:>7.1f}Hz  {t['duration_s']:>7.1f}s"
        )
    return "\n".join(rows)


def make_report(log_path, flow_gyro, fc_gyro, xcorr_x, xcorr_y, output_dir,
                topic_lines):
    log_name = os.path.splitext(os.path.basename(log_path))[0]
    pdf_path = os.path.join(output_dir, f"{log_name}_flow_latency.pdf")

    with PdfPages(pdf_path) as pdf:
        # --- Page 1: Raw gyro overlay ---
        fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
        fig.suptitle(f"Flow Sensor Latency — {log_name}", fontsize=13)

        for ax, axis_label, flow_key, fc_key in [
            (axes[0], "X (roll)", "rate_x", "rate_x"),
            (axes[1], "Y (pitch)", "rate_y", "rate_y"),
        ]:
            ax.plot(fc_gyro["time_s"], fc_gyro[fc_key], "b-", lw=0.4,
                    alpha=0.6, label="FC gyro")
            ax.plot(flow_gyro["time_s"], flow_gyro[flow_key], "r-", lw=0.4,
                    alpha=0.6, label="flow gyro")
            ax.set_ylabel(f"{axis_label} rate [rad/s]")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

        axes[1].set_xlabel("Time [s]")
        plt.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

        # --- Page 2: Cross-correlation ---
        fig, axes = plt.subplots(2, 1, figsize=(12, 7))
        fig.suptitle("Cross-Correlation: flow gyro vs FC gyro", fontsize=13)

        for ax, xc, label in [
            (axes[0], xcorr_x, "X axis (roll)"),
            (axes[1], xcorr_y, "Y axis (pitch)"),
        ]:
            lags_ms = xc["lags_s"] * 1000
            ax.plot(lags_ms, xc["cc"], "b-", lw=1.0)
            peak_ms = xc["peak_lag_s"] * 1000
            ax.axvline(peak_ms, color="r", ls="--", lw=1.5,
                       label=f"peak = {peak_ms:.1f} ms (r = {xc['peak_cc']:.3f})")
            ax.axvline(0, color="gray", ls=":", lw=0.8)
            ax.set_xlabel("Lag [ms] (positive = flow lags FC)")
            ax.set_ylabel("Normalized correlation")
            ax.set_title(label)
            ax.legend(fontsize=10)
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

        # --- Page 3: Aligned overlay ---
        avg_delay = 0.5 * (xcorr_x["peak_lag_s"] + xcorr_y["peak_lag_s"])
        fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
        fig.suptitle(f"After delay correction ({avg_delay*1000:.1f} ms)", fontsize=13)

        t_start = max(flow_gyro["time_s"][0], fc_gyro["time_s"][0])
        t_end = min(flow_gyro["time_s"][-1], fc_gyro["time_s"][-1])
        t_grid = np.arange(t_start, t_end, 1.0 / 1000.0)

        for ax, axis_label, flow_key, fc_key in [
            (axes[0], "X (roll)", "rate_x", "rate_x"),
            (axes[1], "Y (pitch)", "rate_y", "rate_y"),
        ]:
            fc_i = np.interp(t_grid, fc_gyro["time_s"], fc_gyro[fc_key])
            flow_i = np.interp(t_grid, flow_gyro["time_s"] - avg_delay,
                               flow_gyro[flow_key])
            ax.plot(t_grid, fc_i, "b-", lw=0.5, alpha=0.6, label="FC gyro")
            ax.plot(t_grid, flow_i, "r-", lw=0.5, alpha=0.6,
                    label=f"flow gyro (shifted {avg_delay*1000:.1f} ms)")
            ax.set_ylabel(f"{axis_label} rate [rad/s]")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

        axes[1].set_xlabel("Time [s]")
        plt.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

        # --- Page 4: Methodology ---
        delay_x_ms = xcorr_x["peak_lag_s"] * 1000
        delay_y_ms = xcorr_y["peak_lag_s"] * 1000
        grid_hz = xcorr_x["grid_fs"]
        resolution_ms = 1000.0 / grid_hz
        topic_table = format_topic_table(topic_lines)

        fig = plt.figure(figsize=(12, 9))
        fig.text(0.05, 0.95, "Methodology", fontsize=14, fontweight="bold",
                 verticalalignment="top")
        text = (
            "This script estimates the total pipeline delay between the flow sensor\n"
            "measurement and its FC-side timestamp.\n"
            "\n"
            f"{topic_table}\n"
            "\n"
            f"  Cross-correlation grid:  {grid_hz:.0f} Hz  (resolution: {resolution_ms:.1f} ms)\n"
            "\n"
            f"  Result:  X (roll):   {delay_x_ms:+.1f} ± {resolution_ms/2:.1f} ms  "
            f"(r = {xcorr_x['peak_cc']:.3f})\n"
            f"           Y (pitch):  {delay_y_ms:+.1f} ± {resolution_ms/2:.1f} ms  "
            f"(r = {xcorr_y['peak_cc']:.3f})\n"
            f"           Average:    {avg_delay*1000:+.1f} ± {resolution_ms/2:.1f} ms\n"
            "\n"
            "Method:\n"
            "  1. Extract the flow sensor's onboard gyro (test signal):\n"
            "     - Read sensor_optical_flow.delta_angle[0,1] (integrated gyro, radians).\n"
            "     - Divide by integration_timespan_us to get average angular rate (rad/s).\n"
            "     - The flow sensor's IMU (e.g. ICM42688P on ARK Flow) measures the same\n"
            "       rotation as the FC gyro, but its timestamp includes CAN transport delay.\n"
            "\n"
            "  2. Extract the FC's own gyro (reference signal, no CAN delay):\n"
            "     - Read sensor_combined.gyro_rad[0,1] (instantaneous rate, rad/s).\n"
            "     - This is the FC's directly-connected IMU.\n"
            "\n"
            "  3. Cross-correlate each axis independently:\n"
            "     - Interpolate both signals onto a common 1000 Hz grid.\n"
            "     - High-pass filter at 0.5 Hz (zero-phase) to remove bias.\n"
            "     - Compute normalized cross-correlation.\n"
            "     - Parabolic interpolation around the peak gives sub-sample resolution.\n"
            "     Positive lag = flow sensor timestamp is late (delayed).\n"
            "     Accuracy is ± half a grid sample with parabolic interpolation.\n"
        )
        fig.text(0.05, 0.88, text, fontsize=9.5, fontfamily="monospace",
                 verticalalignment="top")
        pdf.savefig(fig)
        plt.close(fig)

    return pdf_path


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Flow sensor CAN latency analysis")
    parser.add_argument("log", help="ULog file (.ulg)")
    parser.add_argument("--output-dir", help="Output directory (default: logs/<name>/)")
    args = parser.parse_args()

    log_path = os.path.abspath(args.log)
    if not os.path.isfile(log_path):
        print(f"Error: {log_path} not found", file=sys.stderr)
        sys.exit(1)

    if args.output_dir:
        output_dir = args.output_dir
    else:
        log_name = os.path.splitext(os.path.basename(log_path))[0]
        script_dir = os.path.dirname(os.path.abspath(__file__))
        px4_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
        output_dir = os.path.join(px4_root, "logs", log_name)
    os.makedirs(output_dir, exist_ok=True)

    print(f"Loading {log_path} ...")
    ulog = ULog(log_path)

    flow_gyro = extract_flow_gyro(ulog)
    if flow_gyro is None or len(flow_gyro["time_s"]) < 50:
        print("Error: insufficient sensor_optical_flow data", file=sys.stderr)
        sys.exit(1)

    fc_gyro = extract_fc_gyro(ulog)
    if fc_gyro is None or len(fc_gyro["time_s"]) < 50:
        print("Error: insufficient sensor_combined data", file=sys.stderr)
        sys.exit(1)

    print(f"Flow gyro: {len(flow_gyro['time_s'])} samples, {flow_gyro['fs']:.0f} Hz")
    print(f"FC gyro:   {len(fc_gyro['time_s'])} samples, {fc_gyro['fs']:.0f} Hz")

    # Cross-correlate each axis independently
    xcorr_x = cross_correlate_delay(
        fc_gyro["time_s"], fc_gyro["rate_x"],
        flow_gyro["time_s"], flow_gyro["rate_x"],
        max_lag_s=0.1, grid_fs=1000.0,
    )
    xcorr_y = cross_correlate_delay(
        fc_gyro["time_s"], fc_gyro["rate_y"],
        flow_gyro["time_s"], flow_gyro["rate_y"],
        max_lag_s=0.1, grid_fs=1000.0,
    )

    if xcorr_x is None or xcorr_y is None:
        print("Error: cross-correlation failed (insufficient overlap)", file=sys.stderr)
        sys.exit(1)

    avg_delay = 0.5 * (xcorr_x["peak_lag_s"] + xcorr_y["peak_lag_s"])

    print(f"\n{'='*50}")
    print(f"Estimated flow sensor pipeline delay:")
    print(f"  X axis (roll):   {xcorr_x['peak_lag_s']*1000:+.1f} ms  "
          f"(r = {xcorr_x['peak_cc']:.3f})")
    print(f"  Y axis (pitch):  {xcorr_y['peak_lag_s']*1000:+.1f} ms  "
          f"(r = {xcorr_y['peak_cc']:.3f})")
    print(f"  Average:         {avg_delay*1000:+.1f} ms")
    print(f"{'='*50}")

    topic_lines = [
        topic_info(flow_gyro["time_s"], "sensor_optical_flow", "delta_angle[0,1]", "test"),
        topic_info(fc_gyro["time_s"],   "sensor_combined",     "gyro_rad[0,1]", "reference"),
    ]

    pdf_path = make_report(log_path, flow_gyro, fc_gyro, xcorr_x, xcorr_y, output_dir,
                           topic_lines)
    print(f"\nReport: {pdf_path}")
    return pdf_path


if __name__ == "__main__":
    main()
