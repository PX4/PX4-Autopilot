#!/usr/bin/env python3
"""
Range sensor CAN transport latency analysis.

Estimates the total pipeline delay from AFBRS50 measurement to the FC-side
distance_sensor timestamp by cross-correlating range rate with IMU-derived
vertical velocity. The lag at peak correlation is the transport delay.

Method:
  1. Rotate body-frame accel to NED using attitude quaternion, subtract gravity
  2. High-pass filter and integrate to get IMU vertical velocity (drift-free)
  3. Differentiate distance_sensor to get range rate
  4. Cross-correlate range rate with -vz_imu (sign: range decreases when vz>0)
  5. Parabolic interpolation around peak for sub-sample resolution

Usage:
    python3 range_sensor_latency.py <log.ulg> [--output-dir <dir>]
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

GRAVITY = 9.80665


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


def butter_lowpass(cutoff_hz, fs, order=2):
    nyq = fs / 2.0
    return sig.butter(order, cutoff_hz / nyq, btype="low")


def quat_rotate_z(q0, q1, q2, q3, ax, ay, az):
    """Rotate body-frame vector to NED and return only the Z (down) component."""
    return (2.0 * (q1 * q3 - q0 * q2) * ax
            + 2.0 * (q2 * q3 + q0 * q1) * ay
            + (q0**2 - q1**2 - q2**2 + q3**2) * az)


def parabolic_peak(cc, lags_s):
    """Refine cross-correlation peak with parabolic interpolation."""
    idx = np.argmax(np.abs(cc))
    if idx == 0 or idx == len(cc) - 1:
        return lags_s[idx], cc[idx]
    y0, y1, y2 = np.abs(cc[idx - 1]), np.abs(cc[idx]), np.abs(cc[idx + 1])
    dt = lags_s[1] - lags_s[0]
    # Parabolic interpolation: offset from idx in samples
    offset = 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2)
    return lags_s[idx] + offset * dt, cc[idx]


# ---------------------------------------------------------------------------
# Data extraction
# ---------------------------------------------------------------------------

def extract_imu_vz(ulog):
    """
    Build IMU-derived vertical velocity:
      1. Body accel from sensor_combined (high rate)
      2. Attitude quaternion from vehicle_attitude (interpolated)
      3. Rotate to NED, subtract gravity → kinematic az
      4. HP-filter + integrate → vz (drift-free)
    """
    sc = get_topic(ulog, "sensor_combined")
    att = get_topic(ulog, "vehicle_attitude")
    if sc is None or att is None:
        return None

    t0 = ulog.start_timestamp
    t_imu = us_to_s(sc.data["timestamp"], t0)
    ax = sc.data["accelerometer_m_s2[0]"]
    ay = sc.data["accelerometer_m_s2[1]"]
    az = sc.data["accelerometer_m_s2[2]"]

    t_att = us_to_s(att.data["timestamp_sample"], t0)
    q0i = np.interp(t_imu, t_att, att.data["q[0]"])
    q1i = np.interp(t_imu, t_att, att.data["q[1]"])
    q2i = np.interp(t_imu, t_att, att.data["q[2]"])
    q3i = np.interp(t_imu, t_att, att.data["q[3]"])

    # NED vertical accel (kinematic = specific_force_ned + gravity)
    az_ned = quat_rotate_z(q0i, q1i, q2i, q3i, ax, ay, az) + GRAVITY

    fs_imu = 1.0 / np.median(np.diff(t_imu))

    # HP filter to remove bias/drift, then integrate for velocity
    b_hp, a_hp = butter_highpass(0.1, fs_imu, order=2)
    az_hp = sig.filtfilt(b_hp, a_hp, az_ned)

    dt = np.diff(t_imu, prepend=t_imu[0] - 1.0 / fs_imu)
    vz_imu = np.cumsum(az_hp * dt)

    # HP filter the integrated velocity too, to remove any remaining drift
    vz_imu = sig.filtfilt(b_hp, a_hp, vz_imu)

    return {"time_s": t_imu, "vz": vz_imu, "fs": fs_imu}


def extract_range(ulog):
    """Extract valid distance sensor measurements."""
    d = get_topic(ulog, "distance_sensor")
    if d is None:
        return None

    t0 = ulog.start_timestamp
    t = us_to_s(d.data["timestamp"], t0)
    dist = d.data["current_distance"]
    quality = d.data["signal_quality"]

    # Filter: valid quality and nonzero distance
    valid = (quality > 0) & (dist > 0.01)
    return {"time_s": t[valid], "distance": dist[valid]}


def extract_ekf_vz(ulog):
    """Extract EKF vertical velocity for comparison."""
    d = get_topic(ulog, "vehicle_local_position")
    if d is None:
        return None
    t0 = ulog.start_timestamp
    return {
        "time_s": us_to_s(d.data["timestamp"], t0),
        "vz": d.data["vz"],
    }


# ---------------------------------------------------------------------------
# Cross-correlation analysis
# ---------------------------------------------------------------------------

def compute_range_rate(range_data, lp_cutoff_hz=5.0):
    """Differentiate and low-pass filter range measurements."""
    t = range_data["time_s"]
    d = range_data["distance"]

    if len(t) < 10:
        return None

    dt = np.diff(t)
    rate = np.diff(d) / dt
    t_rate = 0.5 * (t[:-1] + t[1:])

    # Low-pass filter to reduce derivative noise
    fs = 1.0 / np.median(dt)
    if lp_cutoff_hz < fs / 2:
        b_lp, a_lp = butter_lowpass(lp_cutoff_hz, fs, order=2)
        rate = sig.filtfilt(b_lp, a_lp, rate)

    return {"time_s": t_rate, "rate": rate, "fs": fs}


def cross_correlate_delay(ref_time, ref_signal, test_time, test_signal,
                          max_lag_s=0.1, grid_fs=None):
    """
    Cross-correlate two signals on a common time grid.
    Returns (lags_s, cc_normalized, peak_lag_s, peak_cc).
    """
    # Common time window
    t_start = max(ref_time[0], test_time[0])
    t_end = min(ref_time[-1], test_time[-1])
    if t_end - t_start < 1.0:
        return None

    if grid_fs is None:
        grid_fs = 1.0 / min(np.median(np.diff(ref_time)),
                            np.median(np.diff(test_time)))
    # Bump grid to at least 1000 Hz for reasonable resolution
    grid_fs = max(grid_fs, 1000.0)

    t_grid = np.arange(t_start, t_end, 1.0 / grid_fs)
    ref_i = np.interp(t_grid, ref_time, ref_signal)
    test_i = np.interp(t_grid, test_time, test_signal)

    # Remove means
    ref_i -= np.mean(ref_i)
    test_i -= np.mean(test_i)

    max_lag_samples = int(max_lag_s * grid_fs)
    # np.correlate(a, b)[mid + k] peaks at k = -D when b is delayed by D.
    # Negate the lag axis so positive = test delayed relative to ref.
    cc_full = np.correlate(ref_i, test_i, mode="full")
    mid = len(ref_i) - 1
    cc = cc_full[mid - max_lag_samples:mid + max_lag_samples + 1]
    lags = -np.arange(-max_lag_samples, max_lag_samples + 1) / grid_fs

    # Normalize
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


def make_report(log_path, range_data, range_rate, imu_vz, ekf_vz, xcorr_imu,
                xcorr_ekf, output_dir, topic_lines):
    """Generate PDF report."""
    log_name = os.path.splitext(os.path.basename(log_path))[0]
    pdf_path = os.path.join(output_dir, f"{log_name}_range_latency.pdf")

    with PdfPages(pdf_path) as pdf:
        # --- Page 1: Raw data overview ---
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        fig.suptitle(f"Range Sensor Latency Analysis — {log_name}", fontsize=13)

        axes[0].plot(range_data["time_s"], range_data["distance"], "b-", lw=0.6)
        axes[0].set_ylabel("Range [m]")
        axes[0].set_title("Distance sensor")
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(range_rate["time_s"], range_rate["rate"], "r-", lw=0.6,
                     label="range rate")
        axes[1].set_ylabel("Range rate [m/s]")
        axes[1].set_title(f"Range rate (LP {5.0:.0f} Hz)")
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(imu_vz["time_s"], -imu_vz["vz"], "g-", lw=0.4, alpha=0.7,
                     label="-vz_imu")
        if ekf_vz is not None:
            axes[2].plot(ekf_vz["time_s"], -ekf_vz["vz"], "k-", lw=0.8,
                         alpha=0.8, label="-vz_ekf")
        axes[2].set_ylabel("Velocity [m/s]")
        axes[2].set_xlabel("Time [s]")
        axes[2].set_title("Vertical velocity (negated, for comparison with range rate)")
        axes[2].legend(fontsize=8)
        axes[2].grid(True, alpha=0.3)
        plt.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

        # --- Page 2: Cross-correlation results ---
        n_plots = 1 + (1 if xcorr_ekf is not None else 0)
        fig, axes = plt.subplots(n_plots, 1, figsize=(12, 4 * n_plots))
        if n_plots == 1:
            axes = [axes]
        fig.suptitle("Cross-Correlation: range rate vs vertical velocity", fontsize=13)

        for ax, xc, label in [
            (axes[0], xcorr_imu, "IMU-derived vz"),
        ] + ([(axes[1], xcorr_ekf, "EKF vz")] if xcorr_ekf is not None else []):
            lags_ms = xc["lags_s"] * 1000
            ax.plot(lags_ms, xc["cc"], "b-", lw=1.0)
            peak_ms = xc["peak_lag_s"] * 1000
            ax.axvline(peak_ms, color="r", ls="--", lw=1.5,
                       label=f"peak = {peak_ms:.1f} ms (r = {xc['peak_cc']:.3f})")
            ax.axvline(0, color="gray", ls=":", lw=0.8)
            ax.set_xlabel("Lag [ms] (positive = range lags reference)")
            ax.set_ylabel("Normalized correlation")
            ax.set_title(f"Reference: {label} ({xc['grid_fs']:.0f} Hz grid)")
            ax.legend(fontsize=10)
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

        # --- Page 3: Aligned overlay ---
        fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
        fig.suptitle("Range rate vs -vz_imu: before and after delay correction",
                     fontsize=13)

        # Common time window for overlay
        t_start = max(range_rate["time_s"][0], imu_vz["time_s"][0])
        t_end = min(range_rate["time_s"][-1], imu_vz["time_s"][-1])
        t_grid = np.arange(t_start, t_end, 1.0 / 1000.0)

        rr_i = np.interp(t_grid, range_rate["time_s"], range_rate["rate"])
        vz_i = np.interp(t_grid, imu_vz["time_s"], -imu_vz["vz"])
        # Normalize for visual comparison
        rr_norm = (rr_i - np.mean(rr_i)) / (np.std(rr_i) + 1e-10)
        vz_norm = (vz_i - np.mean(vz_i)) / (np.std(vz_i) + 1e-10)

        axes[0].plot(t_grid, vz_norm, "g-", lw=0.6, alpha=0.7, label="-vz_imu")
        axes[0].plot(t_grid, rr_norm, "r-", lw=0.6, alpha=0.7, label="range rate")
        axes[0].set_title("Before correction (original timestamps)")
        axes[0].set_ylabel("Normalized")
        axes[0].legend(fontsize=8)
        axes[0].grid(True, alpha=0.3)

        # Shift range rate by estimated delay
        delay = xcorr_imu["peak_lag_s"]
        rr_shifted = np.interp(t_grid, range_rate["time_s"] - delay,
                               range_rate["rate"])
        rr_s_norm = (rr_shifted - np.mean(rr_shifted)) / (np.std(rr_shifted) + 1e-10)

        axes[1].plot(t_grid, vz_norm, "g-", lw=0.6, alpha=0.7, label="-vz_imu")
        axes[1].plot(t_grid, rr_s_norm, "r-", lw=0.6, alpha=0.7,
                     label=f"range rate (shifted {delay*1000:.1f} ms)")
        axes[1].set_title(f"After correction (delay = {delay*1000:.1f} ms)")
        axes[1].set_xlabel("Time [s]")
        axes[1].set_ylabel("Normalized")
        axes[1].legend(fontsize=8)
        axes[1].grid(True, alpha=0.3)

        plt.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

        # --- Page 4: Methodology ---
        delay_ms = xcorr_imu["peak_lag_s"] * 1000
        grid_hz = xcorr_imu["grid_fs"]
        resolution_ms = 1000.0 / grid_hz
        topic_table = format_topic_table(topic_lines)

        fig = plt.figure(figsize=(12, 9))
        fig.text(0.05, 0.95, "Methodology", fontsize=14, fontweight="bold",
                 verticalalignment="top")
        text = (
            "This script estimates the total pipeline delay between the range sensor\n"
            "measurement and its FC-side timestamp.\n"
            "\n"
            f"{topic_table}\n"
            "\n"
            f"  Cross-correlation grid:  {grid_hz:.0f} Hz  (resolution: {resolution_ms:.1f} ms)\n"
            "\n"
            f"  Result:  {delay_ms:+.1f} ± {resolution_ms/2:.1f} ms  "
            f"(r = {xcorr_imu['peak_cc']:.3f})\n"
            "\n"
            "Method:\n"
            "  1. Build an IMU-derived vertical velocity (reference signal):\n"
            "     - Rotate sensor_combined accelerometer to NED using vehicle_attitude\n"
            "       quaternion, subtract gravity to get kinematic vertical acceleration.\n"
            "     - High-pass filter at 0.1 Hz (zero-phase) and integrate to get vz.\n"
            "     - High-pass filter the velocity to remove drift.\n"
            "     This signal has no CAN delay (FC IMU is directly connected).\n"
            "\n"
            "  2. Compute range rate (test signal):\n"
            "     - Differentiate distance_sensor.current_distance via finite differences.\n"
            "     - Low-pass filter at 5 Hz (zero-phase) to reduce derivative noise.\n"
            "     For a downward-facing sensor, range_rate ≈ -vz.\n"
            "\n"
            "  3. Cross-correlate:\n"
            "     - Interpolate both signals onto a common 1000 Hz grid.\n"
            "     - Remove means and compute normalized cross-correlation.\n"
            "     - The lag at peak correlation is the estimated delay.\n"
            "     - Parabolic interpolation around the peak gives sub-sample resolution.\n"
            "     Positive lag = range sensor timestamp is late (delayed).\n"
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
    parser = argparse.ArgumentParser(description="Range sensor CAN latency analysis")
    parser.add_argument("log", help="ULog file (.ulg)")
    parser.add_argument("--output-dir", help="Output directory (default: logs/<name>/)")
    args = parser.parse_args()

    log_path = os.path.abspath(args.log)
    if not os.path.isfile(log_path):
        print(f"Error: {log_path} not found", file=sys.stderr)
        sys.exit(1)

    # Output directory
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

    # --- Extract data ---
    range_data = extract_range(ulog)
    if range_data is None or len(range_data["time_s"]) < 20:
        print("Error: insufficient distance_sensor data", file=sys.stderr)
        sys.exit(1)

    range_rate = compute_range_rate(range_data, lp_cutoff_hz=5.0)
    if range_rate is None:
        print("Error: could not compute range rate", file=sys.stderr)
        sys.exit(1)

    imu_vz = extract_imu_vz(ulog)
    if imu_vz is None:
        print("Error: could not build IMU vertical velocity "
              "(need sensor_combined + vehicle_attitude)", file=sys.stderr)
        sys.exit(1)

    ekf_vz = extract_ekf_vz(ulog)

    # --- Cross-correlation ---
    print(f"Distance sensor: {len(range_data['time_s'])} valid samples, "
          f"{range_rate['fs']:.0f} Hz")
    print(f"IMU accel: {len(imu_vz['time_s'])} samples, {imu_vz['fs']:.0f} Hz")

    # range_rate ≈ -vz for downward-facing sensor (range decreases when descending)
    # So correlate range_rate with -vz_imu
    xcorr_imu = cross_correlate_delay(
        imu_vz["time_s"], -imu_vz["vz"],
        range_rate["time_s"], range_rate["rate"],
        max_lag_s=0.1, grid_fs=1000.0,
    )
    if xcorr_imu is None:
        print("Error: cross-correlation failed (insufficient overlap)", file=sys.stderr)
        sys.exit(1)

    xcorr_ekf = None
    if ekf_vz is not None:
        xcorr_ekf = cross_correlate_delay(
            ekf_vz["time_s"], -ekf_vz["vz"],
            range_rate["time_s"], range_rate["rate"],
            max_lag_s=0.1, grid_fs=1000.0,
        )

    # --- Results ---
    delay_ms = xcorr_imu["peak_lag_s"] * 1000
    print(f"\n{'='*50}")
    print(f"Estimated range sensor pipeline delay:")
    print(f"  IMU cross-correlation:  {delay_ms:+.1f} ms  "
          f"(r = {xcorr_imu['peak_cc']:.3f})")
    if xcorr_ekf is not None:
        print(f"  EKF cross-correlation:  {xcorr_ekf['peak_lag_s']*1000:+.1f} ms  "
              f"(r = {xcorr_ekf['peak_cc']:.3f})")
    print(f"{'='*50}")

    if abs(delay_ms) < 1.0:
        print("Note: delay < 1ms — may not be resolvable at this sample rate")
    elif delay_ms < 0:
        print("Note: negative delay suggests range leads IMU — "
              "check sensor orientation / data quality")

    # --- PDF report ---
    # Build topic summary for methodology page
    att = get_topic(ulog, "vehicle_attitude")
    att_time = us_to_s(att.data["timestamp_sample"], ulog.start_timestamp) if att else np.array([])
    topic_lines = [
        topic_info(range_data["time_s"], "distance_sensor", "current_distance", "test"),
        topic_info(imu_vz["time_s"],     "sensor_combined", "accelerometer_m_s2", "reference"),
        topic_info(att_time,             "vehicle_attitude", "q[0..3]", "reference"),
    ]
    if ekf_vz is not None:
        topic_lines.append(
            topic_info(ekf_vz["time_s"], "vehicle_local_position", "vz", "comparison"))

    pdf_path = make_report(log_path, range_data, range_rate, imu_vz, ekf_vz,
                           xcorr_imu, xcorr_ekf, output_dir, topic_lines)
    print(f"\nReport: {pdf_path}")
    return pdf_path


if __name__ == "__main__":
    main()
