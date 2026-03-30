#!/usr/bin/env python3
"""
Barometer thrust compensation analysis tool.

Analyzes baro thrust compensation from flight logs. Three modes:

1. **Estimator review** (baro_thrust_estimate logged, no range sensor):
   Shows online estimator convergence, K/tau identification, residual analysis.

2. **Full validation** (baro_thrust_estimate + range sensor):
   Cross-validates online estimator results against range-sensor ground truth.

3. **Standalone calibration** (range sensor, no estimator):
   Identifies SENS_BARO_PCOEF and SENS_BARO_PTAU from baro vs range error.

The correction model: baro_alt += SENS_BARO_PCOEF * LPF(thrust, SENS_BARO_PTAU)

Requirements:
    - Flight log (.ulg)
    - Python packages: pyulog, numpy, matplotlib

Usage:
    python3 baro_thrust_calibration.py <log.ulg> [--output-dir <dir>]

Outputs:
    - baro_calibration.pdf    Analysis plots
    - Console output with parameters and diagnostics
"""

import argparse
import os
import sys

import numpy as np

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

def get_topic(ulog, topic_name, multi_id=0):
    """Return the first matching dataset for a topic name and multi_id."""
    for d in ulog.data_list:
        if d.name == topic_name and d.multi_id == multi_id:
            return d
    return None


def get_param(ulog, name, default=None):
    """Get an initial parameter value from the log."""
    return ulog.initial_parameters.get(name, default)


def us_to_seconds(ts_us, start_us):
    """Convert microsecond timestamps to seconds relative to log start."""
    return (ts_us.astype(np.int64) - np.int64(start_us)) / 1e6


def thrust_magnitude(thrust_z):
    """Convert NED-Z thrust setpoint to upward magnitude [0, 1]."""
    return np.clip(-thrust_z, 0, 1)


# ---------------------------------------------------------------------------
# Flight phase detection
# ---------------------------------------------------------------------------

def detect_armed_period(ulog):
    """Detect armed start/end times from vehicle_status or actuator_motors."""
    start_us = ulog.start_timestamp

    vstatus = get_topic(ulog, "vehicle_status")
    if vstatus is not None and "arming_state" in vstatus.data:
        ts = us_to_seconds(vstatus.data["timestamp"], start_us)
        armed_idx = np.where(vstatus.data["arming_state"] == 2)[0]
        if len(armed_idx) > 0:
            return ts[armed_idx[0]], ts[armed_idx[-1]]

    # Fallback: look for motor activity
    motors = get_topic(ulog, "actuator_motors")
    if motors is not None:
        ts = us_to_seconds(motors.data["timestamp"], start_us)
        active = np.zeros(len(ts), dtype=bool)
        for i in range(12):
            key = f"control[{i}]"
            if key in motors.data:
                active |= (motors.data[key] > 0.05)
        active_idx = np.where(active)[0]
        if len(active_idx) > 0:
            return ts[active_idx[0]], ts[active_idx[-1]]

    # Last resort: full log duration
    return 0.0, (ulog.last_timestamp - start_us) / 1e6


def find_hover_segment(armed_start, armed_end):
    """Return middle 60% of armed time as the hover analysis window."""
    duration = armed_end - armed_start
    return armed_start + duration * 0.2, armed_end - duration * 0.2


# ---------------------------------------------------------------------------
# Data extraction
# ---------------------------------------------------------------------------

def extract_baro(ulog):
    """Extract barometer altitude from vehicle_air_data."""
    start_us = ulog.start_timestamp
    vad = get_topic(ulog, "vehicle_air_data")
    if vad is None:
        return None
    return {
        "time_s": us_to_seconds(vad.data["timestamp"], start_us),
        "alt_m": vad.data["baro_alt_meter"],
    }


def extract_range(ulog):
    """Extract range sensor distance from distance_sensor."""
    start_us = ulog.start_timestamp
    dist = get_topic(ulog, "distance_sensor")
    if dist is None:
        return None
    return {
        "time_s": us_to_seconds(dist.data["timestamp"], start_us),
        "distance_m": dist.data["current_distance"],
    }


def extract_thrust(ulog):
    """Extract vertical thrust setpoint from vehicle_thrust_setpoint."""
    start_us = ulog.start_timestamp
    thr = get_topic(ulog, "vehicle_thrust_setpoint")
    if thr is None:
        return None
    return {
        "time_s": us_to_seconds(thr.data["timestamp"], start_us),
        "thrust_z": thr.data["xyz[2]"],
    }


def extract_estimator(ulog):
    """Extract online estimator state from baro_thrust_estimate."""
    start_us = ulog.start_timestamp
    est = get_topic(ulog, "baro_thrust_estimate")
    if est is None:
        return None
    return {
        "time_s": us_to_seconds(est.data["timestamp"], start_us),
        "residual": est.data["residual"],
        "k_estimate": est.data["k_estimate"],
        "k_estimate_var": est.data["k_estimate_var"],
        "best_tau": est.data["best_tau"],
        "best_error_var": est.data["best_error_var"],
        "thrust_std": est.data["thrust_std"],
        "converged": est.data["converged"],
        "estimation_active": est.data["estimation_active"],
        "best_bank_idx": est.data["best_bank_idx"],
    }


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------

def compute_baro_error(baro, range_data, armed_start):
    """Compute baro altitude error relative to range sensor.

    Both signals are aligned to a common time base (range sensor timestamps).
    Baro altitude is zeroed at arm time so it represents change from ground.

    Returns: dict with time_s, baro_error (baro - range, positive = baro reads
    higher than truth), and the interpolated baro/range arrays.
    """
    baro_t, baro_alt = baro["time_s"], baro["alt_m"]
    rng_t, rng_dist = range_data["time_s"], range_data["distance_m"]

    # Zero baro at arm time
    arm_baro = np.interp(armed_start, baro_t, baro_alt)
    baro_zeroed = baro_alt - arm_baro

    # Interpolate baro onto range sensor time base
    baro_interp = np.interp(rng_t, baro_t, baro_zeroed)
    error = baro_interp - rng_dist

    return {
        "time_s": rng_t,
        "error": error,
        "baro_alt": baro_interp,
        "range_alt": rng_dist,
        "baro_full_time_s": baro_t,
        "baro_full_alt": baro_zeroed,
    }


def first_order_lpf(signal, time_s, tau):
    """Apply a causal first-order low-pass filter with time constant tau.

    Matches the AlphaFilter used in the EKF: alpha = dt / (dt + tau).
    When tau=0, returns the input unchanged.
    """
    if tau <= 0 or len(signal) < 2:
        return signal.copy()

    out = np.empty_like(signal)
    out[0] = signal[0]
    for i in range(1, len(signal)):
        dt = time_s[i] - time_s[i - 1]
        if dt <= 0:
            out[i] = out[i - 1]
            continue
        alpha = dt / (dt + tau)
        out[i] = out[i - 1] + alpha * (signal[i] - out[i - 1])
    return out


def calibrate(baro_err, thrust_data, hover_start, hover_end):
    """System identification for thrust-based baro compensation.

    Fits: baro_error = K * LPF(thrust_magnitude, tau) + c

    Sweeps over tau candidates and picks the one maximizing R^2.
    Also computes cross-correlation to visualize the delay structure.

    Returns dict with identified K, tau, cross-correlation data, sweep
    results, and the recommended parameters.
    """
    err_t = baro_err["time_s"]
    err = baro_err["error"]

    # Restrict to hover segment
    hov = (err_t >= hover_start) & (err_t <= hover_end)
    if hov.sum() < 20:
        return None

    err_t_hov = err_t[hov]
    err_hov = err[hov]

    # Interpolate thrust onto error timestamps and convert to upward magnitude
    thrust_raw = np.interp(err_t_hov, thrust_data["time_s"],
                           thrust_data["thrust_z"])
    thrust_mag = thrust_magnitude(thrust_raw)

    # Detrend error (remove slow thermal / bias drift)
    err_detrended = err_hov - np.polyval(
        np.polyfit(err_t_hov, err_hov, 1), err_t_hov)
    thrust_detrended = thrust_mag - np.mean(thrust_mag)

    err_var = np.var(err_detrended)
    if err_var < 1e-10:
        return None

    result = {
        "hover_time": err_t_hov,
        "hover_error": err_hov,
        "thrust_mag": thrust_mag,
    }

    # --- Cross-correlation ---
    dt_median = np.median(np.diff(err_t_hov))
    if dt_median > 0:
        max_lag = min(int(2.0 / dt_median), len(err_detrended) // 2)
        if max_lag > 5:
            xcorr = np.correlate(err_detrended, thrust_detrended, "full")
            mid = len(thrust_detrended) - 1
            lags = (np.arange(len(xcorr)) - mid) * dt_median
            norm = np.sqrt(np.sum(err_detrended**2) *
                           np.sum(thrust_detrended**2))
            if norm > 0:
                xcorr = xcorr / norm
            window = (lags >= -2.0) & (lags <= 2.0)
            result["xcorr_lags_s"] = lags[window]
            result["xcorr_values"] = xcorr[window]
            peak_idx = np.argmax(np.abs(xcorr[window]))
            result["xcorr_peak_lag_s"] = float(lags[window][peak_idx])
            result["xcorr_peak_value"] = float(xcorr[window][peak_idx])

    # --- Grid search over time constants ---
    tau_candidates = np.concatenate([
        [0.0],
        np.arange(0.02, 0.2, 0.02),
        np.arange(0.2, 1.01, 0.05),
    ])

    sweep_tau, sweep_r2, sweep_K, sweep_rmse = [], [], [], []

    for tau in tau_candidates:
        thrust_filt = first_order_lpf(thrust_mag, err_t_hov, tau)
        thrust_filt_centered = thrust_filt - np.mean(thrust_filt)

        if np.var(thrust_filt_centered) < 1e-10:
            continue

        # Least-squares: err_detrended = K * thrust_filt_centered
        K = np.sum(err_detrended * thrust_filt_centered) / \
            np.sum(thrust_filt_centered**2)
        residual = err_detrended - K * thrust_filt_centered
        r2 = 1.0 - np.var(residual) / err_var

        sweep_tau.append(float(tau))
        sweep_r2.append(float(r2))
        sweep_K.append(float(K))
        sweep_rmse.append(float(np.sqrt(np.mean(residual**2))))

    if not sweep_tau:
        return None

    result["sweep_tau"] = np.array(sweep_tau)
    result["sweep_r2"] = np.array(sweep_r2)
    result["sweep_K"] = np.array(sweep_K)
    result["sweep_rmse"] = np.array(sweep_rmse)

    # Best fit
    best_idx = int(np.argmax(sweep_r2))
    result["best_tau"] = sweep_tau[best_idx]
    result["best_K"] = sweep_K[best_idx]
    result["best_r2"] = sweep_r2[best_idx]
    result["best_rmse"] = sweep_rmse[best_idx]

    # No-lag baseline for comparison
    result["nolag_K"] = sweep_K[0] if sweep_tau[0] == 0.0 else 0.0
    result["nolag_r2"] = sweep_r2[0] if sweep_tau[0] == 0.0 else 0.0

    # Compensated timeseries (for plotting)
    best_filt = first_order_lpf(thrust_mag, err_t_hov, result["best_tau"])
    result["compensated_error"] = err_hov - result["best_K"] * best_filt

    # Recommended parameters:
    #   baro_error = K * thrust  =>  to cancel: baro_alt += (-K) * thrust
    result["recommended_pcoef"] = -result["best_K"]
    result["recommended_ptau"] = result["best_tau"]

    return result


def reconstruct_raw_error(baro_err, thrust_data, pcoef, ptau):
    """Undo existing baro compensation to reconstruct the raw error.

    During flight the firmware applied: baro_alt += pcoef * LPF(thrust, ptau).
    To recover pre-compensation error: raw = observed - pcoef * LPF(thrust, ptau).
    """
    err_t = baro_err["time_s"]
    thrust_raw = np.interp(err_t, thrust_data["time_s"],
                           thrust_data["thrust_z"])
    thrust_mag = thrust_magnitude(thrust_raw)
    thrust_filt = first_order_lpf(thrust_mag, err_t, ptau)

    raw_error = baro_err["error"] - pcoef * thrust_filt

    result = dict(baro_err)
    result["error"] = raw_error
    return result


# ---------------------------------------------------------------------------
# Plotting — overview and range-based pages
# ---------------------------------------------------------------------------

def plot_overview(baro_err, thrust_data, armed_start, armed_end,
                  hover_start, hover_end):
    """Page 1: Baro vs range altitude and baro error with thrust overlay."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle("Baro Altitude vs Range Sensor", fontsize=14, fontweight="bold")

    t = baro_err["time_s"]
    err = baro_err["error"]

    # Panel 1: Altitude comparison
    ax = axes[0]
    ax.plot(t, baro_err["range_alt"], label="Range sensor (ground truth)",
            color="tab:green", linewidth=1.2)
    ax.plot(baro_err["baro_full_time_s"], baro_err["baro_full_alt"],
            label="Baro alt (zeroed at arm)", color="tab:red",
            linewidth=1.2, alpha=0.85)
    ax.axvspan(armed_start, armed_end, alpha=0.04, color="green", label="Armed")
    ax.axvspan(hover_start, hover_end, alpha=0.08, color="blue",
               label="Hover window")
    ax.set_ylabel("Altitude [m]")
    ax.legend(loc="upper left", fontsize=9)
    ax.grid(True, alpha=0.3)

    # Panel 2: Baro error
    ax = axes[1]
    ax.plot(t, err, color="tab:red", linewidth=0.8, label="Baro error (baro - range)")
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
    ax.axvspan(hover_start, hover_end, alpha=0.08, color="blue")
    ax.set_ylabel("Baro Error [m]")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Panel 3: Thrust
    ax = axes[2]
    if thrust_data is not None:
        thr_t = thrust_data["time_s"]
        thr_mag = thrust_magnitude(thrust_data["thrust_z"])
        ax.plot(thr_t, thr_mag, color="tab:orange", linewidth=0.8,
                label="Thrust magnitude")
        ax.axvspan(hover_start, hover_end, alpha=0.08, color="blue")
    ax.set_ylabel("Thrust [0-1]")
    ax.set_xlabel("Time [s]")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_calibration(calib):
    """Page 2: Calibration results — xcorr, tau sweep, before/after, params."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Thrust Compensation Calibration", fontsize=14,
                 fontweight="bold")

    # Top-left: Cross-correlation
    ax = axes[0, 0]
    if "xcorr_lags_s" in calib:
        ax.plot(calib["xcorr_lags_s"] * 1000, calib["xcorr_values"],
                color="tab:blue", linewidth=1.0)
        peak_lag = calib["xcorr_peak_lag_s"]
        peak_val = calib["xcorr_peak_value"]
        ax.axvline(peak_lag * 1000, color="tab:red", linestyle="--",
                   linewidth=0.8,
                   label=f"Peak: {peak_lag*1000:.0f} ms (r={peak_val:.2f})")
        ax.axvline(0, color="k", linewidth=0.5, linestyle=":")
        ax.legend(fontsize=9)
    ax.set_xlabel("Lag [ms] (positive = error lags thrust)")
    ax.set_ylabel("Cross-correlation")
    ax.set_title("Cross-Correlation: Thrust vs Baro Error")
    ax.grid(True, alpha=0.3)

    # Top-right: R^2 vs tau
    ax = axes[0, 1]
    if "sweep_tau" in calib:
        ax.plot(calib["sweep_tau"], calib["sweep_r2"],
                "o-", color="tab:green", markersize=3, linewidth=1.0)
        ax.axvline(calib["best_tau"], color="tab:red", linestyle="--",
                   linewidth=0.8,
                   label=f"Best: tau={calib['best_tau']:.2f}s "
                         f"(R\u00b2={calib['best_r2']:.3f})")
        ax.legend(fontsize=9)
    ax.set_xlabel("Time constant tau [s]")
    ax.set_ylabel("R\u00b2")
    ax.set_title("Model Fit vs Filter Time Constant")
    ax.grid(True, alpha=0.3)

    # Bottom-left: Before/after compensation
    ax = axes[1, 0]
    if "hover_time" in calib:
        t = calib["hover_time"]
        ax.plot(t, calib["hover_error"], color="tab:red", linewidth=0.8,
                alpha=0.7, label="Raw baro error")
        ax.plot(t, calib["compensated_error"], color="tab:blue", linewidth=0.8,
                label=f"Compensated (K={calib['best_K']:.2f}, "
                      f"tau={calib['best_tau']:.2f}s)")
        ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
        ax.legend(fontsize=9)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Baro Error [m]")
    ax.set_title("Compensation Effect (Hover Segment)")
    ax.grid(True, alpha=0.3)

    # Bottom-right: Recommended parameters
    ax = axes[1, 1]
    ax.axis("off")
    lines = [
        "Recommended Parameters",
        "",
        f"  SENS_BARO_PCOEF  = {calib['recommended_pcoef']:+.2f}  m",
        f"  SENS_BARO_PTAU   = {calib['recommended_ptau']:.2f}  s",
        "",
        f"  Identified gain K  = {calib['best_K']:.3f} m/unit",
        f"  Time constant tau  = {calib['best_tau']:.3f} s",
        f"  Model R\u00b2           = {calib['best_r2']:.3f}",
        f"  Residual RMSE      = {calib['best_rmse']:.3f} m",
        "",
        f"  No-lag model R\u00b2    = {calib['nolag_r2']:.3f}",
        f"  R\u00b2 improvement     = {calib['best_r2'] - calib['nolag_r2']:.3f}",
    ]
    if "xcorr_peak_lag_s" in calib:
        lines.append(
            f"  Cross-corr peak    = {calib['xcorr_peak_lag_s']*1000:.0f} ms")
    ax.text(0.1, 0.95, "\n".join(lines), transform=ax.transAxes,
            fontsize=11, verticalalignment="top", family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#f0f0f0",
                      edgecolor="#cccccc"))

    plt.tight_layout()
    return fig


def plot_scatter(baro_err, thrust_data, calib, hover_start, hover_end):
    """Page 3: Error vs thrust scatter — raw and compensated."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Baro Error vs Thrust (Hover Segment)", fontsize=14,
                 fontweight="bold")

    t = baro_err["time_s"]
    err = baro_err["error"]
    hov = (t >= hover_start) & (t <= hover_end)

    thrust_raw = np.interp(t[hov], thrust_data["time_s"],
                           thrust_data["thrust_z"])
    thr_mag = thrust_magnitude(thrust_raw)
    err_hov = err[hov]

    # Raw
    ax = axes[0]
    ax.scatter(thr_mag, err_hov, s=3, alpha=0.4, color="tab:orange")
    z = np.polyfit(thr_mag, err_hov, 1)
    x_fit = np.linspace(thr_mag.min(), thr_mag.max(), 50)
    ax.plot(x_fit, np.polyval(z, x_fit), "k--", linewidth=1.2)
    r_val = _safe_corrcoef(thr_mag, err_hov)
    ax.set_xlabel("Thrust magnitude [0-1]")
    ax.set_ylabel("Baro Error [m]")
    ax.set_title(f"Raw\nr = {r_val:.3f},  slope = {z[0]:.2f} m/unit")
    ax.grid(True, alpha=0.3)

    # Compensated
    ax = axes[1]
    thrust_filt = first_order_lpf(thr_mag, t[hov], calib["best_tau"])
    comp_err = err_hov - calib["best_K"] * thrust_filt
    ax.scatter(thr_mag, comp_err, s=3, alpha=0.4, color="tab:blue")
    z = np.polyfit(thr_mag, comp_err, 1)
    ax.plot(x_fit, np.polyval(z, x_fit), "k--", linewidth=1.2)
    r_val = _safe_corrcoef(thr_mag, comp_err)
    ax.set_xlabel("Thrust magnitude [0-1]")
    ax.set_ylabel("Compensated Error [m]")
    ax.set_title(f"After Compensation "
                 f"(PCOEF={calib['recommended_pcoef']:+.2f})\n"
                 f"r = {r_val:.3f},  slope = {z[0]:.2f} m/unit")
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--", alpha=0.3)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_validation(calib, existing_pcoef, existing_ptau, baro_err,
                    thrust_data, hover_start, hover_end):
    """Page 2 for calibrated flights: validate existing compensation."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Compensation Validation (Parameters Active During Flight)",
                 fontsize=14, fontweight="bold")

    # Top-left: Cross-correlation (from reconstructed raw error)
    ax = axes[0, 0]
    if "xcorr_lags_s" in calib:
        ax.plot(calib["xcorr_lags_s"] * 1000, calib["xcorr_values"],
                color="tab:blue", linewidth=1.0)
        peak_lag = calib["xcorr_peak_lag_s"]
        peak_val = calib["xcorr_peak_value"]
        ax.axvline(peak_lag * 1000, color="tab:red", linestyle="--",
                   linewidth=0.8,
                   label=f"Peak: {peak_lag*1000:.0f} ms (r={peak_val:.2f})")
        ax.axvline(0, color="k", linewidth=0.5, linestyle=":")
        ax.legend(fontsize=9)
    ax.set_xlabel("Lag [ms] (positive = error lags thrust)")
    ax.set_ylabel("Cross-correlation")
    ax.set_title("Cross-Correlation: Thrust vs Reconstructed Raw Error")
    ax.grid(True, alpha=0.3)

    # Top-right: R^2 vs tau with active PTAU marker
    ax = axes[0, 1]
    if "sweep_tau" in calib:
        ax.plot(calib["sweep_tau"], calib["sweep_r2"],
                "o-", color="tab:green", markersize=3, linewidth=1.0)
        ax.axvline(calib["best_tau"], color="tab:red", linestyle="--",
                   linewidth=0.8,
                   label=f"Best fit: \u03c4={calib['best_tau']:.2f}s "
                         f"(R\u00b2={calib['best_r2']:.3f})")
        ax.axvline(existing_ptau, color="tab:purple", linestyle=":",
                   linewidth=1.5,
                   label=f"Active: PTAU={existing_ptau:.2f}s")
        ax.legend(fontsize=9)
    ax.set_xlabel("Time constant \u03c4 [s]")
    ax.set_ylabel("R\u00b2")
    ax.set_title("Model Fit vs Filter Time Constant")
    ax.grid(True, alpha=0.3)

    # Bottom-left: Reconstructed raw vs actual compensated
    ax = axes[1, 0]
    err_t = baro_err["time_s"]
    hov = (err_t >= hover_start) & (err_t <= hover_end)

    thrust_raw = np.interp(err_t[hov], thrust_data["time_s"],
                           thrust_data["thrust_z"])
    thr_mag = thrust_magnitude(thrust_raw)
    thrust_filt = first_order_lpf(thr_mag, err_t[hov], existing_ptau)
    raw_err_hov = baro_err["error"][hov] - existing_pcoef * thrust_filt
    comp_err_hov = baro_err["error"][hov]

    ax.plot(err_t[hov], raw_err_hov, color="tab:red", linewidth=0.8,
            alpha=0.7, label="Reconstructed raw error")
    ax.plot(err_t[hov], comp_err_hov, color="tab:blue", linewidth=0.8,
            label="Actual compensated error")
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
    ax.legend(fontsize=9)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Baro Error [m]")
    ax.set_title("Compensation Effect (Hover Segment)")
    ax.grid(True, alpha=0.3)

    # Bottom-right: Validation summary
    ax = axes[1, 1]
    ax.axis("off")

    raw_std = float(np.std(raw_err_hov))
    comp_std = float(np.std(comp_err_hov))
    raw_rmse = float(np.sqrt(np.mean(raw_err_hov**2)))
    comp_rmse = float(np.sqrt(np.mean(comp_err_hov**2)))
    variance_reduction = ((1.0 - comp_std**2 / raw_std**2) * 100
                          if raw_std > 0 else 0)

    r_raw = _safe_corrcoef(thr_mag, raw_err_hov)
    r_comp = _safe_corrcoef(thr_mag, comp_err_hov)

    lines = [
        "Validation Summary",
        "",
        "  Active parameters:",
        f"    SENS_BARO_PCOEF  = {existing_pcoef:+.2f}  m",
        f"    SENS_BARO_PTAU   = {existing_ptau:.2f}  s",
        "",
        "  Re-identified (from reconstructed raw):",
        f"    PCOEF = {calib['recommended_pcoef']:+.2f}  "
        f"(active: {existing_pcoef:+.2f})",
        f"    PTAU  = {calib['recommended_ptau']:.2f}  "
        f"(active: {existing_ptau:.2f})",
        f"    R\u00b2    = {calib['best_r2']:.3f}",
        "",
        "  Error reduction:",
        f"    Raw RMSE         = {raw_rmse:.3f} m",
        f"    Compensated RMSE = {comp_rmse:.3f} m",
        f"    Variance reduced = {variance_reduction:.1f}%",
        "",
        "  Thrust correlation:",
        f"    Raw |r|          = {abs(r_raw):.3f}",
        f"    Compensated |r|  = {abs(r_comp):.3f}",
    ]

    ax.text(0.05, 0.95, "\n".join(lines), transform=ax.transAxes,
            fontsize=11, verticalalignment="top", family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#f0f0f0",
                      edgecolor="#cccccc"))

    plt.tight_layout()
    return fig


def plot_scatter_validation(baro_err, thrust_data, hover_start, hover_end,
                            existing_pcoef, existing_ptau):
    """Page 3 for calibrated flights: reconstructed raw vs actual scatter."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Baro Error vs Thrust \u2014 Compensation Validation "
                 "(Hover Segment)", fontsize=14, fontweight="bold")

    t = baro_err["time_s"]
    hov = (t >= hover_start) & (t <= hover_end)

    thrust_raw = np.interp(t[hov], thrust_data["time_s"],
                           thrust_data["thrust_z"])
    thr_mag = thrust_magnitude(thrust_raw)

    # Reconstruct raw error
    thrust_filt = first_order_lpf(thr_mag, t[hov], existing_ptau)
    comp_err = baro_err["error"][hov]
    raw_err = comp_err - existing_pcoef * thrust_filt

    # Left: Reconstructed raw error
    ax = axes[0]
    ax.scatter(thr_mag, raw_err, s=3, alpha=0.4, color="tab:orange")
    z = np.polyfit(thr_mag, raw_err, 1)
    x_fit = np.linspace(thr_mag.min(), thr_mag.max(), 50)
    ax.plot(x_fit, np.polyval(z, x_fit), "k--", linewidth=1.2)
    r_val = _safe_corrcoef(thr_mag, raw_err)
    ax.set_xlabel("Thrust magnitude [0-1]")
    ax.set_ylabel("Baro Error [m]")
    ax.set_title(f"Reconstructed Raw (Before Compensation)\n"
                 f"r = {r_val:.3f},  slope = {z[0]:.2f} m/unit")
    ax.grid(True, alpha=0.3)

    # Right: Actual compensated (as-flown)
    ax = axes[1]
    ax.scatter(thr_mag, comp_err, s=3, alpha=0.4, color="tab:blue")
    z = np.polyfit(thr_mag, comp_err, 1)
    ax.plot(x_fit, np.polyval(z, x_fit), "k--", linewidth=1.2)
    r_val = _safe_corrcoef(thr_mag, comp_err)
    ax.set_xlabel("Thrust magnitude [0-1]")
    ax.set_ylabel("Compensated Error [m]")
    ax.set_title(f"Actual Flight Data "
                 f"(PCOEF={existing_pcoef:+.2f})\n"
                 f"r = {r_val:.3f},  slope = {z[0]:.2f} m/unit")
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--", alpha=0.3)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Plotting — online estimator review
# ---------------------------------------------------------------------------

def plot_estimator_convergence(est_data, armed_start, armed_end,
                               existing_pcoef=0.0):
    """Estimator convergence: K estimate, tau selection, convergence status."""
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle("Online Estimator Convergence", fontsize=14, fontweight="bold")

    t = est_data["time_s"]
    armed = (t >= armed_start) & (t <= armed_end)
    ta = t[armed]

    # Panel 1: K estimate with variance band
    ax = axes[0]
    k = est_data["k_estimate"][armed]
    k_std = np.sqrt(np.clip(est_data["k_estimate_var"][armed], 0, None))
    ax.plot(ta, k, color="tab:blue", linewidth=1.0, label="K estimate")
    ax.fill_between(ta, k - k_std, k + k_std, alpha=0.15, color="tab:blue",
                    label="\u00b11\u03c3")
    if existing_pcoef != 0.0:
        ax.axhline(-existing_pcoef, color="tab:red", linestyle="--",
                   linewidth=0.8, label=f"Saved PCOEF = {existing_pcoef:+.1f}")
    ax.set_ylabel("K [m/unit thrust]")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Panel 2: Best tau selection
    ax = axes[1]
    tau = est_data["best_tau"][armed]
    ax.plot(ta, tau, color="tab:green", linewidth=1.0,
            drawstyle="steps-post", label="Best tau")
    ax.set_ylabel("Tau [s]")
    ax.set_ylim(-0.01, max(0.25, tau.max() * 1.2))
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Panel 3: Prediction error variance + thrust excitation
    ax = axes[2]
    ax.plot(ta, est_data["best_error_var"][armed], color="tab:orange",
            linewidth=0.8, label="Best bank error var")
    ax.set_ylabel("Error Var / Thrust Std")
    ax2 = ax.twinx()
    ax2.plot(ta, est_data["thrust_std"][armed], color="tab:purple",
             linewidth=0.8, alpha=0.7, label="Thrust std")
    ax2.set_ylabel("Thrust Std", color="tab:purple")
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, fontsize=9)
    ax.grid(True, alpha=0.3)

    # Panel 4: Convergence + estimation active flags
    ax = axes[3]
    conv = est_data["converged"][armed].astype(float)
    active = est_data["estimation_active"][armed].astype(float)
    ax.fill_between(ta, 0, active * 0.5, alpha=0.3, color="tab:blue",
                    step="post", label="Estimation active")
    ax.fill_between(ta, 0.5, 0.5 + conv * 0.5, alpha=0.4, color="tab:green",
                    step="post", label="Converged")
    ax.set_ylim(-0.05, 1.05)
    ax.set_yticks([0, 0.5, 1.0])
    ax.set_yticklabels(["", "", ""])
    ax.set_xlabel("Time [s]")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_estimator_residual(est_data, thrust_data, armed_start, armed_end):
    """Estimator residual analysis: residual vs time and vs thrust."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("CF Residual Analysis (Online Estimator)",
                 fontsize=14, fontweight="bold")

    t = est_data["time_s"]
    armed = (t >= armed_start) & (t <= armed_end)

    residual = est_data["residual"][armed]
    ta = t[armed]

    # Left: Residual time series
    ax = axes[0]
    ax.plot(ta, residual, color="tab:red", linewidth=0.6, alpha=0.7)
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("CF Residual [m]")
    ax.set_title("Residual (Baro - Accel Prediction)")
    ax.grid(True, alpha=0.3)

    # Right: Residual vs thrust scatter
    ax = axes[1]
    if thrust_data is not None:
        thr_interp = thrust_magnitude(
            np.interp(ta, thrust_data["time_s"], thrust_data["thrust_z"]))
        ax.scatter(thr_interp, residual, s=2, alpha=0.3, color="tab:orange")
        if np.std(thr_interp) > 1e-6:
            z = np.polyfit(thr_interp, residual, 1)
            x_fit = np.linspace(thr_interp.min(), thr_interp.max(), 50)
            ax.plot(x_fit, np.polyval(z, x_fit), "k--", linewidth=1.2)
            r_val = _safe_corrcoef(thr_interp, residual)
            ax.set_title(f"Residual vs Thrust\n"
                         f"r = {r_val:.3f},  slope = {z[0]:.2f} m/unit")
        else:
            ax.set_title("Residual vs Thrust")
    ax.set_xlabel("Thrust magnitude [0-1]")
    ax.set_ylabel("CF Residual [m]")
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--", alpha=0.3)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _safe_corrcoef(x, y):
    """Pearson correlation, returning 0 if either input has zero variance."""
    if len(x) < 2 or np.std(x) < 1e-10 or np.std(y) < 1e-10:
        return 0.0
    return float(np.corrcoef(x, y)[0, 1])


def _correlation_quality(r):
    """Return a label for correlation magnitude."""
    ar = abs(r)
    if ar > 0.6:
        return "strong"
    if ar > 0.3:
        return "moderate"
    return "weak"


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Analyze barometer thrust compensation from flight logs")
    parser.add_argument("ulog_file", help="Path to .ulg flight log")
    parser.add_argument("--output-dir", "-o", default=None,
                        help="Output directory (default: same dir as log)")
    args = parser.parse_args()

    if not os.path.isfile(args.ulog_file):
        print(f"Error: file not found: {args.ulog_file}", file=sys.stderr)
        sys.exit(1)

    output_dir = args.output_dir or os.path.dirname(
        os.path.abspath(args.ulog_file))
    os.makedirs(output_dir, exist_ok=True)

    # Load log
    print(f"Loading {args.ulog_file} ...")
    ulog = ULog(args.ulog_file)
    duration = (ulog.last_timestamp - ulog.start_timestamp) / 1e6
    print(f"  Duration: {duration:.1f} s")

    # Print relevant parameters
    print("\nParameters:")
    for p in ["EKF2_HGT_REF", "EKF2_BARO_CTRL", "EKF2_BARO_NOISE",
              "SENS_BARO_PCOEF", "SENS_BARO_PTAU", "SENS_BAR_AUTOCAL"]:
        val = get_param(ulog, p)
        if val is not None:
            print(f"  {p:24s} = {val}")

    # Detect what data is available
    existing_pcoef = float(get_param(ulog, 'SENS_BARO_PCOEF', 0.0))
    existing_ptau = float(get_param(ulog, 'SENS_BARO_PTAU', 0.0))
    is_calibrated = (existing_pcoef != 0.0)

    armed_start, armed_end = detect_armed_period(ulog)
    hover_start, hover_end = find_hover_segment(armed_start, armed_end)
    print(f"\nArmed:  {armed_start:.1f}s - {armed_end:.1f}s")
    print(f"Hover:  {hover_start:.1f}s - {hover_end:.1f}s")

    # Extract data
    baro = extract_baro(ulog)
    range_data = extract_range(ulog)
    thrust_data = extract_thrust(ulog)
    est_data = extract_estimator(ulog)

    if baro is None:
        print("Error: no barometer data (vehicle_air_data) in log",
              file=sys.stderr)
        sys.exit(1)
    if thrust_data is None:
        print("Error: no thrust data (vehicle_thrust_setpoint) in log",
              file=sys.stderr)
        sys.exit(1)

    has_range = range_data is not None
    has_estimator = est_data is not None

    print(f"\n  Baro samples:      {len(baro['time_s'])}")
    print(f"  Thrust samples:    {len(thrust_data['time_s'])}")
    print(f"  Range sensor:      {'yes' if has_range else 'no'}")
    print(f"  Online estimator:  {'yes' if has_estimator else 'no'}")
    if has_range:
        print(f"  Range samples:     {len(range_data['time_s'])}")
    if has_estimator:
        print(f"  Estimator samples: {len(est_data['time_s'])}")

    figures = []

    # ----- Online estimator review (always, when available) -----
    if has_estimator:
        print("\n--- Online Estimator Review ---")

        # Find convergence point
        armed_mask = ((est_data["time_s"] >= armed_start)
                      & (est_data["time_s"] <= armed_end))
        conv_mask = armed_mask & (est_data["converged"] > 0)

        if conv_mask.any():
            conv_idx = np.where(conv_mask)[0][-1]
            final_k = float(est_data["k_estimate"][conv_idx])
            final_tau = float(est_data["best_tau"][conv_idx])
            conv_time = float(est_data["time_s"][conv_idx]) - armed_start
            print(f"  Converged at {conv_time:.0f}s after arm")
            print(f"  Final K = {final_k:.3f},  tau = {final_tau:.3f}s")
            print(f"  Implied PCOEF update: {existing_pcoef:.1f} - {final_k:.2f}"
                  f" = {existing_pcoef - final_k:+.2f}")
        else:
            final_k = float(est_data["k_estimate"][armed_mask][-1]) if armed_mask.any() else 0
            final_tau = float(est_data["best_tau"][armed_mask][-1]) if armed_mask.any() else 0
            print("  Did NOT converge during this flight")
            print(f"  Last K = {final_k:.3f},  tau = {final_tau:.3f}s")

        figures.append(plot_estimator_convergence(
            est_data, armed_start, armed_end, existing_pcoef))
        figures.append(plot_estimator_residual(
            est_data, thrust_data, armed_start, armed_end))

    # ----- Range-sensor-based analysis (when range available) -----
    if has_range:
        baro_err = compute_baro_error(baro, range_data, armed_start)
        hov = ((baro_err["time_s"] >= hover_start)
               & (baro_err["time_s"] <= hover_end))

        if hov.sum() < 20:
            print("\nWarning: not enough data in hover segment for "
                  "range-based analysis", file=sys.stderr)

        else:
            err_hov = baro_err["error"][hov]

            if is_calibrated:
                # --- VALIDATION MODE ---
                print(f"\n--- Range-Based Validation ---")
                print(f"  Compensation was ACTIVE (PCOEF={existing_pcoef:+.1f},"
                      f" PTAU={existing_ptau:.2f})")

                print(f"  Observed error:  mean={np.mean(err_hov):+.3f}m,"
                      f" std={np.std(err_hov):.3f}m")

                raw_baro_err = reconstruct_raw_error(
                    baro_err, thrust_data, existing_pcoef, existing_ptau)
                raw_hov = raw_baro_err["error"][hov]
                print(f"  Reconstructed raw: mean={np.mean(raw_hov):+.3f}m,"
                      f" std={np.std(raw_hov):.3f}m")

                thrust_interp_mag = thrust_magnitude(
                    np.interp(baro_err["time_s"][hov],
                              thrust_data["time_s"], thrust_data["thrust_z"]))
                r_raw = _safe_corrcoef(thrust_interp_mag, raw_hov)
                r_comp = _safe_corrcoef(thrust_interp_mag, err_hov)
                print(f"  Thrust |r|: raw={abs(r_raw):.3f} "
                      f"({_correlation_quality(r_raw)}), "
                      f"compensated={abs(r_comp):.3f} "
                      f"({_correlation_quality(r_comp)})")

                calib = calibrate(raw_baro_err, thrust_data,
                                  hover_start, hover_end)
                if calib is not None:
                    print(f"  Re-identified: K={calib['best_K']:.3f},"
                          f" tau={calib['best_tau']:.3f},"
                          f" R\u00b2={calib['best_r2']:.3f}")

                    # Cross-validate with online estimator
                    if has_estimator and conv_mask.any():
                        print(f"\n--- Cross-Validation ---")
                        print(f"  Online:  K={final_k:.3f},"
                              f" tau={final_tau:.3f}")
                        print(f"  Offline: K={calib['best_K']:.3f},"
                              f" tau={calib['best_tau']:.3f}")
                        k_diff = abs(final_k - calib['best_K'])
                        print(f"  K difference: {k_diff:.3f}"
                              f" ({'good' if k_diff < 1.0 else 'notable'})")

                    figures.append(plot_overview(
                        baro_err, thrust_data, armed_start, armed_end,
                        hover_start, hover_end))
                    figures.append(plot_validation(
                        calib, existing_pcoef, existing_ptau, baro_err,
                        thrust_data, hover_start, hover_end))
                    figures.append(plot_scatter_validation(
                        baro_err, thrust_data, hover_start, hover_end,
                        existing_pcoef, existing_ptau))

            else:
                # --- CALIBRATION MODE ---
                print(f"\n--- Range-Based Calibration ---")
                print(f"  No compensation active")
                print(f"  Baro error: mean={np.mean(err_hov):+.3f}m,"
                      f" std={np.std(err_hov):.3f}m")

                thrust_interp = np.interp(
                    baro_err["time_s"][hov], thrust_data["time_s"],
                    thrust_data["thrust_z"])
                r_thrust = _safe_corrcoef(thrust_interp, err_hov)
                print(f"  Thrust correlation: r={r_thrust:+.3f}"
                      f" ({_correlation_quality(r_thrust)})")

                calib = calibrate(baro_err, thrust_data,
                                  hover_start, hover_end)
                if calib is not None:
                    print(f"\n  Identified K  = {calib['best_K']:.3f}")
                    print(f"  Identified tau = {calib['best_tau']:.3f}s")
                    print(f"  Model R\u00b2     = {calib['best_r2']:.3f}")
                    print(f"  Residual RMSE = {calib['best_rmse']:.3f}m")

                    print(f"\n{'='*50}")
                    print(f"  RECOMMENDED PARAMETERS")
                    print(f"{'='*50}")
                    print(f"  SENS_BARO_PCOEF = {calib['recommended_pcoef']:+.2f}")
                    print(f"  SENS_BARO_PTAU  = {calib['recommended_ptau']:.2f}")
                    print(f"{'='*50}")

                    # Cross-validate with online estimator
                    if has_estimator and conv_mask.any():
                        print(f"\n--- Cross-Validation ---")
                        print(f"  Online:  K={final_k:.3f},"
                              f" tau={final_tau:.3f}")
                        print(f"  Offline: K={calib['best_K']:.3f},"
                              f" tau={calib['best_tau']:.3f}")

                    figures.append(plot_overview(
                        baro_err, thrust_data, armed_start, armed_end,
                        hover_start, hover_end))
                    figures.append(plot_calibration(calib))
                    figures.append(plot_scatter(
                        baro_err, thrust_data, calib,
                        hover_start, hover_end))
                else:
                    print("  Calibration failed: insufficient data or "
                          "thrust variation.")

    elif not has_estimator:
        print("\nError: no range sensor and no online estimator data.\n"
              "Need at least one of: distance_sensor or baro_thrust_estimate",
              file=sys.stderr)
        sys.exit(1)

    # Write PDF
    if figures:
        pdf_path = os.path.join(output_dir, "baro_calibration.pdf")
        with PdfPages(pdf_path) as pdf:
            for fig in figures:
                pdf.savefig(fig)
                plt.close(fig)
        print(f"\n  Saved: {pdf_path}")
    else:
        print("\nNo plots generated.")

    print("\nDone.")


if __name__ == "__main__":
    main()
