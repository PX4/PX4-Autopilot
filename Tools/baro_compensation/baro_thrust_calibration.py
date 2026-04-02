#!/usr/bin/env python3
"""
Barometer thrust compensation analysis.

Three-way comparison of K estimates:
  1. Online CF+RLS (from logged baro_thrust_estimate)
  2. Offline CF+RLS replay (baro + accel, mirroring the firmware algorithm)
  3. Distance sensor ground truth (baro vs range, if available)

The correction model: baro_alt += SENS_BARO_PCOEF * |thrust_z|

Usage:
    python3 baro_thrust_calibration.py <log.ulg> [--output-dir <dir>]
        If --output-dir is not given, results go to logs/<log_name>/ in the PX4 root.
"""

import argparse
import os
import shutil
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

GRAVITY = 9.80665


# ---------------------------------------------------------------------------
# ULog helpers
# ---------------------------------------------------------------------------

def get_topic(ulog, name, multi_id=0):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == multi_id:
            return d
    return None


def get_param(ulog, name, default=None):
    return ulog.initial_parameters.get(name, default)


def us_to_s(ts_us, start_us):
    return (ts_us.astype(np.int64) - np.int64(start_us)) / 1e6


def effective_rate(time_s):
    if len(time_s) < 2:
        return 0.0
    dt = np.diff(time_s)
    dt = dt[dt > 0]
    return 1.0 / np.median(dt) if len(dt) > 0 else 0.0


def safe_corrcoef(x, y):
    if len(x) < 2 or np.std(x) < 1e-10 or np.std(y) < 1e-10:
        return 0.0
    return float(np.corrcoef(x, y)[0, 1])


# ---------------------------------------------------------------------------
# Data extraction
# ---------------------------------------------------------------------------

def extract_baro(ulog):
    d = get_topic(ulog, "vehicle_air_data")
    if d is None:
        return None
    return {
        "time_s": us_to_s(d.data["timestamp_sample"], ulog.start_timestamp),
        "alt_m": d.data["baro_alt_meter"],
    }


def extract_accel(ulog):
    d = get_topic(ulog, "vehicle_acceleration")
    if d is None:
        return None
    return {
        "time_s": us_to_s(d.data["timestamp_sample"], ulog.start_timestamp),
        "x": d.data["xyz[0]"],
        "y": d.data["xyz[1]"],
        "z": d.data["xyz[2]"],
    }


def extract_attitude(ulog):
    d = get_topic(ulog, "vehicle_attitude")
    if d is None:
        return None
    return {
        "time_s": us_to_s(d.data["timestamp_sample"], ulog.start_timestamp),
        "qw": d.data["q[0]"],
        "qx": d.data["q[1]"],
        "qy": d.data["q[2]"],
        "qz": d.data["q[3]"],
    }


def extract_thrust(ulog):
    d = get_topic(ulog, "vehicle_thrust_setpoint")
    if d is None:
        return None
    z = d.data.get("xyz[2]", None)
    if z is None:
        return None
    return {
        "time_s": us_to_s(d.data["timestamp"], ulog.start_timestamp),
        "thrust": np.where(np.isfinite(z), np.abs(z), 0.0),
    }


def extract_range(ulog):
    d = get_topic(ulog, "distance_sensor")
    if d is None:
        return None
    t = us_to_s(d.data["timestamp"], ulog.start_timestamp)
    dist = d.data["current_distance"]
    valid = np.isfinite(t) & np.isfinite(dist)
    if "signal_quality" in d.data:
        valid &= d.data["signal_quality"] > 0
    if valid.sum() < 2:
        return None
    return {"time_s": t[valid], "distance_m": dist[valid]}


def extract_online_estimate(ulog):
    d = get_topic(ulog, "baro_thrust_estimate")
    if d is None:
        return None
    return {
        "time_s": us_to_s(d.data["timestamp"], ulog.start_timestamp),
        "residual": d.data["residual"],
        "k_estimate": d.data["k_estimate"],
        "k_estimate_var": d.data["k_estimate_var"],
        "error_var": d.data.get("error_var",
                                np.zeros(len(d.data["timestamp"]))),
        "thrust_std": d.data["thrust_std"],
        "converged": d.data["converged"],
        "estimation_active": d.data["estimation_active"],
    }


def extract_ekf_z(ulog):
    d = get_topic(ulog, "vehicle_local_position")
    if d is None:
        return None
    result = {
        "time_s": us_to_s(d.data["timestamp"], ulog.start_timestamp),
        "z": d.data["z"],
    }
    if "vz" in d.data:
        result["vz"] = d.data["vz"]
    if "vx" in d.data and "vy" in d.data:
        result["vxy"] = np.sqrt(d.data["vx"]**2 + d.data["vy"]**2)
    return result


def extract_ekf_baro_obs(ulog):
    d = get_topic(ulog, "estimator_aid_src_baro_hgt")
    if d is None:
        return None
    return {
        "time_s": us_to_s(d.data["timestamp"], ulog.start_timestamp),
        "observation": d.data["observation"],
    }


def extract_landed(ulog):
    d = get_topic(ulog, "vehicle_land_detected")
    if d is None:
        return None
    return {
        "time_s": us_to_s(d.data["timestamp"], ulog.start_timestamp),
        "landed": d.data["landed"].astype(bool),
    }


def detect_armed_period(ulog):
    start_us = ulog.start_timestamp
    vstatus = get_topic(ulog, "vehicle_status")
    if vstatus is not None and "arming_state" in vstatus.data:
        ts = us_to_s(vstatus.data["timestamp"], start_us)
        armed_idx = np.where(vstatus.data["arming_state"] == 2)[0]
        if len(armed_idx) > 0:
            return float(ts[armed_idx[0]]), float(ts[armed_idx[-1]])
    motors = get_topic(ulog, "actuator_motors")
    if motors is not None:
        ts = us_to_s(motors.data["timestamp"], start_us)
        active = np.zeros(len(ts), dtype=bool)
        for i in range(12):
            key = f"control[{i}]"
            if key in motors.data:
                active |= (motors.data[key] > 0.05)
        active_idx = np.where(active)[0]
        if len(active_idx) > 0:
            return float(ts[active_idx[0]]), float(ts[active_idx[-1]])
    return 0.0, float((ulog.last_timestamp - start_us) / 1e6)


# ---------------------------------------------------------------------------
# Offline CF+RLS estimator  (Python port of baro_thrust_cf_rls.cpp)
# ---------------------------------------------------------------------------

def compute_accel_up(ax, ay, az, qw, qx, qy, qz):
    """Vectorized: body-frame specific force + quaternion -> upward linear accel.

    Rotates body accel to NED (3rd row of quaternion DCM), then converts:
        accel_up = -(specific_force_ned_z + g)
    """
    ned_z = ((2 * (qx * qz - qw * qy)) * ax
             + (2 * (qy * qz + qw * qx)) * ay
             + (1 - 2 * (qx**2 + qy**2)) * az)
    return -(ned_z + GRAVITY)


class CfRls:
    """Python port of BaroThrustCfRls.  Constants match baro_thrust_cf_rls.hpp."""

    # Defaults (matching baro_thrust_cf_rls.hpp)
    DEFAULT_CF_BANDWIDTH = 0.05
    DEFAULT_RLS_LAMBDA = 0.998
    RLS_P_INIT = 100.0
    ERROR_VAR_INIT = 10.0
    ALPHA_ERR = 0.01

    def __init__(self, cf_bandwidth=None, rls_lambda=None):
        bw = cf_bandwidth if cf_bandwidth is not None else self.DEFAULT_CF_BANDWIDTH
        self.CF_OMEGA = 2.0 * np.pi * bw
        self.CF_K1 = 2.0 * self.CF_OMEGA
        self.CF_K2 = self.CF_OMEGA ** 2
        self.RLS_LAMBDA = (rls_lambda if rls_lambda is not None
                           else self.DEFAULT_RLS_LAMBDA)
        self.reset()

    def reset(self):
        self.cf_alt = 0.0
        self.cf_vel = 0.0
        self.cf_init = False
        self.theta = np.zeros(2)                # [K, bias]
        self.P = np.eye(2) * self.RLS_P_INIT
        self.error_var = self.ERROR_VAR_INIT
        self._thrust_mean = 0.0
        self._thrust_var = 0.0
        self._k_smoothed = 0.0

    def update_cf(self, baro_alt, accel_up, dt):
        if not self.cf_init:
            self.cf_alt = baro_alt
            self.cf_vel = 0.0
            self.cf_init = True
            return 0.0
        alt_pred = self.cf_alt + self.cf_vel * dt + 0.5 * accel_up * dt * dt
        vel_pred = self.cf_vel + accel_up * dt
        residual = baro_alt - alt_pred
        self.cf_alt = alt_pred + self.CF_K1 * dt * residual
        self.cf_vel = vel_pred + self.CF_K2 * dt * residual
        if not (np.isfinite(self.cf_alt) and np.isfinite(self.cf_vel)):
            self.cf_alt = baro_alt
            self.cf_vel = 0.0
            return 0.0
        return residual

    def update_rls(self, residual, thrust, dt):
        phi = np.array([thrust, 1.0])
        e = residual - self.theta @ phi

        Pphi = self.P @ phi
        denom = self.RLS_LAMBDA + phi @ Pphi
        if abs(denom) < 1e-10:
            return
        inv = 1.0 / denom

        self.theta += Pphi * inv * e
        self.P = (self.P - np.outer(Pphi, Pphi) * inv) / self.RLS_LAMBDA
        self.error_var = (1 - self.ALPHA_ERR) * self.error_var + self.ALPHA_ERR * e * e

        # Thrust excitation tracking (deviation computed before mean update)
        alpha = dt / (2.0 + dt)
        dev = thrust - self._thrust_mean
        self._thrust_mean = (1 - alpha) * self._thrust_mean + alpha * thrust
        self._thrust_var = (1 - alpha) * self._thrust_var + alpha * dev * dev

        # K smoothing
        alpha_k = dt / (5.0 + dt)
        self._k_smoothed = (1 - alpha_k) * self._k_smoothed + alpha_k * self.theta[0]

        if not (np.isfinite(self.theta).all() and np.isfinite(self.P).all()
                and np.isfinite(self.error_var)):
            self.reset()

    @property
    def k(self):
        return float(self.theta[0])

    @property
    def k_var(self):
        return float(self.P[0, 0])

    @property
    def thrust_std(self):
        return float(np.sqrt(max(self._thrust_var, 0.0)))


def build_estimation_mask(baro_t, armed_start, armed_end, landed,
                          ekf_z=None, range_data=None):
    """Build sample mask matching firmware hard + soft guards.

    Hard gates: armed AND not landed.
    Soft gates (skip RLS but keep CF in firmware — here we skip both since
    the offline replay doesn't separate CF from RLS per-sample):
      - |vz| > 2 m/s
      - vxy > 5 m/s
      - range sensor < 0.5 m (ground effect proximity)
    """
    armed = (baro_t >= armed_start) & (baro_t <= armed_end)

    if landed is not None:
        is_landed = (np.interp(baro_t, landed["time_s"],
                               landed["landed"].astype(float)) > 0.5)
    else:
        is_landed = np.zeros(len(baro_t), dtype=bool)

    mask = armed & ~is_landed

    if ekf_z is not None and "vz" in ekf_z:
        vz = np.interp(baro_t, ekf_z["time_s"], ekf_z["vz"])
        mask &= np.abs(vz) <= 2.0

    if ekf_z is not None and "vxy" in ekf_z:
        vxy = np.interp(baro_t, ekf_z["time_s"], ekf_z["vxy"])
        mask &= vxy <= 5.0

    if range_data is not None:
        rng = np.interp(baro_t, range_data["time_s"],
                        range_data["distance_m"])
        mask &= rng > 0.5

    return mask


def run_offline_cf_rls(baro, accel, attitude, thrust, landed,
                       armed_start, armed_end,
                       cf_bandwidth=None, rls_lambda=None,
                       ekf_z=None, range_data=None):
    """Replay CF+RLS on logged sensor data.  Returns K trace and residuals."""
    baro_t = baro["time_s"]
    baro_alt = baro["alt_m"]

    # Compute accel_up at accel timestamps, then interp to baro timestamps
    qw = np.interp(accel["time_s"], attitude["time_s"], attitude["qw"])
    qx = np.interp(accel["time_s"], attitude["time_s"], attitude["qx"])
    qy = np.interp(accel["time_s"], attitude["time_s"], attitude["qy"])
    qz = np.interp(accel["time_s"], attitude["time_s"], attitude["qz"])
    accel_up_all = compute_accel_up(accel["x"], accel["y"], accel["z"],
                                    qw, qx, qy, qz)

    accel_up = np.interp(baro_t, accel["time_s"], accel_up_all)
    thrust_interp = np.interp(baro_t, thrust["time_s"], thrust["thrust"])

    airborne = build_estimation_mask(baro_t, armed_start, armed_end, landed,
                                     ekf_z, range_data)

    est = CfRls(cf_bandwidth=cf_bandwidth, rls_lambda=rls_lambda)
    n = len(baro_t)
    k_trace = np.full(n, np.nan)
    k_var_trace = np.full(n, np.nan)
    residual = np.full(n, np.nan)
    error_var = np.full(n, np.nan)
    thrust_std = np.full(n, np.nan)

    prev_t = None
    for i in range(n):
        if not airborne[i]:
            continue
        if prev_t is None:
            prev_t = baro_t[i]
            residual[i] = 0.0
            k_trace[i] = est.k
            k_var_trace[i] = est.k_var
            error_var[i] = est.error_var
            thrust_std[i] = est.thrust_std
            continue

        dt = float(np.clip(baro_t[i] - prev_t, 0.001, 0.5))
        prev_t = baro_t[i]

        res = est.update_cf(float(baro_alt[i]), float(accel_up[i]), dt)
        est.update_rls(res, float(thrust_interp[i]), dt)

        residual[i] = res
        k_trace[i] = est.k
        k_var_trace[i] = est.k_var
        error_var[i] = est.error_var
        thrust_std[i] = est.thrust_std

    return {
        "time_s": baro_t,
        "k_trace": k_trace,
        "k_var_trace": k_var_trace,
        "residual": residual,
        "error_var": error_var,
        "thrust_std": thrust_std,
        "final_k": est.k,
        "final_k_var": est.k_var,
    }


# ---------------------------------------------------------------------------
# Range-based calibration  (distance sensor ground truth)
# ---------------------------------------------------------------------------

def run_range_calibration(baro, range_data, thrust,
                          armed_start, armed_end, existing_pcoef=0.0):
    """Least-squares calibration using range sensor as ground truth.

    Undoes existing PCOEF compensation to recover raw baro, then fits:
        raw_baro_error = K_total * thrust + c

    Returns K_total (from-scratch gain), K_residual (remaining after existing
    PCOEF), and data arrays for plotting.
    """
    baro_t, baro_alt = baro["time_s"], baro["alt_m"]
    rng_t, rng_dist = range_data["time_s"], range_data["distance_m"]

    # Zero baro at arm time
    baro_zeroed = baro_alt - np.interp(armed_start, baro_t, baro_alt)

    # Undo existing compensation at baro timestamps
    thrust_at_baro = np.interp(baro_t, thrust["time_s"], thrust["thrust"])
    raw_baro = baro_zeroed - existing_pcoef * thrust_at_baro

    # Interpolate both baro versions to range timestamps
    raw_baro_interp = np.interp(rng_t, baro_t, raw_baro)
    comp_baro_interp = np.interp(rng_t, baro_t, baro_zeroed)
    raw_error = raw_baro_interp - rng_dist
    comp_error = comp_baro_interp - rng_dist

    # Filter: armed, above ground proximity
    MIN_RANGE = 0.5
    mask = ((rng_t >= armed_start) & (rng_t <= armed_end)
            & (rng_dist > MIN_RANGE))
    if mask.sum() < 20:
        return None

    t_fit = rng_t[mask]
    raw_err_fit = raw_error[mask]
    comp_err_fit = comp_error[mask]
    thrust_fit = np.interp(t_fit, thrust["time_s"], thrust["thrust"])

    # Fit on raw (uncompensated) error -> K_total
    A = np.column_stack([thrust_fit, np.ones(len(thrust_fit))])
    coeffs, _, _, _ = np.linalg.lstsq(A, raw_err_fit, rcond=None)
    K_total = float(coeffs[0])

    raw_var = np.var(raw_err_fit)
    resid = raw_err_fit - A @ coeffs
    r2 = 1.0 - np.var(resid) / raw_var if raw_var > 1e-10 else 0.0

    # Compensated fit for comparison
    coeffs_comp, _, _, _ = np.linalg.lstsq(A, comp_err_fit, rcond=None)

    K_residual = K_total + existing_pcoef

    return {
        "K_total": K_total,
        "K_residual": K_residual,
        "r2": float(r2),
        "rmse": float(np.sqrt(np.mean(resid ** 2))),
        "pcoef_recommended": -K_total,
        # Plotting data
        "time_s": rng_t,
        "raw_error": raw_error,
        "comp_error": comp_error,
        "mask": mask,
        "t_fit": t_fit,
        "raw_err_fit": raw_err_fit,
        "comp_err_fit": comp_err_fit,
        "thrust_fit": thrust_fit,
        "raw_coeffs": coeffs,
        "comp_coeffs": coeffs_comp,
    }


# ---------------------------------------------------------------------------
# CF+RLS parameter sweep
# ---------------------------------------------------------------------------

def sweep_cf_params(baro, accel, attitude, thrust, landed,
                    armed_start, armed_end, range_cal, existing_pcoef,
                    ekf_z=None, range_data=None):
    """Sweep CF bandwidth at two lambda values, evaluate against range truth."""
    bandwidths = np.logspace(np.log10(0.01), np.log10(1.0), 30)
    lambdas = [("lambda_0.998", 0.998), ("lambda_1.0", 1.0)]

    # Precompute shared data (same for all parameter combinations)
    baro_t = baro["time_s"]
    baro_alt = baro["alt_m"]

    qw = np.interp(accel["time_s"], attitude["time_s"], attitude["qw"])
    qx = np.interp(accel["time_s"], attitude["time_s"], attitude["qx"])
    qy = np.interp(accel["time_s"], attitude["time_s"], attitude["qy"])
    qz = np.interp(accel["time_s"], attitude["time_s"], attitude["qz"])
    accel_up_all = compute_accel_up(accel["x"], accel["y"], accel["z"],
                                    qw, qx, qy, qz)
    accel_up = np.interp(baro_t, accel["time_s"], accel_up_all)
    thrust_interp = np.interp(baro_t, thrust["time_s"], thrust["thrust"])

    mask = build_estimation_mask(baro_t, armed_start, armed_end, landed,
                                 ekf_z, range_data)
    airborne_idx = np.where(mask)[0]

    raw_err = range_cal["raw_err_fit"]
    thr_fit = range_cal["thrust_fit"]

    results = {"bandwidth": bandwidths}

    for lam_key, lam in lambdas:
        k_list = []
        std_list = []

        for bw in bandwidths:
            est = CfRls(cf_bandwidth=bw, rls_lambda=lam)
            prev_t = None
            for i in airborne_idx:
                if prev_t is None:
                    prev_t = baro_t[i]
                    continue
                dt = float(np.clip(baro_t[i] - prev_t, 0.001, 0.5))
                prev_t = baro_t[i]
                res = est.update_cf(float(baro_alt[i]),
                                    float(accel_up[i]), dt)
                est.update_rls(res, float(thrust_interp[i]), dt)

            total_pcoef = existing_pcoef - est.k
            corrected = raw_err + total_pcoef * thr_fit

            k_list.append(est.k)
            std_list.append(float(np.std(corrected)))

        results[lam_key] = {
            "k_residual": np.array(k_list),
            "comp_error_std": np.array(std_list),
        }

    return results


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

_SUBTITLE_Y = 0.94
_LAYOUT_TOP = 0.93


def plot_altitude_overview(ekf_baro_obs, ekf_z, range_data, thrust_data,
                           armed_start, armed_end):
    """Page 1: Altitude overview — baro, range, EKF, thrust."""
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("Altitude Overview", fontsize=14, fontweight="bold")

    ax = axes[0]
    if range_data is not None:
        ax.plot(range_data["time_s"], range_data["distance_m"],
                label="Distance sensor", color="tab:blue", linewidth=1.2)
    if ekf_baro_obs is not None:
        bt = ekf_baro_obs["time_s"]
        ba = -ekf_baro_obs["observation"]
        ba -= np.interp(armed_start, bt, ba)
        ax.plot(bt, ba, label="Baro observation (EKF input)",
                color="tab:red", linewidth=1.0, alpha=0.8)
    if ekf_z is not None:
        et = ekf_z["time_s"]
        ea = -ekf_z["z"]
        ea -= np.interp(armed_start, et, ea)
        ax.plot(et, ea, label="EKF altitude (-Z)",
                color="tab:green", linewidth=1.0, alpha=0.8)
    ax.axvspan(armed_start, armed_end, alpha=0.04, color="green", label="Armed")
    ax.set_ylabel("Altitude AGL [m]")
    ax.legend(fontsize=9, loc="upper left")
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(thrust_data["time_s"], thrust_data["thrust"],
            color="tab:orange", linewidth=0.8)
    ax.set_ylabel("Thrust |z| [0-1]")
    ax.set_xlabel("Time [s]")
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    return fig


def plot_k_convergence(online, offline, range_cal,
                       armed_start, armed_end, existing_pcoef):
    """Page 2: K estimate convergence — online + offline overlaid."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle("K Estimate Convergence", fontsize=14, fontweight="bold")
    fig.text(0.5, _SUBTITLE_Y,
             "Online (firmware) and offline (replay) CF+RLS. "
             "Range K shown as ground-truth reference.",
             ha="center", va="top", fontsize=9, style="italic", color="0.4")

    # --- Panel 1: K traces with variance bands ---
    ax = axes[0]
    if online is not None:
        t = online["time_s"]
        m = (t >= armed_start) & (t <= armed_end)
        k = online["k_estimate"][m]
        ks = np.sqrt(np.clip(online["k_estimate_var"][m], 0, None))
        ax.plot(t[m], k, color="tab:blue", linewidth=1.0, label="Online K")
        ax.fill_between(t[m], k - ks, k + ks, alpha=0.1, color="tab:blue")

    if offline is not None:
        t = offline["time_s"]
        v = np.isfinite(offline["k_trace"])
        k = offline["k_trace"][v]
        ks = np.sqrt(np.clip(offline["k_var_trace"][v], 0, None))
        ax.plot(t[v], k, color="tab:red", linewidth=1.0,
                label="Offline K", alpha=0.8)
        ax.fill_between(t[v], k - ks, k + ks, alpha=0.1, color="tab:red")

    if range_cal is not None:
        ax.axhline(range_cal["K_residual"], color="tab:green", linestyle="--",
                   linewidth=1.0,
                   label=f"Range K\u2081 = {range_cal['K_residual']:.2f}")

    ax.set_ylabel("K [m / unit thrust]")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # --- Panel 2: Error variance ---
    ax = axes[1]
    if online is not None:
        t = online["time_s"]
        m = (t >= armed_start) & (t <= armed_end)
        ax.plot(t[m], online["error_var"][m], color="tab:blue",
                linewidth=0.8, label="Online")
    if offline is not None:
        t = offline["time_s"]
        v = np.isfinite(offline["error_var"])
        ax.plot(t[v], offline["error_var"][v], color="tab:red",
                linewidth=0.8, label="Offline", alpha=0.8)
    ax.set_ylabel("Error Variance [m\u00b2]")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # --- Panel 3: Thrust excitation + convergence flags ---
    ax = axes[2]
    if online is not None:
        t = online["time_s"]
        m = (t >= armed_start) & (t <= armed_end)
        conv = online["converged"][m].astype(float)
        ax.fill_between(t[m], 0, conv * 0.08, alpha=0.3, color="tab:green",
                        step="post", label="Converged")
        ax.plot(t[m], online["thrust_std"][m], color="tab:blue",
                linewidth=0.8, label="Online thrust std")
    if offline is not None:
        t = offline["time_s"]
        v = np.isfinite(offline["thrust_std"])
        ax.plot(t[v], offline["thrust_std"][v], color="tab:red",
                linewidth=0.8, label="Offline thrust std", alpha=0.8)
    ax.axhline(0.05, color="k", linestyle=":", linewidth=0.5,
               label="Min excitation (0.05)")
    ax.set_ylabel("Thrust Std")
    ax.set_xlabel("Time [s]")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, _LAYOUT_TOP])
    return fig


def plot_residual_comparison(online, offline, thrust,
                             armed_start, armed_end):
    """Page 3: CF residual vs thrust — online and offline side by side."""
    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1])
    fig.suptitle("CF Residual Analysis", fontsize=14, fontweight="bold")
    fig.text(0.5, _SUBTITLE_Y,
             "Top: residual time series (online blue, offline red). "
             "Bottom: residual vs thrust scatter. Slope = K.",
             ha="center", va="top", fontsize=9, style="italic", color="0.4")

    # --- Top: overlaid time series ---
    ax_ts = fig.add_subplot(gs[0, :])
    if online is not None:
        t = online["time_s"]
        m = (t >= armed_start) & (t <= armed_end)
        ax_ts.plot(t[m], online["residual"][m], color="tab:blue",
                   linewidth=0.6, alpha=0.7, label="Online")
    if offline is not None:
        t = offline["time_s"]
        v = np.isfinite(offline["residual"])
        ax_ts.plot(t[v], offline["residual"][v], color="tab:red",
                   linewidth=0.6, alpha=0.7, label="Offline")
    ax_ts.axhline(0, color="k", linewidth=0.5, linestyle="--")
    ax_ts.set_ylabel("CF Residual [m]")
    ax_ts.set_xlabel("Time [s]")
    ax_ts.legend(fontsize=9)
    ax_ts.grid(True, alpha=0.3)

    # --- Bottom: scatter plots ---
    datasets = [
        (online, "Online", "tab:blue", gs[1, 0]),
        (offline, "Offline", "tab:red", gs[1, 1]),
    ]
    for data, label, color, gs_pos in datasets:
        ax = fig.add_subplot(gs_pos)
        if data is None:
            ax.text(0.5, 0.5, f"No {label.lower()} data",
                    transform=ax.transAxes, ha="center", va="center")
            ax.set_title(label)
            continue

        t = data["time_s"]
        res = data["residual"]
        m = (t >= armed_start) & (t <= armed_end) & np.isfinite(res)
        res_m = res[m]
        thr_m = np.interp(t[m], thrust["time_s"], thrust["thrust"])

        ax.scatter(thr_m, res_m, s=2, alpha=0.3, color=color)
        if np.std(thr_m) > 1e-6 and len(thr_m) > 5:
            z = np.polyfit(thr_m, res_m, 1)
            x_fit = np.linspace(thr_m.min(), thr_m.max(), 50)
            ax.plot(x_fit, np.polyval(z, x_fit), "k--", linewidth=1.2)
            r = safe_corrcoef(thr_m, res_m)
            ax.set_title(f"{label}: slope = {z[0]:.2f}, r = {r:.3f}")
        else:
            ax.set_title(label)
        ax.set_xlabel("Thrust [0-1]")
        ax.set_ylabel("CF Residual [m]")
        ax.axhline(0, color="k", linewidth=0.5, linestyle="--", alpha=0.3)
        ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, _LAYOUT_TOP])
    return fig


def plot_ground_truth(range_cal, existing_pcoef):
    """Page 4: Range-sensor ground truth — raw and compensated error."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Ground Truth Validation (Distance Sensor)",
                 fontsize=14, fontweight="bold")
    fig.text(0.5, _SUBTITLE_Y,
             "Left: raw baro error (compensation undone). "
             "Right: as-flown (with existing PCOEF). "
             "Slope = thrust-correlated error.",
             ha="center", va="top", fontsize=9, style="italic", color="0.4")

    thrust = range_cal["thrust_fit"]
    x_fit = np.linspace(thrust.min(), thrust.max(), 50)

    # Top-left: Raw error vs thrust scatter
    ax = axes[0, 0]
    ax.scatter(thrust, range_cal["raw_err_fit"], s=2, alpha=0.3,
               color="tab:orange")
    c = range_cal["raw_coeffs"]
    ax.plot(x_fit, c[0] * x_fit + c[1], "k--", linewidth=1.2)
    r = safe_corrcoef(thrust, range_cal["raw_err_fit"])
    ax.set_title(f"Raw: K = {c[0]:.1f}, r = {r:.3f}, R\u00b2 = {range_cal['r2']:.3f}")
    ax.set_xlabel("Thrust [0-1]")
    ax.set_ylabel("Baro Error [m]")
    ax.grid(True, alpha=0.3)

    # Top-right: Compensated error vs thrust scatter
    ax = axes[0, 1]
    ax.scatter(thrust, range_cal["comp_err_fit"], s=2, alpha=0.3,
               color="tab:blue")
    cc = range_cal["comp_coeffs"]
    ax.plot(x_fit, cc[0] * x_fit + cc[1], "k--", linewidth=1.2)
    r = safe_corrcoef(thrust, range_cal["comp_err_fit"])
    ax.set_title(f"Compensated (PCOEF={existing_pcoef:+.1f}): "
                 f"slope = {cc[0]:.2f}, r = {r:.3f}")
    ax.set_xlabel("Thrust [0-1]")
    ax.set_ylabel("Baro Error [m]")
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--", alpha=0.3)
    ax.grid(True, alpha=0.3)

    # Sync top row y-axes
    ylim = [min(axes[0, 0].get_ylim()[0], axes[0, 1].get_ylim()[0]),
            max(axes[0, 0].get_ylim()[1], axes[0, 1].get_ylim()[1])]
    axes[0, 0].set_ylim(ylim)
    axes[0, 1].set_ylim(ylim)

    # Bottom-left: Raw error time series
    mask = range_cal["mask"]
    t = range_cal["t_fit"]
    ax = axes[1, 0]
    ax.plot(t, range_cal["raw_err_fit"], color="tab:orange", linewidth=0.8)
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
    std_raw = float(np.std(range_cal["raw_err_fit"]))
    ax.set_title(f"Raw error: std = {std_raw:.2f} m")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Baro Error [m]")
    ax.grid(True, alpha=0.3)

    # Bottom-right: Compensated error time series
    ax = axes[1, 1]
    ax.plot(t, range_cal["comp_err_fit"], color="tab:blue", linewidth=0.8)
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
    std_comp = float(np.std(range_cal["comp_err_fit"]))
    ax.set_title(f"Compensated error: std = {std_comp:.2f} m")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Baro Error [m]")
    ax.grid(True, alpha=0.3)

    # Sync bottom row y-axes
    ylim = [min(axes[1, 0].get_ylim()[0], axes[1, 1].get_ylim()[0]),
            max(axes[1, 0].get_ylim()[1], axes[1, 1].get_ylim()[1])]
    axes[1, 0].set_ylim(ylim)
    axes[1, 1].set_ylim(ylim)

    plt.tight_layout(rect=[0, 0, 1, _LAYOUT_TOP])
    return fig


def plot_cf_tuning(sweep, range_cal, existing_pcoef):
    """Page: CF parameter sensitivity — K and error std vs bandwidth."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("CF+RLS Parameter Sensitivity", fontsize=14, fontweight="bold")
    fig.text(0.5, _SUBTITLE_Y,
             "Sweeping CF bandwidth: how does K and compensation quality "
             "change? Range K is ground truth. Lower error std = better.",
             ha="center", va="top", fontsize=9, style="italic", color="0.4")

    bw = sweep["bandwidth"]
    default_bw = CfRls.DEFAULT_CF_BANDWIDTH
    range_k = range_cal["K_residual"]

    styles = [
        ("lambda_0.998", "\u03bb=0.998 (default)", "tab:red"),
        ("lambda_1.0", "\u03bb=1.0 (no forget)", "tab:purple"),
    ]

    # --- Left: K vs bandwidth ---
    ax = axes[0]
    for key, label, color in styles:
        ax.semilogx(bw, sweep[key]["k_residual"], "o-", color=color,
                    markersize=3, linewidth=1.0, label=label)
    ax.axhline(range_k, color="tab:green", linestyle="--", linewidth=1.0,
               label=f"Range K = {range_k:.2f}")
    ax.axvline(default_bw, color="tab:gray", linestyle=":", linewidth=0.8,
               label=f"Default ({default_bw} Hz)")

    # Best K match (default lambda)
    k_def = sweep["lambda_0.998"]["k_residual"]
    idx_best = int(np.argmin(np.abs(k_def - range_k)))
    best_bw = float(bw[idx_best])
    ax.axvline(best_bw, color="tab:blue", linestyle="--", linewidth=0.8,
               label=f"Best K match ({best_bw:.3f} Hz)")

    ax.set_xlabel("CF Bandwidth [Hz]")
    ax.set_ylabel("K (residual)")
    ax.set_title("K vs CF Bandwidth")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- Right: Error std vs bandwidth ---
    ax = axes[1]
    for key, label, color in styles:
        ax.semilogx(bw, sweep[key]["comp_error_std"], "o-", color=color,
                    markersize=3, linewidth=1.0, label=label)

    ideal_err = (range_cal["raw_err_fit"]
                 + range_cal["pcoef_recommended"] * range_cal["thrust_fit"])
    ideal_std = float(np.std(ideal_err))
    ax.axhline(ideal_std, color="tab:green", linestyle="--", linewidth=1.0,
               label=f"Range optimal: {ideal_std:.2f} m")

    current_std = float(np.std(range_cal["comp_err_fit"]))
    ax.axhline(current_std, color="tab:orange", linestyle=":", linewidth=1.0,
               label=f"Current PCOEF: {current_std:.2f} m")

    ax.axvline(default_bw, color="tab:gray", linestyle=":", linewidth=0.8)
    ax.axvline(best_bw, color="tab:blue", linestyle="--", linewidth=0.8)

    # Min error std
    std_def = sweep["lambda_0.998"]["comp_error_std"]
    idx_min = int(np.argmin(std_def))
    min_bw = float(bw[idx_min])
    min_std = float(std_def[idx_min])
    if abs(min_bw - best_bw) / best_bw > 0.1:  # only annotate if different
        ax.axvline(min_bw, color="tab:red", linestyle="--", linewidth=0.8,
                   alpha=0.5, label=f"Min std ({min_bw:.3f} Hz)")

    ax.set_xlabel("CF Bandwidth [Hz]")
    ax.set_ylabel("Compensated Error Std [m]")
    ax.set_title("Compensation Quality vs CF Bandwidth")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, _LAYOUT_TOP])
    return fig, best_bw, min_bw, min_std


def plot_summary(text_lines):
    """Final page: text summary."""
    fig, ax = plt.subplots(1, 1, figsize=(14, 10))
    ax.axis("off")
    fig.suptitle("Analysis Summary", fontsize=14, fontweight="bold")
    ax.text(0.02, 0.98, "\n".join(text_lines), transform=ax.transAxes,
            fontsize=10, verticalalignment="top", family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#f8f8f8",
                      edgecolor="#cccccc"))
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Barometer thrust compensation analysis")
    parser.add_argument("ulog_file", help="Path to .ulg flight log")
    px4_root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    default_log_dir = os.path.join(px4_root, "logs")
    parser.add_argument("--output-dir", "-o", default=default_log_dir,
                        help="Output base directory (default: <PX4_ROOT>/logs/)")
    args = parser.parse_args()

    if not os.path.isfile(args.ulog_file):
        print(f"Error: file not found: {args.ulog_file}", file=sys.stderr)
        sys.exit(1)

    log_name = os.path.splitext(os.path.basename(args.ulog_file))[0]
    if args.output_dir == default_log_dir:
        output_dir = os.path.join(args.output_dir, log_name)
    else:
        output_dir = args.output_dir
    os.makedirs(output_dir, exist_ok=True)

    ulg_dest = os.path.join(output_dir, os.path.basename(args.ulog_file))
    if not os.path.exists(ulg_dest):
        shutil.copy2(args.ulog_file, ulg_dest)

    # ── Load ──
    print(f"Loading {args.ulog_file}")
    ulog = ULog(args.ulog_file)
    duration = (ulog.last_timestamp - ulog.start_timestamp) / 1e6
    existing_pcoef = float(get_param(ulog, "SENS_BARO_PCOEF", 0.0))
    armed_start, armed_end = detect_armed_period(ulog)

    # ── Extract ──
    baro = extract_baro(ulog)
    accel = extract_accel(ulog)
    attitude = extract_attitude(ulog)
    thrust = extract_thrust(ulog)
    range_data = extract_range(ulog)
    online = extract_online_estimate(ulog)
    ekf_z = extract_ekf_z(ulog)
    ekf_baro_obs = extract_ekf_baro_obs(ulog)
    landed = extract_landed(ulog)

    if baro is None or thrust is None:
        print("Error: missing barometer or thrust data", file=sys.stderr)
        sys.exit(1)

    # ── Summary header ──
    summary = []
    summary.append(f"Log: {log_name}")
    summary.append(f"Duration: {duration:.1f}s")
    summary.append(f"Armed: {armed_start:.1f}s - {armed_end:.1f}s")
    summary.append(f"SENS_BARO_PCOEF: {existing_pcoef:+.2f}")
    summary.append("")

    for pname in ["EKF2_HGT_REF", "EKF2_BARO_CTRL", "EKF2_BARO_NOISE",
                  "SENS_BAR_AUTOCAL"]:
        val = get_param(ulog, pname)
        if val is not None:
            summary.append(f"  {pname:24s} = {val}")
    summary.append("")

    summary.append("Data rates:")
    summary.append(f"  Baro:    {effective_rate(baro['time_s']):5.1f} Hz  "
                   f"({len(baro['time_s'])} samples)")
    summary.append(f"  Thrust:  {effective_rate(thrust['time_s']):5.1f} Hz  "
                   f"({len(thrust['time_s'])} samples)")
    if accel is not None:
        summary.append(f"  Accel:   {effective_rate(accel['time_s']):5.1f} Hz  "
                       f"({len(accel['time_s'])} samples)")
    if attitude is not None:
        summary.append(f"  Attitude:{effective_rate(attitude['time_s']):5.1f} Hz  "
                       f"({len(attitude['time_s'])} samples)")
    if range_data is not None:
        summary.append(f"  Range:   {effective_rate(range_data['time_s']):5.1f} Hz  "
                       f"({len(range_data['time_s'])} samples)")
    if online is not None:
        summary.append(f"  Online:  {effective_rate(online['time_s']):5.1f} Hz  "
                       f"({len(online['time_s'])} samples)")
    summary.append("")

    # ── Analysis ──
    figures = []
    figures.append(plot_altitude_overview(
        ekf_baro_obs, ekf_z, range_data, thrust, armed_start, armed_end))

    # 1. Online estimator results
    online_k = None
    if online is not None:
        armed_mask = ((online["time_s"] >= armed_start)
                      & (online["time_s"] <= armed_end))
        conv_mask = armed_mask & (online["converged"] > 0)
        if conv_mask.any():
            conv_idx = np.where(conv_mask)[0][0]
            online_k = float(online["k_estimate"][conv_idx])
            conv_time = float(online["time_s"][conv_idx]) - armed_start
            summary.append(f"Online CF+RLS:")
            summary.append(f"  Converged at {conv_time:.0f}s after arm")
            summary.append(f"  K = {online_k:.3f}")
            summary.append(f"  Total PCOEF: {existing_pcoef:+.2f} "
                           f"- {online_k:.2f} = "
                           f"{existing_pcoef - online_k:+.2f}")
        else:
            last_k = online["k_estimate"][armed_mask]
            online_k = float(last_k[-1]) if len(last_k) > 0 else None
            if online_k is not None:
                summary.append(f"Online CF+RLS: did NOT converge "
                               f"(last K = {online_k:.3f})")
            else:
                summary.append("Online CF+RLS: no armed data")
        summary.append("")

    # 2. Offline CF+RLS replay
    offline = None
    if accel is not None and attitude is not None:
        offline = run_offline_cf_rls(baro, accel, attitude, thrust, landed,
                                     armed_start, armed_end,
                                     ekf_z=ekf_z, range_data=range_data)
        summary.append(f"Offline CF+RLS:")
        summary.append(f"  Final K = {offline['final_k']:.3f}")
        summary.append(f"  Total PCOEF: {existing_pcoef:+.2f} "
                       f"- {offline['final_k']:.2f} = "
                       f"{existing_pcoef - offline['final_k']:+.2f}")
        summary.append("")
    else:
        summary.append("Offline CF+RLS: skipped (missing accel or attitude)")
        summary.append("")

    # 3. Range calibration
    range_cal = None
    if range_data is not None:
        range_cal = run_range_calibration(baro, range_data, thrust,
                                          armed_start, armed_end,
                                          existing_pcoef)
        if range_cal is not None:
            summary.append(f"Range ground truth:")
            summary.append(f"  K_total = {range_cal['K_total']:.3f}  "
                           f"(R\u00b2 = {range_cal['r2']:.3f}, "
                           f"RMSE = {range_cal['rmse']:.3f} m)")
            summary.append(f"  K_residual = {range_cal['K_residual']:.3f}")
            summary.append(f"  PCOEF recommended: "
                           f"{range_cal['pcoef_recommended']:+.2f}")
            summary.append("")
        else:
            summary.append("Range calibration: insufficient data")
            summary.append("")

    # ── Comparison table ──
    sep = "=" * 60
    summary.append(sep)
    r2_label = "R\u00b2"
    summary.append(f"  {'Method':<22} {'K':>8} {'Total PCOEF':>12} "
                   f"{r2_label:>6}")
    summary.append("-" * 60)

    if online_k is not None:
        pcoef = existing_pcoef - online_k
        summary.append(f"  {'Online CF+RLS':<22} {online_k:>8.3f} "
                       f"{pcoef:>+12.2f}")

    if offline is not None:
        pcoef = existing_pcoef - offline["final_k"]
        summary.append(f"  {'Offline CF+RLS':<22} "
                       f"{offline['final_k']:>8.3f} {pcoef:>+12.2f}")

    if range_cal is not None:
        summary.append(f"  {'Range ground truth':<22} "
                       f"{range_cal['K_residual']:>8.3f} "
                       f"{range_cal['pcoef_recommended']:>+12.2f} "
                       f"{range_cal['r2']:>6.3f}")

    summary.append(sep)

    # Pairwise differences
    if online_k is not None and offline is not None:
        d = abs(online_k - offline["final_k"])
        summary.append(f"  Online vs Offline:  \u0394K = {d:.3f}")
    if online_k is not None and range_cal is not None:
        d = abs(online_k - range_cal["K_residual"])
        summary.append(f"  Online vs Range:    \u0394K = {d:.3f}")
    if offline is not None and range_cal is not None:
        d = abs(offline["final_k"] - range_cal["K_residual"])
        summary.append(f"  Offline vs Range:   \u0394K = {d:.3f}")

    summary.append("")

    # Print to console
    for line in summary:
        print(line)

    # ── Generate plots ──
    figures.append(plot_k_convergence(
        online, offline, range_cal, armed_start, armed_end, existing_pcoef))

    if online is not None or offline is not None:
        figures.append(plot_residual_comparison(
            online, offline, thrust, armed_start, armed_end))

    if range_cal is not None:
        figures.append(plot_ground_truth(range_cal, existing_pcoef))

    # ── Parameter sweep (when range ground truth available) ──
    if (range_cal is not None and accel is not None
            and attitude is not None):
        print("\nSweeping CF bandwidth...")
        sweep = sweep_cf_params(baro, accel, attitude, thrust, landed,
                                armed_start, armed_end, range_cal,
                                existing_pcoef,
                                ekf_z=ekf_z, range_data=range_data)
        fig, best_bw, min_bw, min_std = plot_cf_tuning(
            sweep, range_cal, existing_pcoef)
        figures.append(fig)

        range_k = range_cal["K_residual"]
        k_def = sweep["lambda_0.998"]["k_residual"]
        idx_best = int(np.argmin(np.abs(k_def - range_k)))
        best_k = float(k_def[idx_best])

        summary.append("")
        summary.append("CF Bandwidth Tuning:")
        summary.append(f"  Default (0.050 Hz):  K = {float(k_def[np.argmin(np.abs(sweep['bandwidth'] - 0.05))]):+.3f}")
        summary.append(f"  Best K match:        {best_bw:.3f} Hz  "
                       f"(K = {best_k:+.3f})")
        summary.append(f"  Min error std:       {min_bw:.3f} Hz  "
                       f"(std = {min_std:.3f} m)")
        summary.append("")

        for line in summary[-6:]:
            print(line)

    figures.append(plot_summary(summary))

    # ── Save PDF ──
    pdf_path = os.path.join(output_dir, f"{log_name}.pdf")
    with PdfPages(pdf_path) as pdf:
        for fig in figures:
            pdf.savefig(fig)
            plt.close(fig)
    print(f"\nSaved: {pdf_path}")


if __name__ == "__main__":
    main()
