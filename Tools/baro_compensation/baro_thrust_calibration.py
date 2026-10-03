#!/usr/bin/env python3
"""
Identify SENS_BARO_K_T, the barometer thrust compensation coefficient, from a flight log.

Firmware applies:  baro_alt += SENS_BARO_K_T * |thrust_z|

Sign convention used throughout: the fitted model is
    baro_error = baro_alt_raw - truth = K * thrust + bias
so the coefficient that cancels it is SENS_BARO_K_T = -K.

Two methods, chosen from the log contents (--method overrides):
  range   distance_sensor as ground truth, least-squares fit of baro error against thrust
  cf_rls  no range sensor: a baro/accel complementary filter isolates the thrust-correlated
          baro error and a recursive least-squares fit extracts K

Usage:
    python3 baro_thrust_calibration.py <log.ulg> [--output-dir <dir>] [--method range|cf_rls]
        The report is written next to the log unless --output-dir is given.
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

GRAVITY = 9.80665

# Below this height the range sensor sees ground effect, not propwash
MIN_RANGE_M = 0.5


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
        # None on logs that predate the field
        "correction_m": d.data.get("baro_alt_correction"),
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


def existing_k_t(ulog):
    """SENS_BARO_K_T in effect during the flight."""
    val = get_param(ulog, "SENS_BARO_K_T")
    return float(val) if val is not None else 0.0


def recover_raw_altitude(baro, thrust, k_t_flown):
    """Undo the compensation the firmware applied so the fit identifies the full coefficient."""
    if baro["correction_m"] is not None:
        return baro["alt_m"] - baro["correction_m"], "subtracted logged baro_alt_correction"
    if abs(k_t_flown) > 0.0:
        thrust_at_baro = np.interp(baro["time_s"], thrust["time_s"], thrust["thrust"])
        return (baro["alt_m"] - k_t_flown * thrust_at_baro,
                f"undid K_T = {k_t_flown:+.2f} (static pressure terms, if any, cannot be undone)")
    return baro["alt_m"].copy(), "no thrust compensation was active"


# ---------------------------------------------------------------------------
# Complementary filter + recursive least squares
# ---------------------------------------------------------------------------

def compute_accel_up(ax, ay, az, qw, qx, qy, qz):
    """Body-frame specific force + quaternion -> upward linear acceleration.

    Rotates body accel to NED (3rd row of quaternion DCM), then
        accel_up = -(specific_force_ned_z + g)
    """
    ned_z = ((2 * (qx * qz - qw * qy)) * ax
             + (2 * (qy * qz + qw * qx)) * ay
             + (1 - 2 * (qx**2 + qy**2)) * az)
    return -(ned_z + GRAVITY)


class CfRls:
    """Isolate the thrust-correlated baro error and fit residual = K * thrust + bias.

    The complementary filter fuses baro altitude with double-integrated accel at a
    low crossover frequency: accel explains fast altitude changes, baro the slow
    ones, and the residual (baro minus accel prediction) keeps the baro error at
    the frequencies where thrust varies. RLS with a forgetting factor then fits K.
    """

    DEFAULT_CF_BANDWIDTH = 0.05
    DEFAULT_RLS_LAMBDA = 0.998
    RLS_P_INIT = 100.0
    ERROR_VAR_INIT = 10.0
    ALPHA_ERR = 0.01

    def __init__(self, cf_bandwidth=None, rls_lambda=None):
        bw = cf_bandwidth if cf_bandwidth is not None else self.DEFAULT_CF_BANDWIDTH
        self.CF_OMEGA = 2.0 * np.pi * bw
        # critically damped 2nd order: K1 = 2w, K2 = w^2
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

        # deviation uses the mean before this sample so the sample does not bias it
        alpha = dt / (2.0 + dt)
        dev = thrust - self._thrust_mean
        self._thrust_mean = (1 - alpha) * self._thrust_mean + alpha * thrust
        self._thrust_var = (1 - alpha) * self._thrust_var + alpha * dev * dev

        if not (np.isfinite(self.theta).all() and np.isfinite(self.P).all()
                and np.isfinite(self.error_var)):
            self.reset()

    @property
    def k(self):
        return float(self.theta[0])

    @property
    def bias(self):
        return float(self.theta[1])

    @property
    def k_var(self):
        return float(self.P[0, 0])

    @property
    def thrust_std(self):
        return float(np.sqrt(max(self._thrust_var, 0.0)))


def build_estimation_mask(baro_t, armed_start, armed_end, landed,
                          ekf_z=None, range_data=None):
    """Samples worth fitting: armed, airborne, slow, and clear of ground effect.

    Fast vertical motion and forward flight add baro errors that are not propwash
    (dynamic pressure, accel scale error) but still correlate with thrust.
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
        mask &= rng > MIN_RANGE_M

    return mask


def run_cf_rls(baro_t, raw_alt, accel, attitude, thrust, landed,
               armed_start, armed_end, cf_bandwidth=None,
               ekf_z=None, range_data=None):
    """Replay the CF+RLS over the raw altitude. Returns the K trace, residuals and fit quality."""
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

    est = CfRls(cf_bandwidth=cf_bandwidth)
    n = len(baro_t)
    k_trace = np.full(n, np.nan)
    k_var_trace = np.full(n, np.nan)
    residual = np.full(n, np.nan)

    prev_t = None
    for i in range(n):
        if not mask[i]:
            continue
        if prev_t is None:
            prev_t = baro_t[i]
            residual[i] = 0.0
            k_trace[i] = est.k
            k_var_trace[i] = est.k_var
            continue

        dt = float(np.clip(baro_t[i] - prev_t, 0.001, 0.5))
        prev_t = baro_t[i]

        res = est.update_cf(float(raw_alt[i]), float(accel_up[i]), dt)
        est.update_rls(res, float(thrust_interp[i]), dt)

        residual[i] = res
        k_trace[i] = est.k
        k_var_trace[i] = est.k_var

    if mask.sum() < 20:
        return None

    valid = np.isfinite(residual)
    res_v = residual[valid]
    thr_v = thrust_interp[valid]
    after = res_v - (est.k * thr_v + est.bias)
    var_before = np.var(res_v)

    return {
        "time_s": baro_t,
        "k_trace": k_trace,
        "k_var_trace": k_var_trace,
        "residual": residual,
        "thrust_at_baro": thrust_interp,
        "K": est.k,
        "bias": est.bias,
        "k_var": est.k_var,
        "thrust_std": est.thrust_std,
        "r_before": safe_corrcoef(thr_v, res_v),
        "r_after": safe_corrcoef(thr_v, after),
        "r2": float(1.0 - np.var(after) / var_before) if var_before > 1e-10 else 0.0,
        "rmse_before": float(np.std(res_v)),
        "rmse_after": float(np.std(after)),
        "n_samples": int(valid.sum()),
    }


# ---------------------------------------------------------------------------
# Range sensor ground truth
# ---------------------------------------------------------------------------

def run_range_calibration(baro_t, raw_alt, range_data, thrust,
                          armed_start, armed_end):
    """Least-squares fit of (raw baro - range) against thrust.

    Baro is MSL and range is AGL; zeroing baro at arm time leaves a constant
    offset that the bias term absorbs.
    """
    rng_t, rng_dist = range_data["time_s"], range_data["distance_m"]

    raw_zeroed = raw_alt - np.interp(armed_start, baro_t, raw_alt)
    raw_error = np.interp(rng_t, baro_t, raw_zeroed) - rng_dist

    mask = ((rng_t >= armed_start) & (rng_t <= armed_end)
            & (rng_dist > MIN_RANGE_M))
    if mask.sum() < 20:
        return None

    t_fit = rng_t[mask]
    err_fit = raw_error[mask]
    thrust_fit = np.interp(t_fit, thrust["time_s"], thrust["thrust"])

    A = np.column_stack([thrust_fit, np.ones(len(thrust_fit))])
    coeffs, _, _, _ = np.linalg.lstsq(A, err_fit, rcond=None)
    K = float(coeffs[0])

    after = err_fit - A @ coeffs
    var_before = np.var(err_fit)

    return {
        "K": K,
        "bias": float(coeffs[1]),
        "r_before": safe_corrcoef(thrust_fit, err_fit),
        "r_after": safe_corrcoef(thrust_fit, after),
        "r2": float(1.0 - np.var(after) / var_before) if var_before > 1e-10 else 0.0,
        "rmse_before": float(np.std(err_fit)),
        "rmse_after": float(np.std(after)),
        "n_samples": int(mask.sum()),
        "t_fit": t_fit,
        "err_fit": err_fit,
        "thrust_fit": thrust_fit,
        "coeffs": coeffs,
    }


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

_SUBTITLE_Y = 0.94
_LAYOUT_TOP = 0.93


def plot_altitude_overview(baro_t, raw_alt, flown_alt, ekf_z, range_data,
                           thrust_data, armed_start, armed_end):
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("Altitude Overview", fontsize=14, fontweight="bold")

    ax = axes[0]
    if range_data is not None:
        ax.plot(range_data["time_s"], range_data["distance_m"],
                label="Distance sensor", color="tab:blue", linewidth=1.2)
    zero = np.interp(armed_start, baro_t, raw_alt)
    ax.plot(baro_t, raw_alt - zero, label="Baro raw",
            color="tab:red", linewidth=1.0, alpha=0.8)
    if not np.allclose(raw_alt, flown_alt):
        ax.plot(baro_t, flown_alt - np.interp(armed_start, baro_t, flown_alt),
                label="Baro as flown (compensated)",
                color="tab:purple", linewidth=1.0, alpha=0.8)
    if ekf_z is not None:
        et = ekf_z["time_s"]
        ea = -ekf_z["z"]
        ea -= np.interp(armed_start, et, ea)
        ax.plot(et, ea, label="EKF altitude (-Z)",
                color="tab:green", linewidth=1.0, alpha=0.8)
    ax.axvspan(armed_start, armed_end, alpha=0.04, color="green", label="Armed")
    ax.set_ylabel("Altitude relative to arming [m]")
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


def plot_cf_rls(cf, armed_start, armed_end):
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle("CF+RLS Identification", fontsize=14, fontweight="bold")
    fig.text(0.5, _SUBTITLE_Y,
             "Residual = baro minus accel-predicted altitude. Slope against thrust = K.",
             ha="center", va="top", fontsize=9, style="italic", color="0.4")

    t = cf["time_s"]
    v = np.isfinite(cf["k_trace"])

    ax = axes[0]
    k = cf["k_trace"][v]
    ks = np.sqrt(np.clip(cf["k_var_trace"][v], 0, None))
    ax.plot(t[v], k, color="tab:red", linewidth=1.0, label="K estimate")
    ax.fill_between(t[v], k - ks, k + ks, alpha=0.1, color="tab:red")
    ax.axhline(cf["K"], color="k", linestyle="--", linewidth=0.8,
               label=f"Final K = {cf['K']:.2f}")
    ax.set_ylabel("K [m / unit thrust]")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(t[v], cf["residual"][v], color="tab:red", linewidth=0.6, alpha=0.8)
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
    ax.set_ylabel("CF residual [m]")
    ax.set_xlabel("Time [s]")
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    thr = cf["thrust_at_baro"][v]
    res = cf["residual"][v]
    ax.scatter(thr, res, s=2, alpha=0.3, color="tab:red")
    x_fit = np.linspace(thr.min(), thr.max(), 50)
    ax.plot(x_fit, cf["K"] * x_fit + cf["bias"], "k--", linewidth=1.2)
    ax.set_title(f"K = {cf['K']:.2f}, r = {cf['r_before']:.3f}, "
                 f"R² = {cf['r2']:.3f}, thrust std = {cf['thrust_std']:.3f}")
    ax.set_xlabel("Thrust [0-1]")
    ax.set_ylabel("CF residual [m]")
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, _LAYOUT_TOP])
    return fig


def plot_ground_truth(cal):
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Range Sensor Ground Truth", fontsize=14, fontweight="bold")
    fig.text(0.5, _SUBTITLE_Y,
             "Left: raw baro error against range. Right: with the recommended SENS_BARO_K_T applied.",
             ha="center", va="top", fontsize=9, style="italic", color="0.4")

    thrust = cal["thrust_fit"]
    err = cal["err_fit"]
    after = err - cal["K"] * thrust
    x_fit = np.linspace(thrust.min(), thrust.max(), 50)
    c = cal["coeffs"]

    ax = axes[0, 0]
    ax.scatter(thrust, err, s=2, alpha=0.3, color="tab:orange")
    ax.plot(x_fit, c[0] * x_fit + c[1], "k--", linewidth=1.2)
    ax.set_title(f"Raw: K = {c[0]:.2f}, r = {cal['r_before']:.3f}, R² = {cal['r2']:.3f}")
    ax.set_xlabel("Thrust [0-1]")
    ax.set_ylabel("Baro error [m]")
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.scatter(thrust, after, s=2, alpha=0.3, color="tab:blue")
    ax.axhline(c[1], color="k", linewidth=0.8, linestyle="--")
    ax.set_title(f"Compensated (K_T = {-c[0]:+.2f}): r = {cal['r_after']:.3f}")
    ax.set_xlabel("Thrust [0-1]")
    ax.set_ylabel("Baro error [m]")
    ax.grid(True, alpha=0.3)

    ylim = [min(axes[0, 0].get_ylim()[0], axes[0, 1].get_ylim()[0]),
            max(axes[0, 0].get_ylim()[1], axes[0, 1].get_ylim()[1])]
    axes[0, 0].set_ylim(ylim)
    axes[0, 1].set_ylim(ylim)

    t = cal["t_fit"]
    ax = axes[1, 0]
    ax.plot(t, err, color="tab:orange", linewidth=0.8)
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
    ax.set_title(f"Raw error: std = {cal['rmse_before']:.2f} m")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Baro error [m]")
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(t, after, color="tab:blue", linewidth=0.8)
    ax.axhline(c[1], color="k", linewidth=0.5, linestyle="--")
    ax.set_title(f"Compensated error: std = {cal['rmse_after']:.2f} m")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Baro error [m]")
    ax.grid(True, alpha=0.3)

    ylim = [min(axes[1, 0].get_ylim()[0], axes[1, 1].get_ylim()[0]),
            max(axes[1, 0].get_ylim()[1], axes[1, 1].get_ylim()[1])]
    axes[1, 0].set_ylim(ylim)
    axes[1, 1].set_ylim(ylim)

    plt.tight_layout(rect=[0, 0, 1, _LAYOUT_TOP])
    return fig


def plot_summary(text_lines):
    fig, ax = plt.subplots(1, 1, figsize=(14, 10))
    ax.axis("off")
    fig.suptitle("Summary", fontsize=14, fontweight="bold")
    ax.text(0.02, 0.98, "\n".join(text_lines), transform=ax.transAxes,
            fontsize=10, verticalalignment="top", family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#f8f8f8",
                      edgecolor="#cccccc"))
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def fit_lines(label, fit):
    return [
        f"{label}:",
        f"  K = {fit['K']:+.3f} m/unit thrust  ({fit['n_samples']} samples)",
        f"  correlation(thrust, error)  before {fit['r_before']:+.3f}   after {fit['r_after']:+.3f}",
        f"  error std                   before {fit['rmse_before']:.3f} m   after {fit['rmse_after']:.3f} m",
        f"  R² = {fit['r2']:.3f}",
    ]


def main():
    parser = argparse.ArgumentParser(
        description="Identify SENS_BARO_K_T (barometer thrust compensation) from a flight log")
    parser.add_argument("ulog_file", help="Path to .ulg flight log")
    parser.add_argument("--output-dir", "-o", default=None,
                        help="Directory for the report (default: next to the log)")
    parser.add_argument("--method", choices=["range", "cf_rls"], default=None,
                        help="Force a method (default: range when a distance sensor is logged)")
    parser.add_argument("--cf-bandwidth", type=float, default=CfRls.DEFAULT_CF_BANDWIDTH,
                        help="Complementary filter crossover frequency [Hz] for cf_rls")
    args = parser.parse_args()

    if not os.path.isfile(args.ulog_file):
        print(f"Error: file not found: {args.ulog_file}", file=sys.stderr)
        sys.exit(1)

    log_name = os.path.splitext(os.path.basename(args.ulog_file))[0]
    output_dir = args.output_dir or os.path.dirname(os.path.abspath(args.ulog_file))
    os.makedirs(output_dir, exist_ok=True)

    print(f"Loading {args.ulog_file}")
    ulog = ULog(args.ulog_file)
    duration = (ulog.last_timestamp - ulog.start_timestamp) / 1e6
    k_t_flown = existing_k_t(ulog)
    armed_start, armed_end = detect_armed_period(ulog)

    baro = extract_baro(ulog)
    accel = extract_accel(ulog)
    attitude = extract_attitude(ulog)
    thrust = extract_thrust(ulog)
    range_data = extract_range(ulog)
    ekf_z = extract_ekf_z(ulog)
    landed = extract_landed(ulog)

    if baro is None or thrust is None:
        print("Error: missing vehicle_air_data or vehicle_thrust_setpoint", file=sys.stderr)
        sys.exit(1)

    baro_t = baro["time_s"]
    raw_alt, raw_how = recover_raw_altitude(baro, thrust, k_t_flown)

    summary = [
        f"Log: {log_name}",
        f"Duration: {duration:.1f} s, armed {armed_start:.1f} s - {armed_end:.1f} s",
        f"SENS_BARO_K_T during flight: {k_t_flown:+.2f}",
        f"Raw altitude: {raw_how}",
        "",
    ]
    static_k = [get_param(ulog, n, 0.0) for n in
                ("SENS_BARO_K_XP", "SENS_BARO_K_XN", "SENS_BARO_K_YP", "SENS_BARO_K_YN", "SENS_BARO_K_Z")]
    if baro["correction_m"] is None and any(abs(float(k)) > 0.0 for k in static_k):
        summary.append("Warning: SENS_BARO_K_* static pressure compensation was active and is "
                       "still in the altitude used here")
        summary.append("")

    for pname in ["EKF2_HGT_REF", "EKF2_BARO_CTRL", "EKF2_BARO_NOISE"]:
        val = get_param(ulog, pname)
        if val is not None:
            summary.append(f"  {pname:16s} = {val}")
    summary.append("")

    summary.append("Data rates:")
    summary.append(f"  Baro:    {effective_rate(baro_t):5.1f} Hz  ({len(baro_t)} samples)")
    summary.append(f"  Thrust:  {effective_rate(thrust['time_s']):5.1f} Hz  ({len(thrust['time_s'])} samples)")
    if range_data is not None:
        summary.append(f"  Range:   {effective_rate(range_data['time_s']):5.1f} Hz  "
                       f"({len(range_data['time_s'])} samples)")
    summary.append("")

    figures = [plot_altitude_overview(baro_t, raw_alt, baro["alt_m"], ekf_z, range_data,
                                      thrust, armed_start, armed_end)]

    range_cal = None
    if range_data is not None and args.method != "cf_rls":
        range_cal = run_range_calibration(baro_t, raw_alt, range_data, thrust,
                                          armed_start, armed_end)
        if range_cal is None:
            summary.append(f"Range sensor: fewer than 20 samples above {MIN_RANGE_M} m while armed, not used")
            summary.append("")

    cf = None
    if accel is not None and attitude is not None:
        cf = run_cf_rls(baro_t, raw_alt, accel, attitude, thrust, landed,
                        armed_start, armed_end, cf_bandwidth=args.cf_bandwidth,
                        ekf_z=ekf_z, range_data=range_data)

    if args.method == "range" and range_cal is None:
        print("Error: --method range needs a usable distance_sensor topic", file=sys.stderr)
        sys.exit(1)
    if args.method == "cf_rls" and cf is None:
        print("Error: --method cf_rls needs vehicle_acceleration and vehicle_attitude", file=sys.stderr)
        sys.exit(1)

    if range_cal is not None:
        chosen, chosen_label = range_cal, "range"
        why = "distance sensor logged; it is the ground truth"
        figures.append(plot_ground_truth(range_cal))
    elif cf is not None:
        chosen, chosen_label = cf, "cf_rls"
        why = ("no usable distance sensor; accel-predicted altitude is the reference"
               if range_data is None else "range fit not possible; accel-predicted altitude is the reference")
        figures.append(plot_cf_rls(cf, armed_start, armed_end))
    else:
        print("Error: no range sensor and no accel/attitude data, nothing to fit", file=sys.stderr)
        sys.exit(1)

    if range_cal is not None:
        summary += fit_lines("Range sensor ground truth", range_cal) + [""]
    if cf is not None:
        summary += fit_lines("CF+RLS (accel reference)", cf) + [""]
    if range_cal is not None and cf is not None:
        summary.append(f"Methods differ by {abs(range_cal['K'] - cf['K']):.2f} m/unit thrust")
        summary.append("")

    # baro_error = K * thrust + bias, so the coefficient that cancels it is -K
    k_t = -chosen["K"]
    summary.append(f"Method used: {chosen_label} ({why})")
    summary.append(f"SENS_BARO_K_T = -K = {k_t:+.2f}")
    if baro["correction_m"] is None and abs(k_t_flown) > 0.0:
        summary.append(f"  (equivalently SENS_BARO_K_T {k_t_flown:+.2f} minus the residual fit "
                       f"{k_t_flown - k_t:+.2f} on the as-flown altitude)")
    summary.append("")
    if chosen["r2"] < 0.3 or abs(chosen["r_before"]) < 0.3:
        summary.append("Weak fit: thrust explains less than 30 % of the baro error. "
                       "Look for another error source before applying this value.")
    elif abs(k_t) < 0.5:
        summary.append("Thrust effect is below 0.5 m at full thrust; leaving SENS_BARO_K_T at 0 is fine.")
    summary.append(f"param set SENS_BARO_K_T {k_t:.2f}")

    for line in summary:
        print(line)

    figures.append(plot_summary(summary))

    pdf_path = os.path.join(output_dir, f"{log_name}.pdf")
    with PdfPages(pdf_path) as pdf:
        for fig in figures:
            pdf.savefig(fig)
            plt.close(fig)
    print(f"\nSaved: {pdf_path}")


if __name__ == "__main__":
    main()
