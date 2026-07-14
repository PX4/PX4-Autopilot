#!/usr/bin/env python3
"""
Barometer thrust compensation analysis.

Three-way comparison of K estimates:
  1. Online CF+RLS (from logged baro_thrust_estimate)
  2. Offline CF+RLS replay (baro + accel, mirroring the firmware algorithm)
  3. Height reference fit (distance sensor preferred, VIO fallback)

The correction model: baro_alt += SENS_BARO_K_T * |thrust_z|

Usage:
    python3 baro_thrust_calibration.py <log.ulg> [--output-dir <dir>]
        If --output-dir is not given, results go to logs/<log_name>/ in the PX4 root.
"""

import argparse
import os
import shutil
import sys
from dataclasses import dataclass

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
DEFAULT_TIME_S = 20.0
# Note: This is platform dependant on thrust ranges. May need to be tuned.
DEFAULT_MIN_THRUST_EXCITATION_PCT = 5.0
# Height reference thresholds
DEFAULT_MIN_HEIGHT_REF_M = 1.0
# CFLS speed gates (avoids coupling)
DEFAULT_MAX_VERTICAL_SPEED_M_S = 2.0
DEFAULT_MAX_HORIZONTAL_SPEED_M_S = 5.0
DEFAULT_CF_BANDWIDTH_HZ = 0.05
DEFAULT_RLS_LAMBDA_FACTOR = 0.998
# Accel age gate for CF
DEFAULT_MAX_ACCEL_SAMPLE_AGE_S = 0.1
# Attitude age gate for CF
DEFAULT_MAX_ATTITUDE_SAMPLE_AGE_S = 0.1
# Thrust age gate for RLS
DEFAULT_MAX_THRUST_SAMPLE_AGE_S = 0.5
# Offline convergence thresholds
DEFAULT_CONVERGENCE_VAR_THR = 3.0
DEFAULT_CONVERGENCE_ERR_THR = 0.5
DEFAULT_CONVERGENCE_ERR_REL_THR = 0.4
DEFAULT_CONVERGENCE_ERR_MAX_THR = 4.0
DEFAULT_MIN_EXCITATION_TIME_S = 5.0
DEFAULT_MIN_ESTIMATION_TIME_S = 30.0
DEFAULT_K_STABILITY_TIME_S = 10.0
DEFAULT_CONVERGENCE_HOLD_TIME_S = 10.0
DEFAULT_K_STABILITY_DIFF_THR = 0.5
DEFAULT_MIN_K_UPDATE_THRESHOLD = 0.1
DEFAULT_K_T_MAX = 30.0
STATIC_PRESSURE_PARAM_EPS = 1e-6
STATIC_PRESSURE_PARAMS = [
    ("SENS_BARO_K_XP", "EKF2_PCOEF_XP"),
    ("SENS_BARO_K_XN", "EKF2_PCOEF_XN"),
    ("SENS_BARO_K_YP", "EKF2_PCOEF_YP"),
    ("SENS_BARO_K_YN", "EKF2_PCOEF_YN"),
    ("SENS_BARO_K_Z", "EKF2_PCOEF_Z"),
]


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


def get_static_pressure_params(ulog):
    params = []

    for name, legacy_name in STATIC_PRESSURE_PARAMS:
        value = get_param(ulog, name)

        if value is None:
            value = get_param(ulog, legacy_name, 0.0)

        params.append((name, float(value)))

    return params


def get_hover_thrust(ulog):
    hover = get_param(ulog, "MPC_THR_HOVER")

    if hover is not None and np.isfinite(hover) and hover > 0.0:
        return float(hover)

    print("Error: MPC_THR_HOVER is missing, non-finite, or <= 0; "
          "required for percent-based thrust excitation gates",
          file=sys.stderr)
    sys.exit(1)


def active_static_pressure_params(static_pressure_params):
    return [
        (name, value) for name, value in static_pressure_params
        if abs(value) > STATIC_PRESSURE_PARAM_EPS
    ]


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


POSE_FRAME_NED = 1
POSE_FRAME_FRD = 2
POSE_FRAME_NAMES = {
    POSE_FRAME_NED: "NED",
    POSE_FRAME_FRD: "FRD",
}
VALID_HEIGHT_POSE_FRAMES = set(POSE_FRAME_NAMES.keys())


def pose_frame_label(pose_frame):
    frames = sorted(set(int(f) for f in pose_frame
                        if int(f) in POSE_FRAME_NAMES))
    return "/".join(POSE_FRAME_NAMES[f] for f in frames)


@dataclass
class HeightReference:
    """Selected independent height reference used for calibration."""

    source: str
    label: str
    time_s: np.ndarray
    height_m: np.ndarray
    variance: np.ndarray = None
    min_valid_height_m: float = DEFAULT_MIN_HEIGHT_REF_M

    @classmethod
    def from_range(cls, range_data,
                   min_valid_height_m=DEFAULT_MIN_HEIGHT_REF_M):
        return cls(
            source="range",
            label="Distance sensor",
            time_s=range_data["time_s"],
            height_m=range_data["distance_m"],
            variance=range_data.get("variance"),
            min_valid_height_m=min_valid_height_m,
        )

    @classmethod
    def from_vis(cls, vis_data,
                 min_valid_height_m=DEFAULT_MIN_HEIGHT_REF_M):
        frame_label = pose_frame_label(vis_data["pose_frame"])
        return cls(
            source="vio",
            label=f"VIO altitude (-Z, {frame_label})",
            time_s=vis_data["time_s"],
            height_m=-vis_data["z"],
            variance=vis_data.get("z_var"),
            min_valid_height_m=min_valid_height_m,
        )

    def height_at(self, time_s):
        return np.interp(time_s, self.time_s, self.height_m)

    def valid_at(self, time_s):
        height = self.height_at(time_s)
        return np.isfinite(height) & (height > self.min_valid_height_m)


def select_height_reference(range_data, vis_data,
                            min_valid_height_m=DEFAULT_MIN_HEIGHT_REF_M):
    if range_data is not None:
        return HeightReference.from_range(range_data, min_valid_height_m)
    elif vis_data is not None:
        return HeightReference.from_vis(vis_data, min_valid_height_m)
    else:
        print("Error: missing height reference", file=sys.stderr)
        sys.exit(1)


# ---------------------------------------------------------------------------
# Data extraction
# ---------------------------------------------------------------------------

def extract_baro(ulog):
    d = get_topic(ulog, "vehicle_air_data")
    if d is None:
        print("Error: missing barometer data", file=sys.stderr)
        sys.exit(1)
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


def extract_wind(ulog):
    d = get_topic(ulog, "wind")
    if d is None:
        d = get_topic(ulog, "estimator_wind")
    if d is None:
        return None
    required = ["windspeed_north", "windspeed_east"]
    if not all(field in d.data for field in required):
        return None
    t = us_to_s(d.data["timestamp"], ulog.start_timestamp)
    north = d.data["windspeed_north"]
    east = d.data["windspeed_east"]
    valid = np.isfinite(t) & np.isfinite(north) & np.isfinite(east)
    if valid.sum() < 2:
        return None
    return {
        "time_s": t[valid],
        "north": north[valid],
        "east": east[valid],
    }


def extract_thrust(ulog):
    d = get_topic(ulog, "vehicle_thrust_setpoint")
    if d is None:
        print("Error: missing thrust data", file=sys.stderr)
        sys.exit(1)
    else:
        z = d.data.get("xyz[2]", None)
        if z is None:
            print("Error: missing z thrust data", file=sys.stderr)
            sys.exit(1)

    return {
        "time_s": us_to_s(d.data["timestamp"], ulog.start_timestamp),
        "thrust": np.where(np.isfinite(z), np.abs(z), 0.0),
    }

def extract_rpm(ulog):
    d = get_topic(ulog, "esc_status")
    if d is None:
        print("No rpm data logged, skipping", file=sys.stderr)
        return None

    out = {"time_s": us_to_s(d.data["timestamp"], ulog.start_timestamp)}
    for n in range(8):
        r = d.data.get(f"esc[{n}].esc_rpm")

        if r is None:
            r = d.data.get(f"esc.0{n}/esc_rpm")

        if r is None:
            r = d.data.get(f"esc.0{n}")

        if r is None:
            continue

        out[f"rpm{n}"] = r

    if len(out) == 1:
        print("No rpm fields logged, skipping", file=sys.stderr)
        return None

    return out


def extract_range(ulog):
    d = get_topic(ulog, "distance_sensor")

    if d is None:
        print(f"No distance_sensor in {ulog}")
        return None

    t = us_to_s(d.data["timestamp"], ulog.start_timestamp)
    dist = d.data["current_distance"]
    valid = np.isfinite(t) & np.isfinite(dist)

    if "signal_quality" in d.data:
        # TFmini publishes -1 when quality is unknown, and 0 when the range is
        # invalid. Accept unknown quality, but reject known-invalid samples.
        valid &= d.data["signal_quality"] != 0

    if valid.sum() < 2:
        return None

    result = {
        "time_s": t[valid],
        "distance_m": dist[valid],
        "raw_time_s": t,
        "raw_distance_m": dist,
    }

    if "variance" in d.data:
        variance = d.data["variance"]
        result["variance"] = np.where(np.isfinite(variance) & (variance > 0),
                                      variance, np.nan)[valid]

    if "signal_quality" in d.data:
        result["signal_quality"] = d.data["signal_quality"][valid]
        result["raw_signal_quality"] = d.data["signal_quality"]

    if "min_distance" in d.data:
        result["raw_min_distance"] = d.data["min_distance"]

    if "max_distance" in d.data:
        result["raw_max_distance"] = d.data["max_distance"]

    return result


def extract_vis(ulog):
    d = get_topic(ulog, "vehicle_visual_odometry")
    if d is None:
        print(f"No vehicle_visual_odometry in {ulog}")
        return None
    t = us_to_s(d.data["timestamp"], ulog.start_timestamp)

    required = ["pose_frame", "position[0]", "position[1]", "position[2]"]
    if not all(field in d.data for field in required):
        return None

    pose_frame = d.data["pose_frame"]
    x = d.data["position[0]"]
    y = d.data["position[1]"]
    z = d.data["position[2]"]
    valid_pose_frame = np.isin(pose_frame,
                               list(VALID_HEIGHT_POSE_FRAMES))
    valid = (np.isfinite(t) & np.isfinite(x) & np.isfinite(y)
             & np.isfinite(z) & valid_pose_frame)

    if valid.sum() < 2:
        return None

    result = {
        "time_s": t[valid],
        "pose_frame": pose_frame[valid],
        "x": x[valid],
        "y": y[valid],
        "z": z[valid],
    }

    variance_fields = ["position_variance[0]", "position_variance[1]",
                       "position_variance[2]"]

    if all(field in d.data for field in variance_fields):
        x_var = d.data["position_variance[0]"]
        y_var = d.data["position_variance[1]"]
        z_var = d.data["position_variance[2]"]
        result["x_var"] = np.where(np.isfinite(x_var) & (x_var >= 0),
                                   x_var, np.nan)[valid]
        result["y_var"] = np.where(np.isfinite(y_var) & (y_var >= 0),
                                   y_var, np.nan)[valid]
        result["z_var"] = np.where(np.isfinite(z_var) & (z_var >= 0),
                                   z_var, np.nan)[valid]

    return result


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
        result["vx"] = d.data["vx"]
        result["vy"] = d.data["vy"]
        result["vxy"] = np.sqrt(d.data["vx"]**2 + d.data["vy"]**2)
    if "xy_valid" in d.data:
        result["xy_valid"] = d.data["xy_valid"].astype(bool)
    if "v_xy_valid" in d.data:
        result["v_xy_valid"] = d.data["v_xy_valid"].astype(bool)
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


def rotate_earth_to_body(vx, vy, vz, qw, qx, qy, qz):
    """Rotate NED vector samples into body frame using q body->NED."""
    norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    norm = np.where(norm > 1e-6, norm, 1.0)
    qw = qw / norm
    qx = qx / norm
    qy = qy / norm
    qz = qz / norm

    r00 = 1 - 2 * (qy**2 + qz**2)
    r01 = 2 * (qx * qy - qw * qz)
    r02 = 2 * (qx * qz + qw * qy)
    r10 = 2 * (qx * qy + qw * qz)
    r11 = 1 - 2 * (qx**2 + qz**2)
    r12 = 2 * (qy * qz - qw * qx)
    r20 = 2 * (qx * qz - qw * qy)
    r21 = 2 * (qy * qz + qw * qx)
    r22 = 1 - 2 * (qx**2 + qy**2)

    return {
        "x": r00 * vx + r10 * vy + r20 * vz,
        "y": r01 * vx + r11 * vy + r21 * vz,
        "z": r02 * vx + r12 * vy + r22 * vz,
    }


def nearest_sample_age(query_t, sample_t):
    indices = np.searchsorted(sample_t, query_t)
    prev_idx = np.clip(indices - 1, 0, len(sample_t) - 1)
    next_idx = np.clip(indices, 0, len(sample_t) - 1)
    return np.minimum(np.abs(query_t - sample_t[prev_idx]),
                      np.abs(query_t - sample_t[next_idx]))


def estimate_static_pressure_compensation(ulog, baro_t,
                                          static_pressure_params,
                                          ekf_z, attitude, wind):
    active_params = active_static_pressure_params(static_pressure_params)
    if not active_params:
        return {
            "configured": False,
            "active_params": [],
        }

    missing = []
    if wind is None:
        missing.append("wind")
    if attitude is None:
        missing.append("vehicle_attitude")
    if ekf_z is None:
        missing.append("vehicle_local_position")
    elif not all(field in ekf_z for field in ["vx", "vy", "vz"]):
        missing.append("vehicle_local_position velocity")

    if missing:
        return {
            "configured": True,
            "evaluable": False,
            "active_params": active_params,
            "missing": missing,
        }

    param = dict(static_pressure_params)
    vmax = float(get_param(ulog, "SENS_BARO_VMAX",
                           get_param(ulog, "EKF2_ASPD_MAX", 20.0)))
    vmax_sq = vmax * vmax

    vx = np.interp(baro_t, ekf_z["time_s"], ekf_z["vx"])
    vy = np.interp(baro_t, ekf_z["time_s"], ekf_z["vy"])
    vz = np.interp(baro_t, ekf_z["time_s"], ekf_z["vz"])
    wn = np.interp(baro_t, wind["time_s"], wind["north"])
    we = np.interp(baro_t, wind["time_s"], wind["east"])

    qw = np.interp(baro_t, attitude["time_s"], attitude["qw"])
    qx = np.interp(baro_t, attitude["time_s"], attitude["qx"])
    qy = np.interp(baro_t, attitude["time_s"], attitude["qy"])
    qz = np.interp(baro_t, attitude["time_s"], attitude["qz"])

    valid = (np.isfinite(vx) & np.isfinite(vy) & np.isfinite(vz)
             & np.isfinite(wn) & np.isfinite(we)
             & np.isfinite(qw) & np.isfinite(qx)
             & np.isfinite(qy) & np.isfinite(qz))

    if "xy_valid" in ekf_z:
        xy_valid = np.interp(baro_t, ekf_z["time_s"],
                             ekf_z["xy_valid"].astype(float)) > 0.5
        valid &= xy_valid

    if "v_xy_valid" in ekf_z:
        v_xy_valid = np.interp(baro_t, ekf_z["time_s"],
                               ekf_z["v_xy_valid"].astype(float)) > 0.5
        valid &= v_xy_valid

    valid &= nearest_sample_age(baro_t, wind["time_s"]) <= 1.0

    if not valid.any():
        return {
            "configured": True,
            "evaluable": True,
            "active_params": active_params,
            "valid_fraction": 0.0,
            "correction_m": np.full(len(baro_t), np.nan),
            "max_abs_m": 0.0,
            "rms_m": 0.0,
            "vmax": vmax,
        }

    airspeed_body = rotate_earth_to_body(vx - wn, vy - we, vz,
                                         qw, qx, qy, qz)

    kx = np.where(airspeed_body["x"] >= 0.0,
                  param["SENS_BARO_K_XP"], param["SENS_BARO_K_XN"])
    ky = np.where(airspeed_body["y"] >= 0.0,
                  param["SENS_BARO_K_YP"], param["SENS_BARO_K_YN"])
    kz = param["SENS_BARO_K_Z"]

    airspeed_x_sq = np.minimum(airspeed_body["x"]**2, vmax_sq)
    airspeed_y_sq = np.minimum(airspeed_body["y"]**2, vmax_sq)
    airspeed_z_sq = np.minimum(airspeed_body["z"]**2, vmax_sq)

    correction = 0.5 * (airspeed_x_sq * kx
                        + airspeed_y_sq * ky
                        + airspeed_z_sq * kz) / GRAVITY
    correction = np.where(valid, correction, np.nan)
    valid_correction = correction[np.isfinite(correction)]

    return {
        "configured": True,
        "evaluable": True,
        "active_params": active_params,
        "valid_fraction": float(len(valid_correction) / len(baro_t)),
        "correction_m": correction,
        "max_abs_m": float(np.max(np.abs(valid_correction))),
        "rms_m": float(np.sqrt(np.mean(valid_correction**2))),
        "vmax": vmax,
    }


class CfRls:
    """Python port of BaroThrustCfRls.  Constants match baro_thrust_cf_rls.hpp."""

    # Defaults (matching baro_thrust_cf_rls.hpp)
    DEFAULT_CF_BANDWIDTH = DEFAULT_CF_BANDWIDTH_HZ
    DEFAULT_RLS_LAMBDA = DEFAULT_RLS_LAMBDA_FACTOR
    RLS_P_INIT = 100.0
    ERROR_VAR_INIT = 10.0
    ALPHA_ERR = 0.01

    def __init__(self, cf_bandwidth=None, rls_lambda=None,
                 hover_thrust=None,
                 min_thrust_excitation_pct=DEFAULT_MIN_THRUST_EXCITATION_PCT):
        bw = cf_bandwidth if cf_bandwidth is not None else self.DEFAULT_CF_BANDWIDTH
        self.CF_OMEGA = 2.0 * np.pi * bw
        self.CF_K1 = 2.0 * self.CF_OMEGA
        self.CF_K2 = self.CF_OMEGA ** 2
        self.RLS_LAMBDA = (rls_lambda if rls_lambda is not None
                           else self.DEFAULT_RLS_LAMBDA)
        if hover_thrust is None or not np.isfinite(hover_thrust) or hover_thrust <= 0.0:
            raise ValueError("hover_thrust must be finite and > 0")

        self.hover_thrust = float(hover_thrust)
        self.min_thrust_excitation_ratio = (
            float(min_thrust_excitation_pct) / 100.0)
        self.min_thrust_std = (self.min_thrust_excitation_ratio
                               * self.hover_thrust)
        self.min_thrust_var = self.min_thrust_std ** 2
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
        self._converged = False
        self._converged_locked = False
        self._converged_elapsed_s = 0.0
        self._excitation_elapsed_s = 0.0
        self._k_stable_elapsed_s = 0.0

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

    def check_convergence(self, elapsed_since_start_s, dt):
        # Convergence locking and stability
        if self._converged_locked:
            return

        variance_ok = self.k_var < DEFAULT_CONVERGENCE_VAR_THR

        if self._thrust_var > self.min_thrust_var:
            self._excitation_elapsed_s += dt

        excitation_ok = self._excitation_elapsed_s > DEFAULT_MIN_EXCITATION_TIME_S

        explained_var = self.k * self.k * max(self._thrust_var, 0.0)
        total_var = explained_var + self.error_var
        error_ok = (self.error_var < DEFAULT_CONVERGENCE_ERR_THR
                    or (total_var > 1.0
                        and self.error_var / total_var < DEFAULT_CONVERGENCE_ERR_REL_THR
                        and self.error_var < DEFAULT_CONVERGENCE_ERR_MAX_THR))
        time_ok = elapsed_since_start_s > DEFAULT_MIN_ESTIMATION_TIME_S

        if abs(self._k_smoothed - self.k) > DEFAULT_K_STABILITY_DIFF_THR:
            self._k_stable_elapsed_s = 0.0
        else:
            self._k_stable_elapsed_s += dt

        stability_ok = self._k_stable_elapsed_s > DEFAULT_K_STABILITY_TIME_S
        self._converged = (variance_ok and error_ok and excitation_ok
                           and time_ok and stability_ok)

        hold_ok = variance_ok and error_ok and time_ok and stability_ok

        if self._converged or (self._converged_elapsed_s > 0.0 and hold_ok):
            self._converged_elapsed_s += dt

            if self._converged_elapsed_s > DEFAULT_CONVERGENCE_HOLD_TIME_S:
                self._converged_locked = True
        else:
            self._converged_elapsed_s = 0.0

    @property
    def k(self):
        return float(self.theta[0])

    @property
    def k_var(self):
        return float(self.P[0, 0])

    @property
    def thrust_std(self):
        return float(np.sqrt(max(self._thrust_var, 0.0)))

    @property
    def thrust_excitation_ratio(self):
        return self.thrust_std / self.hover_thrust

    @property
    def converged(self):
        return self._converged

    @property
    def converged_locked(self):
        return self._converged_locked

    @property
    def excitation_elapsed_s(self):
        return self._excitation_elapsed_s

    @property
    def k_stable_elapsed_s(self):
        return self._k_stable_elapsed_s

    @property
    def convergence_hold_s(self):
        return self._converged_elapsed_s


def previous_sample_age(query_t, sample_t):
    """Age of the latest sample at each query time; NaN if no previous sample."""
    age = np.full(len(query_t), np.nan)
    valid_sample_t = sample_t[np.isfinite(sample_t)]

    if len(valid_sample_t) == 0:
        return age

    idx = np.searchsorted(valid_sample_t, query_t, side="right") - 1
    valid = idx >= 0
    age[valid] = query_t[valid] - valid_sample_t[idx[valid]]
    return age


def build_cf_rls_masks(baro_t, baro_alt, accel_up, accel_t, attitude_t,
                       thrust_t, thrust_interp, armed_start, armed_end,
                       landed, ekf_z=None,
                       max_vz=DEFAULT_MAX_VERTICAL_SPEED_M_S,
                       max_vxy=DEFAULT_MAX_HORIZONTAL_SPEED_M_S):
    """Build online-equivalent offline masks.

    CF runs only through hard guards. RLS additionally pauses through soft
    guards, matching the firmware behavior.
    """
    armed = (baro_t >= armed_start) & (baro_t <= armed_end)

    if landed is not None:
        is_landed = (np.interp(baro_t, landed["time_s"],
                               landed["landed"].astype(float)) > 0.5)
    else:
        is_landed = np.zeros(len(baro_t), dtype=bool)

    accel_age = previous_sample_age(baro_t, accel_t)
    attitude_age = previous_sample_age(baro_t, attitude_t)
    thrust_age = previous_sample_age(baro_t, thrust_t)

    # Separate state level updates and RLS updates to mimic offline mode
    cf_active = (armed & ~is_landed
                 & np.isfinite(baro_alt)
                 & np.isfinite(accel_up)
                 & np.isfinite(accel_age)
                 & (accel_age <= DEFAULT_MAX_ACCEL_SAMPLE_AGE_S)
                 & np.isfinite(attitude_age)
                 & (attitude_age <= DEFAULT_MAX_ATTITUDE_SAMPLE_AGE_S))

    rls_active = (cf_active
                  & np.isfinite(thrust_interp)
                  & np.isfinite(thrust_age)
                  & (thrust_age <= DEFAULT_MAX_THRUST_SAMPLE_AGE_S))

    if ekf_z is not None and "vz" in ekf_z:
        vz = np.interp(baro_t, ekf_z["time_s"], ekf_z["vz"])
        rls_active &= np.abs(vz) <= max_vz

    if ekf_z is not None and "vxy" in ekf_z:
        vxy = np.interp(baro_t, ekf_z["time_s"], ekf_z["vxy"])
        rls_active &= vxy <= max_vxy

    return cf_active, rls_active


def offline_save_status(offline, existing_k_t):
    reasons = []

    if not offline["would_lock_online"]:
        reasons.append("not locked")

    k_est = offline["final_k"]
    k_t_new = existing_k_t - k_est

    if abs(k_est) < DEFAULT_MIN_K_UPDATE_THRESHOLD:
        reasons.append("K below update threshold")

    if not np.isfinite(k_t_new):
        reasons.append("non-finite K_T")

    if np.isfinite(k_t_new) and abs(k_t_new) > DEFAULT_K_T_MAX:
        reasons.append("K_T out of range")

    return {
        "would_save": len(reasons) == 0,
        "k_t_new": k_t_new,
        "reasons": reasons,
    }


def offline_lock_reasons(offline):
    reasons = []

    if offline["final_k_var"] >= DEFAULT_CONVERGENCE_VAR_THR:
        reasons.append("K variance")

    error_var = offline["final_error_var"]
    explained_var = (offline["final_k"] * offline["final_k"]
                     * offline["final_thrust_std"] ** 2)
    total_var = explained_var + error_var
    error_ok = (error_var < DEFAULT_CONVERGENCE_ERR_THR
                or (total_var > 1.0
                    and error_var / total_var < DEFAULT_CONVERGENCE_ERR_REL_THR
                    and error_var < DEFAULT_CONVERGENCE_ERR_MAX_THR))

    if not error_ok:
        reasons.append("prediction error")

    if offline["excitation_elapsed_s"] <= DEFAULT_MIN_EXCITATION_TIME_S:
        reasons.append(
            f"thrust excitation "
            f"({100.0 * offline['final_thrust_excitation_ratio']:.1f}% "
            f"< {100.0 * offline['min_thrust_excitation_ratio']:.1f}% of hover)")

    if offline["estimation_elapsed_s"] <= DEFAULT_MIN_ESTIMATION_TIME_S:
        reasons.append("estimation time")

    if offline["k_stable_elapsed_s"] <= DEFAULT_K_STABILITY_TIME_S:
        reasons.append("K stability")

    if offline["convergence_hold_s"] <= DEFAULT_CONVERGENCE_HOLD_TIME_S:
        reasons.append("convergence hold")

    return reasons


def run_offline_cf_rls(baro, accel, attitude, thrust, landed,
                       armed_start, armed_end,
                       cf_bandwidth=None, rls_lambda=None,
                       ekf_z=None, height_ref=None,
                       max_vz=DEFAULT_MAX_VERTICAL_SPEED_M_S,
                       max_vxy=DEFAULT_MAX_HORIZONTAL_SPEED_M_S,
                       hover_thrust=None,
                       min_thrust_excitation_pct=DEFAULT_MIN_THRUST_EXCITATION_PCT):
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

    cf_active, rls_active = build_cf_rls_masks(
        baro_t, baro_alt, accel_up, accel["time_s"], attitude["time_s"],
        thrust["time_s"], thrust_interp, armed_start, armed_end, landed,
        ekf_z, max_vz, max_vxy)

    est = CfRls(cf_bandwidth=cf_bandwidth, rls_lambda=rls_lambda,
                hover_thrust=hover_thrust,
                min_thrust_excitation_pct=min_thrust_excitation_pct)
    n = len(baro_t)
    k_trace = np.full(n, np.nan)
    k_var_trace = np.full(n, np.nan)
    residual = np.full(n, np.nan)
    error_var = np.full(n, np.nan)
    thrust_std = np.full(n, np.nan)
    converged = np.zeros(n, dtype=bool)
    converged_locked = np.zeros(n, dtype=bool)
    estimation_active = np.zeros(n, dtype=bool)

    prev_t = None
    estimation_start_t = None
    lock_time_s = np.nan
    last_cf_t = np.nan

    for i in range(n):
        if not cf_active[i]:
            continue

        if prev_t is None:
            prev_t = baro_t[i]
            estimation_start_t = baro_t[i]
            last_cf_t = baro_t[i]
            residual[i] = 0.0
            k_trace[i] = est.k
            k_var_trace[i] = est.k_var
            error_var[i] = est.error_var
            thrust_std[i] = est.thrust_std
            continue

        dt = float(np.clip(baro_t[i] - prev_t, 0.001, 0.5))
        prev_t = baro_t[i]
        last_cf_t = baro_t[i]

        res = est.update_cf(float(baro_alt[i]), float(accel_up[i]), dt)

        if rls_active[i] and not est.converged_locked:
            est.update_rls(res, float(thrust_interp[i]), dt)

        elapsed_s = float(baro_t[i] - estimation_start_t)
        est.check_convergence(elapsed_s, dt)

        if est.converged_locked and not np.isfinite(lock_time_s):
            lock_time_s = float(baro_t[i] - armed_start)

        residual[i] = res
        k_trace[i] = est.k
        k_var_trace[i] = est.k_var
        error_var[i] = est.error_var
        thrust_std[i] = est.thrust_std
        converged[i] = est.converged
        converged_locked[i] = est.converged_locked
        estimation_active[i] = rls_active[i] and not est.converged_locked

    return {
        "time_s": baro_t,
        "k_trace": k_trace,
        "k_var_trace": k_var_trace,
        "residual": residual,
        "error_var": error_var,
        "thrust_std": thrust_std,
        "converged": converged,
        "converged_locked": converged_locked,
        "estimation_active": estimation_active,
        "cf_active": cf_active,
        "rls_active": rls_active,
        "final_k": est.k,
        "final_k_var": est.k_var,
        "final_error_var": est.error_var,
        "final_thrust_std": est.thrust_std,
        "final_thrust_excitation_ratio": est.thrust_excitation_ratio,
        "max_thrust_excitation_ratio": float(
            np.nanmax(thrust_std / est.hover_thrust)
            if np.isfinite(thrust_std).any() else np.nan),
        "hover_thrust": est.hover_thrust,
        "min_thrust_excitation_ratio": est.min_thrust_excitation_ratio,
        "min_thrust_std": est.min_thrust_std,
        "excitation_elapsed_s": est.excitation_elapsed_s,
        "k_stable_elapsed_s": est.k_stable_elapsed_s,
        "convergence_hold_s": est.convergence_hold_s,
        "estimation_elapsed_s": (
            float(last_cf_t - estimation_start_t)
            if estimation_start_t is not None and np.isfinite(last_cf_t)
            else 0.0
        ),
        "would_lock_online": est.converged_locked,
        "lock_time_s": lock_time_s,
        "cf_update_count": int(np.count_nonzero(cf_active)),
        "rls_update_count": int(np.count_nonzero(rls_active)),
    }


# ---------------------------------------------------------------------------
# Height-reference calibration  (distance sensor preferred, VIO fallback)
# ---------------------------------------------------------------------------

def fit_thrust_error_model(thrust, error, model_terms):
    """Fit error = sum(c_i * thrust**term_i). Term 0 is the bias column."""
    terms = []
    for term in model_terms:
        if term not in terms:
            terms.append(term)

    if 0 not in terms:
        terms.append(0)

    A = np.column_stack([
        np.ones(len(thrust)) if term == 0 else thrust ** term
        for term in terms
    ])

    coeffs, _, _, _ = np.linalg.lstsq(A, error, rcond=None)
    prediction = A @ coeffs
    residual = error - prediction
    error_var = np.var(error)

    return {
        "terms": terms,
        "coeffs": coeffs,
        "coeff_by_term": dict(zip(terms, coeffs)),
        "prediction": prediction,
        "residual": residual,
        "r2": float(1.0 - np.var(residual) / error_var
                    if error_var > 1e-10 else 0.0),
        "rmse": float(np.sqrt(np.mean(residual ** 2))),
    }


def eval_thrust_error_model(thrust, fit):
    prediction = np.zeros_like(thrust, dtype=float)

    for term, coeff in zip(fit["terms"], fit["coeffs"]):
        prediction += coeff if term == 0 else coeff * thrust ** term

    return prediction


def run_height_reference_calibration(baro, height_ref, thrust,
                                     armed_start, armed_end,
                                     existing_k_t=0.0,
                                     hover_thrust=None):
    """Least-squares calibration using a selected height reference.

    Undoes existing SENS_BARO_K_T compensation to recover raw baro, then fits:
        raw_baro_error = K_total * thrust + c

    Returns K_total (from-scratch gain), K_residual (remaining after existing
    SENS_BARO_K_T), recommended K_T, and data arrays for plotting.
    """
    baro_t, baro_alt = baro["time_s"], baro["alt_m"]
    ref_t, ref_height = height_ref.time_s, height_ref.height_m

    # Zero baro at arm time
    baro_zeroed = baro_alt - np.interp(armed_start, baro_t, baro_alt)

    # Undo existing compensation at baro timestamps
    thrust_at_baro = np.interp(baro_t, thrust["time_s"], thrust["thrust"])
    raw_baro = baro_zeroed - existing_k_t * thrust_at_baro

    # Interpolate both baro versions to reference timestamps
    raw_baro_interp = np.interp(ref_t, baro_t, raw_baro)
    comp_baro_interp = np.interp(ref_t, baro_t, baro_zeroed)
    raw_error = raw_baro_interp - ref_height
    comp_error = comp_baro_interp - ref_height

    # Filter: armed, above ground proximity
    mask = ((ref_t >= armed_start) & (ref_t <= armed_end)
            & (ref_height > height_ref.min_valid_height_m))
    if mask.sum() < 20:
        return None

    t_fit = ref_t[mask]
    raw_err_fit = raw_error[mask]
    comp_err_fit = comp_error[mask]
    thrust_fit = np.interp(t_fit, thrust["time_s"], thrust["thrust"])
    dt = np.diff(ref_t)
    valid_pair = mask[1:] & mask[:-1]
    valid_time_s = float(np.sum(np.clip(dt[valid_pair], 0.001, 0.5)))
    thrust_var = float(np.var(thrust_fit))
    thrust_std = float(np.sqrt(thrust_var))
    thrust_excitation_ratio = thrust_std / hover_thrust

    raw_linear_fit = fit_thrust_error_model(thrust_fit, raw_err_fit, [1, 0])
    raw_quadratic_fit = fit_thrust_error_model(thrust_fit, raw_err_fit,
                                               [2, 1, 0])
    K_total = float(raw_linear_fit["coeff_by_term"][1])

    # Compensated fit for comparison
    comp_linear_fit = fit_thrust_error_model(thrust_fit, comp_err_fit, [1, 0])

    K_residual = K_total + existing_k_t
    recommended_k_t = -K_total

    return {
        "K_total": K_total,
        "K_residual": K_residual,
        "recommended_k_t": recommended_k_t,
        "r2": raw_linear_fit["r2"],
        "rmse": raw_linear_fit["rmse"],
        "height_ref_source": height_ref.source,
        "height_ref_label": height_ref.label,
        "valid_time_s": valid_time_s,
        "thrust_std": thrust_std,
        "thrust_excitation_ratio": thrust_excitation_ratio,
        # Plotting data
        "time_s": ref_t,
        "dt": dt,
        "raw_error": raw_error,
        "comp_error": comp_error,
        "mask": mask,
        "t_fit": t_fit,
        "raw_err_fit": raw_err_fit,
        "comp_err_fit": comp_err_fit,
        "thrust_fit": thrust_fit,
        "thrust_var": thrust_var,
        "raw_coeffs": raw_linear_fit["coeffs"],
        "comp_coeffs": comp_linear_fit["coeffs"],
        "model_fits": {
            "raw_linear": raw_linear_fit,
            "raw_quadratic": raw_quadratic_fit,
            "comp_linear": comp_linear_fit,
        },
    }

def check_height_cal(height_cal, min_time_s=DEFAULT_TIME_S,
                     min_thrust_excitation_pct=DEFAULT_MIN_THRUST_EXCITATION_PCT):
    # Check various time / thrust thresholds
    if height_cal is None:
        print("Error: Failed height reference calibration")
        return False

    if height_cal["valid_time_s"] < min_time_s:
        print(f"Error: Failed valid time threshold ({height_cal['valid_time_s']:.2f} < {min_time_s:.2f})")
        return False

    min_ratio = min_thrust_excitation_pct / 100.0
    if height_cal["thrust_excitation_ratio"] < min_ratio:
        print("Error: Failed valid thrust excitation threshold "
              f"({100.0 * height_cal['thrust_excitation_ratio']:.1f}% "
              f"< {min_thrust_excitation_pct:.1f}% of hover)")
        return False

    return True

# ---------------------------------------------------------------------------
# CF+RLS parameter sweep
# ---------------------------------------------------------------------------

def sweep_cf_params(baro, accel, attitude, thrust, landed,
                    armed_start, armed_end, height_cal, existing_k_t,
                    ekf_z=None, height_ref=None,
                    max_vz=DEFAULT_MAX_VERTICAL_SPEED_M_S,
                    max_vxy=DEFAULT_MAX_HORIZONTAL_SPEED_M_S,
                    hover_thrust=None,
                    min_thrust_excitation_pct=DEFAULT_MIN_THRUST_EXCITATION_PCT):
    """Sweep CF bandwidth at two lambda values, evaluate against height reference."""
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

    cf_active, rls_active = build_cf_rls_masks(
        baro_t, baro_alt, accel_up, accel["time_s"], attitude["time_s"],
        thrust["time_s"], thrust_interp, armed_start, armed_end, landed,
        ekf_z, max_vz, max_vxy)
    cf_idx = np.where(cf_active)[0]

    raw_err = height_cal["raw_err_fit"]
    thr_fit = height_cal["thrust_fit"]

    results = {"bandwidth": bandwidths}

    for lam_key, lam in lambdas:
        k_list = []
        std_list = []

        for bw in bandwidths:
            est = CfRls(cf_bandwidth=bw, rls_lambda=lam,
                        hover_thrust=hover_thrust,
                        min_thrust_excitation_pct=min_thrust_excitation_pct)
            prev_t = None
            for i in cf_idx:
                if prev_t is None:
                    prev_t = baro_t[i]
                    continue
                dt = float(np.clip(baro_t[i] - prev_t, 0.001, 0.5))
                prev_t = baro_t[i]
                res = est.update_cf(float(baro_alt[i]),
                                    float(accel_up[i]), dt)

                if rls_active[i]:
                    est.update_rls(res, float(thrust_interp[i]), dt)

            candidate_k_t = existing_k_t - est.k
            corrected = raw_err + candidate_k_t * thr_fit

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


def plot_altitude_overview(ekf_baro_obs, ekf_z, height_ref, thrust_data,
                           armed_start, armed_end):
    """Page 1: Altitude overview — baro, height reference, EKF, thrust."""
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("Altitude Overview", fontsize=14, fontweight="bold")

    ax = axes[0]
    if height_ref is not None:
        ax.plot(height_ref.time_s, height_ref.height_m,
                label=height_ref.label, color="tab:blue", linewidth=1.2)
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


def plot_range_signal_quality(range_data):
    """Raw distance sensor samples grouped by signal_quality."""
    if range_data is None or "raw_signal_quality" not in range_data:
        return None

    t = range_data["raw_time_s"]
    dist = range_data["raw_distance_m"]
    quality = range_data["raw_signal_quality"]

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("Distance Sensor Signal Quality",
                 fontsize=14, fontweight="bold")
    fig.text(0.5, _SUBTITLE_Y,
             "PX4 distance_sensor quality: -1 = unknown, 0 = invalid, "
             ">0 = reported quality percentage.",
             ha="center", va="top", fontsize=9, style="italic", color="0.4")

    ax = axes[0]
    quality_groups = [
        (quality == -1, "unknown (-1)", "tab:blue"),
        (quality == 0, "invalid (0)", "tab:red"),
        (quality > 0, "reported >0", "tab:green"),
    ]

    for mask, label, color in quality_groups:
        if np.any(mask):
            ax.scatter(t[mask], dist[mask], s=4, alpha=0.55,
                       color=color, label=f"{label}: {int(np.sum(mask))}")

    if "raw_min_distance" in range_data:
        ax.plot(t, range_data["raw_min_distance"], color="0.25",
                linestyle="--", linewidth=0.8, label="min distance")

    if "raw_max_distance" in range_data:
        ax.plot(t, range_data["raw_max_distance"], color="0.45",
                linestyle=":", linewidth=0.8, label="max distance")

    ax.set_ylabel("Distance [m]")
    ax.set_title("Raw Distance Samples by Quality Class")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.step(t, quality, where="post", color="tab:purple", linewidth=0.8)
    unique_quality = sorted(set(int(q) for q in quality))
    if len(unique_quality) <= 8:
        ax.set_yticks(unique_quality)
    ax.axhline(0, color="tab:red", linestyle="--", linewidth=0.8,
               alpha=0.7)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("signal_quality")
    ax.set_title("Raw signal_quality Value")
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, _LAYOUT_TOP])
    return fig

def plot_rpm_overview(rpm):
    """Page 1: Altitude overview — baro, height reference, EKF, thrust."""
    fig, ax = plt.subplots(1, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("RPM Overview", fontsize=14, fontweight="bold")

    for i in range(8):
        field = f"rpm{i}"
        if field in rpm:
            ax.plot(rpm["time_s"], rpm[field], linewidth=0.8,
                    label=field)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("RPM")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    return fig

def plot_k_convergence(online, offline, height_cal,
                       armed_start, armed_end, existing_k_t,
                       hover_thrust, min_thrust_excitation_pct):
    """Page 2: K estimate convergence — online + offline overlaid."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle("K Estimate Convergence", fontsize=14, fontweight="bold")
    fig.text(0.5, _SUBTITLE_Y,
             "Online (firmware) and offline (replay) CF+RLS. "
             "Height-reference K shown when available.",
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

    if height_cal is not None:
        ax.axhline(height_cal["K_residual"], color="tab:green", linestyle="--",
                   linewidth=1.0,
                   label=f"{height_cal['height_ref_label']} K = "
                         f"{height_cal['K_residual']:.2f}")

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
        ax.fill_between(t[m], 0, conv * min_thrust_excitation_pct,
                        alpha=0.3, color="tab:green",
                        step="post", label="Converged")
        ax.plot(t[m], 100.0 * online["thrust_std"][m] / hover_thrust,
                color="tab:blue", linewidth=0.8,
                label="Online thrust excitation")
    if offline is not None:
        t = offline["time_s"]
        v = np.isfinite(offline["thrust_std"])
        ax.plot(t[v], 100.0 * offline["thrust_std"][v] / hover_thrust,
                color="tab:red", linewidth=0.8,
                label="Offline thrust excitation", alpha=0.8)
    ax.axhline(min_thrust_excitation_pct, color="k",
               linestyle=":", linewidth=0.5,
               label=f"Min excitation ({min_thrust_excitation_pct:.1f}% hover)")
    ax.set_ylabel("Thrust Excitation [% hover]")
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


def plot_height_reference_fit(height_cal, existing_k_t):
    """Page 4: selected height reference — raw and compensated error."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    ref_label = height_cal["height_ref_label"]
    fig.suptitle(f"Height Reference Validation ({ref_label})",
                 fontsize=14, fontweight="bold")
    fig.text(0.5, _SUBTITLE_Y,
             "Left: raw baro error (compensation undone). "
             "Right: as-flown (with existing SENS_BARO_K_T). "
             "Slope = thrust-correlated error.",
             ha="center", va="top", fontsize=9, style="italic", color="0.4")

    thrust = height_cal["thrust_fit"]
    x_fit = np.linspace(thrust.min(), thrust.max(), 50)

    # Top-left: Raw error vs thrust scatter
    ax = axes[0, 0]
    ax.scatter(thrust, height_cal["raw_err_fit"], s=2, alpha=0.3,
               color="tab:orange")
    c = height_cal["raw_coeffs"]
    ax.plot(x_fit, c[0] * x_fit + c[1], "k--", linewidth=1.2,
            label="linear")
    quad_fit = height_cal["model_fits"]["raw_quadratic"]
    ax.plot(x_fit, eval_thrust_error_model(x_fit, quad_fit),
            color="tab:purple", linestyle=":", linewidth=1.2,
            label=(f"quadratic R\u00b2={quad_fit['r2']:.3f}, "
                   f"RMSE={quad_fit['rmse']:.2f} m"))
    r = safe_corrcoef(thrust, height_cal["raw_err_fit"])
    ax.set_title(f"Raw: K = {c[0]:.1f}, r = {r:.3f}, "
                 f"R\u00b2 = {height_cal['r2']:.3f}")
    ax.set_xlabel("Thrust [0-1]")
    ax.set_ylabel("Baro Error [m]")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Top-right: Compensated error vs thrust scatter
    ax = axes[0, 1]
    ax.scatter(thrust, height_cal["comp_err_fit"], s=2, alpha=0.3,
               color="tab:blue")
    cc = height_cal["comp_coeffs"]
    ax.plot(x_fit, cc[0] * x_fit + cc[1], "k--", linewidth=1.2)
    r = safe_corrcoef(thrust, height_cal["comp_err_fit"])
    ax.set_title(f"Compensated (K_T={existing_k_t:+.1f}): "
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
    t = height_cal["t_fit"]
    ax = axes[1, 0]
    ax.plot(t, height_cal["raw_err_fit"], color="tab:orange", linewidth=0.8)
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
    std_raw = float(np.std(height_cal["raw_err_fit"]))
    ax.set_title(f"Raw error: std = {std_raw:.2f} m")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Baro Error [m]")
    ax.grid(True, alpha=0.3)

    # Bottom-right: Compensated error time series
    ax = axes[1, 1]
    ax.plot(t, height_cal["comp_err_fit"], color="tab:blue", linewidth=0.8)
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
    std_comp = float(np.std(height_cal["comp_err_fit"]))
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


def plot_cf_tuning(sweep, height_cal, existing_k_t):
    """Page: CF parameter sensitivity — K and error std vs bandwidth."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("CF+RLS Parameter Sensitivity", fontsize=14, fontweight="bold")
    fig.text(0.5, _SUBTITLE_Y,
             "Sweeping CF bandwidth: how does K and compensation quality "
             "change? Height-reference K is the comparison. Lower error std = better.",
             ha="center", va="top", fontsize=9, style="italic", color="0.4")

    bw = sweep["bandwidth"]
    default_bw = CfRls.DEFAULT_CF_BANDWIDTH
    ref_k = height_cal["K_residual"]
    ref_label = height_cal["height_ref_label"]

    styles = [
        ("lambda_0.998", "\u03bb=0.998 (default)", "tab:red"),
        ("lambda_1.0", "\u03bb=1.0 (no forget)", "tab:purple"),
    ]

    # --- Left: K vs bandwidth ---
    ax = axes[0]
    for key, label, color in styles:
        ax.semilogx(bw, sweep[key]["k_residual"], "o-", color=color,
                    markersize=3, linewidth=1.0, label=label)
    ax.axhline(ref_k, color="tab:green", linestyle="--", linewidth=1.0,
               label=f"{ref_label} K = {ref_k:.2f}")
    ax.axvline(default_bw, color="tab:gray", linestyle=":", linewidth=0.8,
               label=f"Default ({default_bw} Hz)")

    # Best K match (default lambda)
    k_def = sweep["lambda_0.998"]["k_residual"]
    idx_best = int(np.argmin(np.abs(k_def - ref_k)))
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

    ideal_err = (height_cal["raw_err_fit"]
                 + height_cal["recommended_k_t"] * height_cal["thrust_fit"])
    ideal_std = float(np.std(ideal_err))
    ax.axhline(ideal_std, color="tab:green", linestyle="--", linewidth=1.0,
               label=f"{ref_label} optimal: {ideal_std:.2f} m")

    current_std = float(np.std(height_cal["comp_err_fit"]))
    ax.axhline(current_std, color="tab:orange", linestyle=":", linewidth=1.0,
               label=f"Current K_T: {current_std:.2f} m")

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

    def non_negative_float(value):
        try:
            value = float(value)
        except ValueError as exc:
            raise argparse.ArgumentTypeError("must be a number") from exc

        if value < 0:
            raise argparse.ArgumentTypeError("must be non-negative")

        return value

    parser.add_argument("--output-dir", "-o", default=default_log_dir,
                        help="Output base directory (default: <PX4_ROOT>/logs/)")
    parser.add_argument("--min-height-ref-m", type=non_negative_float,
                        default=DEFAULT_MIN_HEIGHT_REF_M,
                        help=("Minimum selected height reference used for "
                              "calibration and offline RLS updates "
                              f"(default: {DEFAULT_MIN_HEIGHT_REF_M} m)"))
    parser.add_argument("--max-vz", type=non_negative_float,
                        default=DEFAULT_MAX_VERTICAL_SPEED_M_S,
                        help=("Maximum absolute vertical speed used for "
                              "offline RLS updates "
                              f"(default: {DEFAULT_MAX_VERTICAL_SPEED_M_S} m/s)"))
    parser.add_argument("--max-vxy", type=non_negative_float,
                        default=DEFAULT_MAX_HORIZONTAL_SPEED_M_S,
                        help=("Maximum horizontal speed used for offline RLS "
                              f"updates (default: {DEFAULT_MAX_HORIZONTAL_SPEED_M_S} m/s)"))
    parser.add_argument("--cf-bandwidth", type=non_negative_float,
                        default=DEFAULT_CF_BANDWIDTH_HZ,
                        help=("Offline CF crossover frequency "
                              f"(default: {DEFAULT_CF_BANDWIDTH_HZ} Hz)"))
    parser.add_argument("--rls-lambda", type=non_negative_float,
                        default=DEFAULT_RLS_LAMBDA_FACTOR,
                        help=("Offline RLS forgetting factor "
                              f"(default: {DEFAULT_RLS_LAMBDA_FACTOR})"))
    parser.add_argument("--min-cal-time-s", type=non_negative_float,
                        default=DEFAULT_TIME_S,
                        help=("Minimum valid height-reference calibration time "
                              f"(default: {DEFAULT_TIME_S} s)"))
    parser.add_argument("--min-thrust-excitation-pct",
                        type=non_negative_float,
                        default=DEFAULT_MIN_THRUST_EXCITATION_PCT,
                        help=("Minimum thrust standard deviation as a "
                              "percentage of MPC_THR_HOVER required to "
                              "recommend SENS_BARO_K_T "
                              f"(default: {DEFAULT_MIN_THRUST_EXCITATION_PCT:.1f}%%)"))
    args = parser.parse_args()

    if args.max_vz <= 0.0 or args.max_vxy <= 0.0:
        print("Error: --max-vz and --max-vxy must be > 0", file=sys.stderr)
        sys.exit(1)

    if args.cf_bandwidth <= 0.0:
        print("Error: --cf-bandwidth must be > 0", file=sys.stderr)
        sys.exit(1)

    if not 0.0 < args.rls_lambda <= 1.0:
        print("Error: --rls-lambda must be in (0, 1]", file=sys.stderr)
        sys.exit(1)

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
    existing_k_t = float(get_param(ulog, "SENS_BARO_K_T", 0.0))
    hover_thrust = get_hover_thrust(ulog)
    static_pressure_params = get_static_pressure_params(ulog)
    armed_start, armed_end = detect_armed_period(ulog)

    # ── Extract ──
    baro = extract_baro(ulog)
    accel = extract_accel(ulog)
    attitude = extract_attitude(ulog)
    wind = extract_wind(ulog)
    thrust = extract_thrust(ulog)
    rpm = extract_rpm(ulog)

    range_data = extract_range(ulog)
    vis_data = extract_vis(ulog)
    if range_data is None and vis_data is None:
        print("Error: missing height reference", file=sys.stderr)
        sys.exit(1)

    height_ref = select_height_reference(
        range_data, vis_data, args.min_height_ref_m)

    online = extract_online_estimate(ulog)
    ekf_z = extract_ekf_z(ulog)
    ekf_baro_obs = extract_ekf_baro_obs(ulog)
    landed = extract_landed(ulog)

    static_pressure = estimate_static_pressure_compensation(
        ulog, baro["time_s"], static_pressure_params, ekf_z, attitude, wind)

    # ── Summary header ──
    summary = []
    summary.append(f"Log: {log_name}")
    summary.append(f"Duration: {duration:.1f}s")
    summary.append(f"Armed: {armed_start:.1f}s - {armed_end:.1f}s")
    summary.append(f"SENS_BARO_K_T: {existing_k_t:+.2f}")
    summary.append(f"MPC_THR_HOVER: {hover_thrust:.3f}")
    summary.append("")
    summary.append("Analysis settings:")
    summary.append(f"  Min height reference:   {args.min_height_ref_m:.2f} m")
    summary.append(f"  Max vertical speed:     {args.max_vz:.2f} m/s")
    summary.append(f"  Max horizontal speed:   {args.max_vxy:.2f} m/s")
    summary.append(f"  Offline CF bandwidth:   {args.cf_bandwidth:.3f} Hz")
    summary.append(f"  Offline RLS lambda:     {args.rls_lambda:.3f}")
    summary.append(f"  Min calibration time:   {args.min_cal_time_s:.1f} s")
    summary.append(f"  Min thrust excitation:  "
                   f"{args.min_thrust_excitation_pct:.1f}% of hover")
    summary.append(f"  Max accel age:          {DEFAULT_MAX_ACCEL_SAMPLE_AGE_S:.2f} s")
    summary.append(f"  Max attitude age:       {DEFAULT_MAX_ATTITUDE_SAMPLE_AGE_S:.2f} s")
    summary.append(f"  Max thrust age:         {DEFAULT_MAX_THRUST_SAMPLE_AGE_S:.2f} s")
    summary.append("")

    for pname in ["EKF2_HGT_REF", "EKF2_BARO_CTRL", "EKF2_BARO_NOISE",
                  "SENS_BAR_AUTOCAL"]:
        val = get_param(ulog, pname)
        if val is not None:
            summary.append(f"  {pname:24s} = {val}")
    summary.append("")

    if static_pressure["configured"]:
        summary.append("Warning:")
        summary.append("  Static pressure compensation coefficients are nonzero.")

        if static_pressure.get("evaluable"):
            summary.append("  The log contains enough data to replay the "
                           "static-pressure correction.")
            summary.append(f"  Estimated static correction: "
                           f"RMS {static_pressure['rms_m']:.3f} m, "
                           f"max {static_pressure['max_abs_m']:.3f} m, "
                           f"valid {100.0 * static_pressure['valid_fraction']:.1f}%")

            if static_pressure["valid_fraction"] > 0.0:
                summary.append("  Thrust calibration is fitting already "
                               "airspeed-corrected baro data.")
            else:
                summary.append("  The logged validity gates indicate the "
                               "correction did not run during baro samples.")
        else:
            missing = ", ".join(static_pressure.get("missing", []))
            summary.append("  The log is missing data needed to confirm "
                           "when the correction ran.")
            summary.append(f"  Missing: {missing}")

        for name, value in static_pressure["active_params"]:
            summary.append(f"  {name:24s} = {value:+.3f}")

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
    if wind is not None:
        summary.append(f"  Wind:    {effective_rate(wind['time_s']):5.1f} Hz  "
                       f"({len(wind['time_s'])} samples)")
    if range_data is not None:
        range_note = ""
        if "raw_signal_quality" in range_data:
            q = range_data["raw_signal_quality"]
            range_note = (f", q=-1:{int(np.sum(q == -1))} "
                          f"q=0:{int(np.sum(q == 0))} "
                          f"q>0:{int(np.sum(q > 0))}")
        summary.append(f"  Range:   {effective_rate(range_data['time_s']):5.1f} Hz  "
                       f"({len(range_data['time_s'])} samples{range_note})")
    if vis_data is not None:
        summary.append(f"  VIO:     {effective_rate(vis_data['time_s']):5.1f} Hz  "
                       f"({len(vis_data['time_s'])} samples, "
                       f"{pose_frame_label(vis_data['pose_frame'])})")
    if online is not None:
        summary.append(f"  Online:  {effective_rate(online['time_s']):5.1f} Hz  "
                       f"({len(online['time_s'])} samples)")
    summary.append("")

    # ── Analysis ──
    figures = []
    figures.append(plot_altitude_overview(
        ekf_baro_obs, ekf_z, height_ref, thrust, armed_start, armed_end))

    if not(rpm is None):
        figures.append(plot_rpm_overview(rpm))

    range_quality_fig = plot_range_signal_quality(range_data)
    if range_quality_fig is not None:
        figures.append(range_quality_fig)

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
            summary.append(f"  SENS_BARO_K_T: {existing_k_t:+.2f} "
                           f"- {online_k:.2f} = "
                           f"{existing_k_t - online_k:+.2f}")
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
                                     cf_bandwidth=args.cf_bandwidth,
                                     rls_lambda=args.rls_lambda,
                                     ekf_z=ekf_z,
                                     height_ref=height_ref,
                                     max_vz=args.max_vz,
                                     max_vxy=args.max_vxy,
                                     hover_thrust=hover_thrust,
                                     min_thrust_excitation_pct=args.min_thrust_excitation_pct)
        offline_save = offline_save_status(offline, existing_k_t)
        summary.append(f"Offline CF+RLS:")
        summary.append(f"  Final K = {offline['final_k']:.3f}")
        summary.append(f"  CF updates = {offline['cf_update_count']}, "
                       f"RLS updates = {offline['rls_update_count']}")
        summary.append(f"  Thrust excitation = "
                       f"{100.0 * offline['final_thrust_excitation_ratio']:.1f}% "
                       f"of hover final, "
                       f"{100.0 * offline['max_thrust_excitation_ratio']:.1f}% "
                       f"max, "
                       f"{offline['excitation_elapsed_s']:.1f}s above "
                       f"{100.0 * offline['min_thrust_excitation_ratio']:.1f}%")
        if offline["would_lock_online"]:
            summary.append(f"  Would lock with configured gates: yes "
                           f"({offline['lock_time_s']:.1f}s after arm)")
        else:
            reasons = ", ".join(offline_lock_reasons(offline))
            summary.append(f"  Would lock with configured gates: no ({reasons})")

        if offline_save["would_save"]:
            summary.append(f"  Would save SENS_BARO_K_T: "
                           f"{offline_save['k_t_new']:+.2f}")
        else:
            reasons = ", ".join(offline_save["reasons"])
            summary.append(f"  Would save SENS_BARO_K_T: no ({reasons})")
        summary.append("")
    else:
        summary.append("Offline CF+RLS: skipped (missing accel or attitude)")
        summary.append("")

    # 3. Height reference calibration
    height_cal = None
    height_cal_recommended = False
    if height_ref is not None:
        height_cal = run_height_reference_calibration(baro, height_ref, thrust,
                                                     armed_start, armed_end,
                                                     existing_k_t,
                                                     hover_thrust)

        height_cal_recommended = check_height_cal(height_cal,
                                                  args.min_cal_time_s,
                                                  args.min_thrust_excitation_pct)

        if height_cal_recommended:
            quad_fit = height_cal["model_fits"]["raw_quadratic"]
            summary.append(f"{height_ref.label} reference:")
            summary.append(f"  K_total = {height_cal['K_total']:.3f}  "
                           f"(R\u00b2 = {height_cal['r2']:.3f}, "
                           f"RMSE = {height_cal['rmse']:.3f} m)")
            summary.append(f"  Quadratic diagnostic: "
                           f"R\u00b2 = {quad_fit['r2']:.3f}, "
                           f"RMSE = {quad_fit['rmse']:.3f} m")
            summary.append(f"  K_residual = {height_cal['K_residual']:.3f}")
            summary.append(f"  Valid time = {height_cal['valid_time_s']:.1f}s, "
                           f"thrust excitation = "
                           f"{100.0 * height_cal['thrust_excitation_ratio']:.1f}% "
                           f"of hover")
            summary.append(f"SENS_BARO_K_T: "
                           f"{height_cal['recommended_k_t']:+.2f}")
            summary.append("")
        else:
            if height_cal is not None:
                summary.append(f"{height_ref.label} calibration: insufficient data "
                               f"(valid time = {height_cal['valid_time_s']:.1f}s, "
                               f"thrust excitation = "
                               f"{100.0 * height_cal['thrust_excitation_ratio']:.1f}% "
                               f"of hover)")
            else:
                summary.append(f"{height_ref.label} calibration: insufficient data")
            summary.append("")

    # ── Comparison table ──
    sep = "=" * 60
    summary.append(sep)
    r2_label = "R\u00b2"
    summary.append(f"  {'Method':<22} {'K':>8} {'Set K_T':>12} "
                   f"{r2_label:>6}")
    summary.append("-" * 60)

    if online_k is not None:
        k_t = existing_k_t - online_k
        summary.append(f"  {'Online CF+RLS':<22} {online_k:>8.3f} "
                       f"{k_t:>+12.2f}")

    if offline is not None:
        offline_save = offline_save_status(offline, existing_k_t)
        k_t_text = (f"{offline_save['k_t_new']:+.2f}"
                    if offline_save["would_save"] else "n/a")
        summary.append(f"  {'Offline CF+RLS':<22} "
                       f"{offline['final_k']:>8.3f} {k_t_text:>12}")

    if height_cal is not None:
        label = f"{height_cal['height_ref_source'].upper()} reference"
        if height_cal_recommended:
            summary.append(f"  {label:<22} "
                           f"{height_cal['K_residual']:>8.3f} "
                           f"{height_cal['recommended_k_t']:>+12.2f} "
                           f"{height_cal['r2']:>6.3f}")
        else:
            summary.append(f"  {label:<22} "
                           f"{height_cal['K_residual']:>8.3f} "
                           f"{'n/a':>12} "
                           f"{height_cal['r2']:>6.3f}")

    summary.append(sep)

    # Pairwise differences
    if online_k is not None and offline is not None:
        d = abs(online_k - offline["final_k"])
        summary.append(f"  Online vs Offline:  \u0394K = {d:.3f}")

    if online_k is not None and height_cal is not None and height_cal_recommended:
        d = abs(online_k - height_cal["K_residual"])
        summary.append(f"  Online vs Reference:  \u0394K = {d:.3f}")

    if offline is not None and height_cal is not None and height_cal_recommended:
        d = abs(offline["final_k"] - height_cal["K_residual"])
        summary.append(f"  Offline vs Reference: \u0394K = {d:.3f}")

    summary.append("")

    # Print to console
    for line in summary:
        print(line)

    # ── Generate plots ──
    figures.append(plot_k_convergence(
        online, offline, height_cal, armed_start, armed_end, existing_k_t,
        hover_thrust, args.min_thrust_excitation_pct))

    if online is not None or offline is not None:
        figures.append(plot_residual_comparison(
            online, offline, thrust, armed_start, armed_end))

    if height_cal is not None:
        figures.append(plot_height_reference_fit(height_cal, existing_k_t))

    # ── Parameter sweep (when height reference available) ──
    if (height_cal is not None and height_cal_recommended and accel is not None
            and attitude is not None):
        print("\nSweeping CF bandwidth...")
        sweep = sweep_cf_params(baro, accel, attitude, thrust, landed,
                                armed_start, armed_end, height_cal,
                                existing_k_t,
                                ekf_z=ekf_z,
                                height_ref=height_ref,
                                max_vz=args.max_vz,
                                max_vxy=args.max_vxy,
                                hover_thrust=hover_thrust,
                                min_thrust_excitation_pct=args.min_thrust_excitation_pct)
        fig, best_bw, min_bw, min_std = plot_cf_tuning(
            sweep, height_cal, existing_k_t)
        figures.append(fig)

        ref_k = height_cal["K_residual"]
        k_def = sweep["lambda_0.998"]["k_residual"]
        idx_best = int(np.argmin(np.abs(k_def - ref_k)))
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
