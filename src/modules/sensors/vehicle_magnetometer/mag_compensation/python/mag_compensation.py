#!/usr/bin/env python3
"""
Identify mag current/throttle compensation from a ulog.

Estimates a bulk delay dt from |m| vs the power signal, then fits
    m(t) = R(t)^T B + b + k * power(t - dt)
and prints CAL_MAG* parameters.

Usage:
    python mag_compensation.py LOG.ulg
    python mag_compensation.py LOG.ulg current --instance 0
    python mag_compensation.py LOG.ulg thrust
"""

import argparse
import sys

import numpy as np
from pyulog import ULog

# PX4 rot_lookup (roll, pitch, yaw deg), index = Rotation enum. See rotation.h.
_ROT_LOOKUP = (
    (0, 0, 0), (0, 0, 45), (0, 0, 90), (0, 0, 135), (0, 0, 180), (0, 0, 225),
    (0, 0, 270), (0, 0, 315), (180, 0, 0), (180, 0, 45), (180, 0, 90),
    (180, 0, 135), (0, 180, 0), (180, 0, 225), (180, 0, 270), (180, 0, 315),
    (90, 0, 0), (90, 0, 45), (90, 0, 90), (90, 0, 135), (270, 0, 0),
    (270, 0, 45), (270, 0, 90), (270, 0, 135), (0, 90, 0), (0, 270, 0),
    (0, 180, 90), (0, 180, 270), (90, 90, 0), (180, 90, 0), (270, 90, 0),
    (90, 180, 0), (270, 180, 0), (90, 270, 0), (180, 270, 0), (270, 270, 0),
    (90, 180, 90), (90, 0, 270), (90, 68, 293), (0, 315, 0), (90, 315, 0),
)

ARMING_STATE_ARMED = 2


def get_data(log, topic, field, instance=0):
    try:
        return log.get_dataset(topic, instance).data[field]
    except Exception:
        return np.array([])


def get_param(log, name, default=None):
    if name in log.initial_parameters:
        return log.initial_parameters[name]
    return default


def us_to_s(t):
    return t * 1e-6 if len(t) else t


def topic_time(log, topic, instance=0):
    ts = get_data(log, topic, 'timestamp_sample', instance)
    t = get_data(log, topic, 'timestamp', instance)
    if len(ts) == len(t) and np.any(ts):
        return us_to_s(ts)
    return us_to_s(t)


def dcm_from_quat(q):
    """v_ned = R v_body. q is (N, 4) Hamilton wxyz, matching matrix::Dcm(Quat)."""
    a, b, c, d = q.T
    ab, ac, ad = a * b, a * c, a * d
    bb, bc, bd = b * b, b * c, b * d
    cc, cd, dd = c * c, c * d, d * d
    R = np.empty((q.shape[0], 3, 3))
    R[:, 0, 0] = 1 - 2 * (cc + dd)
    R[:, 0, 1] = 2 * (bc - ad)
    R[:, 0, 2] = 2 * (ac + bd)
    R[:, 1, 0] = 2 * (bc + ad)
    R[:, 1, 1] = 1 - 2 * (bb + dd)
    R[:, 1, 2] = 2 * (cd - ab)
    R[:, 2, 0] = 2 * (bd - ac)
    R[:, 2, 1] = 2 * (ab + cd)
    R[:, 2, 2] = 1 - 2 * (bb + cc)
    return R


def dcm_from_euler_deg(roll, pitch, yaw):
    """3-2-1 intrinsic, matching matrix::Dcm(Eulerf)."""
    phi, theta, psi = np.deg2rad([roll, pitch, yaw])
    cp, sp = np.cos(phi), np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    cs, ss = np.cos(psi), np.sin(psi)
    return np.array([
        [ct * cs, -cp * ss + sp * st * cs, sp * ss + cp * st * cs],
        [ct * ss, cp * cs + sp * st * ss, -sp * cs + cp * st * ss],
        [-st, sp * ct, cp * ct],
    ])


def dcm_from_rot_enum(rot):
    if rot == 100:
        raise ValueError('custom rotation needs euler angles')
    if rot < 0 or rot >= len(_ROT_LOOKUP):
        rot = 0
    return dcm_from_euler_deg(*_ROT_LOOKUP[rot])


def sensor_to_body_dcm(log, cal_instance):
    rot = int(get_param(log, f'CAL_MAG{cal_instance}_ROT', -1))
    if rot < 0:
        rot = int(get_param(log, 'SENS_BOARD_ROT', 0))
    if rot == 100:
        return dcm_from_euler_deg(
            float(get_param(log, f'CAL_MAG{cal_instance}_ROLL', 0)),
            float(get_param(log, f'CAL_MAG{cal_instance}_PITCH', 0)),
            float(get_param(log, f'CAL_MAG{cal_instance}_YAW', 0)),
        )
    return dcm_from_rot_enum(rot)


def unwrap_quats(q):
    q = np.array(q, dtype=float, copy=True)
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] *= -1
    return q


def interp_quats(t_src, q_src, t_dst):
    q_src = unwrap_quats(q_src)
    q = np.column_stack([np.interp(t_dst, t_src, q_src[:, i]) for i in range(4)])
    n = np.linalg.norm(q, axis=1, keepdims=True)
    n = np.clip(n, 1e-12, None)
    return q / n


def armed_mask(log, t):
    armed = get_data(log, 'vehicle_status', 'arming_state', 0)
    t_armed = topic_time(log, 'vehicle_status', 0)
    if len(armed) == 0:
        return np.ones(len(t), dtype=bool)
    # step-hold arming onto mag times
    idx = np.searchsorted(t_armed, t, side='right') - 1
    idx = np.clip(idx, 0, len(armed) - 1)
    return armed[idx] == ARMING_STATE_ARMED


def estimate_dt(t_mag, mag_norm, t_p, power, dt_max=0.08, dt_step=0.002):
    """Lag that maximises |corr| of first-differences of |m| and power."""
    dt_grid = np.arange(-dt_max, dt_max + dt_step * 0.5, dt_step)
    mag_d = np.diff(mag_norm)
    best_dt = 0.0
    best_c = 0.0
    corr = np.zeros_like(dt_grid)
    if len(mag_d) < 20 or np.std(mag_d) < 1e-9:
        return best_dt, best_c, dt_grid, corr
    for i, dt in enumerate(dt_grid):
        p = np.interp(t_mag - dt, t_p, power)
        p_d = np.diff(p)
        if np.std(p_d) < 1e-12:
            continue
        c = np.corrcoef(mag_d, p_d)[0, 1]
        if not np.isfinite(c):
            continue
        corr[i] = c
        if abs(c) > abs(best_c):
            best_c = c
            best_dt = dt
    return best_dt, best_c, dt_grid, corr


def fit_comp(mag, power, R_ns=None):
    """
    mag (N,3), power (N,).
    If R_ns (N,3,3) maps sensor → NED, fit m = R^T B + b + k * power.
    Else fit m = b + k * power.
    Returns k, b, B (or None), r2 of the power term.
    """
    n = mag.shape[0]
    if n < 10:
        return np.zeros(3), np.zeros(3), None, 0.0

    if R_ns is None:
        A = np.column_stack([np.ones(n), power])
        k = np.zeros(3)
        b = np.zeros(3)
        for i in range(3):
            coef, *_ = np.linalg.lstsq(A, mag[:, i], rcond=None)
            b[i], k[i] = coef
        resid = mag - (b + np.outer(power, k))
        demeaned = mag - mag.mean(axis=0)
        r2 = 1.0 - np.sum(resid ** 2) / max(np.sum(demeaned ** 2), 1e-18)
        return k, b, None, r2

    y = mag.reshape(n * 3)
    A = np.zeros((n * 3, 9))
    Rt = np.transpose(R_ns, (0, 2, 1))
    A[:, 0:3] = Rt.reshape(n * 3, 3)
    A[:, 3:6] = np.tile(np.eye(3), (n, 1))
    A[:, 6:9] = (power[:, None, None] * np.eye(3)[None, :, :]).reshape(n * 3, 3)
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    B, b, k = x[0:3], x[3:6], x[6:9]
    earth_b = np.einsum('nji,j->ni', R_ns, B) + b
    resid = mag - earth_b - np.outer(power, k)
    ref = mag - earth_b
    r2 = 1.0 - np.sum(resid ** 2) / max(np.sum(ref ** 2), 1e-18)
    return k, b, B, r2


def load_power(log, comp_type, instance):
    if comp_type == 'current':
        current = get_data(log, 'battery_status', 'current_a', instance)
        t = topic_time(log, 'battery_status', instance)
        if len(current) == 0:
            return None, None, None, None
        power = current / 1000.0  # kA, matches VehicleMagnetometer
        return power, t, 2 + instance, '[G/kA]'

    xyz = [get_data(log, 'vehicle_thrust_setpoint', f'xyz[{i}]', instance) for i in range(3)]
    if all(len(a) for a in xyz):
        power = np.linalg.norm(np.column_stack(xyz), axis=1)
        t = topic_time(log, 'vehicle_thrust_setpoint', instance)
        return power, t, 1, '[G]'

    thrust_z = get_data(log, 'vehicle_rates_setpoint', 'thrust_body[2]', instance)
    t = topic_time(log, 'vehicle_rates_setpoint', instance)
    if len(thrust_z) == 0:
        return None, None, None, None
    power = np.abs(thrust_z)
    return power, t, 1, '[G]'


def load_mags(log):
    mags = []
    for inst in range(4):
        x = get_data(log, 'sensor_mag', 'x', inst)
        if len(x) == 0:
            continue
        mag = np.column_stack([
            x,
            get_data(log, 'sensor_mag', 'y', inst),
            get_data(log, 'sensor_mag', 'z', inst),
        ])
        t = topic_time(log, 'sensor_mag', inst)
        device_id = int(get_data(log, 'sensor_mag', 'device_id', inst)[0])
        mags.append((inst, device_id, t, mag))
    return mags


def load_attitude(log):
    t = topic_time(log, 'vehicle_attitude', 0)
    q = [get_data(log, 'vehicle_attitude', f'q[{i}]', 0) for i in range(4)]
    if len(t) == 0 or any(len(c) == 0 for c in q):
        return None, None
    return t, np.column_stack(q)


def calibration_instance(log, device_id):
    for j in range(4):
        if int(get_param(log, f'CAL_MAG{j}_ID', 0)) == device_id:
            return j
    return None


def main():
    parser = argparse.ArgumentParser(description='Mag current/throttle compensation from a ulog')
    parser.add_argument('logfile', help='path to .ulg')
    parser.add_argument('type', nargs='?', choices=['current', 'thrust'],
                        help='power signal; default is current if present, else thrust')
    parser.add_argument('--instance', type=int, default=0, help='battery/thrust instance')
    parser.add_argument('--no-plot', action='store_true')
    args = parser.parse_args()

    log = ULog(args.logfile)

    comp_type = args.type
    if comp_type is None:
        if len(get_data(log, 'battery_status', 'current_a', args.instance)) > 0:
            comp_type = 'current'
        else:
            comp_type = 'thrust'

    power, t_power, comp_type_param, unit = load_power(log, comp_type, args.instance)
    if power is None:
        print(f'no {comp_type} data in log', file=sys.stderr)
        sys.exit(1)

    mags = load_mags(log)
    if not mags:
        print('no sensor_mag data in log', file=sys.stderr)
        sys.exit(1)

    t_att, q_att = load_attitude(log)

    print(f'\n{comp_type}-based compensation ({args.logfile}), COMP units {unit}')
    print(f'param set CAL_MAG_COMP_TYP {comp_type_param}')

    if not args.no_plot:
        import matplotlib.pyplot as plt

    for log_inst, device_id, t_mag, mag in mags:
        cal_inst = calibration_instance(log, device_id)
        if cal_inst is None:
            print(f'\nMag log instance {log_inst} device ID {device_id}: no CAL_MAGx_ID match, skip')
            continue

        finite = np.isfinite(mag).all(axis=1)
        mask = finite & armed_mask(log, t_mag)
        if mask.sum() < 50:
            mask = finite
            print(f'\nCAL_MAG{cal_inst} (id {device_id}): no armed interval, using the whole log')
        if mask.sum() < 50:
            print(f'\nCAL_MAG{cal_inst} (id {device_id}): not enough samples, skip')
            continue

        t = t_mag[mask]
        m = mag[mask]
        dt_med = np.median(np.diff(t)) if len(t) > 1 else 1.0
        rate = 1.0 / dt_med if dt_med > 0 else 0.0

        dt, dt_corr, dt_grid, dt_corr_curve = estimate_dt(t, np.linalg.norm(m, axis=1), t_power, power)
        p = np.interp(t - dt, t_power, power)

        R_ns = None
        used_attitude = False
        if t_att is not None:
            q = interp_quats(t_att, q_att, t)
            R_nb = dcm_from_quat(q)
            R_cal = sensor_to_body_dcm(log, cal_inst)
            R_ns = R_nb @ R_cal
            used_attitude = True

        k, b, B, r2 = fit_comp(m, p, R_ns)
        # runtime: data + power * COMP, so COMP cancels k in m = ... + k * power
        comp = -k

        print(f'\nCAL_MAG{cal_inst} device ID {device_id}')
        print(f'  mag rate {rate:.1f} Hz, dt {dt * 1e3:.1f} ms (|m| vs {comp_type} diff corr {dt_corr:.2f})')
        print(f'  power-term R^2 {r2:.2f}' + (' (Earth field removed via attitude)' if used_attitude else ' (no attitude, constant-offset fit)'))
        if rate < 20:
            print('  warning: mag < 20 Hz; enable SDLOG_PROFILE high-rate sensors (bit 11) for a usable dt')
        if np.std(p) < (0.001 if comp_type == 'current' else 0.05):
            print('  warning: little variation in the power signal')
        if used_attitude and B is not None and np.linalg.norm(B) < 0.1:
            print('  warning: fitted |B_earth| is low; check mag / attitude alignment')
        if r2 < 0.15:
            print('  warning: weak current/throttle correlation; compensation may not help')
        print(f'param set CAL_MAG{cal_inst}_XCOMP {comp[0]:.3f}')
        print(f'param set CAL_MAG{cal_inst}_YCOMP {comp[1]:.3f}')
        print(f'param set CAL_MAG{cal_inst}_ZCOMP {comp[2]:.3f}')

        if args.no_plot:
            continue

        earth_b = b if R_ns is None else np.einsum('nji,j->ni', R_ns, B) + b
        m_corr = m - np.outer(p, k)

        fig, axes = plt.subplots(1, 3, figsize=(14, 5))
        fig.suptitle(f'CAL_MAG{cal_inst} id {device_id}  dt={dt * 1e3:.1f} ms  R²={r2:.2f}')
        for i, name in enumerate('XYZ'):
            axes[i].plot(p, m[:, i] - earth_b[:, i], '.', color='C0', alpha=0.4, label='residual')
            xline = np.linspace(p.min(), p.max(), 50)
            axes[i].plot(xline, k[i] * xline, 'k--', label='fit')
            axes[i].set_xlabel('current [kA]' if comp_type == 'current' else 'thrust')
            axes[i].set_ylabel(f'mag {name} minus Earth/offset [G]')
            axes[i].legend()
        plt.tight_layout()

        fig2, axes2 = plt.subplots(4, 1, figsize=(14, 8), sharex=True)
        fig2.suptitle(f'CAL_MAG{cal_inst} original vs compensated')
        for i, name in enumerate('XYZ'):
            axes2[i].plot(t, m[:, i], label='original')
            axes2[i].plot(t, m_corr[:, i], label='compensated')
            axes2[i].set_ylabel(f'mag {name} [G]')
            axes2[i].legend(loc='upper right')
        axes2[3].plot(t, p)
        axes2[3].set_ylabel(comp_type)
        axes2[3].set_xlabel('time [s]')
        plt.tight_layout()

        fig3, ax3 = plt.subplots(figsize=(8, 4))
        ax3.plot(dt_grid * 1e3, dt_corr_curve)
        ax3.axvline(dt * 1e3, color='k', linestyle='--')
        ax3.set_xlabel('dt [ms] (positive = mag lags power)')
        ax3.set_ylabel('corr(d|m|, d power)')
        ax3.set_title(f'CAL_MAG{cal_inst} delay from field norm')
        plt.tight_layout()

    if not args.no_plot:
        plt.show()


if __name__ == '__main__':
    main()
