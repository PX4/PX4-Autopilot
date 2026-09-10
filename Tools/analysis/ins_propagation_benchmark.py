#!/usr/bin/env python3
"""
Benchmark of EKF2 strapdown propagation schemes against a finely integrated truth.

Answers whether the closed-form solution of Goppert et al. (arXiv:2310.04886, PX4
PR #22291) beats the trapezoidal integration it replaces, when both are fed what
ImuDownSampler actually produces.

Neither scheme is intrinsically more accurate. Each is exact for one input convention
and wrong by (theta/2) * |delta_vel| for the other, since J1 - expm(Theta) = -Theta/2
- Theta^2/3. The closed form assumes delta_vel is the raw body-frame integral over the
interval, resolved in the body frame at its start. ImuDownSampler instead rotates each
sub-sample delta velocity forward into the body frame at the *end* of the accumulation
interval (sculling compensation) and accumulates delta angle as a quaternion product,
so the consumer matching its convention is R_end * delta_vel - which is what the pre-PR
code did by updating the quaternion first.

run_control() demonstrates that symmetry on the motion the closed form solves exactly.
run_varying() then shows what happens on motion that varies inside the interval: the
downsampler composes the rotation per sub-sample at the IMU rate, which tracks the
variation that the closed form's constant-rate, constant-specific-force model over a
whole EKF2_PREDICT_US interval cannot.

Truth is an explicit fine integration of the same motion; the IMU sub-samples are
formed the way a strapdown sensor forms them (raw body-frame accel integral, no
rotation compensation), then passed through a transcription of ImuDownSampler.

Errors are reported over a sweep of motion start phases, since a single phase is not
representative: the schemes' errors depend on where in the motion the interval falls.
"""

import numpy as np

G = np.array([0.0, 0.0, 9.80665])

# EKF2_PREDICT_US default
DT = 0.010


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def expm_so3(w):
    theta = np.linalg.norm(w)

    if theta < 1e-14:
        return np.eye(3)

    k = skew(w / theta)
    return np.eye(3) + np.sin(theta) * k + (1 - np.cos(theta)) * (k @ k)


def logm_so3(r):
    cos_theta = min(1.0, max(-1.0, (np.trace(r) - 1) / 2))
    theta = np.arccos(cos_theta)

    if theta < 1e-12:
        return np.zeros(3)

    return theta / (2 * np.sin(theta)) * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])


def coefficients(theta_sq):
    """the c1, c2, c3 of the closed form, in the proof-consistent forms"""
    theta = np.sqrt(theta_sq)

    if theta < 1e-6:
        return 0.5 - theta_sq / 24, 1 / 6 - theta_sq / 120, 1 / 24 - theta_sq / 720

    return ((1 - np.cos(theta)) / theta_sq,
            (theta - np.sin(theta)) / (theta_sq * theta),
            (0.5 * theta_sq + np.cos(theta) - 1) / (theta_sq * theta_sq))


# Motion families. Each returns (omega_body [rad/s], accel_body [m/s^2]) at time t.
# "smooth" advances only ~0.1 rad of phase per 10 ms interval, so omega is nearly
# constant within a step: the closed form's best case.
# "agile" adds content an order of magnitude faster, so omega and accel vary
# appreciably inside a single interval.
def motion_smooth(t, amp):
    omega = amp * np.array([np.sin(7 * t) + 0.6, 0.5 * np.cos(11 * t), -0.3 + 0.4 * np.sin(5 * t)])
    accel = np.array([0.8 * np.sin(9 * t), -0.5 * np.cos(6 * t), -9.7 + 0.9 * np.sin(13 * t)])
    return omega, accel


def motion_agile(t, amp):
    omega = amp * np.array([np.sin(90 * t) + 0.6, 0.5 * np.cos(140 * t), -0.3 + 0.4 * np.sin(60 * t)])
    accel = np.array([2.5 * np.sin(120 * t), -1.8 * np.cos(80 * t), -9.7 + 3.0 * np.sin(170 * t)])
    return omega, accel


MOTIONS = {"smooth": motion_smooth, "agile": motion_agile}


def integrate_truth(motion, t0, dt, r0, v0, amp, n=20000):
    h = dt / n
    r, v, p = r0.copy(), v0.copy(), np.zeros(3)

    for i in range(n):
        omega, accel = motion(t0 + (i + 0.5) * h, amp)
        accel_earth = r @ accel + G
        p += v * h + 0.5 * accel_earth * h * h
        v += accel_earth * h
        r = r @ expm_so3(omega * h)

    return v, p


def imu_subsamples(motion, t0, dt, r0, amp, n_sub, n_fine=400):
    """delta angle and delta velocity per sub-sample, as a strapdown IMU forms them"""
    h_sub = dt / n_sub
    samples = []
    r = r0.copy()

    for k in range(n_sub):
        h = h_sub / n_fine
        r_start = r.copy()
        delta_vel = np.zeros(3)

        for i in range(n_fine):
            omega, accel = motion(t0 + k * h_sub + (i + 0.5) * h, amp)
            # raw body-frame integral: the sensor does not rotation-compensate
            delta_vel += accel * h
            r = r @ expm_so3(omega * h)

        samples.append((logm_so3(r_start.T @ r), delta_vel))

    return samples


def downsample(samples):
    """transcription of ImuDownSampler::update"""
    rotation_accumulated = np.eye(3)
    delta_vel = np.zeros(3)

    for delta_ang, delta_vel_sub in samples:
        # maps a vector from the previous body frame into the new one
        delta_r = expm_so3(delta_ang).T
        delta_vel = delta_r @ delta_vel
        delta_vel = delta_vel + 0.5 * (delta_vel_sub + delta_r @ delta_vel_sub)
        rotation_accumulated = rotation_accumulated @ expm_so3(delta_ang)

    return logm_so3(rotation_accumulated), delta_vel


def scheme_trapezoidal(r0, v0, delta_ang, delta_vel, dt):
    r_end = r0 @ expm_so3(delta_ang)
    vel_new = v0 + r_end @ delta_vel + G * dt
    return vel_new, (v0 + vel_new) * 0.5 * dt


def scheme_closed_form(r0, v0, delta_ang, delta_vel, dt):
    c1, c2, c3 = coefficients(delta_ang @ delta_ang)
    t = skew(delta_ang)
    t_sq = t @ t
    j1 = np.eye(3) + c1 * t + c2 * t_sq
    j2 = 0.5 * np.eye(3) + c2 * t + c3 * t_sq
    return (v0 + r0 @ (j1 @ delta_vel) + G * dt,
            v0 * dt + r0 @ (j2 @ delta_vel) * dt + G * (0.5 * dt * dt))


def scheme_closed_form_raw(r0, v0, samples, dt):
    """the input convention the closed form was derived for: no sculling compensation"""
    rotation_accumulated = np.eye(3)

    for delta_ang, _ in samples:
        rotation_accumulated = rotation_accumulated @ expm_so3(delta_ang)

    delta_vel_raw = sum(delta_vel for _, delta_vel in samples)
    return scheme_closed_form(r0, v0, logm_so3(rotation_accumulated), delta_vel_raw, dt)


R0 = expm_so3(np.array([0.3, -0.5, 1.9]))
V0 = np.array([9.0, -3.0, 0.7])


def integrate_truth_constant(omega, accel, dt, n=200000):
    """fine integration for a genuinely constant body rate and body specific force"""
    h = dt / n
    r, v, p = R0.copy(), V0.copy(), np.zeros(3)

    for _ in range(n):
        accel_earth = r @ accel + G
        p += v * h + 0.5 * accel_earth * h * h
        v += accel_earth * h
        r = r @ expm_so3(omega * h)

    return v, p


def run_control():
    """Constant omega and accel: the model the closed form solves exactly.

    Each scheme is exact given the input convention it was derived for, and wrong by the
    same half-interval rotation given the other's. The residual on the matching rows is
    the fine integrator's own error, not scheme error.
    """
    print("CONTROL: constant body rate and body specific force\n")
    print("{:>6s} {:>7s} | {:<46s} {:>11s} {:>11s}".format("|w|", "theta", "scheme", "dv err", "dp err"))

    for rate in [1.0, 6.0, 20.0]:
        omega = np.array([1.0, 0.5, -0.3])
        omega = omega / np.linalg.norm(omega) * rate
        accel = np.array([0.8, -0.5, -9.7])

        vel_truth, pos_truth = integrate_truth_constant(omega, accel, DT)
        samples = [(omega * DT / 10, accel * DT / 10) for _ in range(10)]
        delta_ang_ds, delta_vel_ds = downsample(samples)
        delta_ang_raw, delta_vel_raw = omega * DT, accel * DT

        rows = [
            ("closed form <- raw a_b*dt  (matching input)",
             scheme_closed_form(R0, V0, delta_ang_raw, delta_vel_raw, DT)),
            ("trapezoidal <- raw a_b*dt  (mismatched)",
             scheme_trapezoidal(R0, V0, delta_ang_raw, delta_vel_raw, DT)),
            ("trapezoidal <- downsampled (matching input)",
             scheme_trapezoidal(R0, V0, delta_ang_ds, delta_vel_ds, DT)),
            ("closed form <- downsampled (mismatched)",
             scheme_closed_form(R0, V0, delta_ang_ds, delta_vel_ds, DT)),
        ]

        print("{:6.1f} {:7.4f} |".format(rate, np.linalg.norm(delta_ang_raw)))

        for name, (vel, pos) in rows:
            print("{:6s} {:7s} | {:<46s} {:11.3e} {:11.3e}".format(
                "", "", name, np.max(np.abs(vel - vel_truth)), np.max(np.abs(pos - pos_truth))))


def run_varying():
    r0, v0 = R0, V0
    start_phases = np.linspace(0.0, 1.0, 24)

    print("max error over {} motion start phases, dt = {:.0f} ms\n".format(len(start_phases), DT * 1e3))
    print("{:8s} {:>9s} {:>7s} {:>8s} | {:>11s} {:>11s} {:>11s} | {:>11s} {:>11s} {:>11s}".format(
        "motion", "peak |w|", "imu Hz", "theta",
        "dv trap", "dv CF", "dv CF raw", "dp trap", "dp CF", "dp CF raw"))

    for name, motion in MOTIONS.items():
        for amp in [0.15, 1.5, 4.0]:
            for n_sub in [10, 4]:
                worst = np.zeros(6)
                theta_max = 0.0
                peak_rate = 0.0

                for t0 in start_phases:
                    vel_truth, pos_truth = integrate_truth(motion, t0, DT, r0, v0, amp)
                    samples = imu_subsamples(motion, t0, DT, r0, amp, n_sub)
                    delta_ang, delta_vel = downsample(samples)

                    results = [scheme_trapezoidal(r0, v0, delta_ang, delta_vel, DT),
                               scheme_closed_form(r0, v0, delta_ang, delta_vel, DT),
                               scheme_closed_form_raw(r0, v0, samples, DT)]

                    errors = np.array([np.max(np.abs(results[i][0] - vel_truth)) for i in range(3)]
                                      + [np.max(np.abs(results[i][1] - pos_truth)) for i in range(3)])
                    worst = np.maximum(worst, errors)
                    theta_max = max(theta_max, np.linalg.norm(delta_ang))
                    peak_rate = max(peak_rate, np.linalg.norm(motion(t0, amp)[0]))

                print("{:8s} {:9.2f} {:7.0f} {:8.4f} | {:11.3e} {:11.3e} {:11.3e} | "
                      "{:11.3e} {:11.3e} {:11.3e}".format(
                          name, peak_rate, 1 / (DT / n_sub), theta_max, *worst))


if __name__ == "__main__":
    run_control()
    print()
    run_varying()
