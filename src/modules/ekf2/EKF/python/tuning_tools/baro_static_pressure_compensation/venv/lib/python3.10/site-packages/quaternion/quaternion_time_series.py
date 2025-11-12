# -*- coding: utf-8 -*-

# Copyright (c) 2021, Michael Boyle
# See LICENSE file for details: <https://github.com/moble/quaternion/blob/main/LICENSE>

from __future__ import print_function, division, absolute_import

import numpy as np
import quaternion
from quaternion.numba_wrapper import njit


def unflip_rotors(q, axis=-1, inplace=False):
    """Flip signs of quaternions along axis to ensure continuity

    Quaternions form a "double cover" of the rotation group, meaning that if `q`
    represents a rotation, then `-q` represents the same rotation.  This is clear
    from the way a quaternion is used to rotate a vector `v`: the rotated vector is
    `q * v * q.conjugate()`, which is precisely the same as the vector resulting
    from `(-q) * v * (-q).conjugate()`.  Some ways of constructing quaternions
    (such as converting from rotation matrices or other representations) can result
    in unexpected sign choices.  For many applications, this will not be a problem.
    But if, for example, the quaternions need to be interpolated or differentiated,
    the results may be surprising.  This function flips the signs of successive
    quaternions (along some chosen axis, if relevant), so that successive
    quaternions are as close as possible while still representing the same
    rotations.

    Parameters
    ----------
    q : array_like
        Quaternion array to modify
    axis : int, optional
        Axis along which successive quaternions will be compared.  Default value is
        the last axis of the quaternion array.
    inplace : bool, optional
        If True, modify the data in place without creating a copy; if False (the
        default), a new array is created and returned.

    Returns
    -------
    q_out : array_like
        An array of precisely the same shape as the input array, differing only by
        factors of precisely -1 in some elements.

    """
    q = np.asarray(q, dtype=np.quaternion)
    ndim = q.ndim
    if abs(axis) > ndim:
        raise ValueError("Requested axis {0} is outside the input array's shape {1}".format(axis, q.shape))
    f = quaternion.as_float_array(q)
    flip = np.linalg.norm(np.diff(f, axis=(axis % ndim)), axis=-1) > 1.4142135623730950488016887242097
    factors = np.insert(-2 * np.mod(np.cumsum(flip, axis=axis, dtype=int), 2) + 1, 0, 1, axis=axis)
    if inplace:
        f *= factors[..., np.newaxis]
    else:
        f = factors[..., np.newaxis] * f
    return quaternion.as_quat_array(f)


def slerp(R1, R2, t1, t2, t_out):
    """Spherical linear interpolation of rotors

    This function uses a simpler interface than the more fundamental
    `slerp_evaluate` and `slerp_vectorized` functions.  The latter
    are fast, being implemented at the C level, but take input `tau`
    instead of time.  This function adjusts the time accordingly.

    Parameters
    ----------
    R1 : quaternion
        Quaternion at beginning of interpolation
    R2 : quaternion
        Quaternion at end of interpolation
    t1 : float
        Time corresponding to R1
    t2 : float
        Time corresponding to R2
    t_out : float or array of floats
        Times to which the rotors should be interpolated

    """
    tau = (t_out-t1)/(t2-t1)
    return np.slerp_vectorized(R1, R2, tau)


def squad(R_in, t_in, t_out, unflip_input_rotors=False):
    """Spherical "quadrangular" interpolation of rotors with a cubic spline

    This is typically the best way to interpolate rotation timeseries.
    It uses the analog of a cubic spline, except that the interpolant
    is confined to the rotor manifold in a natural way.  Alternative
    methods involving interpolation of other coordinates on the
    rotation group or normalization of interpolated values give bad
    results.  The results from this method are continuous in value and
    first derivative everywhere, including around the sampling
    locations.

    The input `R_in` rotors are assumed to be reasonably continuous (no
    sign flips), and the input `t` arrays are assumed to be sorted.  No
    checking is done for either case, and you may get silently bad
    results if these conditions are violated.  The first dimension of
    `R_in` must have the same size as `t_in`, but may have additional
    axes following.

    This function simplifies the calling, compared to `squad_evaluate`
    (which takes a set of four quaternions forming the edges of the
    "quadrangle", and the normalized time `tau`) and `squad_vectorized`
    (which takes the same arguments, but in array form, and efficiently
    loops over them).

    Parameters
    ----------
    R_in : array of quaternions
        A time-series of rotors (unit quaternions) to be interpolated
    t_in : array of float
        The times corresponding to R_in
    t_out : array of float
        The times to which R_in should be interpolated
    unflip_input_rotors : bool, optional
        If True, this function calls `unflip_rotors` on the input, to
        ensure that the rotors are more continuous than not.  Defaults
        to False.

    """
    from functools import partial

    roll = partial(np.roll, axis=0)

    if R_in.size == 0 or t_out.size == 0:
        return np.array((), dtype=np.quaternion)

    if unflip_input_rotors:
        R_in = unflip_rotors(R_in, axis=0)

    # This list contains an index for each `t_out` such that
    # t_in[i-1] <= t_out < t_in[i]
    # Note that `side='right'` is much faster in my tests
    # i_in_for_out = t_in.searchsorted(t_out, side='left')
    # np.clip(i_in_for_out, 0, len(t_in) - 1, out=i_in_for_out)
    i_in_for_out = t_in.searchsorted(t_out, side='right')-1

    # Compute shapes used to broadcast `t` arrays against `R` arrays
    t_in_broadcast_shape = t_in.shape + (1,)*len(R_in.shape[1:])
    t_out_broadcast_shape = t_out.shape + (1,)*len(R_in.shape[1:])

    # Now, for each index `i` in `i_in`, we need to compute the
    # interpolation "coefficients" (`A_i`, `B_ip1`).
    #
    # I previously tested an explicit version of the loops below,
    # comparing `stride_tricks.as_strided` with explicit
    # implementation via `roll` (as seen here).  I found that the
    # `roll` was significantly more efficient for simple calculations,
    # though the difference is probably totally washed out here.  In
    # any case, it might be useful to test again.
    #
    A = R_in * np.exp((- np.log((~R_in) * roll(R_in, -1))
                       + np.log((~roll(R_in, 1)) * R_in)
                       * np.reshape((roll(t_in, -1) - t_in) / (t_in - roll(t_in, 1)), t_in_broadcast_shape)
                       ) * 0.25)
    B = roll(R_in, -1) * np.exp((np.log((~roll(R_in, -1)) * roll(R_in, -2))
                                    * np.reshape((roll(t_in, -1) - t_in) / (roll(t_in, -2) - roll(t_in, -1)), t_in_broadcast_shape)
                                    - np.log((~R_in) * roll(R_in, -1))) * -0.25)

    # Correct the first and last A time steps, and last two B time steps.  We extend R_in with the following wrap-around
    # values:
    # R_in[0-1] = R_in[0]*(~R_in[1])*R_in[0]
    # R_in[n+0] = R_in[-1] * (~R_in[-2]) * R_in[-1]
    # R_in[n+1] = R_in[0] * (~R_in[-1]) * R_in[0]
    #           = R_in[-1] * (~R_in[-2]) * R_in[-1] * (~R_in[-1]) * R_in[-1] * (~R_in[-2]) * R_in[-1]
    #           = R_in[-1] * (~R_in[-2]) * R_in[-1] * (~R_in[-2]) * R_in[-1]
    # A[i] = R_in[i] * np.exp((- np.log((~R_in[i]) * R_in[i+1])
    #                          + np.log((~R_in[i-1]) * R_in[i]) * ((t_in[i+1] - t_in[i]) / (t_in[i] - t_in[i-1]))
    #                          ) * 0.25)
    # A[0] = R_in[0] * np.exp((- np.log((~R_in[0]) * R_in[1]) + np.log((~R_in[0])*R_in[1]*(~R_in[0])) * R_in[0]) * 0.25)
    #      = R_in[0]
    A[0] = R_in[0]
    # A[-1] = R_in[-1] * np.exp((- np.log((~R_in[-1]) * R_in[n+0])
    #                          + np.log((~R_in[-2]) * R_in[-1]) * ((t_in[n+0] - t_in[-1]) / (t_in[-1] - t_in[-2]))
    #                          ) * 0.25)
    #       = R_in[-1] * np.exp((- np.log((~R_in[-1]) * R_in[n+0]) + np.log((~R_in[-2]) * R_in[-1])) * 0.25)
    #       = R_in[-1] * np.exp((- np.log((~R_in[-1]) * R_in[-1] * (~R_in[-2]) * R_in[-1])
    #                           + np.log((~R_in[-2]) * R_in[-1])) * 0.25)
    #       = R_in[-1] * np.exp((- np.log((~R_in[-2]) * R_in[-1]) + np.log((~R_in[-2]) * R_in[-1])) * 0.25)
    #       = R_in[-1]
    A[-1] = R_in[-1]
    # B[i] = R_in[i+1] * np.exp((np.log((~R_in[i+1]) * R_in[i+2]) * ((t_in[i+1] - t_in[i]) / (t_in[i+2] - t_in[i+1]))
    #                            - np.log((~R_in[i]) * R_in[i+1])) * -0.25)
    # B[-2] = R_in[-1] * np.exp((np.log((~R_in[-1]) * R_in[0]) * ((t_in[-1] - t_in[-2]) / (t_in[0] - t_in[-1]))
    #                            - np.log((~R_in[-2]) * R_in[-1])) * -0.25)
    #       = R_in[-1] * np.exp((np.log((~R_in[-1]) * R_in[0]) - np.log((~R_in[-2]) * R_in[-1])) * -0.25)
    #       = R_in[-1] * np.exp((np.log((~R_in[-1]) * R_in[-1] * (~R_in[-2]) * R_in[-1])
    #                            - np.log((~R_in[-2]) * R_in[-1])) * -0.25)
    #       = R_in[-1] * np.exp((np.log((~R_in[-2]) * R_in[-1]) - np.log((~R_in[-2]) * R_in[-1])) * -0.25)
    #       = R_in[-1]
    B[-2] = R_in[-1]
    # B[-1] = R_in[0]
    # B[-1] = R_in[0] * np.exp((np.log((~R_in[0]) * R_in[1]) - np.log((~R_in[-1]) * R_in[0])) * -0.25)
    #       = R_in[-1] * (~R_in[-2]) * R_in[-1]
    #         * np.exp((np.log((~(R_in[-1] * (~R_in[-2]) * R_in[-1])) * R_in[-1] * (~R_in[-2]) * R_in[-1] * (~R_in[-2]) * R_in[-1])
    #                  - np.log((~R_in[-1]) * R_in[-1] * (~R_in[-2]) * R_in[-1])) * -0.25)
    #       = R_in[-1] * (~R_in[-2]) * R_in[-1]
    #         * np.exp((np.log(((~R_in[-1]) * R_in[-2] * (~R_in[-1])) * R_in[-1] * (~R_in[-2]) * R_in[-1] * (~R_in[-2]) * R_in[-1])
    #                  - np.log((~R_in[-1]) * R_in[-1] * (~R_in[-2]) * R_in[-1])) * -0.25)
    #         * np.exp((np.log((~R_in[-2]) * R_in[-1])
    #                  - np.log((~R_in[-2]) * R_in[-1])) * -0.25)
    B[-1] = R_in[-1] * (~R_in[-2]) * R_in[-1]

    # Use the coefficients at the corresponding t_out indices to
    # compute the squad interpolant
    # R_ip1 = np.array(roll(R_in, -1)[i_in_for_out])
    # R_ip1[-1] = R_in[-1]*(~R_in[-2])*R_in[-1]
    R_ip1 = roll(R_in, -1)
    R_ip1[-1] = R_in[-1]*(~R_in[-2])*R_in[-1]
    R_ip1 = np.array(R_ip1[i_in_for_out])
    t_inp1 = roll(t_in, -1)
    t_inp1[-1] = t_in[-1] + (t_in[-1] - t_in[-2])
    tau = np.reshape((t_out - t_in[i_in_for_out]) / ((t_inp1 - t_in)[i_in_for_out]), t_out_broadcast_shape)
    # tau = (t_out - t_in[i_in_for_out]) / ((roll(t_in, -1) - t_in)[i_in_for_out])
    R_out = np.squad_vectorized(tau, R_in[i_in_for_out], A[i_in_for_out], B[i_in_for_out], R_ip1)

    return R_out


@njit
def frame_from_angular_velocity_integrand(rfrak, Omega):
    import math
    from numpy import dot, cross
    from .numpy_quaternion import _eps
    rfrakMag = math.sqrt(rfrak[0] * rfrak[0] + rfrak[1] * rfrak[1] + rfrak[2] * rfrak[2])
    OmegaMag = math.sqrt(Omega[0] * Omega[0] + Omega[1] * Omega[1] + Omega[2] * Omega[2])
    # If the matrix is really close to the identity, return
    if rfrakMag < _eps * OmegaMag:
        return Omega[0] / 2.0, Omega[1] / 2.0, Omega[2] / 2.0
    # If the matrix is really close to singular, it's equivalent to the identity, so return
    if abs(math.sin(rfrakMag)) < _eps:
        return Omega[0] / 2.0, Omega[1] / 2.0, Omega[2] / 2.0

    OmegaOver2 = Omega[0] / 2.0, Omega[1] / 2.0, Omega[2] / 2.0
    rfrakHat = rfrak[0] / rfrakMag, rfrak[1] / rfrakMag, rfrak[2] / rfrakMag

    return ((OmegaOver2 - rfrakHat * dot(rfrakHat, OmegaOver2)) * (rfrakMag / math.tan(rfrakMag))
            + rfrakHat * dot(rfrakHat, OmegaOver2) + cross(OmegaOver2, rfrak))


class appending_array(object):
    def __init__(self, shape, dtype=np.float64, initial_array=None):
        shape = list(shape)
        if shape[0] < 4:
            shape[0] = 4
        self._a = np.empty(shape, dtype=dtype)
        self.n = 0
        if initial_array is not None:
            assert initial_array.dtype == dtype
            assert initial_array.shape[1:] == shape[1:]
            assert initial_array.shape[0] <= shape[0]
            self.n = initial_array.shape[0]
            self._a[:self.n, ...] = initial_array[:]

    def append(self, row):
        self.n += 1
        if self.n > self._a.shape[0]:
            self._a = np.resize(self._a, (2*self._a.shape[0],)+self._a.shape[1:])
        self._a[self.n-1, ...] = row

    @property
    def a(self):
        return self._a[:self.n, ...]


def integrate_angular_velocity(Omega, t0, t1, R0=None, tolerance=1e-12):
    """Compute frame with given angular velocity

    Parameters
    ----------
    Omega : tuple or callable
        Angular velocity from which to compute frame.  Can be
          1) a 2-tuple of float arrays (t, v) giving the angular velocity vector at a series of times,
          2) a function of time that returns the 3-vector angular velocity, or
          3) a function of time and orientation (t, R) that returns the 3-vector angular velocity
        In case 1, the angular velocity will be interpolated to the required times.  Note that accuracy
        is poor in case 1.
    t0 : float
        Initial time
    t1 : float
        Final time
    R0 : quaternion, optional
        Initial frame orientation.  Defaults to 1 (the identity orientation).
    tolerance : float, optional
        Absolute tolerance used in integration.  Defaults to 1e-12.

    Returns
    -------
    t : float array
    R : quaternion array

    """
    import warnings
    from scipy.integrate import ode
    from scipy.interpolate import CubicSpline

    if R0 is None:
        R0 = quaternion.one

    input_is_tabulated = False

    try:
        t_Omega, v = Omega
        Omega = CubicSpline(t_Omega, v)
        def Omega_func(t, R):
            return Omega(t)
        Omega_func(t0, R0)
        input_is_tabulated = True
    except (TypeError, ValueError):
        def Omega_func(t, R):
            return Omega(t, R)
        try:
            Omega_func(t0, R0)
        except TypeError:
            def Omega_func(t, R):
                return Omega(t)
            Omega_func(t0, R0)

    def RHS(t, y):
        R = quaternion.quaternion(*y)
        return (0.5 * quaternion.quaternion(0.0, *Omega_func(t, R)) * R).components

    y0 = R0.components

    if input_is_tabulated:
        from scipy.integrate import solve_ivp
        t = t_Omega
        t_span = [t_Omega[0], t_Omega[-1]]
        solution = solve_ivp(RHS, t_span, y0, t_eval=t_Omega, atol=tolerance, rtol=100*np.finfo(float).eps)
        R = quaternion.from_float_array(solution.y.T)
    else:
        solver = ode(RHS)
        solver.set_initial_value(y0, t0)
        solver.set_integrator('dop853', nsteps=1, atol=tolerance, rtol=0.0)
        solver._integrator.iwork[2] = -1  # suppress Fortran-printed warning
        t = appending_array((int(t1-t0),))
        t.append(solver.t)
        R = appending_array((int(t1-t0), 4))
        R.append(solver.y)
        warnings.filterwarnings("ignore", category=UserWarning)
        t_last = solver.t
        while solver.t < t1:
            solver.integrate(t1, step=True)
            if solver.t > t_last:
                t.append(solver.t)
                R.append(solver.y)
                t_last = solver.t
        warnings.resetwarnings()
        t = t.a
        R = quaternion.as_quat_array(R.a)

    return t, R


def minimal_rotation(R, t, iterations=2):
    """Adjust frame so that there is no rotation about z' axis

    The output of this function is a frame that rotates the z axis onto the same z' axis as the
    input frame, but with minimal rotation about that axis.  This is done by pre-composing the input
    rotation with a rotation about the z axis through an angle gamma, where

        dgamma/dt = 2*(dR/dt * z * R.conjugate()).w

    This ensures that the angular velocity has no component along the z' axis.

    Note that this condition becomes easier to impose the closer the input rotation is to a
    minimally rotating frame, which means that repeated application of this function improves its
    accuracy.  By default, this function is iterated twice, though a few more iterations may be
    called for.

    Parameters
    ----------
    R : quaternion array
        Time series describing rotation
    t : float array
        Corresponding times at which R is measured
    iterations : int [defaults to 2]
        Repeat the minimization to refine the result

    """
    from scipy.interpolate import CubicSpline
    if iterations == 0:
        return R
    R = quaternion.as_float_array(R)
    Rdot = CubicSpline(t, R).derivative()(t)
    R = quaternion.from_float_array(R)
    Rdot = quaternion.from_float_array(Rdot)
    halfgammadot = quaternion.as_float_array(Rdot * quaternion.z * np.conjugate(R))[:, 0]
    halfgamma = CubicSpline(t, halfgammadot).antiderivative()(t)
    Rgamma = np.exp(quaternion.z * halfgamma)
    return minimal_rotation(R * Rgamma, t, iterations=iterations-1)


def angular_velocity(R, t):
    """Approximate angular velocity of a rotating frame

    Parameters
    ----------
    R : array_like
        Quaternion-valued function of time evaluated at a set of times.  This
        represents the quaternion that rotates the standard (x,y,z) frame into the
        moving frame at each instant.
    t : array_like
        Times at which `R` is evaluated.

    Returns
    -------
    Omega : array_like
        The angular velocity (three-vector) as a function of time `t`.  A vector
        that is fixed in the moving frame rotates with this angular velocity with
        respect to the inertial frame.

    Notes
    -----
    The angular velocity at each instant is given by 2 * (dR/dt) / R.  This
    function approximates the input `R` using a cubic spline, and differentiates it
    as such.

    """
    from scipy.interpolate import CubicSpline
    R = quaternion.as_float_array(R)
    Rdot = CubicSpline(t, R).derivative()(t)
    R = quaternion.from_float_array(R)
    Rdot = quaternion.from_float_array(Rdot)
    return quaternion.as_float_array(2*Rdot/R)[:, 1:]
    
