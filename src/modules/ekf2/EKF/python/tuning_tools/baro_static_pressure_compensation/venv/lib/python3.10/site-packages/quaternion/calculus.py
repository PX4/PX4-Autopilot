# -*- coding: utf-8 -*-

# Copyright (c) 2021, Michael Boyle
# See LICENSE file for details: <https://github.com/moble/quaternion/blob/main/LICENSE>

from __future__ import division, print_function, absolute_import
import numpy as np
from quaternion.numba_wrapper import njit, jit, xrange


def fd_derivative(f, t):
    """Fourth-order finite-differencing with non-uniform time steps

    The formula for this finite difference comes from Eq. (A 5b) of "Derivative formulas and errors for non-uniformly
    spaced points" by M. K. Bowen and Ronald Smith.  As explained in their Eqs. (B 9b) and (B 10b), this is a
    fourth-order formula -- though that's a squishy concept with non-uniform time steps.

    TODO: If there are fewer than five points, the function should revert to simpler (lower-order) formulas.

    """
    dfdt = np.empty_like(f)
    if (f.ndim == 1):
        _derivative(f, t, dfdt)
    elif (f.ndim == 2):
        _derivative_2d(f, t, dfdt)
    elif (f.ndim == 3):
        _derivative_3d(f, t, dfdt)
    else:
        raise NotImplementedError("Taking derivatives of {0}-dimensional arrays is not yet implemented".format(f.ndim))
    return dfdt


@njit
def _derivative(f, t, dfdt):
    for i in xrange(2):
        t_i = t[i]
        t1 = t[0]
        t2 = t[1]
        t3 = t[2]
        t4 = t[3]
        t5 = t[4]
        h1 = t1 - t_i
        h2 = t2 - t_i
        h3 = t3 - t_i
        h4 = t4 - t_i
        h5 = t5 - t_i
        h12 = t1 - t2
        h13 = t1 - t3
        h14 = t1 - t4
        h15 = t1 - t5
        h23 = t2 - t3
        h24 = t2 - t4
        h25 = t2 - t5
        h34 = t3 - t4
        h35 = t3 - t5
        h45 = t4 - t5
        dfdt[i] = (-((h2 * h3 * h4 + h2 * h3 * h5 + h2 * h4 * h5 + h3 * h4 * h5) / (h12 * h13 * h14 * h15)) * f[0]
                   + ((h1 * h3 * h4 + h1 * h3 * h5 + h1 * h4 * h5 + h3 * h4 * h5) / (h12 * h23 * h24 * h25)) * f[1]
                   - ((h1 * h2 * h4 + h1 * h2 * h5 + h1 * h4 * h5 + h2 * h4 * h5) / (h13 * h23 * h34 * h35)) * f[2]
                   + ((h1 * h2 * h3 + h1 * h2 * h5 + h1 * h3 * h5 + h2 * h3 * h5) / (h14 * h24 * h34 * h45)) * f[3]
                   - ((h1 * h2 * h3 + h1 * h2 * h4 + h1 * h3 * h4 + h2 * h3 * h4) / (h15 * h25 * h35 * h45)) * f[4])

    for i in xrange(2, len(t) - 2):
        t1 = t[i - 2]
        t2 = t[i - 1]
        t3 = t[i]
        t4 = t[i + 1]
        t5 = t[i + 2]
        h1 = t1 - t3
        h2 = t2 - t3
        h4 = t4 - t3
        h5 = t5 - t3
        h12 = t1 - t2
        h13 = t1 - t3
        h14 = t1 - t4
        h15 = t1 - t5
        h23 = t2 - t3
        h24 = t2 - t4
        h25 = t2 - t5
        h34 = t3 - t4
        h35 = t3 - t5
        h45 = t4 - t5
        dfdt[i] = (-((h2 * h4 * h5) / (h12 * h13 * h14 * h15)) * f[i - 2]
                   + ((h1 * h4 * h5) / (h12 * h23 * h24 * h25)) * f[i - 1]
                   - ((h1 * h2 * h4 + h1 * h2 * h5 + h1 * h4 * h5 + h2 * h4 * h5) / (h13 * h23 * h34 * h35)) * f[i]
                   + ((h1 * h2 * h5) / (h14 * h24 * h34 * h45)) * f[i + 1]
                   - ((h1 * h2 * h4) / (h15 * h25 * h35 * h45)) * f[i + 2])

    for i in xrange(len(t) - 2, len(t)):
        t_i = t[i]
        t1 = t[-5]
        t2 = t[-4]
        t3 = t[-3]
        t4 = t[-2]
        t5 = t[-1]
        h1 = t1 - t_i
        h2 = t2 - t_i
        h3 = t3 - t_i
        h4 = t4 - t_i
        h5 = t5 - t_i
        h12 = t1 - t2
        h13 = t1 - t3
        h14 = t1 - t4
        h15 = t1 - t5
        h23 = t2 - t3
        h24 = t2 - t4
        h25 = t2 - t5
        h34 = t3 - t4
        h35 = t3 - t5
        h45 = t4 - t5
        dfdt[i] = (-((h2 * h3 * h4 + h2 * h3 * h5 + h2 * h4 * h5 + h3 * h4 * h5) / (h12 * h13 * h14 * h15)) * f[-5]
                   + ((h1 * h3 * h4 + h1 * h3 * h5 + h1 * h4 * h5 + h3 * h4 * h5) / (h12 * h23 * h24 * h25)) * f[-4]
                   - ((h1 * h2 * h4 + h1 * h2 * h5 + h1 * h4 * h5 + h2 * h4 * h5) / (h13 * h23 * h34 * h35)) * f[-3]
                   + ((h1 * h2 * h3 + h1 * h2 * h5 + h1 * h3 * h5 + h2 * h3 * h5) / (h14 * h24 * h34 * h45)) * f[-2]
                   - ((h1 * h2 * h3 + h1 * h2 * h4 + h1 * h3 * h4 + h2 * h3 * h4) / (h15 * h25 * h35 * h45)) * f[-1])

    return


@njit
def _derivative_2d(f, t, dfdt):
    for i in xrange(2):
        t_i = t[i]
        t1 = t[0]
        t2 = t[1]
        t3 = t[2]
        t4 = t[3]
        t5 = t[4]
        h1 = t1 - t_i
        h2 = t2 - t_i
        h3 = t3 - t_i
        h4 = t4 - t_i
        h5 = t5 - t_i
        h12 = t1 - t2
        h13 = t1 - t3
        h14 = t1 - t4
        h15 = t1 - t5
        h23 = t2 - t3
        h24 = t2 - t4
        h25 = t2 - t5
        h34 = t3 - t4
        h35 = t3 - t5
        h45 = t4 - t5
        for k in xrange(f.shape[1]):
            dfdt[i, k] = (
            -((h2 * h3 * h4 + h2 * h3 * h5 + h2 * h4 * h5 + h3 * h4 * h5) / (h12 * h13 * h14 * h15)) * f[0, k]
            + ((h1 * h3 * h4 + h1 * h3 * h5 + h1 * h4 * h5 + h3 * h4 * h5) / (h12 * h23 * h24 * h25)) * f[1, k]
            - ((h1 * h2 * h4 + h1 * h2 * h5 + h1 * h4 * h5 + h2 * h4 * h5) / (h13 * h23 * h34 * h35)) * f[2, k]
            + ((h1 * h2 * h3 + h1 * h2 * h5 + h1 * h3 * h5 + h2 * h3 * h5) / (h14 * h24 * h34 * h45)) * f[3, k]
            - ((h1 * h2 * h3 + h1 * h2 * h4 + h1 * h3 * h4 + h2 * h3 * h4) / (h15 * h25 * h35 * h45)) * f[4, k])

    for i in xrange(2, len(t) - 2):
        t1 = t[i - 2]
        t2 = t[i - 1]
        t3 = t[i]
        t4 = t[i + 1]
        t5 = t[i + 2]
        h1 = t1 - t3
        h2 = t2 - t3
        h4 = t4 - t3
        h5 = t5 - t3
        h12 = t1 - t2
        h13 = t1 - t3
        h14 = t1 - t4
        h15 = t1 - t5
        h23 = t2 - t3
        h24 = t2 - t4
        h25 = t2 - t5
        h34 = t3 - t4
        h35 = t3 - t5
        h45 = t4 - t5
        for k in xrange(f.shape[1]):
            dfdt[i, k] = (-((h2 * h4 * h5) / (h12 * h13 * h14 * h15)) * f[i - 2, k]
                          + ((h1 * h4 * h5) / (h12 * h23 * h24 * h25)) * f[i - 1, k]
                          - ((h1 * h2 * h4 + h1 * h2 * h5 + h1 * h4 * h5 + h2 * h4 * h5) / (h13 * h23 * h34 * h35))
                            * f[i, k]
                          + ((h1 * h2 * h5) / (h14 * h24 * h34 * h45)) * f[i + 1, k]
                          - ((h1 * h2 * h4) / (h15 * h25 * h35 * h45)) * f[i + 2, k])

    for i in xrange(len(t) - 2, len(t)):
        t_i = t[i]
        t1 = t[-5]
        t2 = t[-4]
        t3 = t[-3]
        t4 = t[-2]
        t5 = t[-1]
        h1 = t1 - t_i
        h2 = t2 - t_i
        h3 = t3 - t_i
        h4 = t4 - t_i
        h5 = t5 - t_i
        h12 = t1 - t2
        h13 = t1 - t3
        h14 = t1 - t4
        h15 = t1 - t5
        h23 = t2 - t3
        h24 = t2 - t4
        h25 = t2 - t5
        h34 = t3 - t4
        h35 = t3 - t5
        h45 = t4 - t5
        for k in xrange(f.shape[1]):
            dfdt[i, k] = (
            -((h2 * h3 * h4 + h2 * h3 * h5 + h2 * h4 * h5 + h3 * h4 * h5) / (h12 * h13 * h14 * h15)) * f[-5, k]
            + ((h1 * h3 * h4 + h1 * h3 * h5 + h1 * h4 * h5 + h3 * h4 * h5) / (h12 * h23 * h24 * h25)) * f[-4, k]
            - ((h1 * h2 * h4 + h1 * h2 * h5 + h1 * h4 * h5 + h2 * h4 * h5) / (h13 * h23 * h34 * h35)) * f[-3, k]
            + ((h1 * h2 * h3 + h1 * h2 * h5 + h1 * h3 * h5 + h2 * h3 * h5) / (h14 * h24 * h34 * h45)) * f[-2, k]
            - ((h1 * h2 * h3 + h1 * h2 * h4 + h1 * h3 * h4 + h2 * h3 * h4) / (h15 * h25 * h35 * h45)) * f[-1, k])

    return


@njit
def _derivative_3d(f, t, dfdt):
    for i in xrange(2):
        t_i = t[i]
        t1 = t[0]
        t2 = t[1]
        t3 = t[2]
        t4 = t[3]
        t5 = t[4]
        h1 = t1 - t_i
        h2 = t2 - t_i
        h3 = t3 - t_i
        h4 = t4 - t_i
        h5 = t5 - t_i
        h12 = t1 - t2
        h13 = t1 - t3
        h14 = t1 - t4
        h15 = t1 - t5
        h23 = t2 - t3
        h24 = t2 - t4
        h25 = t2 - t5
        h34 = t3 - t4
        h35 = t3 - t5
        h45 = t4 - t5
        for k in xrange(f.shape[1]):
            for m in xrange(f.shape[1]):
                dfdt[i, k, m] = (
                -((h2 * h3 * h4 + h2 * h3 * h5 + h2 * h4 * h5 + h3 * h4 * h5) / (h12 * h13 * h14 * h15)) * f[0, k, m]
                + ((h1 * h3 * h4 + h1 * h3 * h5 + h1 * h4 * h5 + h3 * h4 * h5) / (h12 * h23 * h24 * h25)) * f[1, k, m]
                - ((h1 * h2 * h4 + h1 * h2 * h5 + h1 * h4 * h5 + h2 * h4 * h5) / (h13 * h23 * h34 * h35)) * f[2, k, m]
                + ((h1 * h2 * h3 + h1 * h2 * h5 + h1 * h3 * h5 + h2 * h3 * h5) / (h14 * h24 * h34 * h45)) * f[3, k, m]
                - ((h1 * h2 * h3 + h1 * h2 * h4 + h1 * h3 * h4 + h2 * h3 * h4) / (h15 * h25 * h35 * h45)) * f[4, k, m])

    for i in xrange(2, len(t) - 2):
        t1 = t[i - 2]
        t2 = t[i - 1]
        t3 = t[i]
        t4 = t[i + 1]
        t5 = t[i + 2]
        h1 = t1 - t3
        h2 = t2 - t3
        h4 = t4 - t3
        h5 = t5 - t3
        h12 = t1 - t2
        h13 = t1 - t3
        h14 = t1 - t4
        h15 = t1 - t5
        h23 = t2 - t3
        h24 = t2 - t4
        h25 = t2 - t5
        h34 = t3 - t4
        h35 = t3 - t5
        h45 = t4 - t5
        for k in xrange(f.shape[1]):
            for m in xrange(f.shape[1]):
                dfdt[i, k, m] = (-((h2 * h4 * h5) / (h12 * h13 * h14 * h15)) * f[i - 2, k, m]
                              + ((h1 * h4 * h5) / (h12 * h23 * h24 * h25)) * f[i - 1, k, m]
                              - ((h1 * h2 * h4 + h1 * h2 * h5 + h1 * h4 * h5 + h2 * h4 * h5) / (h13 * h23 * h34 * h35))
                                 * f[i, k, m]
                              + ((h1 * h2 * h5) / (h14 * h24 * h34 * h45)) * f[i + 1, k, m]
                              - ((h1 * h2 * h4) / (h15 * h25 * h35 * h45)) * f[i + 2, k, m])

    for i in xrange(len(t) - 2, len(t)):
        t_i = t[i]
        t1 = t[-5]
        t2 = t[-4]
        t3 = t[-3]
        t4 = t[-2]
        t5 = t[-1]
        h1 = t1 - t_i
        h2 = t2 - t_i
        h3 = t3 - t_i
        h4 = t4 - t_i
        h5 = t5 - t_i
        h12 = t1 - t2
        h13 = t1 - t3
        h14 = t1 - t4
        h15 = t1 - t5
        h23 = t2 - t3
        h24 = t2 - t4
        h25 = t2 - t5
        h34 = t3 - t4
        h35 = t3 - t5
        h45 = t4 - t5
        for k in xrange(f.shape[1]):
            for m in xrange(f.shape[1]):
                dfdt[i, k, m] = (
                -((h2 * h3 * h4 + h2 * h3 * h5 + h2 * h4 * h5 + h3 * h4 * h5) / (h12 * h13 * h14 * h15)) * f[-5, k, m]
                + ((h1 * h3 * h4 + h1 * h3 * h5 + h1 * h4 * h5 + h3 * h4 * h5) / (h12 * h23 * h24 * h25)) * f[-4, k, m]
                - ((h1 * h2 * h4 + h1 * h2 * h5 + h1 * h4 * h5 + h2 * h4 * h5) / (h13 * h23 * h34 * h35)) * f[-3, k, m]
                + ((h1 * h2 * h3 + h1 * h2 * h5 + h1 * h3 * h5 + h2 * h3 * h5) / (h14 * h24 * h34 * h45)) * f[-2, k, m]
                - ((h1 * h2 * h3 + h1 * h2 * h4 + h1 * h3 * h4 + h2 * h3 * h4) / (h15 * h25 * h35 * h45)) * f[-1, k, m])

    return


@njit
def fd_indefinite_integral(f, t):
    Sfdt = np.empty_like(f)
    Sfdt[0] = 0.0
    for i in xrange(1, len(t)):
        for j in xrange(f.shape[1]):
            Sfdt[i, j] = Sfdt[i - 1, j] + (f[i, j] + f[i - 1, j]) * ((t[i] - t[i - 1]) / 2.0)
    return Sfdt


def fd_definite_integral(f, t):
    Sfdt = np.zeros_like(f)
    Sfdt[1:, ...] = (f[1:, ...] + f[:-1, ...]) * ((t[1:] - t[:-1]) / 2.0).reshape((-1,) + (1,)*(f.ndim-1))
    return np.sum(Sfdt, axis=0)


def spline_evaluation(f, t, t_out=None, axis=None, spline_degree=3,
                      derivative_order=0, definite_integral_bounds=None):
    """Approximate input data using a spline and evaluate

    Note that this function is somewhat more general than it needs to be, so that it can be reused
    for closely related functions involving derivatives, antiderivatives, and integrals.

    Parameters
    ==========
    f : (..., N, ...) array_like
        Real or complex function values to be interpolated.

    t : (N,) array_like
        An N-D array of increasing real values. The length of f along the interpolation axis must be
        equal to the length of t.  The number of data points must be larger than the spline degree.

    t_out : None or (M,) array_like [defaults to None]
        The new values of `t` on which to evaluate the result.  If None, it is assumed that some
        other feature of the data is needed, like a derivative or antiderivative, which are then
        output using the same `t` values as the input.

    axis : None or int [defaults to None]
        The axis of `f` with length equal to the length of `t`.  If None, this function searches for
        an axis of equal length in reverse order -- that is, starting from the last axis of `f`.
        Note that this feature is helpful when `f` is one-dimensional or will always satisfy that
        criterion, but is dangerous otherwise.  Caveat emptor.

    spline_degree : int [defaults to 3]
        Degree of the interpolating spline. Must be 1 <= spline_degree <= 5.

    derivative_order : int [defaults to 0]
        The order of the derivative to apply to the data.  Note that this may be negative, in which
        case the corresponding antiderivative is returned.

    definite_integral_bounds : None or (2,) array_like [defaults to None]
        If this is not None, the `t_out` and `derivative_order` parameters are ignored, and the
        returned values are just the (first) definite integrals of the splines between these limits,
        along each remaining axis.

    """
    import numpy as np
    from scipy.interpolate import InterpolatedUnivariateSpline

    # Process input arguments and get data into correct shape
    if not 1 <= spline_degree <= 5:
        raise ValueError('The spline degree must be between 1 and 5 (inclusive); it is {0}.'.format(spline_degree))
    t = np.asarray(t, dtype=float, order='C')
    if t.ndim != 1:
        raise ValueError('Input t values must be a one-dimensional array; this input has {0}.'.format(t.ndim))
    n = t.size
    if spline_degree >= n:
        raise ValueError('The spline degree ({0}) must be less than the number of data points ({1}).'.format(spline_degree, n))
    f = np.asanyarray(f)
    if axis is None:
        try:
            axis = f.ndim - 1 - list(reversed(f.shape)).index(n)
        except ValueError:
            axis = None
    if axis is None or f.shape[axis] != n:
        raise ValueError((
            "Input function values `f` [shape {0}] should have at least one "
            "axis with the same length as input `t` [{1}], or bad axis input."
            ).format(f.shape, n))
    shape = list(f.shape)
    if definite_integral_bounds is not None:
        shape[axis] = 1  # We'll keep this axis for now (set to length 1) for uniform treatment, and remove it before returning
        definite_integral_bounds = np.array(definite_integral_bounds, dtype=float)
        if definite_integral_bounds.shape != (2,):
            raise ValueError("Expected exactly two bounds for the definite integral; got {0}.".format(definite_integral_bounds.shape))
        f_out = np.empty(shape, dtype=f.dtype)
        t_a, t_b = definite_integral_bounds
        def evaluator(s):
            return s.integral(t_a, t_b)
        axis_slice = slice(max(0, np.argmin(np.abs(t-t_a))-10), min(n, np.argmin(np.abs(t-t_b))+11))
    else:
        if t_out is None:
            t_out = t
            axis_slice = slice(None)
        else:
            axis_slice = slice(max(0, np.argmin(np.abs(t-t_out[0]))-10), min(n, np.argmin(np.abs(t-t_out[-1]))+11))
        shape[axis] = t_out.size
        if derivative_order != 0 and derivative_order > spline_degree:
            raise ValueError("Order of derivative ({0}) must not be greater than degree of spline ({1})".format(derivative_order, spline_degree))
        f_out = np.empty(shape, dtype=f.dtype)
        if derivative_order < 0:
            def evaluator(s):
                return s.antiderivative(n=-derivative_order)(t_out)
        elif derivative_order > 0:
            def evaluator(s):
                return s.derivative(n=derivative_order)(t_out)
        else:
            def evaluator(s):
                return s(t_out)
    def spline(f, t):
        return InterpolatedUnivariateSpline(t[axis_slice], f[axis_slice], k=spline_degree)

    # Move the axis to the end so that we can just iterate over all but the last index
    if axis != -1 and axis != n-1:
        f = np.moveaxis(f, axis, -1)
        f_out = np.moveaxis(f_out, axis, -1)

    # Iterate over all extra axes and evaluate
    complex_valued = np.iscomplexobj(f)
    for index in np.ndindex(f.shape[:-1]):
        if complex_valued:
            f_out[index] = evaluator(spline(f[index].real, t)) + 1j * evaluator(spline(f[index].imag, t))
        else:
            f_out[index] = evaluator(spline(f[index], t))

    # Undo the axis move we did previously to the output (input doesn't matter any more)
    if axis != -1 and axis != n-1:
        f_out = np.moveaxis(f_out, -1, axis)

    # If this is a definite integral, remove that extraneous axis
    if definite_integral_bounds is not None:
        f_out = np.squeeze(f_out, axis=axis)

    return f_out


def spline_derivative(f, t, derivative_order=1, axis=0):
    return spline_evaluation(f, t, axis=axis, derivative_order=derivative_order)


def spline_indefinite_integral(f, t, integral_order=1, axis=0):
    return spline_evaluation(f, t, axis=axis, derivative_order=-integral_order)


def spline_definite_integral(f, t, t1=None, t2=None, axis=0):
    if t1 is None:
        t1 = t[0]
    if t2 is None:
        t2 = t[-1]
    return spline_evaluation(f, t, axis=axis, definite_integral_bounds=(t1, t2))


try:
    from scipy.interpolate import InterpolatedUnivariateSpline
    spline = spline_evaluation
    derivative = spline_derivative
    antiderivative = spline_indefinite_integral
    indefinite_integral = spline_indefinite_integral
    definite_integral = spline_definite_integral
except ImportError:
    # import warnings
    # warning_text = (
    #     "\n\n" + "!" * 57 + "\n" +
    #     "Could not import from scipy, which means that derivatives\n" +
    #     "and integrals will use less accurate finite-differencing\n" +
    #     "techniques.  You may want to install scipy." +
    #     "\n" + "!" * 57 + "\n"
    # )
    # warnings.warn(warning_text)
    derivative = fd_derivative
    antiderivative = fd_indefinite_integral
    indefinite_integral = fd_indefinite_integral
    definite_integral = fd_definite_integral
