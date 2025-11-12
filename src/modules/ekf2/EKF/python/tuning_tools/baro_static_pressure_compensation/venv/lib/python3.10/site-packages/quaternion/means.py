# -*- coding: utf-8 -*-

# Copyright (c) 2021, Michael Boyle
# See LICENSE file for details: <https://github.com/moble/quaternion/blob/main/LICENSE>

from __future__ import division, print_function, absolute_import



def mean_rotor_in_chordal_metric(R, t=None):
    """Return rotor that is closest to all R in the least-squares sense

    This can be done (quasi-)analytically because of the simplicity of
    the chordal metric function.  It is assumed that the input R values
    all are normalized (or at least have the same norm).

    Note that the `t` argument is optional.  If it is present, the
    times are used to weight the corresponding integral.  If it is not
    present, a simple sum is used instead (which may be slightly
    faster).  However, because a spline is used to do this integral,
    the number of input points must be at least 4 (one more than the
    degree of the spline).

    """
    import numpy as np
    from . import as_float_array
    from .calculus import definite_integral
    if t is None:
        return np.sum(R).normalized()
    if len(t) < 4 or len(R) < 4:
        raise ValueError('Input arguments must have length greater than 3; their lengths are {0} and {1}.'.format(len(R), len(t)))
    mean = definite_integral(as_float_array(R), t)
    return np.quaternion(*mean).normalized()


def optimal_alignment_in_chordal_metric(Ra, Rb, t=None):
    """Return Rd such that Rd*Rb is as close to Ra as possible

    This function simply encapsulates the mean rotor of Ra/Rb.

    As in the `mean_rotor_in_chordal_metric` function, the `t` argument is
    optional.  If it is present, the times are used to weight the corresponding
    integral.  If it is not present, a simple sum is used instead (which may be
    slightly faster).

    Notes
    =====
    The idea here is to find Rd such that

        ∫ |Rd*Rb - Ra|^2 dt

    is minimized.  [Note that the integrand is the distance in the chordal metric.]
    We can ensure that this quantity is minimized by multiplying Rd by an
    exponential, differentiating with respect to the argument of the exponential,
    and setting that argument to 0.  This derivative should be 0 at the minimum.
    We have

        ∂ᵢ ∫ |exp[vᵢ]*Rd*Rb-Ra|^2 dt  →  2 ⟨ eᵢ * Rd * ∫ Rb*R̄a dt ⟩₀

    where → denotes taking vᵢ→0, the symbol ⟨⟩₀ denotes taking the scalar part, and
    eᵢ is the unit quaternionic vector in the `i` direction.  The only way for this
    quantity to be zero for each choice of `i` is if

        Rd * ∫ Rb*R̄a dt

    is itself a pure scalar.  This, in turn, can only happen if either (1) the
    integral is 0 or (2) if Rd is proportional to the conjugate of the integral:

        Rd ∝ ∫ Ra*R̄b dt

    Now, since we want Rd to be a rotor, we simply define it to be the normalized
    integral.

    """
    return mean_rotor_in_chordal_metric(Ra / Rb, t)


def optimal_alignment_in_Euclidean_metric(avec, bvec, t=None):
    """Return rotor R such that R*b⃗*R̄ is as close to a⃗ as possible

    As in the `optimal_alignment_in_chordal_metric` function, the `t` argument is
    optional.  If it is present, the times are used to weight the corresponding
    integral.  If it is not present, a simple sum is used instead (which may be
    slightly faster).

    The task of finding `R` is called "Wahba's problem"
    <https://en.wikipedia.org/wiki/Wahba%27s_problem>, and has a simple solution
    using eigenvectors.  In their book "Fundamentals of Spacecraft Attitude
    Determination and Control" (2014), Markley and Crassidis say that "Davenport’s
    method remains the best method for solving Wahba’s problem".  This constructs a
    simple matrix from a sum over the input vectors, and extracts the optimal rotor
    as the dominant eigenvector (the one with the largest eigenvalue).

    """
    import numpy as np
    from scipy.linalg import eigh
    from scipy.interpolate import InterpolatedUnivariateSpline as spline
    from . import quaternion

    avec = np.asarray(avec, dtype=float)
    bvec = np.asarray(bvec, dtype=float)
    if avec.shape != bvec.shape:
        raise ValueError("Input vectors must have same shape; avec.shape={!r}, bvec.shape={!r}".format(avec.shape, bvec.shape))
    if avec.shape[-1] != 3:
        raise ValueError("Final dimension of avec and bvec must have size 3; it is {!r}".format(avec.shape[-1]))
    if t is not None:
        if avec.ndim != 2:
            raise ValueError("If t is not None, avec and bvec must have exactly 2 dimensions; they have {!r}".format(avec.ndim))
        t = np.asarray(t, dtype=float)
        if avec.shape[0] != len(t):
            raise ValueError("Input time must have same length as first dimension of vectors; len(t)={!r}".format(len(t)))

    # This constructs the matrix given by Eq. (5.11) of Markley and Crassidis
    S = np.empty((3, 3))
    for i in range(3):
        for j in range(3):
            if t is None:
                S[i, j] = np.sum(avec[..., i] * bvec[..., j])
            else:
                S[i, j] = spline(t, avec[:, i] * bvec[:, j]).integral(t[0], t[-1])

    # This is Eq. (5.17) from Markley and Crassidis, modified to suit our
    # conventions by flipping the sign of ``z``, and moving the final dimension
    # to the first dimension.
    M = [
            [S[0,0]+S[1,1]+S[2,2],      S[2,1]-S[1,2],         S[0,2]-S[2,0],           S[1,0]-S[0,1],    ],
            [    S[2,1]-S[1,2],      S[0,0]-S[1,1]-S[2,2],     S[0,1]+S[1,0],           S[0,2]+S[2,0],    ],
            [    S[0,2]-S[2,0],         S[0,1]+S[1,0],      -S[0,0]+S[1,1]-S[2,2],      S[1,2]+S[2,1],    ],
            [    S[1,0]-S[0,1],         S[0,2]+S[2,0],         S[1,2]+S[2,1],       -S[0,0]-S[1,1]+S[2,2],],
    ]

    # This extracts the dominant eigenvector, and interprets it as a rotor.  In
    # particular, note that the *last* eigenvector output by `eigh` (the 3rd)
    # has the largest eigenvalue.
    eigenvector = eigh(M, subset_by_index=(3, 3))[1][:, 0]
    return quaternion(*eigenvector)


def mean_rotor_in_intrinsic_metric(R, t=None):
    raise NotImplementedError()
