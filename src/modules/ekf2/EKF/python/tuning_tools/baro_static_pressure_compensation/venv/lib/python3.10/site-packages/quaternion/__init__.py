# -*- coding: utf-8 -*-

# Copyright (c) 2021, Michael Boyle
# See LICENSE file for details: <https://github.com/moble/quaternion/blob/main/LICENSE>

__version__ = "2022.4.3"
__doc_title__ = "Quaternion dtype for NumPy"
__doc__ = "Adds a quaternion dtype to NumPy."
__all__ = ['quaternion',
           'as_quat_array', 'as_spinor_array',
           'as_float_array', 'from_float_array',
           'as_vector_part', 'from_vector_part',
           'as_rotation_matrix', 'from_rotation_matrix',
           'as_rotation_vector', 'from_rotation_vector',
           'as_euler_angles', 'from_euler_angles',
           'as_spherical_coords', 'from_spherical_coords',
           'rotate_vectors', 'allclose',
           'rotor_intrinsic_distance', 'rotor_chordal_distance',
           'rotation_intrinsic_distance', 'rotation_chordal_distance',
           'slerp_evaluate', 'squad_evaluate',
           'zero', 'one', 'x', 'y', 'z', 'integrate_angular_velocity',
           'squad', 'slerp', 'derivative', 'definite_integral', 'indefinite_integral']


import numpy as np

from .numpy_quaternion import (
    quaternion, _eps, slerp_evaluate, squad_evaluate,
    # slerp_vectorized, squad_vectorized, slerp, squad,
)
from .quaternion_time_series import (
    unflip_rotors, slerp, squad, integrate_angular_velocity, minimal_rotation, angular_velocity,
)
from .calculus import (
    derivative, antiderivative, definite_integral, indefinite_integral,
    fd_derivative, fd_definite_integral, fd_indefinite_integral,
    spline_derivative, spline_definite_integral, spline_indefinite_integral,
)
try:
    from .calculus import spline
except:
    pass
from .means import (
    mean_rotor_in_chordal_metric,
    optimal_alignment_in_chordal_metric,
    optimal_alignment_in_Euclidean_metric
)

np.quaternion = quaternion
np.sctypeDict['quaternion'] = np.dtype(quaternion)

zero = np.quaternion(0, 0, 0, 0)
one = np.quaternion(1, 0, 0, 0)
x = np.quaternion(0, 1, 0, 0)
y = np.quaternion(0, 0, 1, 0)
z = np.quaternion(0, 0, 0, 1)

rotor_intrinsic_distance = np.rotor_intrinsic_distance
rotor_chordal_distance = np.rotor_chordal_distance
rotation_intrinsic_distance = np.rotation_intrinsic_distance
rotation_chordal_distance = np.rotation_chordal_distance


def as_float_array(a):
    """View the quaternion array as an array of floats

    This function is fast (of order 1 microsecond) because no data is
    copied; the returned quantity is just a "view" of the original.

    The output view has one more dimension (of size 4) than the input
    array, but is otherwise the same shape.  The components along
    that last dimension represent the scalar and vector components of
    each quaternion in that order: `w`, `x`, `y`, `z`.

    """
    return np.asarray(a, dtype=np.quaternion).view((np.double, 4))


def as_quat_array(a):
    """View a float array as an array of quaternions

    The input array must have a final dimension whose size is
    divisible by four (or better yet *is* 4), because successive
    indices in that last dimension will be considered successive
    components of the output quaternion.  Each set of 4 components
    will be interpreted as the scalar and vector components of a
    quaternion in that order: `w`, `x`, `y`, `z`.

    This function is usually fast (of order 1 microsecond) because no
    data is copied; the returned quantity is just a "view" of the
    original.  However, if the input array is not C-contiguous
    (basically, as you increment the index into the last dimension of
    the array, you just move to the neighboring float in memory), the
    data will need to be copied which may be quite slow.  Therefore,
    you should try to ensure that the input array is in that order.
    Slices and transpositions will frequently break that rule.

    We will not convert back from a two-spinor array because there is
    no unique convention for them, so I don't want to mess with that.
    Also, we want to discourage users from the slow, memory-copying
    process of swapping columns required for useful definitions of
    the two-spinors.

    """
    a = np.asarray(a, dtype=np.double)

    # fast path
    if a.shape == (4,):
        return quaternion(a[0], a[1], a[2], a[3])

    # view only works if the last axis is C-contiguous
    if not a.flags['C_CONTIGUOUS'] or a.strides[-1] != a.itemsize:
        a = a.copy(order='C')
    try:
        av = a.view(np.quaternion)
    except ValueError as e:
        message = (str(e) + '\n            '
                   + 'Failed to view input data as a series of quaternions.  '
                   + 'Please ensure that the last dimension has size divisible by 4.\n            '
                   + 'Input data has shape {0} and dtype {1}.'.format(a.shape, a.dtype))
        raise ValueError(message)

    # special case: don't create an axis for a single quaternion, to
    # match the output of `as_float_array`
    if av.shape[-1] == 1:
        av = av.reshape(a.shape[:-1])

    return av


def from_float_array(a):
    return as_quat_array(a)


def from_vector_part(v, vector_axis=-1):
    """Create a quaternion array from an array of the vector parts.

    Essentially, this just inserts a 0 in front of each vector part, and
    re-interprets the result as a quaternion.

    Parameters
    ----------
    v : array_like
        Array of vector parts of quaternions.  When interpreted as a numpy array,
        if the dtype is `quaternion`, the array is returned immediately, and the
        following argument is ignored.  Otherwise, it it must be a float array with
        dimension `vector_axis` of size 3 or 4.
    vector_axis : int, optional
        The axis to interpret as containing the vector components.  The default is
        -1.

    Returns
    -------
    q : array of quaternions
        Quaternions with vector parts corresponding to input vectors.

    """
    v = np.asarray(v)
    if v.dtype != np.quaternion:
        input_shape = v.shape
        if vector_axis != -1:
            v = np.moveaxis(v, vector_axis, -1)
        if v.shape[-1] == 3:
            v = from_float_array(np.insert(v, 0, 0.0, axis=-1))
        elif v.shape[-1] == 4:
            v = v.copy()
            v[..., 0] = 0.0
            v = from_float_array(v)
        else:
            raise ValueError(
                "Vector input has shape {0}, which cannot be interpreted as quaternions ".format(input_shape)
                + "with vector axis {0}".format(vector_axis)
            )
    return v


def as_vector_part(q):
    """Create an array of vector parts from an array of quaternions.

    Parameters
    ----------
    q : quaternion array_like
        Array of quaternions.

    Returns
    -------
    v : array
        Float array of shape `q.shape + (3,)`

    """
    q = np.asarray(q, dtype=np.quaternion)
    return as_float_array(q)[..., 1:]


def as_spinor_array(a):
    """View a quaternion array as spinors in two-complex representation

    This function is relatively slow and scales poorly, because memory
    copying is apparently involved -- I think it's due to the
    "advanced indexing" required to swap the columns.

    """
    a = np.atleast_1d(a)
    assert a.dtype == np.dtype(np.quaternion)
    # I'm not sure why it has to be so complicated, but all of these steps
    # appear to be necessary in this case.
    return a.view(np.float64).reshape(a.shape + (4,))[..., [0, 3, 2, 1]].ravel().view(np.complex128).reshape(a.shape + (2,))


def as_rotation_matrix(q):
    """Convert input quaternion to 3x3 rotation matrix

    For any quaternion `q`, this function returns a matrix `m` such that, for every
    vector `v`, we have

        m @ v.vec == q * v * q.conjugate()

    Here, `@` is the standard python matrix multiplication operator and `v.vec` is
    the 3-vector part of the quaternion `v`.

    Parameters
    ----------
    q : quaternion or array of quaternions
        The quaternion(s) need not be normalized, but must all be nonzero

    Returns
    -------
    m : float array
        Output shape is q.shape+(3,3).  This matrix should multiply (from
        the left) a column vector to produce the rotated column vector.

    Raises
    ------
    ZeroDivisionError
        If any of the input quaternions have norm 0.0.

    """
    if q.shape == () and not isinstance(q, np.ndarray):  # This is just a single quaternion
        n = q.norm()
        if n == 0.0:
            raise ZeroDivisionError("Input to `as_rotation_matrix({0})` has zero norm".format(q))
        elif abs(n-1.0) < _eps:  # Input q is basically normalized
            return np.array([
                [1 - 2*(q.y**2 + q.z**2),   2*(q.x*q.y - q.z*q.w),      2*(q.x*q.z + q.y*q.w)],
                [2*(q.x*q.y + q.z*q.w),     1 - 2*(q.x**2 + q.z**2),    2*(q.y*q.z - q.x*q.w)],
                [2*(q.x*q.z - q.y*q.w),     2*(q.y*q.z + q.x*q.w),      1 - 2*(q.x**2 + q.y**2)]
            ])
        else:  # Input q is not normalized
            return np.array([
                [1 - 2*(q.y**2 + q.z**2)/n,   2*(q.x*q.y - q.z*q.w)/n,      2*(q.x*q.z + q.y*q.w)/n],
                [2*(q.x*q.y + q.z*q.w)/n,     1 - 2*(q.x**2 + q.z**2)/n,    2*(q.y*q.z - q.x*q.w)/n],
                [2*(q.x*q.z - q.y*q.w)/n,     2*(q.y*q.z + q.x*q.w)/n,      1 - 2*(q.x**2 + q.y**2)/n]
            ])
    else:  # This is an array of quaternions
        n = np.norm(q)
        if np.any(n == 0.0):
            raise ZeroDivisionError("Array input to `as_rotation_matrix` has at least one element with zero norm")
        else:  # Assume input q is not normalized
            m = np.empty(q.shape + (3, 3))
            q = as_float_array(q)
            m[..., 0, 0] = 1.0 - 2*(q[..., 2]**2 + q[..., 3]**2)/n
            m[..., 0, 1] = 2*(q[..., 1]*q[..., 2] - q[..., 3]*q[..., 0])/n
            m[..., 0, 2] = 2*(q[..., 1]*q[..., 3] + q[..., 2]*q[..., 0])/n
            m[..., 1, 0] = 2*(q[..., 1]*q[..., 2] + q[..., 3]*q[..., 0])/n
            m[..., 1, 1] = 1.0 - 2*(q[..., 1]**2 + q[..., 3]**2)/n
            m[..., 1, 2] = 2*(q[..., 2]*q[..., 3] - q[..., 1]*q[..., 0])/n
            m[..., 2, 0] = 2*(q[..., 1]*q[..., 3] - q[..., 2]*q[..., 0])/n
            m[..., 2, 1] = 2*(q[..., 2]*q[..., 3] + q[..., 1]*q[..., 0])/n
            m[..., 2, 2] = 1.0 - 2*(q[..., 1]**2 + q[..., 2]**2)/n
            return m


def from_rotation_matrix(rot, nonorthogonal=True):
    """Convert input 3x3 rotation matrix to unit quaternion

    For any orthogonal matrix `rot`, this function returns a quaternion `q` such
    that, for every pure-vector quaternion `v`, we have

        q * v * q.conjugate() == rot @ v.vec

    Here, `@` is the standard python matrix multiplication operator and `v.vec` is
    the 3-vector part of the quaternion `v`.  If `rot` is not orthogonal the
    "closest" orthogonal matrix is used; see Notes below.

    Parameters
    ----------
    rot : (..., N, 3, 3) float array
        Each 3x3 matrix represents a rotation by multiplying (from the left) a
        column vector to produce a rotated column vector.  Note that this input may
        actually have ndims>3; it is just assumed that the last two dimensions have
        size 3, representing the matrix.
    nonorthogonal : bool, optional
        If scipy.linalg is available, use the more robust algorithm of Bar-Itzhack.
        Default value is True.

    Returns
    -------
    q : array of quaternions
        Unit quaternions resulting in rotations corresponding to input rotations.
        Output shape is rot.shape[:-2].

    Raises
    ------
    LinAlgError
        If any of the eigenvalue solutions does not converge

    Notes
    -----
    By default, if scipy.linalg is available, this function uses Bar-Itzhack's
    algorithm to allow for non-orthogonal matrices.  [J. Guidance, Vol. 23, No. 6,
    p. 1085 <http://dx.doi.org/10.2514/2.4654>] This will almost certainly be quite
    a bit slower than simpler versions, though it will be more robust to numerical
    errors in the rotation matrix.  Also note that Bar-Itzhack uses some pretty
    weird conventions.  The last component of the quaternion appears to represent
    the scalar, and the quaternion itself is conjugated relative to the convention
    used throughout this module.

    If scipy.linalg is not available or if the optional `nonorthogonal` parameter
    is set to `False`, this function falls back to the possibly faster, but less
    robust, algorithm of Markley [J. Guidance, Vol. 31, No. 2, p. 440
    <http://dx.doi.org/10.2514/1.31730>].

    """
    try:
        from scipy import linalg
    except ImportError:
        linalg = False

    rot = np.array(rot, copy=False)
    shape = rot.shape[:-2]

    if linalg and nonorthogonal:
        from operator import mul
        from functools import reduce

        K3 = np.empty(shape+(4, 4))
        K3[..., 0, 0] = (rot[..., 0, 0] - rot[..., 1, 1] - rot[..., 2, 2])/3.0
        K3[..., 0, 1] = (rot[..., 1, 0] + rot[..., 0, 1])/3.0
        K3[..., 0, 2] = (rot[..., 2, 0] + rot[..., 0, 2])/3.0
        K3[..., 0, 3] = (rot[..., 1, 2] - rot[..., 2, 1])/3.0
        K3[..., 1, 0] = K3[..., 0, 1]
        K3[..., 1, 1] = (rot[..., 1, 1] - rot[..., 0, 0] - rot[..., 2, 2])/3.0
        K3[..., 1, 2] = (rot[..., 2, 1] + rot[..., 1, 2])/3.0
        K3[..., 1, 3] = (rot[..., 2, 0] - rot[..., 0, 2])/3.0
        K3[..., 2, 0] = K3[..., 0, 2]
        K3[..., 2, 1] = K3[..., 1, 2]
        K3[..., 2, 2] = (rot[..., 2, 2] - rot[..., 0, 0] - rot[..., 1, 1])/3.0
        K3[..., 2, 3] = (rot[..., 0, 1] - rot[..., 1, 0])/3.0
        K3[..., 3, 0] = K3[..., 0, 3]
        K3[..., 3, 1] = K3[..., 1, 3]
        K3[..., 3, 2] = K3[..., 2, 3]
        K3[..., 3, 3] = (rot[..., 0, 0] + rot[..., 1, 1] + rot[..., 2, 2])/3.0

        if not shape:
            q = zero.copy()
            eigvals, eigvecs = linalg.eigh(K3.T, subset_by_index=(3, 3))
            q.components[0] = eigvecs[-1]
            q.components[1:] = -eigvecs[:-1].flatten()
            return q
        else:
            q = np.empty(shape+(4,), dtype=np.float64)
            for flat_index in range(reduce(mul, shape)):
                multi_index = np.unravel_index(flat_index, shape)
                eigvals, eigvecs = linalg.eigh(K3[multi_index], subset_by_index=(3, 3))
                q[multi_index+(0,)] = eigvecs[-1]
                q[multi_index+(slice(1,None),)] = -eigvecs[:-1].flatten()
            return as_quat_array(q)

    else:  # No scipy.linalg or not `nonorthogonal`
        diagonals = np.empty(shape+(4,))
        diagonals[..., 0] = rot[..., 0, 0]
        diagonals[..., 1] = rot[..., 1, 1]
        diagonals[..., 2] = rot[..., 2, 2]
        diagonals[..., 3] = rot[..., 0, 0] + rot[..., 1, 1] + rot[..., 2, 2]

        indices = np.argmax(diagonals, axis=-1)

        q = diagonals  # reuse storage space
        indices_i = (indices == 0)
        if np.any(indices_i):
            if indices_i.shape == ():
                indices_i = Ellipsis
            rot_i = rot[indices_i, :, :]
            q[indices_i, 0] = rot_i[..., 2, 1] - rot_i[..., 1, 2]
            q[indices_i, 1] = 1 + rot_i[..., 0, 0] - rot_i[..., 1, 1] - rot_i[..., 2, 2]
            q[indices_i, 2] = rot_i[..., 0, 1] + rot_i[..., 1, 0]
            q[indices_i, 3] = rot_i[..., 0, 2] + rot_i[..., 2, 0]
        indices_i = (indices == 1)
        if np.any(indices_i):
            if indices_i.shape == ():
                indices_i = Ellipsis
            rot_i = rot[indices_i, :, :]
            q[indices_i, 0] = rot_i[..., 0, 2] - rot_i[..., 2, 0]
            q[indices_i, 1] = rot_i[..., 1, 0] + rot_i[..., 0, 1]
            q[indices_i, 2] = 1 - rot_i[..., 0, 0] + rot_i[..., 1, 1] - rot_i[..., 2, 2]
            q[indices_i, 3] = rot_i[..., 1, 2] + rot_i[..., 2, 1]
        indices_i = (indices == 2)
        if np.any(indices_i):
            if indices_i.shape == ():
                indices_i = Ellipsis
            rot_i = rot[indices_i, :, :]
            q[indices_i, 0] = rot_i[..., 1, 0] - rot_i[..., 0, 1]
            q[indices_i, 1] = rot_i[..., 2, 0] + rot_i[..., 0, 2]
            q[indices_i, 2] = rot_i[..., 2, 1] + rot_i[..., 1, 2]
            q[indices_i, 3] = 1 - rot_i[..., 0, 0] - rot_i[..., 1, 1] + rot_i[..., 2, 2]
        indices_i = (indices == 3)
        if np.any(indices_i):
            if indices_i.shape == ():
                indices_i = Ellipsis
            rot_i = rot[indices_i, :, :]
            q[indices_i, 0] = 1 + rot_i[..., 0, 0] + rot_i[..., 1, 1] + rot_i[..., 2, 2]
            q[indices_i, 1] = rot_i[..., 2, 1] - rot_i[..., 1, 2]
            q[indices_i, 2] = rot_i[..., 0, 2] - rot_i[..., 2, 0]
            q[indices_i, 3] = rot_i[..., 1, 0] - rot_i[..., 0, 1]

        q /= np.linalg.norm(q, axis=-1)[..., np.newaxis]

        return as_quat_array(q)


def as_rotation_vector(q):
    """Convert input quaternion to the axis-angle representation

    Note that if any of the input quaternions has norm zero, no error is
    raised, but NaNs will appear in the output.

    Parameters
    ----------
    q : quaternion or array of quaternions
        The quaternion(s) need not be normalized, but must all be nonzero

    Returns
    -------
    rot : float array
        Output shape is q.shape+(3,).  Each vector represents the axis of
        the rotation, with norm proportional to the angle of the rotation in
        radians.

    """
    return as_float_array(2*np.log(np.normalized(q)))[..., 1:]


def from_rotation_vector(rot):
    """Convert input 3-vector in axis-angle representation to unit quaternion

    Parameters
    ----------
    rot : (Nx3) float array
        Each vector represents the axis of the rotation, with norm
        proportional to the angle of the rotation in radians.

    Returns
    -------
    q : array of quaternions
        Unit quaternions resulting in rotations corresponding to input
        rotations.  Output shape is rot.shape[:-1].

    """
    rot = np.array(rot, copy=False)
    quats = np.zeros(rot.shape[:-1]+(4,))
    quats[..., 1:] = rot[...]/2
    quats = as_quat_array(quats)
    return np.exp(quats)


def as_euler_angles(q):
    """Open Pandora's Box

    If somebody is trying to make you use Euler angles, tell them no, and
    walk away, and go and tell your mum.

    You don't want to use Euler angles.  They are awful.  Stay away.  It's
    one thing to convert from Euler angles to quaternions; at least you're
    moving in the right direction.  But to go the other way?!  It's just not
    right.

    Assumes the Euler angles correspond to the quaternion R via

        R = exp(alpha*z/2) * exp(beta*y/2) * exp(gamma*z/2)

    The angles are naturally in radians.

    NOTE: Before opening an issue reporting something "wrong" with this
    function, be sure to read all of the following page, *especially* the
    very last section about opening issues or pull requests.
    <https://github.com/moble/quaternion/wiki/Euler-angles-are-horrible>

    Parameters
    ----------
    q : quaternion or array of quaternions
        The quaternion(s) need not be normalized, but must all be nonzero

    Returns
    -------
    alpha_beta_gamma : float array
        Output shape is q.shape+(3,).  These represent the angles (alpha,
        beta, gamma) in radians, where the normalized input quaternion
        represents `exp(alpha*z/2) * exp(beta*y/2) * exp(gamma*z/2)`.

    Raises
    ------
    AllHell
        ...if you try to actually use Euler angles, when you could have
        been using quaternions like a sensible person.

    """
    alpha_beta_gamma = np.empty(q.shape + (3,), dtype=np.float64)
    n = np.norm(q)
    q = as_float_array(q)
    alpha_beta_gamma[..., 0] = np.arctan2(q[..., 3], q[..., 0]) + np.arctan2(-q[..., 1], q[..., 2])
    alpha_beta_gamma[..., 1] = 2*np.arccos(np.sqrt((q[..., 0]**2 + q[..., 3]**2)/n))
    alpha_beta_gamma[..., 2] = np.arctan2(q[..., 3], q[..., 0]) - np.arctan2(-q[..., 1], q[..., 2])
    return alpha_beta_gamma


def from_euler_angles(alpha_beta_gamma, beta=None, gamma=None):
    """Improve your life drastically

    Assumes the Euler angles correspond to the quaternion R via

        R = exp(alpha*z/2) * exp(beta*y/2) * exp(gamma*z/2)

    The angles naturally must be in radians for this to make any sense.

    NOTE: Before opening an issue reporting something "wrong" with this
    function, be sure to read all of the following page, *especially* the
    very last section about opening issues or pull requests.
    <https://github.com/moble/quaternion/wiki/Euler-angles-are-horrible>

    Parameters
    ----------
    alpha_beta_gamma : float or array of floats
        This argument may either contain an array with last dimension of
        size 3, where those three elements describe the (alpha, beta, gamma)
        radian values for each rotation; or it may contain just the alpha
        values, in which case the next two arguments must also be given.
    beta : None, float, or array of floats
        If this array is given, it must be able to broadcast against the
        first and third arguments.
    gamma : None, float, or array of floats
        If this array is given, it must be able to broadcast against the
        first and second arguments.

    Returns
    -------
    R : quaternion array
        The shape of this array will be the same as the input, except that
        the last dimension will be removed.

    """
    # Figure out the input angles from either type of input
    if gamma is None:
        alpha_beta_gamma = np.asarray(alpha_beta_gamma, dtype=np.double)
        alpha = alpha_beta_gamma[..., 0]
        beta  = alpha_beta_gamma[..., 1]
        gamma = alpha_beta_gamma[..., 2]
    else:
        alpha = np.asarray(alpha_beta_gamma, dtype=np.double)
        beta  = np.asarray(beta, dtype=np.double)
        gamma = np.asarray(gamma, dtype=np.double)

    # Set up the output array
    R = np.empty(np.broadcast(alpha, beta, gamma).shape + (4,), dtype=np.double)

    # Compute the actual values of the quaternion components
    R[..., 0] =  np.cos(beta/2)*np.cos((alpha+gamma)/2)  # scalar quaternion components
    R[..., 1] = -np.sin(beta/2)*np.sin((alpha-gamma)/2)  # x quaternion components
    R[..., 2] =  np.sin(beta/2)*np.cos((alpha-gamma)/2)  # y quaternion components
    R[..., 3] =  np.cos(beta/2)*np.sin((alpha+gamma)/2)  # z quaternion components

    return as_quat_array(R)


def as_spherical_coords(q):
    """Return the spherical coordinates corresponding to this quaternion

    Obviously, spherical coordinates do not contain as much information as a
    quaternion, so this function does lose some information.  However, the
    returned spherical coordinates will represent the point(s) on the sphere
    to which the input quaternion(s) rotate the z axis.

    Parameters
    ----------
    q : quaternion or array of quaternions
        The quaternion(s) need not be normalized, but must be nonzero

    Returns
    -------
    vartheta_varphi : float array
        Output shape is q.shape+(2,).  These represent the angles (vartheta,
        varphi) in radians, where the normalized input quaternion represents
        `exp(varphi*z/2) * exp(vartheta*y/2)`, up to an arbitrary inital
        rotation about `z`.

    """
    return as_euler_angles(q)[..., 1::-1]


def from_spherical_coords(theta_phi, phi=None):
    """Return the quaternion corresponding to these spherical coordinates

    Assumes the spherical coordinates correspond to the quaternion R via

        R = exp(phi*z/2) * exp(theta*y/2)

    The angles naturally must be in radians for this to make any sense.

    Note that this quaternion rotates `z` onto the point with the given
    spherical coordinates, but also rotates `x` and `y` onto the usual basis
    vectors (theta and phi, respectively) at that point.

    Parameters
    ----------
    theta_phi : float or array of floats
        This argument may either contain an array with last dimension of
        size 2, where those two elements describe the (theta, phi) values in
        radians for each point; or it may contain just the theta values in
        radians, in which case the next argument must also be given.
    phi : None, float, or array of floats
        If this array is given, it must be able to broadcast against the
        first argument.

    Returns
    -------
    R : quaternion array
        If the second argument is not given to this function, the shape
        will be the same as the input shape except for the last dimension,
        which will be removed.  If the second argument is given, this
        output array will have the shape resulting from broadcasting the
        two input arrays against each other.

    """
    # Figure out the input angles from either type of input
    if phi is None:
        theta_phi = np.asarray(theta_phi, dtype=np.double)
        theta = theta_phi[..., 0]
        phi  = theta_phi[..., 1]
    else:
        theta = np.asarray(theta_phi, dtype=np.double)
        phi = np.asarray(phi, dtype=np.double)

    # Set up the output array
    R = np.empty(np.broadcast(theta, phi).shape + (4,), dtype=np.double)

    # Compute the actual values of the quaternion components
    R[..., 0] =  np.cos(phi/2)*np.cos(theta/2)  # scalar quaternion components
    R[..., 1] = -np.sin(phi/2)*np.sin(theta/2)  # x quaternion components
    R[..., 2] =  np.cos(phi/2)*np.sin(theta/2)  # y quaternion components
    R[..., 3] =  np.sin(phi/2)*np.cos(theta/2)  # z quaternion components

    return as_quat_array(R)


def rotate_vectors(R, v, axis=-1):
    """Rotate vectors by given quaternions

    This function is for the case where each quaternion (possibly the only input
    quaternion) is used to rotate multiple vectors.  If each quaternion is only
    rotating a single vector, it is more efficient to use the standard formula

        vprime = R * v * R.conjugate()

    (Note that `from_vector_part` and `as_vector_part` may be helpful.)

    Parameters
    ----------
    R : quaternion array
        Quaternions by which to rotate the input vectors
    v : float array
        Three-vectors to be rotated.
    axis : int
        Axis of the `v` array to use as the vector dimension.  This
        axis of `v` must have length 3.

    Returns
    -------
    vprime : float array
        The rotated vectors.  This array has shape R.shape+v.shape.

    Notes
    -----
    For simplicity, this function converts the input quaternion(s) to matrix form,
    and rotates the input vector(s) by the usual matrix multiplication.  As noted
    above, if each input quaternion is only used to rotate a single vector, this is
    not the most efficient approach.  The simple formula shown above is faster than
    this function, though it should be noted that the most efficient approach (in
    terms of operation counts) is to use the formula

      v' = v + 2 * r x (s * v + r x v) / m

    where x represents the cross product, s and r are the scalar and vector parts
    of the quaternion, respectively, and m is the sum of the squares of the
    components of the quaternion.  If you are looping over a very large number of
    quaternions, and just rotating a single vector each time, you might want to
    implement that alternative algorithm using numba (or something that doesn't use
    python).

    """
    R = np.asarray(R, dtype=np.quaternion)
    v = np.asarray(v, dtype=float)
    if v.ndim < 1 or 3 not in v.shape:
        raise ValueError("Input `v` does not have at least one dimension of length 3")
    if v.shape[axis] != 3:
        raise ValueError("Input `v` axis {0} has length {1}, not 3.".format(axis, v.shape[axis]))
    m = as_rotation_matrix(R)
    tensordot_axis = m.ndim-2
    final_axis = tensordot_axis + (axis % v.ndim)
    return np.moveaxis(
        np.tensordot(m, v, axes=(-1, axis)),
        tensordot_axis, final_axis
    )


def isclose(a, b, rtol=4*np.finfo(float).eps, atol=0.0, equal_nan=False):
    """
    Returns a boolean array where two arrays are element-wise equal within a
    tolerance.

    This function is essentially a copy of the `numpy.isclose` function,
    with different default tolerances and one minor changes necessary to
    deal correctly with quaternions.

    The tolerance values are positive, typically very small numbers.  The
    relative difference (`rtol` * abs(`b`)) and the absolute difference
    `atol` are added together to compare against the absolute difference
    between `a` and `b`.

    Parameters
    ----------
    a, b : array_like
        Input arrays to compare.
    rtol : float
        The relative tolerance parameter (see Notes).
    atol : float
        The absolute tolerance parameter (see Notes).
    equal_nan : bool
        Whether to compare NaN's as equal.  If True, NaN's in `a` will be
        considered equal to NaN's in `b` in the output array.

    Returns
    -------
    y : array_like
        Returns a boolean array of where `a` and `b` are equal within the
        given tolerance. If both `a` and `b` are scalars, returns a single
        boolean value.

    See Also
    --------
    allclose

    Notes
    -----
    For finite values, isclose uses the following equation to test whether
    two floating point values are equivalent:

      absolute(`a` - `b`) <= (`atol` + `rtol` * absolute(`b`))

    The above equation is not symmetric in `a` and `b`, so that
    `isclose(a, b)` might be different from `isclose(b, a)` in
    some rare cases.

    Examples
    --------
    >>> quaternion.isclose([1e10*quaternion.x, 1e-7*quaternion.y], [1.00001e10*quaternion.x, 1e-8*quaternion.y],
    ...     rtol=1.e-5, atol=1.e-8)
    array([True, False])
    >>> quaternion.isclose([1e10*quaternion.x, 1e-8*quaternion.y], [1.00001e10*quaternion.x, 1e-9*quaternion.y],
    ...     rtol=1.e-5, atol=1.e-8)
    array([True, True])
    >>> quaternion.isclose([1e10*quaternion.x, 1e-8*quaternion.y], [1.0001e10*quaternion.x, 1e-9*quaternion.y],
    ...     rtol=1.e-5, atol=1.e-8)
    array([False, True])
    >>> quaternion.isclose([quaternion.x, np.nan*quaternion.y], [quaternion.x, np.nan*quaternion.y])
    array([True, False])
    >>> quaternion.isclose([quaternion.x, np.nan*quaternion.y], [quaternion.x, np.nan*quaternion.y], equal_nan=True)
    array([True, True])
    """
    def within_tol(x, y, atol, rtol):
        with np.errstate(invalid='ignore'):
            result = np.less_equal(abs(x-y), atol + rtol * abs(y))
        return result[()]

    x = np.array(a, copy=False, subok=True, ndmin=1)
    y = np.array(b, copy=False, subok=True, ndmin=1)

    # Make sure y is an inexact type to avoid bad behavior on abs(MIN_INT).
    # This will cause casting of x later. Also, make sure to allow subclasses
    # (e.g., for numpy.ma).
    try:
        dt = np.result_type(y, 1.)
    except TypeError:
        dt = np.dtype(np.quaternion)
    y = np.array(y, dtype=dt, copy=False, subok=True)

    xfin = np.isfinite(x)
    yfin = np.isfinite(y)
    if np.all(xfin) and np.all(yfin):
        return within_tol(x, y, atol, rtol)
    else:
        finite = xfin & yfin
        cond = np.zeros_like(finite, subok=True)
        # Because we're using boolean indexing, x & y must be the same shape.
        # Ideally, we'd just do x, y = broadcast_arrays(x, y). It's in
        # lib.stride_tricks, though, so we can't import it here.
        x = x * np.ones_like(cond)
        y = y * np.ones_like(cond)
        # Avoid subtraction with infinite/nan values...
        cond[finite] = within_tol(x[finite], y[finite], atol, rtol)
        # Check for equality of infinite values...
        cond[~finite] = (x[~finite] == y[~finite])
        if equal_nan:
            # Make NaN == NaN
            both_nan = np.isnan(x) & np.isnan(y)
            cond[both_nan] = both_nan[both_nan]

        return cond[()]


def allclose(a, b, rtol=4*np.finfo(float).eps, atol=0.0, equal_nan=False, verbose=False):
    """Returns True if two arrays are element-wise equal within a tolerance.

    This function is essentially a wrapper for the `quaternion.isclose`
    function, but returns a single boolean value of True if all elements
    of the output from `quaternion.isclose` are True, and False otherwise.
    This function also adds the option.

    Note that this function has stricter tolerances than the
    `numpy.allclose` function, as well as the additional `verbose` option.

    Parameters
    ----------
    a, b : array_like
        Input arrays to compare.
    rtol : float
        The relative tolerance parameter (see Notes).
    atol : float
        The absolute tolerance parameter (see Notes).
    equal_nan : bool
        Whether to compare NaN's as equal.  If True, NaN's in `a` will be
        considered equal to NaN's in `b` in the output array.
    verbose : bool
        If the return value is False, all the non-close values are printed,
        iterating through the non-close indices in order, displaying the
        array values along with the index, with a separate line for each
        pair of values.

    See Also
    --------
    isclose, numpy.all, numpy.any, numpy.allclose

    Returns
    -------
    allclose : bool
        Returns True if the two arrays are equal within the given
        tolerance; False otherwise.


    Notes
    -----
    If the following equation is element-wise True, then allclose returns
    True.

      absolute(`a` - `b`) <= (`atol` + `rtol` * absolute(`b`))

    The above equation is not symmetric in `a` and `b`, so that
    `allclose(a, b)` might be different from `allclose(b, a)` in
    some rare cases.

    """
    close = isclose(a, b, rtol=rtol, atol=atol, equal_nan=equal_nan)
    result = np.all(close)
    if verbose and not result:
        a, b = np.atleast_1d(a), np.atleast_1d(b)
        a, b = np.broadcast_arrays(a, b)
        print('Non-close values:')
        for i in np.nonzero(close == False):
            print('    a[{0}]={1}\n    b[{0}]={2}'.format(i, a[i], b[i]))
    return result
