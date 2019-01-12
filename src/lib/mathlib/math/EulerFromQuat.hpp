/**
 * @file EulerFromQuat.hpp
 *
 * All rotations and axis systems follow the right-hand rule
 *
 * IMPORTANT:
 *
 * Pitch-Roll-Yaw (the origin MC control is Roll-Pitch-Yaw which will come to singular when pitch is near pi/2)
 * _psi: yaw    _theta: pitch     _phi: roll
 *
 * @author Cris.Wei <buaaxw@gmail.com>
 */

#pragma once

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif

namespace matrix
{

template <typename Type>
class Dcm;

template <typename Type>
class Quaternion;

/**
 * EulerFromQuat angles class
 *
 * This class describes the rotation from frame 1
 * to frame 2 via 3-2-1 intrinsic Tait-Bryan rotation sequence.
 */
template<typename Type>
class EulerFromQuat : public Vector<Type, 3>
{
public:
    /**
     * Standard constructor
     */
    EulerFromQuat() = default;

    /**
     * Copy constructor
     *
     * @param other vector to copy
     */
    EulerFromQuat(const Vector<Type, 3> &other) :
        Vector<Type, 3>(other)
    {
    }

    /**
     * Constructor from Matrix31
     *
     * @param other Matrix31 to copy
     */
    EulerFromQuat(const Matrix<Type, 3, 1> &other) :
        Vector<Type, 3>(other)
    {
    }

    /**
     * Constructor from euler angles
     *
     * Instance is initialized from an 3-2-1 intrinsic Tait-Bryan
     * rotation sequence representing transformation from frame 1
     * to frame 2.
     *
     * @param phi_ rotation angle about X axis
     * @param theta_ rotation angle about Y axis
     * @param psi_ rotation angle about Z axis
     */
    EulerFromQuat(Type phi_, Type theta_, Type psi_) : Vector<Type, 3>()
    {
        phi() = phi_;
        theta() = theta_;
        psi() = psi_;
    }

    /**
     * Constructor from DCM matrix
     *
     * Instance is set from Dcm representing transformation from
     * frame 2 to frame 1.
     * This instance will hold the angles defining the 3-2-1 intrinsic
     * Tait-Bryan rotation sequence from frame 1 to frame 2.
     *
     * @param dcm Direction cosine matrix
    */
    EulerFromQuat(const Dcm<Type> &dcm)
    {
        Type phi_val = Type(atan2(dcm(2, 1), dcm(2, 2)));
        Type theta_val = Type(asin(-dcm(2, 0)));
        Type psi_val = Type(atan2(dcm(1, 0), dcm(0, 0)));
        Type pi = Type(M_PI);

        if (Type(fabs(theta_val - pi / Type(2))) < Type(1.0e-3)) {
            phi_val = Type(0.0);
            psi_val = Type(atan2(dcm(1, 2), dcm(0, 2)));

        } else if (Type(fabs(theta_val + pi / Type(2))) < Type(1.0e-3)) {
            phi_val = Type(0.0);
            psi_val = Type(atan2(-dcm(1, 2), -dcm(0, 2)));
        }

        phi() = phi_val;
        theta() = theta_val;
        psi() = psi_val;
    }

    /**
     * Constructor from quaternion instance.
     *
     * Instance is set from a quaternion representing transformation
     * from frame 2 to frame 1.
     * This instance will hold the angles defining the 3-2-1 intrinsic
     * Tait-Bryan rotation sequence from frame 1 to frame 2.
     *
     * @param q quaternion
    */
    EulerFromQuat(const Quaternion<Type> &q)
    {
        Type phi_val = Type(asin(Type(2.0) * (q(0) * q(1) + q(2) * q(3))));
        Type theta_val = Type(atan2(Type(-2.0) * Type(q(1) * q(3) - q(0) * q(2)), Type(1.0) - Type(2.0) * Type(q(1) * q(1) + q(2) * q(2))));
        Type psi_val = Type(atan2(Type(-2.0) * Type(q(1) * q(2) - q(0) * q(3)), Type(1.0) - Type(2.0) * Type(q(1) * q(1) + q(3) * q(3))));

        theta() = theta_val;//pitch
        phi() = phi_val;//roll
        psi() = psi_val;//yaw
    }

    inline Type phi() const
    {
        return (*this)(0);
    }
    inline Type theta() const
    {
        return (*this)(1);
    }
    inline Type psi() const
    {
        return (*this)(2);
    }

    inline Type &phi()
    {
        return (*this)(0);
    }
    inline Type &theta()
    {
        return (*this)(1);
    }
    inline Type &psi()
    {
        return (*this)(2);
    }

};

typedef EulerFromQuat<float> EulerFromQuatf;

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
