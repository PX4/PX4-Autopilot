/**
 * @file AxisAngle.hpp
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"
#include "helper_functions.hpp"

namespace matrix
{

template <typename Type>
class Dcm;

template <typename Type>
class Euler;

template <typename Type>
class AxisAngle;

/**
 * AxisAngle class
 *
 * The rotation between two coordinate frames is
 * described by this class.
 */
template<typename Type>
class AxisAngle : public Vector<Type, 3>
{
public:
    virtual ~AxisAngle() {};

    typedef Matrix<Type, 3, 1> Matrix31;

    /**
     * Constructor from array
     *
     * @param data_ array
     */
    AxisAngle(const Type *data_) :
        Vector<Type, 3>(data_)
    {
    }

    /**
     * Standard constructor
     */
    AxisAngle() :
        Vector<Type, 3>()
    {
    }

    /**
     * Constructor from Matrix31
     *
     * @param other Matrix31 to copy
     */
    AxisAngle(const Matrix31 &other) :
        Vector<Type, 3>(other)
    {
    }

    /**
     * Constructor from quaternion
     *
     * This sets the instance from a quaternion representing coordinate transformation from
     * frame 2 to frame 1 where the rotation from frame 1 to frame 2 is described
     * by a 3-2-1 intrinsic Tait-Bryan rotation sequence.
     *
     * @param q quaternion
     */
    AxisAngle(const Quaternion<Type> &q) :
        Vector<Type, 3>()
    {
        AxisAngle &v = *this;
        Type ang = (Type)2.0f*acosf(q(0));
        Type mag = sinf(ang/2.0f);
        if (fabs(ang) < 1e-10f) {
            v(0) = 0;
            v(1) = 0;
            v(2) = 0;
        } else {
            v(0) = ang*q(1)/mag;
            v(1) = ang*q(2)/mag;
            v(2) = ang*q(3)/mag;
        }
    }

    /**
     * Constructor from dcm
     *
     * Instance is initialized from a dcm representing coordinate transformation
     * from frame 2 to frame 1.
     *
     * @param dcm dcm to set quaternion to
     */
    AxisAngle(const Dcm<Type> &dcm) :
        Vector<Type, 3>()
    {
        AxisAngle &v = *this;
        v = Quaternion<Type>(dcm);
    }

    /**
     * Constructor from euler angles
     *
     * This sets the instance to a quaternion representing coordinate transformation from
     * frame 2 to frame 1 where the rotation from frame 1 to frame 2 is described
     * by a 3-2-1 intrinsic Tait-Bryan rotation sequence.
     *
     * @param euler euler angle instance
     */
    AxisAngle(const Euler<Type> &euler) :
        Vector<Type, 3>()
    {
        AxisAngle &v = *this;
        v = Quaternion<Type>(euler);
    }

    /**
     * Constructor from 3 axis angle values (unit vector * angle)
     *
     * @param x r_x*angle
     * @param y r_y*angle
     * @param z r_z*angle
     */
    AxisAngle(Type x, Type y, Type z) :
        Vector<Type, 3>()
    {
        AxisAngle &v = *this;
        v(0) = x;
        v(1) = y;
        v(2) = z;
    }

    /**
     * Constructor from axis and angle
     *
     * @param axis An axis of rotation, normalized if not unit length
     * @param angle The amount to rotate
     */
    AxisAngle(const Matrix31 & axis_, Type angle_) :
        Vector<Type, 3>()
    {
        AxisAngle &v = *this;
        // make sure axis is a unit vector
        Vector<Type, 3> a = axis_;
        a = a.unit();
        v(0) = a(0)*angle_;
        v(1) = a(1)*angle_;
        v(2) = a(2)*angle_;
    }


    Vector<Type, 3> axis() {
        return Vector<Type, 3>::unit();
    }

    Type angle() {
        return Vector<Type, 3>::norm();
    }
};

typedef AxisAngle<float> AxisAnglef;

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
