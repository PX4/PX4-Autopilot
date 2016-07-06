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
        Type angle = (Type)2.0f*acosf(q(0));
        Type mag = sinf(angle/2.0f);
        v(0) = angle*q(1)/mag;
        v(1) = angle*q(2)/mag;
        v(2) = angle*q(3)/mag;
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
        v = Quaternion<float>(dcm);
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
        v = Quaternion<float>(euler);
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
     * @param x r_x*angle
     * @param x r_x*angle
     * @param y r_y*angle
     * @param z r_z*angle
     */
    AxisAngle(const Matrix31 & axis, Type angle) :
        Vector<Type, 3>()
    {
        AxisAngle &v = *this;
        // make sure axis is a unit vector
        Vector<Type, 3> a = axis;
        a = a.unit();
        v(0) = a(0)*angle;
        v(1) = a(1)*angle;
        v(2) = a(2)*angle;
    }
};

typedef AxisAngle<float> AxisAnglef;

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
