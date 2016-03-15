/**
 * @file Quaternion.hpp
 *
 * A quaternion class.
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

template<typename Type>
class Quaternion : public Vector<Type, 4>
{
public:
    virtual ~Quaternion() {};

    typedef Matrix<Type, 4, 1> Matrix41;
    typedef Matrix<Type, 3, 1> Matrix31;

    Quaternion(const Type *data_) :
        Vector<Type, 4>(data_)
    {
    }

    Quaternion() :
        Vector<Type, 4>()
    {
        Quaternion &q = *this;
        q(0) = 1;
        q(1) = 0;
        q(2) = 0;
        q(3) = 0;
    }

    Quaternion(const Matrix41 & other) :
        Vector<Type, 4>(other)
    {
    }

    Quaternion(const Dcm<Type> & dcm) :
        Vector<Type, 4>()
    {
        Quaternion &q = *this;
        q(0) = Type(0.5 * sqrt(1 + dcm(0, 0) +
                               dcm(1, 1) + dcm(2, 2)));
        q(1) = Type((dcm(2, 1) - dcm(1, 2)) /
                    (4 * q(0)));
        q(2) = Type((dcm(0, 2) - dcm(2, 0)) /
                    (4 * q(0)));
        q(3) = Type((dcm(1, 0) - dcm(0, 1)) /
                    (4 * q(0)));
    }

    Quaternion(const Euler<Type> & euler) :
        Vector<Type, 4>()
    {
        Quaternion &q = *this;
        Type cosPhi_2 = Type(cos(euler.phi() / (Type)2.0));
        Type cosTheta_2 = Type(cos(euler.theta() / (Type)2.0));
        Type cosPsi_2 = Type(cos(euler.psi() / (Type)2.0));
        Type sinPhi_2 = Type(sin(euler.phi() / (Type)2.0));
        Type sinTheta_2 = Type(sin(euler.theta() / (Type)2.0));
        Type sinPsi_2 = Type(sin(euler.psi() / (Type)2.0));
        q(0) = cosPhi_2 * cosTheta_2 * cosPsi_2 +
               sinPhi_2 * sinTheta_2 * sinPsi_2;
        q(1) = sinPhi_2 * cosTheta_2 * cosPsi_2 -
               cosPhi_2 * sinTheta_2 * sinPsi_2;
        q(2) = cosPhi_2 * sinTheta_2 * cosPsi_2 +
               sinPhi_2 * cosTheta_2 * sinPsi_2;
        q(3) = cosPhi_2 * cosTheta_2 * sinPsi_2 -
               sinPhi_2 * sinTheta_2 * cosPsi_2;
    }

    Quaternion(Type a, Type b, Type c, Type d) :
        Vector<Type, 4>()
    {
        Quaternion &q = *this;
        q(0) = a;
        q(1) = b;
        q(2) = c;
        q(3) = d;
    }

    Quaternion operator*(const Quaternion &q) const
    {
        const Quaternion &p = *this;
        Quaternion r;
        r(0) = p(0)*q(0) - p(1)*q(1) - p(2)*q(2) - p(3)*q(3);
        r(1) = p(0)*q(1) + p(1)*q(0) - p(2)*q(3) + p(3)*q(2);
        r(2) = p(0)*q(2) + p(1)*q(3) + p(2)*q(0) - p(3)*q(1);
        r(3) = p(0)*q(3) - p(1)*q(2) + p(2)*q(1) + p(3)*q(0);
        return r;
    }

    void operator*=(const Quaternion & other)
    {
        Quaternion &self = *this;
        self = self * other;
    }

    Quaternion operator*(Type scalar) const
    {
        const Quaternion &q = *this;
        return scalar * q;
    }

    void operator*=(Type scalar)
    {
        Quaternion &q = *this;
        q = q * scalar;
    }

    Matrix41 derivative(const Matrix31 & w) const {
        const Quaternion &q = *this;
        Type dataQ[] = {
            q(0), -q(1), -q(2), -q(3),
            q(1),  q(0), -q(3),  q(2),
            q(2),  q(3),  q(0), -q(1),
            q(3), -q(2),  q(1),  q(0)
        };
        Matrix<Type, 4, 4> Q(dataQ);
        Vector<Type, 4> v;
        v(0) = 0;
        v(1) = w(0,0);
        v(2) = w(1,0);
        v(3) = w(2,0);
        return Q * v * Type(0.5);
    }

    void invert() {
        Quaternion &q = *this;
        q(1) *= -1;
        q(2) *= -1;
        q(3) *= -1;
    }

    Quaternion inversed() {
        Quaternion &q = *this;
        Quaternion ret;
        ret(0) = q(0);
        ret(1) = -q(1);
        ret(2) = -q(2);
        ret(3) = -q(3);
        return ret;
    }

    void rotate(const Vector<Type, 3> &vec) {
        Quaternion res;
        res.from_axis_angle(vec);
        (*this) = (*this) * res;
    }

    void from_axis_angle(Vector<Type, 3> vec) {
        Quaternion &q = *this;
        Type theta = vec.norm();
        if(theta < (Type)1e-10) {
            q(0) = (Type)1.0;
            q(1)=q(2)=q(3)=0;
            return;
        }
        vec /= theta;
        from_axis_angle(vec,theta);
    }

    void from_axis_angle(const Vector<Type, 3> &axis, Type theta) {
        Quaternion &q = *this;
        if(theta < (Type)1e-10) {
            q(0) = (Type)1.0;
            q(1)=q(2)=q(3)=0;
        }
        Type magnitude = sinf(theta/2.0f);

        q(0) = cosf(theta/2.0f);
        q(1) = axis(0) * magnitude;
        q(2) = axis(1) * magnitude;
        q(3) = axis(2) * magnitude;
    }

    Vector<Type, 3> to_axis_angle() {
        Quaternion &q = *this;
        Type axis_magnitude = Type(sqrt(q(1) * q(1) + q(2) * q(2) + q(3) * q(3)));
        Vector<Type, 3> vec;
        vec(0) = q(1);
        vec(1) = q(2);
        vec(2) = q(3);
        if(axis_magnitude >= (Type)1e-10) {
            vec = vec / axis_magnitude;
            vec = vec * wrap_pi((Type)2.0 * atan2f(axis_magnitude,q(0)));
        }
        return vec;
    }
};

typedef Quaternion<float> Quatf;
typedef Quaternion<float> Quaternionf;

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
