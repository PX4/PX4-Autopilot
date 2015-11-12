/**
 * @file Quaternion.hpp
 *
 * A quaternion class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"

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
        Type cosPhi_2 = Type(cos(euler.phi() / 2.0));
        Type cosTheta_2 = Type(cos(euler.theta() / 2.0));
        Type cosPsi_2 = Type(cos(euler.psi() / 2.0));
        Type sinPhi_2 = Type(sin(euler.phi() / 2.0));
        Type sinTheta_2 = Type(sin(euler.theta() / 2.0));
        Type sinPsi_2 = Type(sin(euler.psi() / 2.0));
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
};

typedef Quaternion<float> Quatf;

}; // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
