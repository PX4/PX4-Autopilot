/**
 * @file Euler.hpp
 *
 * Euler angle tait-bryan body 3-2-1
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
class Quaternion;

template<typename Type>
class Euler : public Vector<Type, 3>
{
public:
    virtual ~Euler() {};

    Euler() : Vector<Type, 3>()
    {
    }

    Euler(const Vector<Type, 3> & other) :
        Vector<Type, 3>(other)
    {
    }

    Euler(const Matrix<Type, 3, 1> & other) :
        Vector<Type, 3>(other)
    {
    }

    Euler(Type phi_, Type theta_, Type psi_) : Vector<Type, 3>()
    {
        phi() = phi_;
        theta() = theta_;
        psi() = psi_;
    }

    Euler(const Dcm<Type> & dcm) : Vector<Type, 3>()
    {
        Type phi_val = Type(atan2(dcm(2,1), dcm(2,2)));
        Type theta_val = Type(asin(-dcm(2,0)));
        Type psi_val = Type(atan2(dcm(1,0), dcm(0,0)));
        Type pi = Type(M_PI);

        if (fabs(theta_val - pi/2) < 1.0e-3) {
            phi_val = Type(0.0);
            psi_val = Type(atan2(dcm(1,2), dcm(0,2)));
        } else if (Type(fabs(theta_val + pi/2)) < Type(1.0e-3)) {
            phi_val = Type(0.0);
            psi_val = Type(atan2(-dcm(1,2), -dcm(0,2)));
        }

        phi() = phi_val;
        theta() = theta_val;
        psi() = psi_val;
    }

    Euler(const Quaternion<Type> & q) :
        Vector<Type, 3>()
    {
        *this = Euler(Dcm<Type>(q));
    }

    inline Type phi() const {
        return (*this)(0);
    }
    inline Type theta() const {
        return (*this)(1);
    }
    inline Type psi() const {
        return (*this)(2);
    }

    inline Type & phi() {
        return (*this)(0);
    }
    inline Type & theta() {
        return (*this)(1);
    }
    inline Type & psi() {
        return (*this)(2);
    }

};

typedef Euler<float> Eulerf;

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
