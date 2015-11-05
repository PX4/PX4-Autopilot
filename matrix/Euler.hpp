/**
 * @file Euler.hpp
 *
 * Euler angle tait-bryan body 3-2-1
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once
#include <Vector.hpp>
#include <Dcm.hpp>
#include <Quaternion.hpp>

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

    Euler(Type phi, Type theta, Type psi) : Vector<Type, 3>()
    {
        this->phi() = phi;
        this->theta() = theta;
        this->psi() = psi;
    }

    Euler(const Dcm<Type> & dcm) {
        Type theta = asin(-dcm(2, 0));
        Type phi = 0;
        Type psi = 0;

        if (fabs(theta - M_PI_2) < 1.0e-3) {
            psi = atan2(dcm(1, 2) - dcm(0, 1),
                        dcm(0, 2) + dcm(1, 1));

        } else if (fabs(theta + M_PI_2) < 1.0e-3) {
            psi = atan2(dcm(1, 2) - dcm(0, 1),
                        dcm(0, 2) + dcm(1, 1));

        } else {
            phi = atan2f(dcm(2, 1), dcm(2, 2));
            psi = atan2f(dcm(1, 0), dcm(0, 0));
        }
        this->phi() = phi;
        this->theta() = theta;
        this->psi() = psi;
    }

    Euler(const Quaternion<Type> & q) {
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

}; // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
