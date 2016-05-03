/**
 * @file Euler.hpp
 *
 * Euler angle tait-bryan body 3-2-1
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"

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
 * Euler angles class
 *
 * More elaborate class description
 */
template<typename Type>
class Euler : public Vector<Type, 3>
{
public:
    virtual ~Euler() {};

    /**
     * Standard constructor
     *
     * More elaborate function description
     */
    Euler() : Vector<Type, 3>()
    {
    }

    /**
     * Copy constructor
     *
     * More elaborate function description
     *
     * @param other vector to copy
     */
    Euler(const Vector<Type, 3> & other) :
        Vector<Type, 3>(other)
    {
    }

    /**
     * Constructor from Matrix31
     *
     * More elaborate function description
     *
     * @param other Matrix31 to copy
     */
    Euler(const Matrix<Type, 3, 1> & other) :
        Vector<Type, 3>(other)
    {
    }

    /**
     * Constructor from euler angles
     *
     * More elaborate function description
     *
     * @param phi_   roll
     * @param theta_ pitch
     * @param psi_   yaw
     */
    Euler(Type phi_, Type theta_, Type psi_) : Vector<Type, 3>()
    {
        set_from_euler(phi_, theta_, psi_);
    }

    /**
     * Constructor from dcm
     *
     * More elaborate function description
     *
     * @param dcm_ dcm to set angles to
     */
    Euler(const Dcm<Type> & dcm) : Vector<Type, 3>()
    {
        set_from_dcm(dcm);
    }

    /**
     * Constructor from quaternion
     *
     * More elaborate function description
     *
     * @param q quaternion to set angles to
     */
    Euler(const Quaternion<Type> & q) :
        Vector<Type, 3>()
    {
        set_from_quaternion(q);
    }

    /**
     * Set from euler angles
     *
     * More elaborate function description
     *
     * @param phi_   roll
     * @param theta_ pitch
     * @param psi_   yaw
     */
    void set_from_euler(Type phi_, Type theta_, Type psi_)
    {
        phi() = phi_;
        theta() = theta_;
        psi() = psi_;
    }

    /**
     * Set from dcm
     *
     * More elaborate function description
     *
     * @param dcm_ dcm to set angles to
     */
    void set_from_dcm(const Dcm<Type> & dcm)
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

    /**
     * Set from dcm
     *
     * More elaborate function description
     *
     * @param q quaternion to set angles to
     */
    void set_from_quaternion(const Quaternion<Type> & q)
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
