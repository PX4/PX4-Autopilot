/**
 * @file Dcm.hpp
 *
 * A direction cosine matrix class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"


namespace matrix
{

template<typename Type>
class Quaternion;

template<typename Type>
class Euler;

/**
 * Direction cosine matrix class
 *
 * More elaborate class description
 */
template<typename Type>
class Dcm : public Matrix<Type, 3, 3>
{
public:
    virtual ~Dcm() {};

    typedef Matrix<Type, 3, 1> Vector3;

    /**
     * Standard constructor
     *
     * Initializes to identity
     */
    Dcm() : Matrix<Type, 3, 3>()
    {
        (*this) = eye<Type, 3>();
    }

    /**
     * Constructor from array
     *
     * Copies the given scalar value to all matrix fields.
     *
     * @param other array
     */
    Dcm(const Type *data_) : Matrix<Type, 3, 3>(data_)
    {
    }

    /**
     * Copy constructor
     *
     * More elaborate function description
     *
     * @param other Matrix33 to set dcm to
     */
    Dcm(const Matrix<Type, 3, 3> &other) : Matrix<Type, 3, 3>(other)
    {
    }

    /**
     * Constructor from quaternion
     *
     * More elaborate function description
     *
     * @param q quaternion to set dcm to
     */
    Dcm(const Quaternion<Type> & q) {
        set_from_quaternion(q);
    }

    /**
     * Constructor from euler angles
     *
     * More elaborate function description
     *
     * @param euler euler angles to set dcm to
     */
    Dcm(const Euler<Type> & euler) {
        set_from_euler(euler);
    }


    /**
     * Set from quaternion
     *
     * More elaborate function description
     *
     * @param q quaternion to set dcm to
     */
    void set_from_quaternion(const Quaternion<Type> & q) {
        Dcm &dcm = *this;
        Type a = q(0);
        Type b = q(1);
        Type c = q(2);
        Type d = q(3);
        Type aSq = a*a;
        Type bSq = b*b;
        Type cSq = c*c;
        Type dSq = d*d;
        dcm(0, 0) = aSq + bSq - cSq - dSq;
        dcm(0, 1) = 2 * (b * c - a * d);
        dcm(0, 2) = 2 * (a * c + b * d);
        dcm(1, 0) = 2 * (b * c + a * d);
        dcm(1, 1) = aSq - bSq + cSq - dSq;
        dcm(1, 2) = 2 * (c * d - a * b);
        dcm(2, 0) = 2 * (b * d - a * c);
        dcm(2, 1) = 2 * (a * b + c * d);
        dcm(2, 2) = aSq - bSq - cSq + dSq;
    }

    /**
     * Set from euler angles
     *
     * More elaborate function description
     *
     * @param euler euler angles to set dcm to
     */
    void set_from_euler(const Euler<Type> & euler) {
        Dcm &dcm = *this;
        Type cosPhi = Type(cos(euler.phi()));
        Type sinPhi = Type(sin(euler.phi()));
        Type cosThe = Type(cos(euler.theta()));
        Type sinThe = Type(sin(euler.theta()));
        Type cosPsi = Type(cos(euler.psi()));
        Type sinPsi = Type(sin(euler.psi()));

        dcm(0, 0) = cosThe * cosPsi;
        dcm(0, 1) = -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
        dcm(0, 2) = sinPhi * sinPsi + cosPhi * sinThe * cosPsi;

        dcm(1, 0) = cosThe * sinPsi;
        dcm(1, 1) = cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
        dcm(1, 2) = -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;

        dcm(2, 0) = -sinThe;
        dcm(2, 1) = sinPhi * cosThe;
        dcm(2, 2) = cosPhi * cosThe;
    }

    Vector<Type, 3> vee() const {    // inverse to Vector.hat() operation
        const Dcm &A(*this);
        Vector<Type, 3> v;
        v(0) = -A(1,2);
        v(1) =  A(0,2);
        v(2) = -A(0,1);
        return v;
    }
};

typedef Dcm<float> Dcmf;

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
