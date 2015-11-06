/**
 * @file Vector3.hpp
 *
 * 3D vector class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "matrix.hpp"

namespace matrix
{

template <typename Type, size_t M>
class Vector;

template<typename Type>
class Vector3 : public Vector<Type, 3>
{
public:
    virtual ~Vector3() {};

    Vector3() :
        Vector<Type, 3>()
    {
    }

    Vector3(const Vector<Type, 3> & other) :
        Vector<Type, 3>(other)
    {
    }

    Vector3(const Matrix<Type, 3, 1> & other) :
        Vector<Type, 3>(other)
    {
    }

    Vector3(const Type *data_) :
        Vector<Type, 3>(data_)
    {
    }

    Vector3(Type x, Type y, Type z) : Vector<Type, 3>()
    {
        Vector3 &v(*this);
        v(0) = x;
        v(1) = y;
        v(2) = z;
    }

    Vector3 cross(const Vector3 & b)  const {
        const Vector3 &a(*this);
        Vector3 c;
        c(0) = a(1)*b(2) - a(2)*b(1);
        c(1) = -a(0)*b(2) + a(2)*b(0);
        c(2) = a(0)*b(1) - a(1)*b(0);
        return c;
    }
};

typedef Vector3<float> Vector3f;

}; // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
