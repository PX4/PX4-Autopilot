/**
 * @file Vector2.hpp
 *
 * 2D vector class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"

namespace matrix
{

template <typename Type, size_t M>
class Vector;

template<typename Type>
class Vector2 : public Vector<Type, 2>
{
public:

    typedef Matrix<Type, 2, 1> Matrix21;
    typedef Vector<Type, 3> Vector3;

    Vector2() = default;

    Vector2(const Matrix21 & other) :
        Vector<Type, 2>(other)
    {
    }

    explicit Vector2(const Type data_[2]) :
        Vector<Type, 2>(data_)
    {
    }

    Vector2(Type x, Type y)
    {
        Vector2 &v(*this);
        v(0) = x;
        v(1) = y;
    }

    template<size_t P, size_t Q>
    Vector2(const Slice<Type, 2, 1, P, Q>& slice_in) : Vector<Type, 2>(slice_in)
    {
    }

    explicit Vector2(const Vector3 & other)
    {
        Vector2 &v(*this);
        v(0) = other(0);
        v(1) = other(1);
    }

    Type cross(const Matrix21 & b) const {
        const Vector2 &a(*this);
        return a(0)*b(1, 0) - a(1)*b(0, 0);
    }

    Type operator%(const Matrix21 & b) const {
        return (*this).cross(b);
    }

};

typedef Vector2<float> Vector2f;

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
