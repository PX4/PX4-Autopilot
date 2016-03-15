/**
 * @file Vector.hpp
 *
 * Vector class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include <cmath>

#include "math.hpp"

namespace matrix
{

template <typename Type, size_t M, size_t N>
class Matrix;

template<typename Type, size_t M>
class Vector : public Matrix<Type, M, 1>
{
public:
    virtual ~Vector() {};

    typedef Matrix<Type, M, 1> MatrixM1;

    Vector() : MatrixM1()
    {
    }

    Vector(const MatrixM1 & other) :
        MatrixM1(other)
    {
    }

    Vector(const Type *data_) :
        MatrixM1(data_)
    {
    }

    inline Type operator()(size_t i) const
    {
        const MatrixM1 &v = *this;
        return v(i, 0);
    }

    inline Type &operator()(size_t i)
    {
        MatrixM1 &v = *this;
        return v(i, 0);
    }

    Type dot(const MatrixM1 & b) const {
        const Vector &a(*this);
        Type r = 0;
        for (size_t i = 0; i<M; i++) {
            r += a(i)*b(i,0);
        }
        return r;
    }

    Type norm() const {
        const Vector &a(*this);
        return Type(sqrt(a.dot(a)));
    }

    inline void normalize() {
        (*this) /= norm();
    }

    Vector unit() const {
        return (*this) / norm();
    }

    Vector pow(Type v) const {
        const Vector &a(*this);
        Vector r;
        for (size_t i = 0; i<M; i++) {
            r(i) = Type(::pow(a(i), v));
        }
        return r;
    }
};

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
