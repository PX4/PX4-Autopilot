/**
 * @file Vector.hpp
 *
 * Vector class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"

namespace matrix
{

template <typename Type, size_t M, size_t N>
class Matrix;

template<typename Type, size_t M>
class Vector : public Matrix<Type, M, 1>
{
public:
    typedef Matrix<Type, M, 1> MatrixM1;

    Vector() = default;

    Vector(const MatrixM1 & other) :
        MatrixM1(other)
    {
    }

    Vector(const Type data_[M]) :
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

    inline Type operator*(const MatrixM1 & b) const {
        const Vector &a(*this);
        return a.dot(b);
    }

    inline Vector operator*(float b) const {
        return Vector(MatrixM1::operator*(b));
    }

    Type norm() const {
        const Vector &a(*this);
        return Type(sqrt(a.dot(a)));
    }

    Type norm_squared() const {
        const Vector &a(*this);
        return a.dot(a);
    }

    inline Type length() const {
        return norm();
    }

    inline void normalize() {
        (*this) /= norm();
    }

    Vector unit() const {
        return (*this) / norm();
    }

    Vector unit_or_zero(const Type eps = Type(1e-5)) {
        const Type n = norm();
        if (n > eps) {
            return (*this) / n;
        }
        return Vector();
    }

    inline Vector normalized() const {
        return unit();
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
