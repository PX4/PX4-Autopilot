/**
 * @file Vector.hpp
 *
 * Vector class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once
#include <Matrix.hpp>

namespace matrix
{

template<typename Type, size_t N>
class Vector : public Matrix<Type, N, 1>
{
public:
    virtual ~Vector() {};

    Vector() : Matrix<Type, N, 1>()
    {
    }

    inline Type operator()(size_t i) const
    {
        const Matrix<Type, N, 1> &v = *this;
        return v(i, 0);
    }

    inline Type &operator()(size_t i)
    {
        Matrix<Type, N, 1> &v = *this;
        return v(i, 0);
    }

    Type dot(const Vector & b) {
        Vector &a(*this);
        Type r = 0;
        for (int i = 0; i<3; i++) {
            r += a(i)*b(i);
        }
        return r;
    }

    Type norm() {
        Vector &a(*this);
        return sqrt(a.dot(a));
    }
};

}; // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
