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
    virtual ~Vector() {};

    Vector() : Matrix<Type, M, 1>()
    {
    }

    Vector(const Vector<Type, M> & other) :
        Matrix<Type, M, 1>(other)
    {
    }

    Vector(const Matrix<Type, M, 1> & other) :
        Matrix<Type, M, 1>(other)
    {
    }

    Vector(const Type *data_) :
        Matrix<Type, M, 1>(data_)
    {
    }

    inline Type operator()(size_t i) const
    {
        const Matrix<Type, M, 1> &v = *this;
        return v(i, 0);
    }

    inline Type &operator()(size_t i)
    {
        Matrix<Type, M, 1> &v = *this;
        return v(i, 0);
    }

    Type dot(const Vector & b) const {
        const Vector &a(*this);
        Type r = 0;
        for (int i = 0; i<M; i++) {
            r += a(i)*b(i);
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
};

}; // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
