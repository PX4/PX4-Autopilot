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
        return sqrt(a.dot(a));
    }

    inline void normalize() {
        (*this) /= norm();
    }

    /**
     * Vector Operations
     */

    Vector<Type, M> operator+(const Vector<Type, M> &other) const
    {
        Vector<Type, M> res;
        const Vector<Type, M> &self = *this;

        for (size_t i = 0; i < M; i++) {
            res(i) = self(i) + other(i);
        }

        return res;
    }

    bool operator==(const Vector<Type, M> &other) const
    {
        Vector<Type, M> res;
        const Vector<Type, M> &self = *this;

        for (size_t i = 0; i < M; i++) {
            if (self(i) != other(i)) {
                return false;
            }
        }

        return true;
    }

    Vector<Type, M> operator-(const Vector<Type, M> &other) const
    {
        Vector<Type, M> res;
        const Vector<Type, M> &self = *this;

        for (size_t i = 0; i < M; i++) {
            res(i) = self(i) - other(i);
        }

        return res;
    }

    void operator+=(const Vector<Type, M> &other)
    {
        Vector<Type, M> &self = *this;
        self = self + other;
    }

    void operator-=(const Vector<Type, M> &other)
    {
        Vector<Type, M> &self = *this;
        self = self - other;
    }

    /**
     * Scalar Operations
     */

    Vector<Type, M> operator*(Type scalar) const
    {
        Vector<Type, M> res;
        const Vector<Type, M> &self = *this;

        for (size_t i = 0; i < M; i++) {
            res(i) = self(i) * scalar;
        }

        return res;
    }

    Vector<Type, M> operator/(Type scalar) const
    {
        return (*this)*(1.0/scalar);
    }

    Vector<Type, M> operator+(Type scalar) const
    {
        Vector<Type, M> res;
        const Vector<Type, M> &self = *this;

        for (size_t i = 0; i < M; i++) {
            res(i) = self(i) + scalar;
        }

        return res;
    }

    void operator*=(Type scalar)
    {
        Vector<Type, M> &self = *this;

        for (size_t i = 0; i < M; i++) {
            self(i) = self(i) * scalar;
        }
    }

    void operator/=(Type scalar)
    {
        Vector<Type, M> &self = *this;
        self = self * (1.0f / scalar);
    }

};

}; // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
