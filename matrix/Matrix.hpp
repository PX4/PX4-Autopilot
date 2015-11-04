/**
 * @file Matrix.hpp
 *
 * A simple matrix template library.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "Vector.hpp"

namespace matrix
{

template <typename Type, size_t M>
class Vector;

template<typename Type, size_t  M, size_t N>
class Matrix
{

protected:
    Type _data[M][N];

public:

    virtual ~Matrix() {};

    Matrix() :
        _data()
    {
    }

    Matrix(const Type *data) :
        _data()
    {
        memcpy(_data, data, sizeof(_data));
    }

    Matrix(const Matrix &other) :
        _data()
    {
        memcpy(_data, other._data, sizeof(_data));
    }

    /**
     * Accessors/ Assignment etc.
     */

    Type *data()
    {
        return _data[0];
    }

    inline Type operator()(size_t i, size_t j) const
    {
        return _data[i][j];
    }

    inline Type &operator()(size_t i, size_t j)
    {
        return _data[i][j];
    }

    /**
     * Matrix Operations
     */

    // this might use a lot of programming memory
    // since it instantiates a class for every
    // required mult pair, but it provides
    // compile time size_t checking
    template<size_t P>
    Matrix<Type, M, P> operator*(const Matrix<Type, N, P> &other) const
    {
        const Matrix<Type, M, N> &self = *this;
        Matrix<Type, M, P> res;
        res.setZero();

        for (size_t i = 0; i < M; i++) {
            for (size_t k = 0; k < P; k++) {
                for (size_t j = 0; j < N; j++) {
                    res(i, k) += self(i, j) * other(j, k);
                }
            }
        }

        return res;
    }

    Matrix<Type, M, N> operator+(const Matrix<Type, M, N> &other) const
    {
        Matrix<Type, M, N> res;
        const Matrix<Type, M, N> &self = *this;

        for (size_t i = 0; i < M; i++) {
            for (size_t j = 0; j < N; j++) {
                res(i , j) = self(i, j) + other(i, j);
            }
        }

        return res;
    }

    bool operator==(const Matrix<Type, M, N> &other) const
    {
        Matrix<Type, M, N> res;
        const Matrix<Type, M, N> &self = *this;

        for (size_t i = 0; i < M; i++) {
            for (size_t j = 0; j < N; j++) {
                if (self(i , j) != other(i, j)) {
                    return false;
                }
            }
        }

        return true;
    }

    Matrix<Type, M, N> operator-(const Matrix<Type, M, N> &other) const
    {
        Matrix<Type, M, N> res;
        const Matrix<Type, M, N> &self = *this;

        for (size_t i = 0; i < M; i++) {
            for (size_t j = 0; j < N; j++) {
                res(i , j) = self(i, j) - other(i, j);
            }
        }

        return res;
    }

    void operator+=(const Matrix<Type, M, N> &other)
    {
        Matrix<Type, M, N> &self = *this;
        self = self + other;
    }

    void operator-=(const Matrix<Type, M, N> &other)
    {
        Matrix<Type, M, N> &self = *this;
        self = self - other;
    }

    void operator*=(const Matrix<Type, M, N> &other)
    {
        Matrix<Type, M, N> &self = *this;
        self = self * other;
    }

    /**
     * Scalar Operations
     */

    Matrix<Type, M, N> operator*(Type scalar) const
    {
        Matrix<Type, M, N> res;
        const Matrix<Type, M, N> &self = *this;

        for (size_t i = 0; i < M; i++) {
            for (size_t j = 0; j < N; j++) {
                res(i , j) = self(i, j) * scalar;
            }
        }

        return res;
    }

    Matrix<Type, M, N> operator+(Type scalar) const
    {
        Matrix<Type, M, N> res;
        const Matrix<Type, M, N> &self = *this;

        for (size_t i = 0; i < M; i++) {
            for (size_t j = 0; j < N; j++) {
                res(i , j) = self(i, j) + scalar;
            }
        }

        return res;
    }

    void operator*=(Type scalar)
    {
        Matrix<Type, M, N> &self = *this;

        for (size_t i = 0; i < M; i++) {
            for (size_t j = 0; j < N; j++) {
                self(i, j) = self(i, j) * scalar;
            }
        }
    }

    void operator/=(Type scalar)
    {
        Matrix<Type, M, N> &self = *this;
        self = self * (1.0f / scalar);
    }

    /**
     * Misc. Functions
     */

    void print()
    {
        Matrix<Type, M, N> &self = *this;
        printf("\n");

        for (size_t i = 0; i < M; i++) {
            printf("[");

            for (size_t j = 0; j < N; j++) {
                printf("%10g\t", double(self(i, j)));
            }

            printf("]\n");
        }
    }

    Matrix<Type, N, M> transpose() const
    {
        Matrix<Type, N, M> res;
        const Matrix<Type, M, N> &self = *this;

        for (size_t i = 0; i < M; i++) {
            for (size_t j = 0; j < N; j++) {
                res(j, i) = self(i, j);
            }
        }

        return res;
    }


    // tranpose alias
    inline Matrix<Type, N, M> T() const
    {
        return transpose();
    }

    Matrix<Type, M, M> expm(float dt, size_t n) const
    {
        Matrix<float, M, M> res;
        res.setIdentity();
        Matrix<float, M, M> A_pow = *this;
        float k_fact = 1;
        size_t k = 1;

        while (k < n) {
            res += A_pow * (float(pow(dt, k)) / k_fact);

            if (k == n) {
                break;
            }

            A_pow *= A_pow;
            k_fact *= k;
            k++;
        }

        return res;
    }

    Vector<Type, M> diagonal() const
    {
        Vector<Type, M> res;
        // force square for now
        const Matrix<Type, M, M> &self = *this;

        for (size_t i = 0; i < M; i++) {
            res(i) = self(i, i);
        }

        return res;
    }

    void setZero()
    {
        memset(_data, 0, sizeof(_data));
    }

    void setIdentity()
    {
        setZero();
        Matrix<Type, M, N> &self = *this;

        for (size_t i = 0; i < M and i < N; i++) {
            self(i, i) = 1;
        }
    }

    inline void swapRows(size_t a, size_t b)
    {
        if (a == b) {
            return;
        }

        Matrix<Type, M, N> &self = *this;

        for (size_t j = 0; j < N; j++) {
            Type tmp = self(a, j);
            self(a, j) = self(b, j);
            self(b, j) = tmp;
        }
    }

    inline void swapCols(size_t a, size_t b)
    {
        if (a == b) {
            return;
        }

        Matrix<Type, M, N> &self = *this;

        for (size_t i = 0; i < M; i++) {
            Type tmp = self(i, a);
            self(i, a) = self(i, b);
            self(i, b) = tmp;
        }
    }

    /**
     * inverse based on LU factorization with partial pivotting
     */
    Matrix <Type, M, M> inverse() const
    {
        Matrix<Type, M, M> L;
        L.setIdentity();
        const Matrix<Type, M, M> &A = (*this);
        Matrix<Type, M, M> U = A;
        Matrix<Type, M, M> P;
        P.setIdentity();

        //printf("A:\n"); A.print();

        // for all diagonal elements
        for (size_t n = 0; n < N; n++) {

            // if diagonal is zero, swap with row below
            if (fabsf(U(n, n)) < 1e-8f) {
                //printf("trying pivot for row %d\n",n);
                for (size_t i = 0; i < N; i++) {
                    if (i == n) {
                        continue;
                    }

                    //printf("\ttrying row %d\n",i);
                    if (fabsf(U(i, n)) > 1e-8f) {
                        //printf("swapped %d\n",i);
                        U.swapRows(i, n);
                        P.swapRows(i, n);
                    }
                }
            }

#ifdef MATRIX_ASSERT
            //printf("A:\n"); A.print();
            //printf("U:\n"); U.print();
            //printf("P:\n"); P.print();
            //fflush(stdout);
            ASSERT(fabsf(U(n, n)) > 1e-8f);
#endif

            // failsafe, return zero matrix
            if (fabsf(U(n, n)) < 1e-8f) {
                Matrix<Type, M, M> zero;
                zero.setZero();
                return zero;
            }

            // for all rows below diagonal
            for (size_t i = (n + 1); i < N; i++) {
                L(i, n) = U(i, n) / U(n, n);

                // add i-th row and n-th row
                // multiplied by: -a(i,n)/a(n,n)
                for (size_t k = n; k < N; k++) {
                    U(i, k) -= L(i, n) * U(n, k);
                }
            }
        }

        //printf("L:\n"); L.print();
        //printf("U:\n"); U.print();

        // solve LY=P*I for Y by forward subst
        Matrix<Type, M, M> Y = P;

        // for all columns of Y
        for (size_t c = 0; c < N; c++) {
            // for all rows of L
            for (size_t i = 0; i < N; i++) {
                // for all columns of L
                for (size_t j = 0; j < i; j++) {
                    // for all existing y
                    // subtract the component they
                    // contribute to the solution
                    Y(i, c) -= L(i, j) * Y(j, c);
                }

                // divide by the factor
                // on current
                // term to be solved
                // Y(i,c) /= L(i,i);
                // but L(i,i) = 1.0
            }
        }

        //printf("Y:\n"); Y.print();

        // solve Ux=y for x by back subst
        Matrix<Type, M, M> X = Y;

        // for all columns of X
        for (size_t c = 0; c < N; c++) {
            // for all rows of U
            for (size_t k = 0; k < N; k++) {
                // have to go in reverse order
                size_t i = N - 1 - k;

                // for all columns of U
                for (size_t j = i + 1; j < N; j++) {
                    // for all existing x
                    // subtract the component they
                    // contribute to the solution
                    X(i, c) -= U(i, j) * X(j, c);
                }

                // divide by the factor
                // on current
                // term to be solved
                X(i, c) /= U(i, i);
            }
        }

        //printf("X:\n"); X.print();
        return X;
    }

    // inverse alias
    inline Matrix<Type, N, M> I() const
    {
        return inverse();
    }

};


typedef Matrix<float, 3, 3> Matrix3f;

}; // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
