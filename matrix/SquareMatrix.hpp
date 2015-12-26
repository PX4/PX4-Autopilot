/**
 * @file SquareMatrix.hpp
 *
 * A square matrix
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "math.hpp"

namespace matrix
{

template <typename Type, size_t M, size_t N>
class Matrix;

template <typename Type, size_t M>
class Vector;

template<typename Type, size_t  M>
class SquareMatrix : public Matrix<Type, M, M>
{
public:
    SquareMatrix() :
        Matrix<Type, M, M>()
    {
    }

    SquareMatrix(const Type *data_) :
        Matrix<Type, M, M>(data_)
    {
    }

    SquareMatrix(const Matrix<Type, M, M> &other) :
        Matrix<Type, M, M>(other)
    {
    }

    // inverse alias
    inline SquareMatrix<Type, M> I() const
    {
        return inv(*this);
    }

    Vector<Type, M> diag() const
    {
        Vector<Type, M> res;
        const SquareMatrix<Type, M> &self = *this;

        for (size_t i = 0; i < M; i++) {
            res(i) = self(i, i);
        }
        return res;
    }

};

typedef SquareMatrix<float, 3> SquareMatrix3f;

template<typename Type, size_t M>
SquareMatrix<Type, M> eye() {
    SquareMatrix<Type, M> m;
    m.setIdentity();
    return m;
}

template<typename Type, size_t M>
SquareMatrix<Type, M> diag(Vector<Type, M> d) {
    SquareMatrix<Type, M> m;
    for (size_t i=0; i<M; i++) {
        m(i,i) = d(i);
    }
    return m;
}

template<typename Type, size_t M>
SquareMatrix<Type, M> expm(const Matrix<Type, M, M> & A, size_t order=5)
{
    SquareMatrix<Type, M> res;
    SquareMatrix<Type, M> A_pow = A;
    res.setIdentity();
    size_t i_factorial = 1;
    for (size_t i=1; i<=order; i++) {
        i_factorial *= i;
        res += A_pow / Type(i_factorial);
        A_pow *= A_pow;
    }

    return res;
}

/**
 * inverse based on LU factorization with partial pivotting
 */
template<typename Type, size_t M>
SquareMatrix <Type, M> inv(const SquareMatrix<Type, M> & A)
{
    SquareMatrix<Type, M> L;
    L.setIdentity();
    SquareMatrix<Type, M> U = A;
    SquareMatrix<Type, M> P;
    P.setIdentity();

    //printf("A:\n"); A.print();

    // for all diagonal elements
    for (size_t n = 0; n < M; n++) {

        // if diagonal is zero, swap with row below
        if (fabsf(U(n, n)) < 1e-8f) {
            //printf("trying pivot for row %d\n",n);
            for (size_t i = 0; i < M; i++) {
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
            SquareMatrix<Type, M> zero;
            zero.setZero();
            return zero;
        }

        // for all rows below diagonal
        for (size_t i = (n + 1); i < M; i++) {
            L(i, n) = U(i, n) / U(n, n);

            // add i-th row and n-th row
            // multiplied by: -a(i,n)/a(n,n)
            for (size_t k = n; k < M; k++) {
                U(i, k) -= L(i, n) * U(n, k);
            }
        }
    }

    //printf("L:\n"); L.print();
    //printf("U:\n"); U.print();

    // solve LY=P*I for Y by forward subst
    SquareMatrix<Type, M> Y = P;

    // for all columns of Y
    for (size_t c = 0; c < M; c++) {
        // for all rows of L
        for (size_t i = 0; i < M; i++) {
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
    SquareMatrix<Type, M> X = Y;

    // for all columns of X
    for (size_t c = 0; c < M; c++) {
        // for all rows of U
        for (size_t k = 0; k < M; k++) {
            // have to go in reverse order
            size_t i = M - 1 - k;

            // for all columns of U
            for (size_t j = i + 1; j < M; j++) {
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



} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
