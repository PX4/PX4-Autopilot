/**
 * @file LeastSquaresSolver.hpp
 *
 * Least Squares Solver using QR householder decomposition.
 * It calculates x for Ax = b.
 * A = Q*R
 * where R is an upper triangular matrix.
 *
 * R*x = Q^T*b
 * This is efficiently solved for x because of the upper triangular property of R.
 *
 * @author Bart Slinger <bartslinger@gmail.com>
 */

#pragma once

#include "math.hpp"

namespace matrix {

template<typename Type, size_t M, size_t N>
class LeastSquaresSolver
{
public:

    /**
     * @brief Class calculates QR decomposition which can be used for linear
     * least squares
     * @param A Matrix of size MxN
     *
     * Initialize the class with a MxN matrix. The constructor starts the
     * QR decomposition. This class does not check the rank of the matrix.
     * The user needs to make sure that rank(A) = N and N >= M.
     */
    LeastSquaresSolver(Matrix<Type, M, N> A)
    {
        if (M < N) {
            return;
        }
        // Copy contentents of matrix A
        _A = A;

        for (size_t j = 0; j < N; j++) {
            float normx = 0.0f;
            for (size_t i = j; i < M; i++) {
                normx += _A(i,j) * _A(i,j);
            }
            normx = sqrt(normx);
            float s = _A(j,j) > 0 ? -1.0f : 1.0f;
            float u1 = _A(j,j) - s*normx;
            // prevent divide by zero
            // also covers u1. normx is never negative
            if (normx < 1e-8f) {
                break;
            }
            float w[M] = {};
            w[0] = 1.0f;
            for (size_t i = j+1; i < M; i++) {
                w[i-j] = _A(i,j) / u1;
                _A(i,j) = w[i-j];
            }
            _A(j,j) = s*normx;
            _tau(j) = -s*u1/normx;

            for (size_t k = j+1; k < N; k++) {
                float tmp = 0.0f;
                for (size_t i = j; i < M; i++) {
                    tmp += w[i-j] * _A(i,k);
                }
                for (size_t i = j; i < M; i++) {
                    _A(i,k) -= _tau(j) * w[i-j] * tmp;
                }
            }

        }
    }

    /**
     * @brief qtb Calculate Q^T * b
     * @param b
     * @return Q^T*b
     *
     * This function calculates Q^T * b. This is useful for the solver
     * because R*x = Q^T*b.
     */
    Vector<Type, M> qtb(Vector<Type, M> b) {
        Vector<Type, M> qtbv = b;

        for (size_t j = 0; j < N; j++) {
            float w[M];
            w[0] = 1.0f;
            // fill vector w
            for (size_t i = j+1; i < M; i++) {
                w[i-j] = _A(i,j);
            }
            float tmp = 0.0f;
            for (size_t i = j; i < M; i++) {
                tmp += w[i-j] * qtbv(i);
            }

            for (size_t i = j; i < M; i++) {
                qtbv(i) -= _tau(j) * w[i-j] * tmp;
            }
        }
        return qtbv;
    }

    /**
     * @brief Solve Ax=b for x
     * @param b
     * @return Vector x
     *
     * Find x in the equation Ax = b.
     * A is provided in the initializer of the class.
     */
    Vector<Type, N> solve(Vector<Type, M> b) {
        Vector<Type, M> qtbv = qtb(b);
        Vector<Type, N> x;

        for (size_t l = N; l > 0 ; l--) {
            size_t i = l - 1;
            x(i) = qtbv(i);
            for (size_t r = i+1; r < N; r++) {
                x(i) -= _A(i,r) * x(r);
            }
            // divide by zero, return vector of zeros
            if (fabs(_A(i,i)) < 1e-8f) {
                for (size_t z = 0; z < N; z++) {
                    x(z) = 0.0f;
                }
                break;
            }
            x(i) = x(i) / _A(i,i);
        }
        return x;
    }

private:
    Matrix<Type, M, N> _A;
    Vector<Type, N> _tau;

};

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
