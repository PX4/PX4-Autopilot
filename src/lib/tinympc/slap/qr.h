//
// Created by Brian Jackson on 1/22/23.
// Copyright (c) 2023 Robotic Exploration Lab. All rights reserved.
//

#pragma once

#include "matrix.h"

/**
 * @brief Performs QR decomposition
 *
 * Calculates the QR decomposition using Householder reflections.
 * The R factor is stored in the upper triangular portion of A,
 * while the reflection vectors are stored below the diagonal, which
 * can be used to recover the Q matrix.
 *
 * See also: slap_ComputeQ(), slap_Qtb(), slap_LeastSquares()
 *
 * **Header File:** `slap/qr.h`
 * @param A A square or skinny matrix
 * @param[out] betas Stores scaling factors needed to recover Q. Must have
 *                   the same number of rows as A.
 * @param[in] temp A temporary vector with the same number of rows as A.
 */
enum slap_ErrorCode slap_QR(Matrix A, Matrix betas, Matrix temp);

/**
 * @brief Computes the Q matrix from a previously-computed QR decomposition
 *
 * See also: slap_QR(), slap_LeastSquares(), slap_Qtb()
 *
 * **Header File:** `slap/qr.h`
 * @param[out] A A square matrix with the same number of rows as R.
 * @param[in] R A square or skinny matrix containing the results from a QR decomposition.
 * @param[in] betas A vector of scaling factors needed to recover the Q matrix.
 * @param Q_work A temporary matrix of the same size as Q for intermediate calculations
 */
enum slap_ErrorCode slap_ComputeQ(Matrix Q, const Matrix R, const Matrix betas,
                                  Matrix Q_work);

/**
 * @brief Calculates \f$Q^T b\f$ where $Q$ is the orthogonal matrix from a QR decomposition
 *
 * This method is useful for solving linear systems with the QR decomposition, where the
 * right side needs to be multiplied by \f$Q^T\f$.
 *
 * The result is stored in b.
 *
 * See also: slap_QR(), slap_LeastSquares()
 *
 * **Header File:** `slap/qr.h`
 * @param[in] R
 * @param[in] betas Scaling factors for reflections
 * @param[in,out] b Vector to multiply in-place
 */
enum slap_ErrorCode slap_Qtb(const Matrix R, const Matrix betas, Matrix b);

/**
 * @brief Solve a least squares problem
 *
 * Solves \f$ \text{minimize} || A x - b || \f$, where \f$A\f$ has more
 * rows (\f$m\f$) than columns (\f$n\f$).
 *
 * Both `A` and `b` are over-written by this method. `A` is overwritten with the it's QR
 * decomposition, and the first \f$n\f$ rows of `b` contain the solution.
 *
 * See also: slap_QR(), slap_Qtb()
 *
 * **Header File:** `slap/qr.h`
 * @param[in,out] A A "skinny" matrix
 * @param[in,out] b
 * @param[out] betas A vector of scaling factors to recover Q. Must have the same number of
 *                   rows as `A`.
 * @param temp A temporary vector used to compute the QR decomposition of `A`.
 *             Must have the same number of rows as `A`.
 * @return
 */
enum slap_ErrorCode slap_LeastSquares(Matrix A, Matrix b, Matrix betas, Matrix temp);
