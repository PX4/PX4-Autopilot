//
// Created by Brian Jackson on 3/4/23.
// Copyright (c) 2023 Robotic Exploration Lab. All rights reserved.
//

#pragma once

#include "matrix.h"

/**
 * @brief Multiply a matrix by an upper triangular matrix and add to another matrix.
 *
 * Simply ignores entries in the lower triangle of U.
 *
 * @param C Output matrix (m x p)
 * @param U Upper triangular matrix (m x n)
 * @param B Input matrix (n x p)
 * @param alpha Scaling factor for U*B
 * @param beta Scaling factor for C
 */
enum slap_ErrorCode slap_UpperTriMulAdd(Matrix C, const Matrix U, const Matrix B,
                                        double alpha, double beta);

/**
 * @brief Multiply a matrix by a lower triangular matrix and add to another matrix.
 *
 * Simply ignores entries in the upper triangle of L.
 *
 * @param C Output matrix (m x p)
 * @param L Lower triangular matrix (m x n)
 * @param B Input matrix (n x p)
 * @param alpha Scaling factor for L*B
 * @param beta Scaling factor for C
 */
enum slap_ErrorCode slap_LowerTriMulAdd(Matrix C, const Matrix L, const Matrix B,
                                        double alpha, double beta);

/**
 * @brief Check if a matrix is upper triangular.
 */
bool slap_CheckUpperTri(const Matrix A);

/**
 * @brief Check if a matrix is lower triangular.
 */
bool slap_CheckLowerTri(const Matrix A);

/**
 * @brief Make a matrix upper triangular.
 *
 * Sets all of the entries below the diagonal to zero.
 *
 * @param A Matrix to make upper triangular. Can be square or rectangular.
 */
enum slap_ErrorCode slap_MakeUpperTri(Matrix A);

/**
 * @brief Make a matrix lower triangular.
 *
 * Sets all of the entries above the diagonal to zero.
 *
 * @param A Matrix to make lower triangular. Can be square or rectangular.
 */
enum slap_ErrorCode slap_MakeLowerTri(Matrix A);

/**
 * @brief Solve a linear system of equation for a triangular matrix
 *
 * Uses back-substitution to solve a system of equations of the following form:
 * \f[
 *  L x = b
 * \f]
 * for a lower-triangular matrix \f$ L \f$, or
 * \f[
 *  L^T x = b
 * \f]
 * if `slap_IsTransposed(L)` is true, or
 * \f[
 * R x = b
 * \f]
 * if `slap_GetType(L) == slap_TRIANGULAR_UPPER`.
 *
 * **Header File:** `slap/linalg.h`
 * @param[in]          L A triangular matrix. Assumed to be lower-triangular unless its
 *                       slap_MatrixType is slap_TRIANGULAR_UPPER
 * @param[inout]       b The right-hand-side vector. Stores the solution upon completion.
 * @return slap error code
 */
enum slap_ErrorCode slap_TriSolve(Matrix L, Matrix b);
