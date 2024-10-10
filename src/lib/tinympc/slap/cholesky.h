/**
 * @file cholesky.h
 * @author Brian Jackson (bjack205@gmail.com)
 * @brief Methods for solving systems of equations using a Cholesky dcomposition
 * @date 2022-01-30
 *
 * @copyright Copyright (c) 2022
 *
 * @addtogroup LinearAlgebra
 * @{
 */

#pragma once

#include "matrix.h"

/**
 * @brief Perform a Cholesky decomposition
 *
 * Performs a Cholesky decomposition on the square matrix @p A, storing the result in the
 * lower triangular portion of @p A.
 *
 * **Header File:** `slap/linalg.h`
 * @param  A a square symmetric matrix
 * @return slap error code. There is a dedicated code SLAP_CHOLESKY_FAIL if the
 * factorization fails due to a negative value on the diagonal (matrix isn't positive
 * definite).
 */
enum slap_ErrorCode slap_Cholesky(Matrix A);


/**
 * @brief Solve a linear system of equation with a precomputed Cholesky decomposition.
 *
 * Here's a simple derivation of how the decomposition works:
 * \f{align}{
 * &Ax = b \\
 * &L L^T x = b \\
 * &\implies y = L^{-1} b \;\text{ solved using back-substitution} \\
 * &L^T x = y \\
 * &\implies x = L^{-T} y \;\text{ solved using back-substitution} \\
 * \f}
 *
 * # Example
 * ```c
 * enum slap_ErrorCode err;
 * err = slap_Cholesky(A);
 * if (err == SLAP_CHOLESKY_FAIL) {
 *   printf("Matrix not positive definite!\n);
 * }
 * slap_CholeskySolve(A, x);
 * ```
 *
 * **Header File:** `slap/linalg.h`
 * @param[in]    A A square matrix whose Cholesky decomposition is stored in the lower
 *               triangular portion of the matrix
 * @param[inout] b The right-hand-side vector. Stores the solution upon completion of the
 *               function.
 * @return slap error code
 */
enum slap_ErrorCode slap_CholeskySolve(Matrix A, Matrix b);
