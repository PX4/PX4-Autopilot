/**
 * @file binary_ops.h
 * @author Brian Jackson (bjack205@gmail.com)
 * @copyright Copyright (c) 2022
 * @date 2023-01-30
 *
 * @brief Operations between two matrices
 */
#pragma once

#include "matrix.h"

/**
 * @brief Return the normed difference between 2 matrices of the same size
 *
 * Returns \f$ \sqrt{\sum_{i=0}^{m-1} \sum_{j=0}^{n-1} (A_{ij} - B_{ij})^2 } \f$
 *
 * @param A A matrix of dimension (m,n)
 * @param B A matrix of dimension (m,n)
 * @return NAN if input is invalid, normed difference otherwise
 */
float slap_NormedDifference(Matrix A, Matrix B);

/**
 * @brief Add two matrices, with scaling
 *
 * Calculates \f$C = A + \alpha B\f$
 *
 * Matrices must all be the same size.
 *
 * Any of the matrices can be aliased.
 *
 * # Examples
 * Normal matrix addition
 * ```c
 * slap_MatrixAddition(C, A, B, 1.0);
 * ```
 *
 * Subtract a matrix from another, in-place
 * ```c
 * slap_MatrixAddition(A, A, B, -1);
 * ```
 *
 * @param[out] C Destination matrix
 * @param[in] A Input matrix
 * @param[in] B Input matrix
 * @param alpha Scaling on B
 */
enum slap_ErrorCode slap_MatrixAddition(Matrix C, Matrix A, Matrix B, float alpha);
