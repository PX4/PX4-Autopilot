//
// Created by Brian Jackson on 12/19/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#pragma once

#include "matrix.h"

/**
 * @brief Perform in-place Matrix multiplication with addition
 *
 * \f[
 * C = \beta C + \alpha A B
 * \f]
 *
 * The two input matrices can be aliased, but neither can be aliased with the output, doing
 * so will result in undefined behavior.
 *
 * # Examples
 * Normal matrix multiplication:
 * ```c
 * slap_MatMulAdd(C, A, B, 1, 0);
 * ```
 *
 * With transpose:
 * ```c
 * slap_MatMulAdd(C, slap_Transpose(A), B, 1, 0);
 * ```
 *
 * Add result to the output, with aliasing
 * ```c
 * slap_MatMullAdd(C, A, slap_Transpose(A), 1, 1);
 * ```
 *
 * Transposed output and input scaling
 * ```c
 * slap_MatMulAdd(slap_Transpose(C), A, B, 0.5, 1);
 * ```
 *
 * See also: slap_MatMulAB(), slap_MatMulAtB()
 *
 * **Header File:** `slap/linalg.h`
 * @param C Destination matrix
 * @param[in] A Left input matrix
 * @param[in] B Right input matrix
 * @param[in] alpha Scaling factor on output
 * @param[in] beta Scaling factor on input
 */

// #ifdef SLAP_BACKEND_EIGEN
// #ifdef __cplusplus
// extern "C" {
// #endif
// enum slap_ErrorCode slap_MatMulAdd(Matrix C, Matrix A, Matrix B, float alpha, float beta);
// #ifdef __cplusplus
// }
// #endif
// #else
enum slap_ErrorCode slap_MatMulAdd(Matrix C, Matrix A, Matrix B, float alpha, float beta);
// #endif

/**
 * @brief Simple in-place matrix multiplication
 *
 * Calculates
 * \f[
 * C = A B
 * \f]
 *
 * This method performs basic matrix multiplication.
 *
 * This method ignores all metadata besides the size
 * (including transpose state, strides, etc.) so should be used
 * with care.
 *
 * It's provided mainly to give the lowest possible cost for matrix multiplication,
 * by directly indexing into the underlying data without checking for transposes or striding.
 *
 * See also: slap_MatMulAdd(), slap_MatMulAtB()
 *
 * **Header File:** `slap/linalg.h`
 * @param[out] C Destination matrix
 * @param[in] A Left input matrix
 * @param[in] B Right input matrix
 */
enum slap_ErrorCode slap_MatMulAB(Matrix C, Matrix A, Matrix B);

/**
 * @brief Simple matrix-transpose matrix multiplication
 *
 * Calculates
 * \f[
 * C = A^T B
 * \f]
 *
 * Similar to slap_MatMulAB(), this method ignores all metadata information. See that
 * method for more details.
 *
 *
 * See also: slap_MatMulAdd(), slap_MatMulAB()
 *
 * **Header File:** `slap/linalg.h`
 * @param[out] C Destination matrix
 * @param[in] A Left input matrix
 * @param[in] B Right input matrix
 */
enum slap_ErrorCode slap_MatMulAtB(Matrix C, Matrix A, Matrix B);
