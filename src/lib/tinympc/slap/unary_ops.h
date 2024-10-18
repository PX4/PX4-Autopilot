/**
 * @file unary_ops.h
 * @author Brian Jackson (bjack205@gmail.com)
 * @brief Basic unary operations on the Matrix type
 * @date 2022-01-30
 *
 * @copyright Copyright (c) 2022
 *
 * @addtogroup Basics
 * @{
 */

#pragma once

#include "matrix.h"

/**
 * @brief Sets all of the elements in a matrix to a single value
 *
 * **Header File:** `"slap/unary_ops.h"`
 * @param mat Matrix to be modified
 * @param val Value to which each element will be set
 * @return 0 if successful
 */
enum slap_ErrorCode slap_SetConst(Matrix mat, float val);

/**
 * @brief Scale a matrix by a constant factor
 *
 * **Header File:** `"slap/unary_ops.h"`
 * @param mat Fully initialized matrix of non-zero size. Values will be modified.
 * @param alpha scalar by which to multiply the matrix
 */
enum slap_ErrorCode slap_ScaleByConst(Matrix mat, float alpha);

/**
 * @brief Set the diagonal elements of the matrix to val, and the rest to zeros.
 *
 * **Header File:** `"slap/unary_ops.h"`
 * @param mat Square matrix
 * @param val Value for the diagonal elements
 */
enum slap_ErrorCode slap_SetIdentity(Matrix mat, float val);

/**
 * @brief Set the first n elements of a matrix diagonal from an array
 *
 * If @a len is greater than the minimum dimension, only the minimum dimension will be set.
 * Doesn't touch any of the off-diagonal elements.
 *
 * **Header File:** `"slap/unary_ops.h"`
 * @param mat Matrix (nrows >= ncols)
 * @param diag Array of length `nrows`.
 */
enum slap_ErrorCode slap_SetDiagonal(Matrix mat, const float* diag, int len);

/**
 * @brief Add a multiple of the identity to the matrix
 *
 * **Header File:** `"slap/unary_ops.h"`
 * @param mat A valid matrix
 * @param alpha The value to add to the diagonal elements
 */
enum slap_ErrorCode slap_AddIdentity(Matrix mat, float alpha);

/**
 * @brief Sets the matrix to a equally-spaced range
 *
 * Fills the matrix in column-major order.
 *
 * # Example
 * ```c'
 * #include "slap/new_matrix.h"
 * Matrix A = slap_NewMatrix(3, 3);
 * slap_SetRange(A, 1, 9);
 * slap_FreeMatrix(A);
 * ```
 * Creates a matrix with the following elements:
 * ```
 * A = [
 *   1 4 7
 *   2 5 8
 *   3 6 9
 * ]
 * ```
 *
 * **Header File:** `"slap/unary_ops.h"`
 * @param mat A valid, non-empty matrix
 * @param start Value of the first element
 * @param stop Value of the last element
 */
enum slap_ErrorCode slap_SetRange(Matrix mat, float start, float stop);

/**@}*/
