/**
 * @file copy_matrix.h
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
 * @brief Copy a matrix to another matrix
 *
 * @param dest a matrix of size (m,n)
 * @param src a matrix of size (n,m)
 * @return slap error code
 */
enum slap_ErrorCode slap_Copy(Matrix dest, Matrix src);

/**
 * @brief Copy a matrix to another matrix, transposed
 *
 * @param dest a matrix of size (m,n)
 * @param src a matrix of size (n,m)
 * @return slap error code
 */
enum slap_ErrorCode slap_CopyTranspose(Matrix dest, Matrix src);

/**
 * @brief Copy the data from an array into the matrix
 *
 * The data is always copied into the same order as the underlying memory layout, so
 * this method ignores whether the matrix is transposed or not.
 *
 * The source array must be at least as long as the destination Matrix, otherwise this
 * function will result in undefined behavior.
 *
 * @param mat  A valid matrix
 * @param data Data to be copied into the array. Must have length of at least mat.rows * * mat.cols.
 * @return slap error code
 */
enum slap_ErrorCode slap_CopyFromArray(Matrix mat, const float* data);
