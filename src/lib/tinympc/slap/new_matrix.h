/**
 * @file new_matrix.h
 * @author Brian Jackson (bjack205@gmail.com)
 * @copyright Copyright (c) 2022
 * @date 2023-01-16
 *
 * @brief Sole methods that allocate and free memory. Provided to conveniently
 * create and free matrices whose data is stored on the heap.
 *
 */

#pragma once

#include "matrix.h"

/**
 * @brief Allocate a new matrix on the heap
 *
 * Data will not be initialized. Wrapper around a call to `malloc`.
 * Must be followed by a call to `FreeMatrix`.
 *
 * # Example
 * ```c
 * Matrix A = slap_NewMatrix(3,4);
 * slap_FreeMatrix(A);
 * ```
 *
 * **Header File:** `"slap/new_matrix.h"`
 * @param rows number of rows in the matrix
 * @param cols number of columns in the matrix
 * @return A new matrix
 */
Matrix slap_NewMatrix(int rows, int cols);

/**
 * @brief Allocate a new matrix on the heap, initialized with zeros
 *
 * Data will be initialized to zeros. Wrapper around a call to `calloc`.
 * Must be followed by a call to `FreeMatrix`.
 *
 * # Example
 * ```c
 * Matrix A = slap_NewMatrixZeros(3,4);
 * slap_FreeMatrix(A);
 * ```
 *
 * **Header File:** `"slap/new_matrix.h"`
 * @param rows number of rows in the matrix
 * @param cols number of columns in the matrix
 * @return A new matrix
 */
Matrix slap_NewMatrixZeros(int rows, int cols);

/**
 * @brief Free the data for a matrix
 *
 * Note that this methods takes the pointer to the matrix as a argument, so that it can
 * set the internal data pointer to NULL after the free.
 *
 * Note this does NOT attempt to free the matrix object itself, only the data
 * it wraps.
 *
 * Should only be used in conjunction with slap_NewMatrix() or slap_NewMatrixZeros().
 *
 * **Header File:** `"slap/new_matrix.h"`
 */
enum slap_ErrorCode slap_FreeMatrix(Matrix* mat);
