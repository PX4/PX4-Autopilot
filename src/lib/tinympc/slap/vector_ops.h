/**
 * @file vector_ops.h
 * @author Brian Jackson (bjack205@gmail.com)
 * @date 2022-01-30
 * @copyright Copyright (c) 2022
 *
 * @brief Methods that treat the matrix as a vector, including things like
 *        finding minimum or maximum values, norms, sums, etc.
 *
 * @addtogroup Basics
 * @{
 */

#pragma once

#include "matrix.h"
#include "iterator.h"

/**
 * @brief Find the maximum value in the matrix
 *
 * **Header File:** `slap/vector_ops.h`
 * @param[in] mat A valid matrix
 * @param[out] max_value
 * @return A MatrixIterator pointing to the max value
 */
MatrixIterator slap_ArgMax(Matrix mat, float *max_value);

/**
 * @brief Find the minimum value in the matrix
 *
 * **Header File:** `slap/vector_ops.h`
 * @param[in] mat
 * @param[out] min_value
 * @return A MatrixIterator pointing to the min value
 */
MatrixIterator slap_ArgMin(Matrix mat, float *min_value);

/**
 * @brief Find the maximum value in the matrix
 *
 * **Header File:** `slap/vector_ops.h`
 * @param mat A valid Matrix
 * @return The maximum value, or NaN if there was an error
 */
float slap_Max(Matrix mat);

/**
 * @brief Find the minimum value in the matrix
 *
 * **Header File:** `slap/vector_ops.h`
 * @param mat A valid Matrix
 * @return The minimum value, or NaN if there was an error
*/
float slap_Min(Matrix mat);

/**
 * @brief Calculate the squared 2-norm of the matrix (treating it like a vector)
 *
 * **Header File:** `slap/vector_ops.h`
 * @param mat A valid Matrix
 * @return The two norm squared, or NaN if there was an error
 */
float slap_NormTwoSquared(Matrix mat);

/**
 * @brief Calculate the 2-norm, treating the matrix as a vector
 *
 * **Header File:** `slap/vector_ops.h`
 * @param mat A valid matrix
 * @return The two-norm, or NaN if there was an error
 */
float slap_NormTwo(Matrix mat);

/**
 * @brief Calculate the infinity norm, treating the matrix as a vector
 *
 * **Header File:** `slap/vector_ops.h`
 * @param mat A valid matrix
 * @return The infinity norm, or NaN if there was an error
 */
float slap_NormInf(Matrix mat);

/**
 * @brief Calculate the one norm, treating the matrix as a vector
 *
 * **Header File:** `slap/vector_ops.h`
 * @param mat A valid matrix
 * @return The one norm, or NaN if there was an error
 */
float slap_NormOne(Matrix mat);

/**
 * @brief Calculate the sum of all of the elements
 *
 * **Header File:** `slap/vector_ops.h`
 * @param mat A valid matrix
 * @return The sum of all of the elements, or NaN if there was an error
 */
float slap_Sum(Matrix mat);
