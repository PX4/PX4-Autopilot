//
// Created by Brian Jackson on 12/18/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#pragma once

#include "matrix.h"

/**
 * @brief Calculate the inner (dot) product of two vectors
 *
 * If the vectors are of unequal length, the inner product is taken up to the shortest
 * length.
 *
 * **Header File:** `vector_products.h`
 * @param x A vector of length n
 * @param y A vector of length n
 * @return Dot product of x,y. NAN if invalid.
 */
float slap_InnerProduct(Matrix x, Matrix y);

/**
 * @brief Calculate the scaled inner product \f$ y^T A x \f$
 *
 * **Header File:** `vector_products.h`
 * @param x A vector of length n
 * @param A A matrix of size (n,m)
 * @param y A vector of length m
 * @return The dot product, or NAN if invalid.
 */
float slap_QuadraticForm(Matrix y, Matrix Q, Matrix x);

/**
 * @brief Take the outer product of two vectors
 *
 * Calculates x * y'
 *
 * @param[out] C An n x m matrix holding the result of the outer product.
 *               If either dimension is larger than the corresponding vector, the rest will
 *               be untouched.
 *
 * **Header File:** `vector_products.h`
 * @param[in]  x An n-dimensional vector
 * @param[in]  y An m-dimensional vector
 * @return slap return code indicating any errors
 */
enum slap_ErrorCode slap_OuterProduct(Matrix C, Matrix x, Matrix y);

/**
 * @brief take the 3D cross product of 2 vectors
 *
 * The vectors must all have at least length 3. If the the length is greater than three,
 * only the first 3 elements are used.
 *
 * **Header File:** `vector_products.h`
 * @param[out] z cross product vector
 * @param[in]  x first input vector
 * @param[in]  y second input vector (order matters)
 * @return slap return code indicating any errors
 */
enum slap_ErrorCode slap_CrossProduct(Matrix z, Matrix x, Matrix y);
