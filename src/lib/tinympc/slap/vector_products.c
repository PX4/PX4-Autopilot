//
// Created by Brian Jackson on 12/18/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#include "vector_products.h"

#include <math.h>

#include "matrix_checks.h"

float slap_InnerProduct(const Matrix x, const Matrix y) {
  SLAP_ASSERT_DENSE(x, NAN, "InnerProduct: x vector must be dense");
  SLAP_ASSERT_DENSE(y, NAN, "InnerProduct: x vector must be dense");
  int nx = slap_NumElements(x);
  int ny = slap_NumElements(x);
  int n = nx < ny ? nx : ny;
  float dot = 0;
  for (int i = 0; i < n; ++i) {
    dot += x.data[i] * y.data[i];
  }
  return dot;
}

float slap_QuadraticForm(const Matrix y, const Matrix Q, const Matrix x) {
  SLAP_ASSERT_VALID(y, NAN, "QuadraticForm: y vector not valid");
  SLAP_ASSERT_VALID(Q, NAN, "QuadraticForm: Q matrix not valid");
  SLAP_ASSERT_VALID(x, NAN, "QuadraticForm: x vector not valid");
  SLAP_ASSERT_DENSE(y, NAN, "QuadraticForm: y matrix must be dense");
  SLAP_ASSERT_DENSE(x, NAN, "QuadraticForm: x matrix must be dense");

  enum slap_ErrorCode err;
  err = slap_CheckMatrix(y);
  if (err != SLAP_NO_ERROR) {
    (void)SLAP_ERROR(err, "Bad y vector in QuadraticForm");
    return NAN;
  }
  err = slap_CheckMatrix(x);
  if (err != SLAP_NO_ERROR) {
    (void)SLAP_ERROR(err, "Bad x vector in QuadraticForm");
    return NAN;
  }
  err = slap_CheckMatrix(Q);
  if (err != SLAP_NO_ERROR) {
    (void)SLAP_ERROR(err, "Bad Q matrix in QuadraticForm");
    return NAN;
  }

  int n = slap_NumRows(Q);
  int m = slap_NumCols(Q);
  if (slap_NumElements(y) != n || slap_NumElements(x) != m) {
    (void)SLAP_ERROR(SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
                           "Dimensions incompatible for QuadraticForm");
    return NAN;
  }
  float out = 0.0;
  for (int j = 0; j < m; ++j) {
    for (int i = 0; i < n; ++i) {
      float xj = x.data[j];
      float yi = y.data[i];
      float Aij = *slap_GetElementConst(Q, i, j);
      out += yi * Aij * xj;
    }
  }
  return out;
}

enum slap_ErrorCode slap_OuterProduct(Matrix C, Matrix x, Matrix y) {
  SLAP_ASSERT_VALID(C, SLAP_INVALID_MATRIX, "OuterProduct: C matrix is invalid");
  SLAP_ASSERT_VALID(x, SLAP_INVALID_MATRIX, "OuterProduct: x vector is invalid");
  SLAP_ASSERT_VALID(y, SLAP_INVALID_MATRIX, "OuterProduct: y vector is invalid");
  SLAP_ASSERT_DENSE(x, SLAP_MATRIX_NOT_DENSE, "OuterProduct: x must be a dense matrix");
  SLAP_ASSERT_DENSE(y, SLAP_MATRIX_NOT_DENSE, "OuterProduct: y must be a dense matrix");
  int n = slap_NumElements(x);
  int m = slap_NumElements(y);
  SLAP_ASSERT(slap_NumRows(C) >= n, SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              "OuterProduct: Output matrix doesn't have enough rows. Needs %d, has %d", n,
              slap_NumRows(C));
  SLAP_ASSERT(slap_NumCols(C) >= m, SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              "OuterProduct: Output matrix doesn't have enough columns. Needs %d, has %d", m,
              slap_NumCols(C));
  for (int j = 0; j < m; ++j) {
    float yj = y.data[j];
    for (int i = 0; i < n; ++i) {
      float* Cij = slap_GetElement(C, i, j);
      *Cij = x.data[i] * yj;
    }
  }
  return SLAP_NO_ERROR;
}

enum slap_ErrorCode slap_CrossProduct(Matrix z, Matrix x, Matrix y) {
  SLAP_ASSERT_VALID(z, SLAP_INVALID_MATRIX, "CrossProduct: z vector is invalid");
  SLAP_ASSERT_VALID(x, SLAP_INVALID_MATRIX, "CrossProduct: z vector is invalid");
  SLAP_ASSERT_VALID(y, SLAP_INVALID_MATRIX, "CrossProduct: z vector is invalid");
  SLAP_ASSERT(slap_NumElements(z) >= 3, SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              "CrossProduct: z vector must have length greater than or equal to 3 (got %d)",
              slap_NumElements(z));
  SLAP_ASSERT(slap_NumElements(x) >= 3, SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              "CrossProduct: x vector must have length greater than or equal to 3 (got %d)",
              slap_NumElements(x));
  SLAP_ASSERT(slap_NumElements(y) >= 3, SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              "CrossProduct: y vector must have length greater than or equal to 3 (got %d)",
              slap_NumElements(y));
  z.data[0] = x.data[1] * y.data[2] - x.data[2] * y.data[1];
  z.data[1] = x.data[2] * y.data[0] - x.data[0] * y.data[2];
  z.data[2] = x.data[0] * y.data[1] - x.data[1] * y.data[0];
  return SLAP_NO_ERROR;
}
