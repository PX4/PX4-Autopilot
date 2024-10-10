//
// Created by Brian Jackson on 12/18/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#include "binary_ops.h"

#include <math.h>

float slap_NormedDifference(Matrix A, Matrix B) {
  SLAP_ASSERT_VALID(A, NAN, "MatrixNormedDifference: invalid A matrix");
  SLAP_ASSERT_VALID(B, NAN, "MatrixNormedDifference: invalid B matrix");
  SLAP_ASSERT_SAME_SIZE(A, B, NAN, "MatrixNormedDifference");
  float diff = 0;
  for (int i = 0; i < slap_NumElements(A); ++i) {
    float d = A.data[i] - B.data[i];
    diff += d * d;
  }
  return sqrt(diff);
}
enum slap_ErrorCode slap_MatrixAddition(Matrix C, Matrix A, Matrix B, float alpha) {
  SLAP_ASSERT_VALID(C, SLAP_INVALID_MATRIX, "MatAdd: C matrix invalid");
  SLAP_ASSERT_VALID(A, SLAP_INVALID_MATRIX, "MatAdd: A matrix invalid");
  SLAP_ASSERT_VALID(B, SLAP_INVALID_MATRIX, "MatAdd: B matrix invalid");
  SLAP_ASSERT_SAME_SIZE(A, B, SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS, "MatAdd");
  SLAP_ASSERT_SAME_SIZE(C, A, SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS, "MatAdd");
  int n = slap_NumRows(C);
  int m = slap_NumCols(C);
  for (int j = 0; j < m; ++j) {
    for (int i = 0; i < n; ++i) {
      float Aij = *slap_GetElementConst(A, i, j);
      float Bij = *slap_GetElementConst(B, i, j);
      slap_SetElement(C, i, j, Aij + alpha * Bij);
    }
  }
  return SLAP_NO_ERROR;
}
