//
// Created by Brian Jackson on 12/18/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#include "function_mapping.h"

#include "iterator.h"

enum slap_ErrorCode slap_Map(Matrix mat, float (*function)(float)) {
  SLAP_ASSERT_VALID(mat, SLAP_INVALID_MATRIX, "Map: invalid matrix");
  for (MatrixIterator it = slap_Iterator(mat); !slap_IsFinished(&it); slap_Step(&it)) {
    float* value = mat.data + it.index;
    *value = function(*value);
  }
  return SLAP_NO_ERROR;
}
enum slap_ErrorCode slap_BinaryMap(Matrix C, Matrix A, Matrix B,
                                   float (*function)(float, float)) {
  // Check for valid matrices
  SLAP_ASSERT_VALID(C, SLAP_INVALID_MATRIX, "BinaryMap: invalid output C matrix");
  SLAP_ASSERT_VALID(A, SLAP_INVALID_MATRIX, "BinaryMap: invalid input A matrix");
  SLAP_ASSERT_VALID(B, SLAP_INVALID_MATRIX, "BinaryMap: invalid input B matrix");

  // Check that matrices have the same size
  SLAP_ASSERT_SAME_SIZE(C,A, SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS, "slap_BinaryMap");
  SLAP_ASSERT_SAME_SIZE(A,B, SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS, "slap_BinaryMap");

  for (MatrixIterator it = slap_Iterator(C); !slap_IsFinished(&it); slap_Step(&it)) {
    // NOTE: use Cartesian indexing on A,B because they might have different strides
    float *Ck = C.data + it.index;
    float Ak = *slap_GetElement(A, it.i, it.j);
    float Bk = *slap_GetElement(B, it.i, it.j);
    *Ck = function(Ak, Bk);
  }
  return SLAP_NO_ERROR;
}
