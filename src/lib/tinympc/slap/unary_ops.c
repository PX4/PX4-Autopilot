//
// Created by Brian Jackson on 12/18/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#include "unary_ops.h"

#include "iterator.h"
#include "matrix_checks.h"

enum slap_ErrorCode slap_SetConst(Matrix mat, float val) {
  SLAP_ASSERT_VALID(mat, SLAP_INVALID_MATRIX, "SetConst: invalid matrix");
  for (MatrixIterator it = slap_Iterator(mat); !slap_IsFinished(&it); slap_Step(&it)) {
    mat.data[it.index] = val;
  }
  return SLAP_NO_ERROR;
}

enum slap_ErrorCode slap_ScaleByConst(Matrix mat, float alpha) {
  SLAP_ASSERT_VALID(mat, SLAP_INVALID_MATRIX, "ScaleByConst: invalid matrix");
  for (MatrixIterator it = slap_Iterator(mat); !slap_IsFinished(&it); slap_Step(&it)) {
    mat.data[it.index] *= alpha;
  }
  return 0;
}

enum slap_ErrorCode slap_SetIdentity(Matrix mat, float val) {
  SLAP_ASSERT_VALID(mat, SLAP_INVALID_MATRIX, "SetIdentity: invalid matrix");
  slap_SetConst(mat, 0.0);
  for (int k = 0; k < slap_MinDim(mat); ++k) {
    slap_SetElement(mat, k, k, val);
  }
  return 0;
}

enum slap_ErrorCode slap_SetDiagonal(Matrix mat, const float* diag, int len) {
  SLAP_ASSERT_VALID(mat, SLAP_INVALID_MATRIX, "SetDiagonal: invalid matrix");
  SLAP_ASSERT(diag != NULL, SLAP_BAD_POINTER, SLAP_BAD_POINTER,
              "SetDiagonal: pointer to diagonal elements is NULL");
  int n = slap_MinDim(mat);
  n = n <= len ? n : len;
  for (int k = 0; k < n; ++k) {
    slap_SetElement(mat, k, k, diag[k]);
  }
  return SLAP_NO_ERROR;
}

enum slap_ErrorCode slap_AddIdentity(Matrix mat, float alpha) {
  SLAP_ASSERT_VALID(mat, SLAP_INVALID_MATRIX, "AddIdentity: invalid matrix");
  int n = slap_MinDim(mat);
  for (int i = 0; i < n; ++i) {
    float diag = *slap_GetElement(mat, i, i);
    slap_SetElement(mat, i, i, diag + alpha);
  }
  return SLAP_NO_ERROR;
}


enum slap_ErrorCode slap_SetRange(Matrix mat, float start, float stop) {
  SLAP_CHECK_MATRIX(mat);
  float range = stop - start;
  int num_el = slap_NumElements(mat) - 1;
  if (num_el <= 0) {
    return SLAP_EMPTY_MATRIX;
  }

  float step = range / (float)num_el;
  float val = 0;
  int k = 0;
  for (int j = 0; j < slap_NumCols(mat); ++j) {
    for (int i = 0; i < slap_NumRows(mat); ++i) {
      val = start + step * k;
      slap_SetElement(mat, i, j, val);
      ++k;
    }
  }
  return SLAP_NO_ERROR;
}
