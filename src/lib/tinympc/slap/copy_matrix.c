//
// Created by Brian Jackson on 12/18/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#include "copy_matrix.h"

#include "iterator.h"
#include "matrix_checks.h"

enum slap_ErrorCode slap_Copy(Matrix dest, Matrix src) {
  SLAP_ASSERT_VALID(dest, SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
                    "MatrixCopy: invalid destination matrix");
  SLAP_ASSERT_VALID(src, SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
                    "MatrixCopy: invalid source matrix");
  SLAP_ASSERT_SAME_SIZE(dest, src, SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS, "MatrixCopy");
  int n = slap_NumRows(src);
  int m = slap_NumCols(src);
  for (int j = 0; j < m; ++j) {
    for (int i = 0; i < n; ++i) {
      slap_SetElement(dest, i, j, *slap_GetElement(src, i, j));
    }
  }
  //  memcpy(dest.data, src.data, slap_NumElements(dest) * sizeof(float));
  return SLAP_NO_ERROR;
}

enum slap_ErrorCode slap_CopyTranspose(Matrix dest, Matrix src) {
  SLAP_CHECK_MATRIX(dest);
  SLAP_CHECK_MATRIX(src);
  if ((slap_NumRows(dest) != slap_NumCols((src))) ||
      (slap_NumCols(dest) != slap_NumRows(src))) {
    return SLAP_ERROR(SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
               "Matrix sizes are not transposes of each other. Got (%d,%d) and (%d,%d).\n",
               slap_NumRows(src), slap_NumCols(src), slap_NumRows(dest), slap_NumCols(dest));
  }

  for (int i = 0; i < slap_NumRows(dest); ++i) {
    for (int j = 0; j < slap_NumCols(dest); ++j) {
      int dest_index = slap_Cart2Index(dest, i, j);
      int src_index = slap_Cart2Index(src, j, i);
      dest.data[dest_index] = src.data[src_index];
    }
  }
  return SLAP_NO_ERROR;
}

enum slap_ErrorCode slap_CopyFromArray(Matrix mat, const float* data) {
  SLAP_ASSERT_VALID(mat, SLAP_INVALID_MATRIX, "CopyFromArray: invalid matrix");
  SLAP_ASSERT(data != NULL, SLAP_BAD_POINTER, SLAP_BAD_POINTER,
              "CopyFromArray: Can't copy from raw array, pointer is NULL");
  for (MatrixIterator it = slap_Iterator(mat); !slap_IsFinished(&it); slap_Step(&it)) {
    mat.data[it.index] = data[it.k];
  }
  return SLAP_NO_ERROR;
}
