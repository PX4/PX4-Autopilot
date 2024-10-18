//
// Created by Brian Jackson on 12/18/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#include "strided_matrix.h"

Matrix slap_CreateSubMatrix(Matrix mat, int top_left_row, int top_left_col, int new_rows,
                            int new_cols) {
  Matrix new_mat = slap_NullMatrix();
  SLAP_ASSERT_VALID(mat, new_mat, "CreateSubMatrix: invalid parent matrix");
  SLAP_ASSERT(new_rows >= 0, SLAP_INVALID_DIMENSION, new_mat,
              "CreateSubMatrix: number of rows must be non-negative, got %d", new_rows);
  SLAP_ASSERT(new_cols >= 0, SLAP_INVALID_DIMENSION, new_mat,
              "CreateSubMatrix: number of rows must be non-negative, got %d", new_cols);
  SLAP_ASSERT(
      slap_CheckInbounds(mat, top_left_row + new_rows - 1, top_left_col + new_cols - 1),
      SLAP_INDEX_OUT_OF_BOUNDS, new_mat,
      "CreateSubMatrix: sub-matrix out of bounds with top-left corner (%d,%d), "
      "bottom-right corner (%d,%d)",
      top_left_row, top_left_col, top_left_row + new_rows - 1, top_left_col + new_cols - 1);

  new_mat.rows = new_rows;
  new_mat.cols = new_cols;
  new_mat.sy = mat.sy;
  new_mat.is_transposed = mat.is_transposed;
  int top_left_index = slap_Cart2Index(mat, top_left_row, top_left_col);
  new_mat.data = mat.data + top_left_index;
  new_mat.mattype = mat.mattype;
  return new_mat;
}
