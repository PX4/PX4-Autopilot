#include "matrix.h"

Matrix slap_MatrixFromArray(int rows, int cols, float* data) {  // NOLINT(readability-non-const-parameter)
  Matrix mat = {rows, cols, rows, 0, data, slap_DENSE};
  return mat;
}

void slap_Linear2Cart(Matrix mat, int k, int* row, int* col) {  // NOLINT(bugprone-easily-swappable-parameters)
  int rows = slap_NumRows(mat);
  *row = k % rows;
  *col = k / rows;
}

Matrix slap_Flatten(const Matrix mat) {
  SLAP_ASSERT_VALID(mat, slap_NullMatrix(), "Flatten: invalid matrix");
  SLAP_ASSERT_DENSE(mat, slap_NullMatrix(), "Flatten: input matrix must be dense");
  int size = slap_NumElements(mat);
  Matrix vec = {
      .rows = size,
      .cols = 1,
      .sy = mat.sy,
      .is_transposed = mat.is_transposed,
      .data = mat.data,
      .mattype = mat.mattype,
  };
  return vec;
}

Matrix slap_Transpose(Matrix mat) {
  Matrix new_mat = {
      .rows = mat.rows,
      .cols = mat.cols,
      .sy = mat.sy,
      .is_transposed = !mat.is_transposed,
      .data = mat.data,
      .mattype = mat.mattype,
  };
  return new_mat;
}

Matrix slap_Reshape(Matrix mat, int rows, int cols) {
  SLAP_ASSERT_VALID(mat, slap_NullMatrix(), "Reshape: invalid matrix");
  SLAP_ASSERT_DENSE(mat, slap_NullMatrix(), "Reshape: input matrix must be dense");
  SLAP_ASSERT(rows >= 0, SLAP_INVALID_DIMENSION, slap_NullMatrix(),
              "Reshape: number of rows must be positive, got %d", rows);
  SLAP_ASSERT(cols >= 0, SLAP_INVALID_DIMENSION, slap_NullMatrix(),
              "Reshape: number of columns must be positive, got %d", cols);
  Matrix new_mat = {
      .rows = rows,
      .cols = cols,
      .sy = rows,
      .is_transposed = mat.is_transposed,
      .data = mat.data,
      .mattype = mat.mattype,
  };
  new_mat.rows = rows;
  new_mat.cols = cols;
  return new_mat;
}
Matrix slap_UpperTri(Matrix mat) {
  Matrix new_mat = {
      .rows = mat.rows,
      .cols = mat.cols,
      .sy = mat.sy,
      .is_transposed = mat.is_transposed,
      .data = mat.data,
      .mattype = slap_TRIANGULAR_UPPER,
  };
  return new_mat;
}

Matrix slap_LowerTri(Matrix mat) {
  Matrix new_mat = {
      .rows = mat.rows,
      .cols = mat.cols,
      .sy = mat.sy,
      .is_transposed = mat.is_transposed,
      .data = mat.data,
      .mattype = slap_TRIANGULAR_LOWER,
  };
  return new_mat;
}
