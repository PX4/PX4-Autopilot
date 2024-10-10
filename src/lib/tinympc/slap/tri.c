//
// Created by Brian Jackson on 3/4/23.
// Copyright (c) 2023 Robotic Exploration Lab. All rights reserved.
//

#include "tri.h"

#include <math.h>

enum slap_ErrorCode slap_UpperTriMulAdd(Matrix C, const Matrix U, const Matrix B,
                                        double alpha, double beta) {
  SLAP_ASSERT_VALID(C, SLAP_INVALID_MATRIX, "Error in UpperTriMulAdd: Invalid C matrix");
  SLAP_ASSERT_VALID(U, SLAP_INVALID_MATRIX, "Error in UpperTriMulAdd: Invalid U matrix");
  SLAP_ASSERT_VALID(B, SLAP_INVALID_MATRIX, "Error in UpperTriMulAdd: Invalid B matrix");
  SLAP_ASSERT(slap_NumRows(U) == slap_NumRows(U), SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              "Error in UpperTriMulAdd: Rows of C (%d) not equal to Rows of U (%d).",
              slap_NumRows(C), slap_NumRows(U));
  SLAP_ASSERT(slap_NumCols(U) == slap_NumRows(B), SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              "Error in UpperTriMulAdd: Columns of U (%d) not equal to Rows of B (%d).",
              slap_NumCols(U), slap_NumRows(B));
  SLAP_ASSERT(slap_NumCols(C) == slap_NumCols(B), SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              "Error in UpperTriMulAdd: Columns of C (%d) not equal to Columns of B (%d).",
              slap_NumCols(C), slap_NumCols(B));
  int m = slap_NumRows(U);
  int n = slap_NumCols(U);
  int p = slap_NumCols(B);
  for (int j = 0; j < p; ++j) {
    for (int i = 0; i < m; ++i) {
      float* Cij = slap_GetElement(C, i, j);
      *Cij *= (float)beta;
      for (int k = i; k < n; ++k) {
        float Uik = *slap_GetElementConst(U, i, k);
        float Bkj = *slap_GetElementConst(B, k, j);
        *Cij += (float)alpha * Uik * Bkj;
      }
    }
  }
  return SLAP_NO_ERROR;
}
enum slap_ErrorCode slap_LowerTriMulAdd(Matrix C, const Matrix L, const Matrix B,
                                        double alpha, double beta) {
  SLAP_ASSERT_VALID(C, SLAP_INVALID_MATRIX, "Error in UpperTriMulAdd: Invalid C matrix");
  SLAP_ASSERT_VALID(L, SLAP_INVALID_MATRIX, "Error in UpperTriMulAdd: Invalid L matrix");
  SLAP_ASSERT_VALID(B, SLAP_INVALID_MATRIX, "Error in UpperTriMulAdd: Invalid B matrix");
  SLAP_ASSERT(slap_NumRows(L) == slap_NumRows(L), SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              "Error in UpperTriMulAdd: Rows of C (%d) not equal to Rows of L (%d).",
              slap_NumRows(C), slap_NumRows(L));
  SLAP_ASSERT(slap_NumCols(L) == slap_NumRows(B), SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              "Error in UpperTriMulAdd: Columns of L (%d) not equal to Rows of B (%d).",
              slap_NumCols(L), slap_NumRows(B));
  SLAP_ASSERT(slap_NumCols(C) == slap_NumCols(B), SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              "Error in UpperTriMulAdd: Columns of C (%d) not equal to Columns of B (%d).",
              slap_NumCols(C), slap_NumCols(B));
  int m = slap_NumRows(L);
  int n = slap_NumCols(L);
  int p = slap_NumCols(B);
  for (int j = 0; j < p; ++j) {
    for (int i = 0; i < m; ++i) {
      float* Cij = slap_GetElement(C, i, j);
      *Cij *= (float)beta;
      int stop = i < n ? i + 1 : n;
      for (int k = 0; k < stop; ++k) {
        float Uik = *slap_GetElementConst(L, i, k);
        float Bkj = *slap_GetElementConst(B, k, j);
        *Cij += (float)alpha * Uik * Bkj;
      }
    }
  }
  return SLAP_NO_ERROR;
}

bool slap_CheckUpperTri(const Matrix A) {
  SLAP_ASSERT_VALID(A, false, "Can't check UpperTri: A matrix is invalid");
  bool is_triu = true;
  int n_rows = slap_NumRows(A);
  int n_cols = slap_NumCols(A);
  if (n_cols > n_rows) {
    n_cols = n_rows;
  }
  for (int j = 0; j < n_cols; ++j) {
    for (int i = j + 1; i < n_rows; ++i) {
      float val = *slap_GetElementConst(A, i, j);
      if (fabs(val) > 0) {
        is_triu = false;
        break;
      }
    }
    if (!is_triu) { break; }
  }
  return is_triu;
}

bool slap_CheckLowerTri(const Matrix A) {
  SLAP_ASSERT_VALID(A, false, "Can't check LowerTri: A matrix is invalid");
  bool is_tril = true;
  int n_rows = slap_NumRows(A);
  int n_cols = slap_NumCols(A);
  if (n_rows > n_cols) {
    n_rows = n_cols;
  }
  for (int j = 0; j < n_cols; ++j) {
    for (int i = 0; i < j; ++i) {
      float val = *slap_GetElementConst(A, i, j);
      if (fabs(val) > 0) {
        is_tril = false;
        break;
      }
    }
    if (!is_tril) { break; }
  }
  return is_tril;
}

enum slap_ErrorCode slap_MakeUpperTri(Matrix A) {
  SLAP_ASSERT_VALID(A, SLAP_INVALID_MATRIX, "Error in MakeUpperTri: Matrix Invalid");
  int n_rows = slap_NumRows(A);
  int n_cols = slap_NumCols(A);
  if (n_cols > n_rows) {
    n_cols = n_rows;
  }
  for (int j = 0; j < n_cols; ++j) {
    for (int i = j + 1; i < n_rows; ++i) {
      slap_SetElement(A, i, j, 0);
    }
  }
  return SLAP_NO_ERROR;
}

enum slap_ErrorCode slap_MakeLowerTri(Matrix A) {
  SLAP_ASSERT_VALID(A, SLAP_INVALID_MATRIX, "Error in MakeLowerTri: Matrix Invalid");
  int n_rows = slap_NumRows(A);
  int n_cols = slap_NumCols(A);
  if (n_rows > n_cols) {
    n_rows = n_cols;
  }
  for (int j = 0; j < n_cols; ++j) {
    for (int i = 0; i < j; ++i) {
      slap_SetElement(A, i, j, 0);
    }
  }
  return SLAP_NO_ERROR;
}

enum slap_ErrorCode slap_TriSolve(Matrix L, Matrix b) {
  SLAP_ASSERT_VALID(L, SLAP_INVALID_MATRIX, "LowerTriBackSub: L matrix invalid");
  SLAP_ASSERT_VALID(b, SLAP_INVALID_MATRIX, "LowerTriBackSub: b matrix invalid");
  SLAP_ASSERT(slap_NumCols(L) == slap_NumRows(b), SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              "LowerTriBackSub: L has %d columns but b has %d rows", slap_NumCols(L),
              slap_NumRows(b));
  int n = b.rows;
  int m = b.cols;
  bool tL = L.mattype == slap_TRIANGULAR_UPPER || slap_IsTransposed(L);

  for (int j_ = 0; j_ < n; ++j_) {
    int j = tL ? n - j_ - 1 : j_;
    for (int k = 0; k < m; ++k) {
      float* xjk = slap_GetElement(b, j, k);
      float Ljj = *slap_GetElement(L, j, j);
      *xjk /= Ljj;

      for (int i_ = j_ + 1; i_ < n; ++i_) {
        int i = tL ? i_ - (j_ + 1) : i_;
        float* xik = slap_GetElement(b, i, k);
        float Lij = *slap_GetElement(L, i, j);
        *xik -= Lij * (*xjk);
      }
    }
  }
  return SLAP_NO_ERROR;
}
