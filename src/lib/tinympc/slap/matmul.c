//
// Created by Brian Jackson on 12/19/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#include "matmul.h"
#include "tri.h"

enum slap_ErrorCode slap_MatMulAdd(
    Matrix C, Matrix A, Matrix B, float alpha,
    float beta) {

  // Check for special structure
  if (slap_GetType(A) == slap_TRIANGULAR_UPPER) {
    return slap_UpperTriMulAdd(C, A, B, alpha, beta);
  }
  if (slap_GetType(A) == slap_TRIANGULAR_LOWER) {
    return slap_LowerTriMulAdd(C, A, B, alpha, beta);
  }

  SLAP_ASSERT_VALID(C, SLAP_INVALID_MATRIX, "MatMulAdd: invalid C matrix");
  SLAP_ASSERT_VALID(A, SLAP_INVALID_MATRIX, "MatMulAdd: invalid A matrix");
  SLAP_ASSERT_VALID(B, SLAP_INVALID_MATRIX, "MatMulAdd: invalid B matrix");
  int n = slap_NumRows(A);
  int m = slap_NumCols(A);
  int p = slap_NumCols(B);
  SLAP_ASSERT(slap_NumRows(B) == m, SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              "MatMulAdd: dimension mismatch, B has %d rows, expected %d", slap_NumRows(B),
              m);
  SLAP_ASSERT(slap_NumRows(C) == n, SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              "MatMulAdd: dimension mismatch, C has %d rows, expected %d", slap_NumRows(C),
              n);
  SLAP_ASSERT(slap_NumCols(C) == p, SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
              "MatMulAdd: dimension mismatch, C has %d columns, expected %d",
              slap_NumCols(C), p);
  for (int i = 0; i < n; ++i) {    // rows of output
    for (int j = 0; j < p; ++j) {  // Columns of output
      float* Cij = slap_GetElement(C, i, j);
      *Cij *= beta;
      for (int k = 0; k < m; ++k) {  // columns of A, rows of B
        float Aik = *slap_GetElementConst(A, i, k);
        float Bkj = *slap_GetElementConst(B, k, j);
        *Cij += alpha * Aik * Bkj;
      }
    }
  }
  return SLAP_NO_ERROR;
}

enum slap_ErrorCode slap_MatMulAB(Matrix C, Matrix A, Matrix B) {
  // C is n x p
  // A is n x m
  // B is m x p
  SLAP_ASSERT_VALID(C, SLAP_INVALID_MATRIX, "MatMulAB: invalid C matrix");
  SLAP_ASSERT_VALID(A, SLAP_INVALID_MATRIX, "MatMulAB: invalid A matrix");
  SLAP_ASSERT_VALID(B, SLAP_INVALID_MATRIX, "MatMulAB: invalid B matrix");
  int n = A.rows;
  int m = A.cols;
  int p = B.cols;
  float Aik;
  float Bkj;
  float Cij;
  int ij;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < p; ++j) {
      ij = i + j * n;
      Cij = 0;
      for (int k = 0; k < m; ++k) {  // columns of A, rows of B
        Aik = A.data[i + n * k];
        Bkj = B.data[k + m * j];
        Cij += Aik * Bkj;
      }
      C.data[ij] = Cij;
    }
  }
  return SLAP_NO_ERROR;
}

enum slap_ErrorCode slap_MatMulAtB(Matrix C, Matrix A, Matrix B) {
  // C is n x p
  // A is m x n
  // B is m x p
  SLAP_ASSERT_VALID(C, SLAP_INVALID_MATRIX, "MatMulAtB: invalid C matrix");
  SLAP_ASSERT_VALID(A, SLAP_INVALID_MATRIX, "MatMulAtB: invalid A matrix");
  SLAP_ASSERT_VALID(B, SLAP_INVALID_MATRIX, "MatMulAtB: invalid B matrix");
  int n = A.cols;
  int m = A.rows;
  int p = B.cols;
  int ij;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < p; ++j) {
      ij = i + n * j;
      float Cij = 0;
      for (int k = 0; k < m; ++k) {
        float Aki = A.data[k + i * m];
        float Bkj = B.data[k + j * m];
        Cij += Aki * Bkj;
      }
      C.data[ij] = Cij;
    }
  }
  return SLAP_NO_ERROR;
}
