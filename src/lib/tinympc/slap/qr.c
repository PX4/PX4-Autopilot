//
// Created by Brian Jackson on 1/22/23.
// Copyright (c) 2023 Robotic Exploration Lab. All rights reserved.
//

#include "qr.h"

#include <math.h>

#include "printing.h"
#include "unary_ops.h"
#include "strided_matrix.h"
#include "tri.h"

#define ZERO_TOL 1e-10

/**
 * @brief Performs a Householder reflection
 *
 * Performs \f$ v = x \pm ||x|| e_1 \f$ where \f$x$\f are the elements
 * below the kth diagonal of A.
 *
 * Also calculates the scaling \f$beta = \frac{2}{v^T v}\f$
 *
 * @param R Partially computed QR decomposition of a matrix A
 * @param v Destination vector for the reflection. Should have same number of rows as R.
 * @param k The index of the column of R on which to calculate the reflection.
 * @return The scaling factor \f$beta\f$
 */
float Householder(Matrix R, Matrix v, int k) {
  int m = slap_NumRows(R);

  // Copy kth column of R below the diagonal
  float sigma = 0;
  float xk = *slap_GetElement(R, k, k);
  for (int i = k + 1; i < m; ++i) {
    float xi = *slap_GetElement(R, i, k);
    v.data[i] = xi;
    sigma += xi * xi;
  }

  // Check if column is already empty
  if (fabs(sigma) < ZERO_TOL) {
    return 0;
  }

  // Reflect in a way that avoids round-off error due to cancellation
  float norm_v = sqrt(sigma + xk * xk);
  float vk = xk;
  if (xk > 0) {
    vk += norm_v;
  } else {
    vk -= norm_v;
  }
  v.data[k] = vk;

  // Calculate scaling factor
  float beta = 2 / (vk * vk + sigma);

  return beta;
}

/**
 * Calculates \f$ \bar{Q} = Q * (I - \alpha v v^T) \f$
 * where \f$v = [1 \; y^T]^T\f$.
 *
 * @param[out] Q_bar
 * @param[in] Q
 * @param[in] R Scaled Householder reflection
 * @param[in] k Column index
 * @param alpha scaling factor
 */
void Qmuly(Matrix Q_bar, Matrix Q, Matrix R, float alpha, int k) {
  int m = slap_NumRows(Q);

  // Top left corner: Stays the same
  // Bottom left corner: Stays the same

//  printf("Qmuly with k = %d\n", k);
//  printf("  Looping columns %d to %d\n", k, m - 1);
  for (int j = k; j < m; ++j) {
    float v_j = 1;
    if (j > k) {
      v_j = *slap_GetElement(R, j, k);
    }
    float Qiv;

    // Top right corner: Q12 - alpha * Q12 * v * v'
//    printf("    Looping rows %d to %d\n", 0, k - 1);
    for (int i = 0; i < k; ++i) {
      Qiv = 0;

      // Calculate Q[i,:] * v
      for (int kk = k; kk < m; ++kk) {
        float v_kk = 1.0;
        if (kk > k) {
          v_kk = *slap_GetElement(R, kk, k);
        }
        Qiv += *slap_GetElement(Q, i, kk) * v_kk;
      }

      // Calculate the output
      float *Qij = slap_GetElement(Q_bar, i, j);
      *Qij -= alpha * Qiv * v_j;
//      printf("    Qv[%d] = %5.3f, v[%d] = %5.3f\n", i, Qiv, j, v_j);
    }

    // Bottom right corner: Q22 - alpha * A22 * v * v'
    for (int i = k; i < m; ++i) {
      Qiv = 0;

      // Calculate Q[i,:] * v
      for (int kk = k; kk < m; kk++) {
        float v_kk = 1.0;
        if (kk > k) {
          v_kk = *slap_GetElement(R, kk, k);
        }
        Qiv += *slap_GetElement(Q, i, kk) * v_kk;
      }

      // Calculate the output
      float *Qij = slap_GetElement(Q_bar, i, j);
      *Qij -= alpha * Qiv * v_j;
    }
  }
}
enum slap_ErrorCode slap_QR(Matrix A, Matrix betas, Matrix temp) {
  int m = slap_NumRows(A);
  int n = slap_NumCols(A);

  // Rename betas to v
  // The first k elements of v are the previous beta values,
  // while the last n-k are the elements of v
  Matrix v = betas;

  // Loop over columns
  for (int k = 0; k < n; ++k) {
    // Calculate Householder reflection vector
    float beta = Householder(A, betas, k);

    // Perform Householder reflection
    // A = (I - beta * v * v') * A

    // 1. Calculate temp = v'A
    for (int j = k; j < n; ++j) {    // loop over columns
      temp.data[j] = 0;
      for (int i = k; i < m; ++i) {  // loop over rows
        temp.data[j] += *slap_GetElement(A, i, j) * v.data[i];
      }
    }

    // 2. Calculate A = A - beta *  v * temp
    for (int j = k; j < n; ++j) {    // loop over columns
      for (int i = k; i < m; ++i) {  // loop over rows
        float *Aij = slap_GetElement(A, i, j);
        *Aij -= beta * v.data[i] * temp.data[j];
      }
    }

    // Store y = v / v[1] below the diagonal
    // Discards the first element, which is known to be 1
    float v_k = v.data[k];
    for (int i = k + 1; i < m; ++i) {
      slap_SetElement(A, i, k, v.data[i] / v_k);
    }
    v.data[k] = v_k * v_k * beta;  // save the scaling
  }

  return SLAP_NO_ERROR;
}

enum slap_ErrorCode slap_ComputeQ(Matrix Q, const Matrix R, const Matrix betas,
                                  Matrix Q_work) {
  int m = slap_NumRows(R);
  int n = slap_NumCols(R);
  slap_SetIdentity(Q, 1.0);
  slap_SetIdentity(Q_work, 1.0);
  for (int k = 0; k < n; ++k) {
    // Calculate Q_work = Q * (I - beta * v * v')
    Qmuly(Q_work, Q, R, betas.data[k], k);

    // Copy the modified columns on the right back to Q
    for (int j = k; j < m; ++j) {
      for (int i = 0; i < m; ++i) {
        slap_SetElement(Q, i, j, *slap_GetElement(Q_work, i, j));
      }
    }
  }
  return SLAP_NO_ERROR;
}
enum slap_ErrorCode slap_Qtb(const Matrix R, const Matrix betas, Matrix b) {
  int m = slap_NumRows(R);
  int n = slap_NumCols(R);
  for (int k = 0; k < n; ++k) {
    // b[k+1] = b[k] - beta[k] * v[k] * (v[k]'b[k])

    // alpha = beta[k] * v[k]'b[k]
    float alpha = b.data[k];
    for (int i = k + 1; i < m; ++i) {
        alpha += *slap_GetElement(R, i, k) * b.data[i];
    }
    alpha *= betas.data[k];

    // b[k+1] = b[k] - alpha * v[k]
    b.data[k] -= alpha;
    for (int i = k + 1; i < m; ++i) {
      b.data[i] -= alpha * *slap_GetElement(R, i, k);
    }
  }
  return SLAP_NO_ERROR;
}
enum slap_ErrorCode slap_LeastSquares(Matrix A, Matrix b, Matrix betas, Matrix temp) {
  // Perform QR on A
  slap_QR(A, betas, temp);

  // Calculate Q'b
  slap_Qtb(A, betas, b);

  // Triangular solve R x = Q'b
  int n = slap_NumCols(A);
  Matrix R = slap_UpperTri(slap_CreateSubMatrix(A, 0, 0, n, n));
  Matrix x = slap_CreateSubMatrix(b, 0, 0, n, 1);
  slap_TriSolve(R, x);
  return SLAP_NO_ERROR;
}
