//
// Created by Brian Jackson on 12/19/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#include "cholesky.h"

#include <math.h>
#include "tri.h"

enum slap_ErrorCode slap_Cholesky(Matrix A) {
  SLAP_ASSERT_VALID(A, SLAP_INVALID_MATRIX, "Cholesky: matrix invalid");
  int n = slap_MinDim(A);
  for (int j = 0; j < n; ++j) {
    for (int k = 0; k < j; ++k) {
      for (int i = j; i < n; ++i) {
        float* Aij = slap_GetElement(A, i, j);
        float Aik = *slap_GetElement(A, i, k);
        float Ajk = *slap_GetElement(A, j, k);
        *Aij -= Aik * Ajk;
      }
    }
    float Ajj = *slap_GetElement(A, j, j);
    if (Ajj <= 0) {
      return SLAP_CHOLESKY_FAIL;
    }
    float ajj = sqrt(Ajj);

    for (int i = j; i < n; ++i) {
      float* Aij = slap_GetElement(A, i, j);
      *Aij /= ajj;
    }
  }
  return SLAP_NO_ERROR;
}

enum slap_ErrorCode slap_CholeskySolve(const Matrix A, Matrix b) {
  // NOTE: Validity checks are done by the sub-methods
  enum slap_ErrorCode err;
  err = slap_TriSolve(A, b);
  if (err != SLAP_NO_ERROR) {
    return err;
  }
  err = slap_TriSolve(slap_Transpose(A), b);
  if (err != SLAP_NO_ERROR) {
    return err;
  }
  return SLAP_NO_ERROR;
}
