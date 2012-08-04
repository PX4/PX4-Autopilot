/*
 * mrdivide.c
 *
 * Code generation for function 'mrdivide'
 *
 * C source code generated on: Wed Jul 11 08:38:35 2012
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "attitudeKalmanfilter.h"
#include "mrdivide.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */

/*
 *
 */
void mrdivide(const real32_T A[108], const real32_T B[81], real32_T y[108])
{
  int32_T jy;
  int32_T iy;
  real32_T b_A[81];
  int8_T ipiv[9];
  int32_T j;
  int32_T mmj;
  int32_T jj;
  int32_T jp1j;
  int32_T c;
  int32_T ix;
  real32_T temp;
  int32_T k;
  real32_T s;
  int32_T loop_ub;
  real32_T Y[108];
  for (jy = 0; jy < 9; jy++) {
    for (iy = 0; iy < 9; iy++) {
      b_A[iy + 9 * jy] = B[jy + 9 * iy];
    }

    ipiv[jy] = (int8_T)(1 + jy);
  }

  for (j = 0; j < 8; j++) {
    mmj = -j;
    jj = j * 10;
    jp1j = jj + 1;
    c = mmj + 9;
    jy = 0;
    ix = jj;
    temp = fabsf(b_A[jj]);
    for (k = 2; k <= c; k++) {
      ix++;
      s = fabsf(b_A[ix]);
      if (s > temp) {
        jy = k - 1;
        temp = s;
      }
    }

    if ((real_T)b_A[jj + jy] != 0.0) {
      if (jy != 0) {
        ipiv[j] = (int8_T)((j + jy) + 1);
        ix = j;
        iy = j + jy;
        for (k = 0; k < 9; k++) {
          temp = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = temp;
          ix += 9;
          iy += 9;
        }
      }

      loop_ub = (jp1j + mmj) + 8;
      for (iy = jp1j; iy + 1 <= loop_ub; iy++) {
        b_A[iy] /= b_A[jj];
      }
    }

    c = 8 - j;
    jy = jj + 9;
    for (iy = 1; iy <= c; iy++) {
      if ((real_T)b_A[jy] != 0.0) {
        temp = b_A[jy] * -1.0F;
        ix = jp1j;
        loop_ub = (mmj + jj) + 18;
        for (k = 10 + jj; k + 1 <= loop_ub; k++) {
          b_A[k] += b_A[ix] * temp;
          ix++;
        }
      }

      jy += 9;
      jj += 9;
    }
  }

  for (jy = 0; jy < 12; jy++) {
    for (iy = 0; iy < 9; iy++) {
      Y[iy + 9 * jy] = A[jy + 12 * iy];
    }
  }

  for (iy = 0; iy < 9; iy++) {
    if (ipiv[iy] != iy + 1) {
      for (j = 0; j < 12; j++) {
        temp = Y[iy + 9 * j];
        Y[iy + 9 * j] = Y[(ipiv[iy] + 9 * j) - 1];
        Y[(ipiv[iy] + 9 * j) - 1] = temp;
      }
    }
  }

  for (j = 0; j < 12; j++) {
    c = 9 * j;
    for (k = 0; k < 9; k++) {
      jy = 9 * k;
      if ((real_T)Y[k + c] != 0.0) {
        for (iy = k + 2; iy < 10; iy++) {
          Y[(iy + c) - 1] -= Y[k + c] * b_A[(iy + jy) - 1];
        }
      }
    }
  }

  for (j = 0; j < 12; j++) {
    c = 9 * j;
    for (k = 8; k > -1; k += -1) {
      jy = 9 * k;
      if ((real_T)Y[k + c] != 0.0) {
        Y[k + c] /= b_A[k + jy];
        for (iy = 0; iy + 1 <= k; iy++) {
          Y[iy + c] -= Y[k + c] * b_A[iy + jy];
        }
      }
    }
  }

  for (jy = 0; jy < 9; jy++) {
    for (iy = 0; iy < 12; iy++) {
      y[iy + 12 * jy] = Y[jy + 9 * iy];
    }
  }
}

/* End of code generation (mrdivide.c) */
