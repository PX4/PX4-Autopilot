/*
 * mrdivide.c
 *
 * Code generation for function 'mrdivide'
 *
 * C source code generated on: Mon Oct 01 19:38:49 2012
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
void b_mrdivide(const real32_T A[36], const real32_T B[9], real32_T y[36])
{
  int32_T rtemp;
  int32_T k;
  real32_T b_A[9];
  real32_T b_B[36];
  int32_T r1;
  int32_T r2;
  int32_T r3;
  real32_T maxval;
  real32_T a21;
  real32_T Y[36];
  for (rtemp = 0; rtemp < 3; rtemp++) {
    for (k = 0; k < 3; k++) {
      b_A[k + 3 * rtemp] = B[rtemp + 3 * k];
    }
  }

  for (rtemp = 0; rtemp < 12; rtemp++) {
    for (k = 0; k < 3; k++) {
      b_B[k + 3 * rtemp] = A[rtemp + 12 * k];
    }
  }

  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = (real32_T)fabsf(b_A[0]);
  a21 = (real32_T)fabsf(b_A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if ((real32_T)fabsf(b_A[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  b_A[r2] /= b_A[r1];
  b_A[r3] /= b_A[r1];
  b_A[3 + r2] -= b_A[r2] * b_A[3 + r1];
  b_A[3 + r3] -= b_A[r3] * b_A[3 + r1];
  b_A[6 + r2] -= b_A[r2] * b_A[6 + r1];
  b_A[6 + r3] -= b_A[r3] * b_A[6 + r1];
  if ((real32_T)fabsf(b_A[3 + r3]) > (real32_T)fabsf(b_A[3 + r2])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  b_A[3 + r3] /= b_A[3 + r2];
  b_A[6 + r3] -= b_A[3 + r3] * b_A[6 + r2];
  for (k = 0; k < 12; k++) {
    Y[3 * k] = b_B[r1 + 3 * k];
    Y[1 + 3 * k] = b_B[r2 + 3 * k] - Y[3 * k] * b_A[r2];
    Y[2 + 3 * k] = (b_B[r3 + 3 * k] - Y[3 * k] * b_A[r3]) - Y[1 + 3 * k] * b_A[3
      + r3];
    Y[2 + 3 * k] /= b_A[6 + r3];
    Y[3 * k] -= Y[2 + 3 * k] * b_A[6 + r1];
    Y[1 + 3 * k] -= Y[2 + 3 * k] * b_A[6 + r2];
    Y[1 + 3 * k] /= b_A[3 + r2];
    Y[3 * k] -= Y[1 + 3 * k] * b_A[3 + r1];
    Y[3 * k] /= b_A[r1];
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    for (k = 0; k < 12; k++) {
      y[k + 12 * rtemp] = Y[rtemp + 3 * k];
    }
  }
}

/*
 *
 */
void c_mrdivide(const real32_T A[72], const real32_T B[36], real32_T y[72])
{
  int32_T jy;
  int32_T iy;
  real32_T b_A[36];
  int8_T ipiv[6];
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
  real32_T Y[72];
  for (jy = 0; jy < 6; jy++) {
    for (iy = 0; iy < 6; iy++) {
      b_A[iy + 6 * jy] = B[jy + 6 * iy];
    }

    ipiv[jy] = (int8_T)(1 + jy);
  }

  for (j = 0; j < 5; j++) {
    mmj = -j;
    jj = j * 7;
    jp1j = jj + 1;
    c = mmj + 6;
    jy = 0;
    ix = jj;
    temp = (real32_T)fabsf(b_A[jj]);
    for (k = 2; k <= c; k++) {
      ix++;
      s = (real32_T)fabsf(b_A[ix]);
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
        for (k = 0; k < 6; k++) {
          temp = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = temp;
          ix += 6;
          iy += 6;
        }
      }

      loop_ub = (jp1j + mmj) + 5;
      for (iy = jp1j; iy + 1 <= loop_ub; iy++) {
        b_A[iy] /= b_A[jj];
      }
    }

    c = 5 - j;
    jy = jj + 6;
    for (iy = 1; iy <= c; iy++) {
      if ((real_T)b_A[jy] != 0.0) {
        temp = b_A[jy] * -1.0F;
        ix = jp1j;
        loop_ub = (mmj + jj) + 12;
        for (k = 7 + jj; k + 1 <= loop_ub; k++) {
          b_A[k] += b_A[ix] * temp;
          ix++;
        }
      }

      jy += 6;
      jj += 6;
    }
  }

  for (jy = 0; jy < 12; jy++) {
    for (iy = 0; iy < 6; iy++) {
      Y[iy + 6 * jy] = A[jy + 12 * iy];
    }
  }

  for (iy = 0; iy < 6; iy++) {
    if (ipiv[iy] != iy + 1) {
      for (j = 0; j < 12; j++) {
        temp = Y[iy + 6 * j];
        Y[iy + 6 * j] = Y[(ipiv[iy] + 6 * j) - 1];
        Y[(ipiv[iy] + 6 * j) - 1] = temp;
      }
    }
  }

  for (j = 0; j < 12; j++) {
    c = 6 * j;
    for (k = 0; k < 6; k++) {
      jy = 6 * k;
      if ((real_T)Y[k + c] != 0.0) {
        for (iy = k + 2; iy < 7; iy++) {
          Y[(iy + c) - 1] -= Y[k + c] * b_A[(iy + jy) - 1];
        }
      }
    }
  }

  for (j = 0; j < 12; j++) {
    c = 6 * j;
    for (k = 5; k > -1; k += -1) {
      jy = 6 * k;
      if ((real_T)Y[k + c] != 0.0) {
        Y[k + c] /= b_A[k + jy];
        for (iy = 0; iy + 1 <= k; iy++) {
          Y[iy + c] -= Y[k + c] * b_A[iy + jy];
        }
      }
    }
  }

  for (jy = 0; jy < 6; jy++) {
    for (iy = 0; iy < 12; iy++) {
      y[iy + 12 * jy] = Y[jy + 6 * iy];
    }
  }
}

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
    temp = (real32_T)fabsf(b_A[jj]);
    for (k = 2; k <= c; k++) {
      ix++;
      s = (real32_T)fabsf(b_A[ix]);
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
