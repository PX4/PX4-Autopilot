/*
 * mrdivide.c
 *
 * Code generation for function 'mrdivide'
 *
 * C source code generated on: Sat Jan 19 15:25:29 2013
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
  real32_T b_A[9];
  int32_T rtemp;
  int32_T k;
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
  maxval = (real32_T)fabs(b_A[0]);
  a21 = (real32_T)fabs(b_A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if ((real32_T)fabs(b_A[2]) > maxval) {
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
  if ((real32_T)fabs(b_A[3 + r3]) > (real32_T)fabs(b_A[3 + r2])) {
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
  real32_T b_A[36];
  int8_T ipiv[6];
  int32_T i3;
  int32_T iy;
  int32_T j;
  int32_T c;
  int32_T ix;
  real32_T temp;
  int32_T k;
  real32_T s;
  int32_T jy;
  int32_T ijA;
  real32_T Y[72];
  for (i3 = 0; i3 < 6; i3++) {
    for (iy = 0; iy < 6; iy++) {
      b_A[iy + 6 * i3] = B[i3 + 6 * iy];
    }

    ipiv[i3] = (int8_T)(1 + i3);
  }

  for (j = 0; j < 5; j++) {
    c = j * 7;
    iy = 0;
    ix = c;
    temp = (real32_T)fabs(b_A[c]);
    for (k = 2; k <= 6 - j; k++) {
      ix++;
      s = (real32_T)fabs(b_A[ix]);
      if (s > temp) {
        iy = k - 1;
        temp = s;
      }
    }

    if (b_A[c + iy] != 0.0F) {
      if (iy != 0) {
        ipiv[j] = (int8_T)((j + iy) + 1);
        ix = j;
        iy += j;
        for (k = 0; k < 6; k++) {
          temp = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = temp;
          ix += 6;
          iy += 6;
        }
      }

      i3 = (c - j) + 6;
      for (jy = c + 1; jy + 1 <= i3; jy++) {
        b_A[jy] /= b_A[c];
      }
    }

    iy = c;
    jy = c + 6;
    for (k = 1; k <= 5 - j; k++) {
      temp = b_A[jy];
      if (b_A[jy] != 0.0F) {
        ix = c + 1;
        i3 = (iy - j) + 12;
        for (ijA = 7 + iy; ijA + 1 <= i3; ijA++) {
          b_A[ijA] += b_A[ix] * -temp;
          ix++;
        }
      }

      jy += 6;
      iy += 6;
    }
  }

  for (i3 = 0; i3 < 12; i3++) {
    for (iy = 0; iy < 6; iy++) {
      Y[iy + 6 * i3] = A[i3 + 12 * iy];
    }
  }

  for (jy = 0; jy < 6; jy++) {
    if (ipiv[jy] != jy + 1) {
      for (j = 0; j < 12; j++) {
        temp = Y[jy + 6 * j];
        Y[jy + 6 * j] = Y[(ipiv[jy] + 6 * j) - 1];
        Y[(ipiv[jy] + 6 * j) - 1] = temp;
      }
    }
  }

  for (j = 0; j < 12; j++) {
    c = 6 * j;
    for (k = 0; k < 6; k++) {
      iy = 6 * k;
      if (Y[k + c] != 0.0F) {
        for (jy = k + 2; jy < 7; jy++) {
          Y[(jy + c) - 1] -= Y[k + c] * b_A[(jy + iy) - 1];
        }
      }
    }
  }

  for (j = 0; j < 12; j++) {
    c = 6 * j;
    for (k = 5; k > -1; k += -1) {
      iy = 6 * k;
      if (Y[k + c] != 0.0F) {
        Y[k + c] /= b_A[k + iy];
        for (jy = 0; jy + 1 <= k; jy++) {
          Y[jy + c] -= Y[k + c] * b_A[jy + iy];
        }
      }
    }
  }

  for (i3 = 0; i3 < 6; i3++) {
    for (iy = 0; iy < 12; iy++) {
      y[iy + 12 * i3] = Y[i3 + 6 * iy];
    }
  }
}

/*
 *
 */
void mrdivide(const real32_T A[108], const real32_T B[81], real32_T y[108])
{
  real32_T b_A[81];
  int8_T ipiv[9];
  int32_T i2;
  int32_T iy;
  int32_T j;
  int32_T c;
  int32_T ix;
  real32_T temp;
  int32_T k;
  real32_T s;
  int32_T jy;
  int32_T ijA;
  real32_T Y[108];
  for (i2 = 0; i2 < 9; i2++) {
    for (iy = 0; iy < 9; iy++) {
      b_A[iy + 9 * i2] = B[i2 + 9 * iy];
    }

    ipiv[i2] = (int8_T)(1 + i2);
  }

  for (j = 0; j < 8; j++) {
    c = j * 10;
    iy = 0;
    ix = c;
    temp = (real32_T)fabs(b_A[c]);
    for (k = 2; k <= 9 - j; k++) {
      ix++;
      s = (real32_T)fabs(b_A[ix]);
      if (s > temp) {
        iy = k - 1;
        temp = s;
      }
    }

    if (b_A[c + iy] != 0.0F) {
      if (iy != 0) {
        ipiv[j] = (int8_T)((j + iy) + 1);
        ix = j;
        iy += j;
        for (k = 0; k < 9; k++) {
          temp = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = temp;
          ix += 9;
          iy += 9;
        }
      }

      i2 = (c - j) + 9;
      for (jy = c + 1; jy + 1 <= i2; jy++) {
        b_A[jy] /= b_A[c];
      }
    }

    iy = c;
    jy = c + 9;
    for (k = 1; k <= 8 - j; k++) {
      temp = b_A[jy];
      if (b_A[jy] != 0.0F) {
        ix = c + 1;
        i2 = (iy - j) + 18;
        for (ijA = 10 + iy; ijA + 1 <= i2; ijA++) {
          b_A[ijA] += b_A[ix] * -temp;
          ix++;
        }
      }

      jy += 9;
      iy += 9;
    }
  }

  for (i2 = 0; i2 < 12; i2++) {
    for (iy = 0; iy < 9; iy++) {
      Y[iy + 9 * i2] = A[i2 + 12 * iy];
    }
  }

  for (jy = 0; jy < 9; jy++) {
    if (ipiv[jy] != jy + 1) {
      for (j = 0; j < 12; j++) {
        temp = Y[jy + 9 * j];
        Y[jy + 9 * j] = Y[(ipiv[jy] + 9 * j) - 1];
        Y[(ipiv[jy] + 9 * j) - 1] = temp;
      }
    }
  }

  for (j = 0; j < 12; j++) {
    c = 9 * j;
    for (k = 0; k < 9; k++) {
      iy = 9 * k;
      if (Y[k + c] != 0.0F) {
        for (jy = k + 2; jy < 10; jy++) {
          Y[(jy + c) - 1] -= Y[k + c] * b_A[(jy + iy) - 1];
        }
      }
    }
  }

  for (j = 0; j < 12; j++) {
    c = 9 * j;
    for (k = 8; k > -1; k += -1) {
      iy = 9 * k;
      if (Y[k + c] != 0.0F) {
        Y[k + c] /= b_A[k + iy];
        for (jy = 0; jy + 1 <= k; jy++) {
          Y[jy + c] -= Y[k + c] * b_A[jy + iy];
        }
      }
    }
  }

  for (i2 = 0; i2 < 9; i2++) {
    for (iy = 0; iy < 12; iy++) {
      y[iy + 12 * i2] = Y[i2 + 9 * iy];
    }
  }
}

/* End of code generation (mrdivide.c) */
