/*
 * mldivide.c
 *
 * Code generation for function 'mldivide'
 *
 * C source code generated on: Wed May  8 17:06:46 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "frontFlowKalmanFilter.h"
#include "wallEstimationFilter.h"
#include "mldivide.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static real32_T eml_xnrm2(int32_T n, const real32_T x[20], int32_T ix0);
static real32_T rt_hypotf_snf(real32_T u0, real32_T u1);

/* Function Definitions */
static real32_T eml_xnrm2(int32_T n, const real32_T x[20], int32_T ix0)
{
  real32_T y;
  real32_T scale;
  int32_T kend;
  int32_T k;
  real32_T absxk;
  real32_T t;
  y = 0.0F;
  scale = 1.17549435E-38F;
  kend = (ix0 + n) - 1;
  for (k = ix0; k <= kend; k++) {
    absxk = (real32_T)fabs(x[k - 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * (real32_T)sqrt(y);
}

static real32_T rt_hypotf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  real32_T a;
  real32_T b;
  a = (real32_T)fabs(u0);
  b = (real32_T)fabs(u1);
  if (a < b) {
    a /= b;
    y = b * (real32_T)sqrt(a * a + 1.0F);
  } else if (a > b) {
    b /= a;
    y = a * (real32_T)sqrt(b * b + 1.0F);
  } else if (rtIsNaNF(b)) {
    y = b;
  } else {
    y = a * 1.41421354F;
  }

  return y;
}

void mldivide(const real32_T A[20], const real32_T B[10], real32_T Y[2])
{
  real32_T b_B[10];
  int32_T i;
  real32_T b_A[20];
  real32_T tau[2];
  int8_T jpvt[2];
  real32_T work[2];
  int32_T pvt;
  real32_T vn1[2];
  real32_T vn2[2];
  int32_T k;
  real32_T y;
  real32_T wj;
  int32_T iy;
  real32_T temp1;
  real32_T t;
  int32_T i_i;
  int32_T ix;
  int32_T lastv;
  int32_T lastc;
  boolean_T exitg2;
  int32_T exitg1;
  int32_T jy;
  real_T rankR;
  real_T j;
  for (i = 0; i < 10; i++) {
    b_B[i] = B[i];
  }

  memcpy(&b_A[0], &A[0], 20U * sizeof(real32_T));
  for (pvt = 0; pvt < 2; pvt++) {
    jpvt[pvt] = (int8_T)(1 + pvt);
    work[pvt] = 0.0F;
  }

  k = 1;
  for (pvt = 0; pvt < 2; pvt++) {
    y = 0.0F;
    wj = 1.17549435E-38F;
    for (iy = k; iy <= k + 9; iy++) {
      temp1 = (real32_T)fabs(A[iy - 1]);
      if (temp1 > wj) {
        t = wj / temp1;
        y = 1.0F + y * t * t;
        wj = temp1;
      } else {
        t = temp1 / wj;
        y += t * t;
      }
    }

    y = wj * (real32_T)sqrt(y);
    vn1[pvt] = y;
    vn2[pvt] = vn1[pvt];
    k += 10;
  }

  for (i = 0; i < 2; i++) {
    i_i = i + i * 10;
    iy = 0;
    if ((2 - i > 1) && ((real32_T)fabs(vn1[1]) > (real32_T)fabs(vn1[i]))) {
      iy = 1;
    }

    pvt = i + iy;
    if (pvt + 1 != i + 1) {
      ix = 10 * pvt;
      iy = 10 * i;
      for (k = 0; k < 10; k++) {
        wj = b_A[ix];
        b_A[ix] = b_A[iy];
        b_A[iy] = wj;
        ix++;
        iy++;
      }

      iy = jpvt[pvt];
      jpvt[pvt] = jpvt[i];
      jpvt[i] = (int8_T)iy;
      vn1[pvt] = vn1[i];
      vn2[pvt] = vn2[i];
    }

    t = b_A[i_i];
    temp1 = 0.0F;
    wj = eml_xnrm2(9 - i, b_A, i_i + 2);
    if (wj != 0.0F) {
      wj = rt_hypotf_snf((real32_T)fabs(b_A[i_i]), wj);
      if (b_A[i_i] >= 0.0F) {
        wj = -wj;
      }

      if ((real32_T)fabs(wj) < 9.86076132E-32F) {
        iy = 0;
        do {
          iy++;
          pvt = i_i - i;
          for (k = i_i + 1; k + 1 <= pvt + 10; k++) {
            b_A[k] *= 1.01412048E+31F;
          }

          wj *= 1.01412048E+31F;
          t *= 1.01412048E+31F;
        } while (!((real32_T)fabs(wj) >= 9.86076132E-32F));

        wj = rt_hypotf_snf((real32_T)fabs(t), eml_xnrm2(9 - i, b_A, i_i + 2));
        if (t >= 0.0F) {
          wj = -wj;
        }

        temp1 = (wj - t) / wj;
        t = 1.0F / (t - wj);
        pvt = i_i - i;
        for (k = i_i + 1; k + 1 <= pvt + 10; k++) {
          b_A[k] *= t;
        }

        for (k = 1; k <= iy; k++) {
          wj *= 9.86076132E-32F;
        }

        t = wj;
      } else {
        temp1 = (wj - b_A[i_i]) / wj;
        t = 1.0F / (b_A[i_i] - wj);
        pvt = i_i - i;
        for (k = i_i + 1; k + 1 <= pvt + 10; k++) {
          b_A[k] *= t;
        }

        t = wj;
      }
    }

    tau[i] = temp1;
    b_A[i_i] = t;
    if (i + 1 < 2) {
      t = b_A[i_i];
      b_A[i_i] = 1.0F;
      if (tau[0] != 0.0F) {
        lastv = 20;
        iy = i_i;
        while ((lastv - 10 > 0) && (b_A[iy + 9] == 0.0F)) {
          lastv--;
          iy--;
        }

        lastc = 1;
        exitg2 = FALSE;
        while ((exitg2 == FALSE) && (lastc > 0)) {
          iy = 11;
          do {
            exitg1 = 0;
            if (iy <= lastv) {
              if (b_A[iy - 1] != 0.0F) {
                exitg1 = 1;
              } else {
                iy++;
              }
            } else {
              lastc = 0;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = TRUE;
          }
        }
      } else {
        lastv = 10;
        lastc = 0;
      }

      if (lastv - 10 > 0) {
        if (lastc == 0) {
        } else {
          ix = i_i;
          wj = 0.0F;
          for (iy = 11; iy <= lastv; iy++) {
            wj += b_A[iy - 1] * b_A[ix];
            ix++;
          }

          work[0] = wj;
        }

        if (-tau[0] == 0.0F) {
        } else {
          k = 0;
          jy = 0;
          pvt = 1;
          while (pvt <= lastc) {
            if (work[jy] != 0.0F) {
              wj = work[jy] * -tau[0];
              ix = i_i;
              pvt = lastv + k;
              for (iy = k + 10; iy + 1 <= pvt; iy++) {
                b_A[iy] += b_A[ix] * wj;
                ix++;
              }
            }

            jy++;
            k += 10;
            pvt = 2;
          }
        }
      }

      b_A[i_i] = t;
    }

    pvt = i + 2;
    while (pvt < 3) {
      if (vn1[1] != 0.0F) {
        temp1 = (real32_T)fabs(b_A[10 + i]) / vn1[1];
        y = temp1 * temp1;
        temp1 = 1.0F - temp1 * temp1;
        if (1.0F - y < 0.0F) {
          temp1 = 0.0F;
        }

        wj = vn1[1] / vn2[1];
        if (temp1 * (wj * wj) <= 0.000345266977F) {
          y = 0.0F;
          wj = 1.17549435E-38F;
          for (k = i + 11; k + 1 < 21; k++) {
            temp1 = (real32_T)fabs(b_A[k]);
            if (temp1 > wj) {
              t = wj / temp1;
              y = 1.0F + y * t * t;
              wj = temp1;
            } else {
              t = temp1 / wj;
              y += t * t;
            }
          }

          y = wj * (real32_T)sqrt(y);
          vn1[1] = y;
          vn2[1] = y;
        } else {
          vn1[1] *= (real32_T)sqrt(temp1);
        }
      }

      pvt = 3;
    }
  }

  rankR = 0.0;
  k = 0;
  while ((k < 2) && (!((real32_T)fabs(b_A[k + 10 * k]) <= 10.0F * (real32_T)fabs
                       (b_A[0]) * 1.1920929E-7F))) {
    rankR++;
    k++;
  }

  for (i = 0; i < 2; i++) {
    Y[i] = 0.0F;
  }

  for (pvt = 0; pvt < 2; pvt++) {
    if (tau[pvt] != 0.0F) {
      wj = b_B[pvt];
      for (i = 0; i <= 8 - pvt; i++) {
        iy = (pvt + i) + 1;
        wj += b_A[iy + 10 * pvt] * b_B[iy];
      }

      wj *= tau[pvt];
      if (wj != 0.0F) {
        b_B[pvt] -= wj;
        for (i = 0; i <= 8 - pvt; i++) {
          iy = (pvt + i) + 1;
          b_B[iy] -= b_A[iy + 10 * pvt] * wj;
        }
      }
    }
  }

  for (i = 0; i < (int32_T)rankR; i++) {
    Y[jpvt[i] - 1] = b_B[i];
  }

  for (pvt = 0; pvt < (int32_T)-(1.0 + (-1.0 - rankR)); pvt++) {
    j = rankR + -(real_T)pvt;
    Y[jpvt[(int32_T)j - 1] - 1] /= b_A[((int32_T)j + 10 * ((int32_T)j - 1)) - 1];
    i = 0;
    while (i <= (int32_T)j - 2) {
      Y[jpvt[0] - 1] -= Y[jpvt[(int32_T)j - 1] - 1] * b_A[10 * ((int32_T)j - 1)];
      i = 1;
    }
  }
}

/* End of code generation (mldivide.c) */
