/*
 * mldivide.c
 *
 * Code generation for function 'mldivide'
 *
 * C source code generated on: Thu Feb 28 10:58:05 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "flowNavigation.h"
#include "frontFlowKalmanFilter.h"
#include "wallEstimator.h"
#include "mldivide.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static real_T b_eml_xnrm2(int32_T n, const real_T x[20], int32_T ix0);
static real_T eml_matlab_zlarfg(int32_T n, real_T *alpha1, real_T x[20], int32_T
  ix0);
static real_T eml_xnrm2(const real_T x[20], int32_T ix0);
static real_T rt_hypotd_snf(real_T u0, real_T u1);

/* Function Definitions */
static real_T b_eml_xnrm2(int32_T n, const real_T x[20], int32_T ix0)
{
  real_T y;
  real_T scale;
  int32_T kend;
  int32_T k;
  real_T absxk;
  real_T t;
  y = 0.0;
  if (n < 1) {
  } else if (n == 1) {
    y = fabs(x[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0 + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrt(y);
  }

  return y;
}

static real_T eml_matlab_zlarfg(int32_T n, real_T *alpha1, real_T x[20], int32_T
  ix0)
{
  real_T tau;
  real_T xnorm;
  int32_T knt;
  int32_T i3;
  int32_T k;
  tau = 0.0;
  if (n <= 0) {
  } else {
    xnorm = b_eml_xnrm2(n - 1, x, ix0);
    if (xnorm != 0.0) {
      xnorm = rt_hypotd_snf(fabs(*alpha1), fabs(xnorm));
      if (*alpha1 >= 0.0) {
        xnorm = -xnorm;
      }

      if (fabs(xnorm) < 1.0020841800044864E-292) {
        knt = 0;
        do {
          knt++;
          i3 = (ix0 + n) - 2;
          for (k = ix0; k <= i3; k++) {
            x[k - 1] *= 9.9792015476736E+291;
          }

          xnorm *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while (!(fabs(xnorm) >= 1.0020841800044864E-292));

        xnorm = b_eml_xnrm2(n - 1, x, ix0);
        xnorm = rt_hypotd_snf(fabs(*alpha1), fabs(xnorm));
        if (*alpha1 >= 0.0) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0 / (*alpha1 - xnorm);
        i3 = (ix0 + n) - 2;
        for (k = ix0; k <= i3; k++) {
          x[k - 1] *= *alpha1;
        }

        for (k = 1; k <= knt; k++) {
          xnorm *= 1.0020841800044864E-292;
        }

        *alpha1 = xnorm;
      } else {
        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0 / (*alpha1 - xnorm);
        i3 = (ix0 + n) - 2;
        for (k = ix0; k <= i3; k++) {
          x[k - 1] *= *alpha1;
        }

        *alpha1 = xnorm;
      }
    }
  }

  return tau;
}

static real_T eml_xnrm2(const real_T x[20], int32_T ix0)
{
  real_T y;
  real_T scale;
  int32_T k;
  real_T absxk;
  real_T t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = ix0; k <= ix0 + 9; k++) {
    absxk = fabs(x[k - 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0 + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

static real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T a;
  real_T b;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = b * sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    y = a * sqrt(b * b + 1.0);
  } else if (rtIsNaN(b)) {
    y = b;
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

void mldivide(const real_T A[20], const real_T B[10], real_T Y[2])
{
  real_T b_B[10];
  real_T b_A[20];
  real_T tau[2];
  real_T vn1[2];
  int32_T k;
  int8_T jpvt[2];
  real_T work[2];
  real_T vn2[2];
  int32_T pvt;
  real_T wj;
  int32_T i;
  int32_T i_i;
  int32_T iy;
  real_T rankR;
  int32_T ix;
  int32_T lastv;
  int32_T lastc;
  boolean_T exitg2;
  int32_T exitg1;
  int32_T jy;
  real_T y;
  real_T t;
  memcpy(&b_B[0], &B[0], 10U * sizeof(real_T));
  memcpy(&b_A[0], &A[0], 20U * sizeof(real_T));
  k = 1;
  for (pvt = 0; pvt < 2; pvt++) {
    jpvt[pvt] = (int8_T)(1 + pvt);
    work[pvt] = 0.0;
    wj = eml_xnrm2(A, k);
    vn2[pvt] = wj;
    k += 10;
    vn1[pvt] = wj;
  }

  for (i = 0; i < 2; i++) {
    i_i = i + i * 10;
    iy = 0;
    if (2 - i > 1) {
      wj = fabs(vn1[i]);
      k = 2;
      while (k <= 2 - i) {
        rankR = fabs(vn1[1]);
        if (rankR > wj) {
          iy = 1;
          wj = rankR;
        }

        k = 3;
      }
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

    rankR = b_A[i_i];
    tau[i] = eml_matlab_zlarfg(10 - i, &rankR, b_A, i_i + 2);
    b_A[i_i] = rankR;
    if (i + 1 < 2) {
      rankR = b_A[i_i];
      b_A[i_i] = 1.0;
      if (tau[0] != 0.0) {
        lastv = 10;
        iy = i_i + 9;
        while ((lastv > 0) && (b_A[iy] == 0.0)) {
          lastv--;
          iy--;
        }

        lastc = 1;
        exitg2 = FALSE;
        while ((exitg2 == FALSE) && (lastc > 0)) {
          iy = 11;
          do {
            exitg1 = 0;
            if (iy <= 10 + lastv) {
              if (b_A[iy - 1] != 0.0) {
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
        lastv = 0;
        lastc = 0;
      }

      if (lastv > 0) {
        if (lastc == 0) {
        } else {
          ix = i_i;
          wj = 0.0;
          for (iy = 11; iy <= lastv + 10; iy++) {
            wj += b_A[iy - 1] * b_A[ix];
            ix++;
          }

          work[0] = wj;
        }

        if (-tau[0] == 0.0) {
        } else {
          k = 10;
          jy = 0;
          pvt = 1;
          while (pvt <= lastc) {
            if (work[jy] != 0.0) {
              wj = work[jy] * -tau[0];
              ix = i_i;
              pvt = lastv + k;
              for (iy = k; iy + 1 <= pvt; iy++) {
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

      b_A[i_i] = rankR;
    }

    pvt = i + 2;
    while (pvt < 3) {
      if (vn1[1] != 0.0) {
        rankR = fabs(b_A[10 + i]) / vn1[1];
        y = rankR * rankR;
        rankR = 1.0 - rankR * rankR;
        if (1.0 - y < 0.0) {
          rankR = 0.0;
        }

        wj = vn1[1] / vn2[1];
        if (rankR * (wj * wj) <= 1.4901161193847656E-8) {
          y = 0.0;
          wj = 2.2250738585072014E-308;
          for (k = i; k + 12 < 21; k++) {
            rankR = fabs(b_A[k + 11]);
            if (rankR > wj) {
              t = wj / rankR;
              y = 1.0 + y * t * t;
              wj = rankR;
            } else {
              t = rankR / wj;
              y += t * t;
            }
          }

          y = wj * sqrt(y);
          vn1[1] = y;
          vn2[1] = y;
        } else {
          vn1[1] *= sqrt(rankR);
        }
      }

      pvt = 3;
    }
  }

  rankR = 0.0;
  k = 0;
  while ((k < 2) && (!(fabs(b_A[k + 10 * k]) <= 10.0 * fabs(b_A[0]) *
                       2.2204460492503131E-16))) {
    rankR++;
    k++;
  }

  for (i = 0; i < 2; i++) {
    Y[i] = 0.0;
  }

  for (pvt = 0; pvt < 2; pvt++) {
    if (tau[pvt] != 0.0) {
      wj = b_B[pvt];
      for (i = 0; i <= 8 - pvt; i++) {
        iy = (pvt + i) + 1;
        wj += b_A[iy + 10 * pvt] * b_B[iy];
      }

      wj *= tau[pvt];
      if (wj != 0.0) {
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
    wj = rankR + -(real_T)pvt;
    Y[jpvt[(int32_T)wj - 1] - 1] /= b_A[((int32_T)wj + 10 * ((int32_T)wj - 1)) -
      1];
    i = 0;
    while (i <= (int32_T)wj - 2) {
      Y[jpvt[0] - 1] -= Y[jpvt[(int32_T)wj - 1] - 1] * b_A[10 * ((int32_T)wj - 1)];
      i = 1;
    }
  }
}

/* End of code generation (mldivide.c) */
