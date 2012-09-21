/*
 * mrdivide.c
 *
 * Code generation for function 'mrdivide'
 *
 * C source code generated on: Fri Sep 21 13:56:44 2012
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
static real32_T b_eml_matlab_zlarfg(real32_T *alpha1, const real32_T *x);
static void eml_lusolve(const real32_T A_data[81], const int32_T A_sizes[2],
  real32_T B_data[81], int32_T B_sizes[2]);
static real32_T eml_matlab_zlarfg(int32_T n, real32_T *alpha1, real32_T x_data
  [81], int32_T x_sizes[2], int32_T ix0);
static void eml_qrsolve(const real32_T A_data[81], const int32_T A_sizes[2],
  real32_T B_data[81], int32_T B_sizes[2], real32_T Y_data[81], int32_T Y_sizes
  [2]);
static real32_T eml_xnrm2(int32_T n, const real32_T x_data[81], const int32_T
  x_sizes[2], int32_T ix0);

/* Function Definitions */

/*
 *
 */
static real32_T b_eml_matlab_zlarfg(real32_T *alpha1, const real32_T *x)
{
  return 0.0F;
}

/*
 *
 */
static void eml_lusolve(const real32_T A_data[81], const int32_T A_sizes[2],
  real32_T B_data[81], int32_T B_sizes[2])
{
  int32_T loop_ub;
  int32_T iy;
  real32_T b_A_data[81];
  int32_T jA;
  int32_T tmp_data[9];
  int32_T ipiv_data[9];
  int32_T ldap1;
  int32_T j;
  int32_T mmj;
  int32_T jj;
  int32_T ix;
  real32_T temp;
  int32_T k;
  real32_T s;
  int32_T jrow;
  int32_T b_loop_ub;
  int32_T c;
  loop_ub = A_sizes[0] * A_sizes[1] - 1;
  for (iy = 0; iy <= loop_ub; iy++) {
    b_A_data[iy] = A_data[iy];
  }

  iy = A_sizes[1];
  jA = A_sizes[1];
  jA = iy <= jA ? iy : jA;
  if (jA < 1) {
  } else {
    loop_ub = jA - 1;
    for (iy = 0; iy <= loop_ub; iy++) {
      tmp_data[iy] = 1 + iy;
    }

    loop_ub = jA - 1;
    for (iy = 0; iy <= loop_ub; iy++) {
      ipiv_data[iy] = tmp_data[iy];
    }
  }

  ldap1 = A_sizes[1] + 1;
  iy = A_sizes[1] - 1;
  jA = A_sizes[1];
  loop_ub = iy <= jA ? iy : jA;
  for (j = 1; j <= loop_ub; j++) {
    mmj = (A_sizes[1] - j) + 1;
    jj = (j - 1) * ldap1;
    if (mmj < 1) {
      jA = -1;
    } else {
      jA = 0;
      if (mmj > 1) {
        ix = jj;
        temp = (real32_T)fabs(b_A_data[jj]);
        for (k = 1; k + 1 <= mmj; k++) {
          ix++;
          s = (real32_T)fabs(b_A_data[ix]);
          if (s > temp) {
            jA = k;
            temp = s;
          }
        }
      }
    }

    if ((real_T)b_A_data[jj + jA] != 0.0) {
      if (jA != 0) {
        ipiv_data[j - 1] = j + jA;
        jrow = j - 1;
        iy = jrow + jA;
        for (k = 1; k <= A_sizes[1]; k++) {
          temp = b_A_data[jrow];
          b_A_data[jrow] = b_A_data[iy];
          b_A_data[iy] = temp;
          jrow += A_sizes[1];
          iy += A_sizes[1];
        }
      }

      b_loop_ub = jj + mmj;
      for (jA = jj + 1; jA + 1 <= b_loop_ub; jA++) {
        b_A_data[jA] /= b_A_data[jj];
      }
    }

    c = A_sizes[1] - j;
    jA = jj + ldap1;
    iy = jj + A_sizes[1];
    for (jrow = 1; jrow <= c; jrow++) {
      if ((real_T)b_A_data[iy] != 0.0) {
        temp = b_A_data[iy] * -1.0F;
        ix = jj;
        b_loop_ub = (mmj + jA) - 1;
        for (k = jA; k + 1 <= b_loop_ub; k++) {
          b_A_data[k] += b_A_data[ix + 1] * temp;
          ix++;
        }
      }

      iy += A_sizes[1];
      jA += A_sizes[1];
    }
  }

  for (jA = 0; jA + 1 <= A_sizes[1]; jA++) {
    if (ipiv_data[jA] != jA + 1) {
      for (j = 0; j < 9; j++) {
        temp = B_data[jA + B_sizes[0] * j];
        B_data[jA + B_sizes[0] * j] = B_data[(ipiv_data[jA] + B_sizes[0] * j) -
          1];
        B_data[(ipiv_data[jA] + B_sizes[0] * j) - 1] = temp;
      }
    }
  }

  if (B_sizes[0] == 0) {
  } else {
    for (j = 0; j < 9; j++) {
      c = A_sizes[1] * j;
      for (k = 0; k + 1 <= A_sizes[1]; k++) {
        iy = A_sizes[1] * k;
        if ((real_T)B_data[k + c] != 0.0) {
          for (jA = k + 1; jA + 1 <= A_sizes[1]; jA++) {
            B_data[jA + c] -= B_data[k + c] * b_A_data[jA + iy];
          }
        }
      }
    }
  }

  if (B_sizes[0] == 0) {
  } else {
    for (j = 0; j < 9; j++) {
      c = A_sizes[1] * j;
      for (k = A_sizes[1] - 1; k + 1 > 0; k--) {
        iy = A_sizes[1] * k;
        if ((real_T)B_data[k + c] != 0.0) {
          B_data[k + c] /= b_A_data[k + iy];
          for (jA = 0; jA + 1 <= k; jA++) {
            B_data[jA + c] -= B_data[k + c] * b_A_data[jA + iy];
          }
        }
      }
    }
  }
}

/*
 *
 */
static real32_T eml_matlab_zlarfg(int32_T n, real32_T *alpha1, real32_T x_data
  [81], int32_T x_sizes[2], int32_T ix0)
{
  real32_T tau;
  int32_T nm1;
  real32_T xnorm;
  real32_T a;
  int32_T knt;
  int32_T loop_ub;
  int32_T k;
  tau = 0.0F;
  if (n <= 0) {
  } else {
    nm1 = n - 2;
    xnorm = eml_xnrm2(nm1 + 1, x_data, x_sizes, ix0);
    if ((real_T)xnorm != 0.0) {
      a = (real32_T)fabs(*alpha1);
      xnorm = (real32_T)fabs(xnorm);
      if (a < xnorm) {
        a /= xnorm;
        xnorm *= (real32_T)sqrt(a * a + 1.0F);
      } else if (a > xnorm) {
        xnorm /= a;
        xnorm = (real32_T)sqrt(xnorm * xnorm + 1.0F) * a;
      } else if (rtIsNaNF(xnorm)) {
      } else {
        xnorm = a * 1.41421354F;
      }

      if ((real_T)*alpha1 >= 0.0) {
        xnorm = -xnorm;
      }

      if ((real32_T)fabs(xnorm) < 9.86076132E-32F) {
        knt = 0;
        do {
          knt++;
          loop_ub = ix0 + nm1;
          for (k = ix0; k <= loop_ub; k++) {
            x_data[k - 1] *= 1.01412048E+31F;
          }

          xnorm *= 1.01412048E+31F;
          *alpha1 *= 1.01412048E+31F;
        } while (!((real32_T)fabs(xnorm) >= 9.86076132E-32F));

        xnorm = eml_xnrm2(nm1 + 1, x_data, x_sizes, ix0);
        a = (real32_T)fabs(*alpha1);
        xnorm = (real32_T)fabs(xnorm);
        if (a < xnorm) {
          a /= xnorm;
          xnorm *= (real32_T)sqrt(a * a + 1.0F);
        } else if (a > xnorm) {
          xnorm /= a;
          xnorm = (real32_T)sqrt(xnorm * xnorm + 1.0F) * a;
        } else if (rtIsNaNF(xnorm)) {
        } else {
          xnorm = a * 1.41421354F;
        }

        if ((real_T)*alpha1 >= 0.0) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0F / (*alpha1 - xnorm);
        loop_ub = ix0 + nm1;
        for (k = ix0; k <= loop_ub; k++) {
          x_data[k - 1] *= *alpha1;
        }

        for (k = 1; k <= knt; k++) {
          xnorm *= 9.86076132E-32F;
        }

        *alpha1 = xnorm;
      } else {
        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0F / (*alpha1 - xnorm);
        loop_ub = ix0 + nm1;
        for (k = ix0; k <= loop_ub; k++) {
          x_data[k - 1] *= *alpha1;
        }

        *alpha1 = xnorm;
      }
    }
  }

  return tau;
}

/*
 *
 */
static void eml_qrsolve(const real32_T A_data[81], const int32_T A_sizes[2],
  real32_T B_data[81], int32_T B_sizes[2], real32_T Y_data[81], int32_T Y_sizes
  [2])
{
  real_T rankR;
  real_T u1;
  int32_T mn;
  int32_T b_A_sizes[2];
  int32_T loop_ub;
  int32_T k;
  real32_T b_A_data[81];
  int32_T pvt;
  int32_T b_mn;
  int32_T tmp_data[9];
  int32_T jpvt_data[9];
  int8_T unnamed_idx_0;
  real32_T work_data[9];
  int32_T nmi;
  real32_T vn1_data[9];
  real32_T vn2_data[9];
  int32_T i;
  int32_T i_i;
  int32_T mmi;
  int32_T ix;
  real32_T smax;
  real32_T s;
  int32_T iy;
  real32_T atmp;
  real32_T tau_data[9];
  int32_T i_ip1;
  int32_T lastv;
  int32_T lastc;
  boolean_T exitg2;
  int32_T exitg1;
  boolean_T firstNonZero;
  real32_T t;
  rankR = (real_T)A_sizes[0];
  u1 = (real_T)A_sizes[1];
  rankR = rankR <= u1 ? rankR : u1;
  mn = (int32_T)rankR;
  b_A_sizes[0] = A_sizes[0];
  b_A_sizes[1] = A_sizes[1];
  loop_ub = A_sizes[0] * A_sizes[1] - 1;
  for (k = 0; k <= loop_ub; k++) {
    b_A_data[k] = A_data[k];
  }

  k = A_sizes[0];
  pvt = A_sizes[1];
  b_mn = k <= pvt ? k : pvt;
  if (A_sizes[1] < 1) {
  } else {
    loop_ub = A_sizes[1] - 1;
    for (k = 0; k <= loop_ub; k++) {
      tmp_data[k] = 1 + k;
    }

    loop_ub = A_sizes[1] - 1;
    for (k = 0; k <= loop_ub; k++) {
      jpvt_data[k] = tmp_data[k];
    }
  }

  if ((A_sizes[0] == 0) || (A_sizes[1] == 0)) {
  } else {
    unnamed_idx_0 = (int8_T)A_sizes[1];
    loop_ub = unnamed_idx_0 - 1;
    for (k = 0; k <= loop_ub; k++) {
      work_data[k] = 0.0F;
    }

    k = 1;
    for (nmi = 0; nmi + 1 <= A_sizes[1]; nmi++) {
      vn1_data[nmi] = eml_xnrm2(A_sizes[0], A_data, A_sizes, k);
      vn2_data[nmi] = vn1_data[nmi];
      k += A_sizes[0];
    }

    for (i = 0; i + 1 <= b_mn; i++) {
      i_i = i + i * A_sizes[0];
      nmi = A_sizes[1] - i;
      mmi = (A_sizes[0] - i) - 1;
      if (nmi < 1) {
        pvt = -1;
      } else {
        pvt = 0;
        if (nmi > 1) {
          ix = i;
          smax = (real32_T)fabs(vn1_data[i]);
          for (k = 1; k + 1 <= nmi; k++) {
            ix++;
            s = (real32_T)fabs(vn1_data[ix]);
            if (s > smax) {
              pvt = k;
              smax = s;
            }
          }
        }
      }

      pvt += i;
      if (pvt + 1 != i + 1) {
        ix = A_sizes[0] * pvt;
        iy = A_sizes[0] * i;
        for (k = 1; k <= A_sizes[0]; k++) {
          s = b_A_data[ix];
          b_A_data[ix] = b_A_data[iy];
          b_A_data[iy] = s;
          ix++;
          iy++;
        }

        k = jpvt_data[pvt];
        jpvt_data[pvt] = jpvt_data[i];
        jpvt_data[i] = k;
        vn1_data[pvt] = vn1_data[i];
        vn2_data[pvt] = vn2_data[i];
      }

      if (i + 1 < A_sizes[0]) {
        atmp = b_A_data[i_i];
        smax = eml_matlab_zlarfg(mmi + 1, &atmp, b_A_data, b_A_sizes, i_i + 2);
        tau_data[i] = smax;
      } else {
        atmp = b_A_data[i_i];
        smax = b_A_data[i_i];
        s = b_eml_matlab_zlarfg(&atmp, &smax);
        b_A_data[i_i] = smax;
        tau_data[i] = s;
      }

      b_A_data[i_i] = atmp;
      if (i + 1 < A_sizes[1]) {
        atmp = b_A_data[i_i];
        b_A_data[i_i] = 1.0F;
        i_ip1 = (i + (i + 1) * A_sizes[0]) + 1;
        if ((real_T)tau_data[i] != 0.0) {
          lastv = mmi;
          pvt = i_i + mmi;
          while ((lastv + 1 > 0) && ((real_T)b_A_data[pvt] == 0.0)) {
            lastv--;
            pvt--;
          }

          lastc = nmi - 1;
          exitg2 = 0U;
          while ((exitg2 == 0U) && (lastc > 0)) {
            k = i_ip1 + (lastc - 1) * A_sizes[0];
            pvt = k + lastv;
            do {
              exitg1 = 0U;
              if (k <= pvt) {
                if ((real_T)b_A_data[k - 1] != 0.0) {
                  exitg1 = 1U;
                } else {
                  k++;
                }
              } else {
                lastc--;
                exitg1 = 2U;
              }
            } while (exitg1 == 0U);

            if (exitg1 == 1U) {
              exitg2 = 1U;
            }
          }
        } else {
          lastv = -1;
          lastc = 0;
        }

        if (lastv + 1 > 0) {
          if (lastc == 0) {
          } else {
            for (iy = 1; iy <= lastc; iy++) {
              work_data[iy - 1] = 0.0F;
            }

            iy = 0;
            loop_ub = i_ip1 + A_sizes[0] * (lastc - 1);
            pvt = i_ip1;
            while ((A_sizes[0] > 0) && (pvt <= loop_ub)) {
              ix = i_i;
              smax = 0.0F;
              k = pvt + lastv;
              for (nmi = pvt; nmi <= k; nmi++) {
                smax += b_A_data[nmi - 1] * b_A_data[ix];
                ix++;
              }

              work_data[iy] += smax;
              iy++;
              pvt += A_sizes[0];
            }
          }

          smax = -tau_data[i];
          if ((real_T)smax == 0.0) {
          } else {
            pvt = 0;
            for (nmi = 1; nmi <= lastc; nmi++) {
              if ((real_T)work_data[pvt] != 0.0) {
                s = work_data[pvt] * smax;
                ix = i_i;
                loop_ub = lastv + i_ip1;
                for (k = i_ip1 - 1; k + 1 <= loop_ub; k++) {
                  b_A_data[k] += b_A_data[ix] * s;
                  ix++;
                }
              }

              pvt++;
              i_ip1 += A_sizes[0];
            }
          }
        }

        b_A_data[i_i] = atmp;
      }

      for (nmi = i + 1; nmi + 1 <= A_sizes[1]; nmi++) {
        if ((real_T)vn1_data[nmi] != 0.0) {
          smax = (real32_T)fabs(b_A_data[i + b_A_sizes[0] * nmi]) / vn1_data[nmi];
          smax = 1.0F - smax * smax;
          if ((real_T)smax < 0.0) {
            smax = 0.0F;
          }

          s = vn1_data[nmi] / vn2_data[nmi];
          if (smax * (s * s) <= 0.000345266977F) {
            if (i + 1 < A_sizes[0]) {
              k = (i + A_sizes[0] * nmi) + 1;
              smax = 0.0F;
              if (mmi < 1) {
              } else if (mmi == 1) {
                smax = (real32_T)fabs(b_A_data[k]);
              } else {
                s = 0.0F;
                firstNonZero = TRUE;
                pvt = k + mmi;
                while (k + 1 <= pvt) {
                  if (b_A_data[k] != 0.0F) {
                    atmp = (real32_T)fabs(b_A_data[k]);
                    if (firstNonZero) {
                      s = atmp;
                      smax = 1.0F;
                      firstNonZero = FALSE;
                    } else if (s < atmp) {
                      t = s / atmp;
                      smax = 1.0F + smax * t * t;
                      s = atmp;
                    } else {
                      t = atmp / s;
                      smax += t * t;
                    }
                  }

                  k++;
                }

                smax = s * (real32_T)sqrt(smax);
              }

              vn1_data[nmi] = smax;
              vn2_data[nmi] = vn1_data[nmi];
            } else {
              vn1_data[nmi] = 0.0F;
              vn2_data[nmi] = 0.0F;
            }
          } else {
            vn1_data[nmi] *= (real32_T)sqrt(smax);
          }
        }
      }
    }
  }

  rankR = (real_T)A_sizes[0];
  u1 = (real_T)A_sizes[1];
  rankR = rankR >= u1 ? rankR : u1;
  smax = (real32_T)rankR * (real32_T)fabs(b_A_data[0]) * 1.1920929E-7F;
  rankR = 0.0;
  k = 0;
  while ((k + 1 <= mn) && (!((real32_T)fabs(b_A_data[k + b_A_sizes[0] * k]) <=
           smax))) {
    rankR++;
    k++;
  }

  unnamed_idx_0 = (int8_T)A_sizes[1];
  Y_sizes[0] = (int32_T)unnamed_idx_0;
  Y_sizes[1] = 9;
  loop_ub = unnamed_idx_0 * 9 - 1;
  for (k = 0; k <= loop_ub; k++) {
    Y_data[k] = 0.0F;
  }

  for (nmi = 0; nmi + 1 <= mn; nmi++) {
    if ((real_T)tau_data[nmi] != 0.0) {
      for (k = 0; k < 9; k++) {
        smax = B_data[nmi + B_sizes[0] * k];
        for (i = nmi + 1; i + 1 <= A_sizes[0]; i++) {
          smax += b_A_data[i + b_A_sizes[0] * nmi] * B_data[i + B_sizes[0] * k];
        }

        smax *= tau_data[nmi];
        if ((real_T)smax != 0.0) {
          B_data[nmi + B_sizes[0] * k] -= smax;
          for (i = nmi + 1; i + 1 <= A_sizes[0]; i++) {
            B_data[i + B_sizes[0] * k] -= b_A_data[i + b_A_sizes[0] * nmi] *
              smax;
          }
        }
      }
    }
  }

  for (k = 0; k < 9; k++) {
    for (i = 0; (real_T)(i + 1) <= rankR; i++) {
      Y_data[(jpvt_data[i] + Y_sizes[0] * k) - 1] = B_data[i + B_sizes[0] * k];
    }

    for (nmi = (int32_T)rankR - 1; nmi + 1 >= 1; nmi--) {
      Y_data[(jpvt_data[nmi] + Y_sizes[0] * k) - 1] /= b_A_data[nmi + b_A_sizes
        [0] * nmi];
      for (i = 0; i + 1 <= nmi; i++) {
        Y_data[(jpvt_data[i] + Y_sizes[0] * k) - 1] -= Y_data[(jpvt_data[nmi] +
          Y_sizes[0] * k) - 1] * b_A_data[i + b_A_sizes[0] * nmi];
      }
    }
  }
}

/*
 *
 */
static real32_T eml_xnrm2(int32_T n, const real32_T x_data[81], const int32_T
  x_sizes[2], int32_T ix0)
{
  real32_T y;
  real32_T scale;
  boolean_T firstNonZero;
  int32_T kend;
  int32_T k;
  real32_T absxk;
  real32_T t;
  y = 0.0F;
  if (n < 1) {
  } else if (n == 1) {
    y = (real32_T)fabs(x_data[ix0 - 1]);
  } else {
    scale = 0.0F;
    firstNonZero = TRUE;
    kend = (ix0 + n) - 1;
    for (k = ix0 - 1; k + 1 <= kend; k++) {
      if (x_data[k] != 0.0F) {
        absxk = (real32_T)fabs(x_data[k]);
        if (firstNonZero) {
          scale = absxk;
          y = 1.0F;
          firstNonZero = FALSE;
        } else if (scale < absxk) {
          t = scale / absxk;
          y = 1.0F + y * t * t;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }
    }

    y = scale * (real32_T)sqrt(y);
  }

  return y;
}

/*
 *
 */
void mrdivide(const real32_T A_data[81], const int32_T A_sizes[2], const
              real32_T B_data[81], const int32_T B_sizes[2], real32_T y_data[81],
              int32_T y_sizes[2])
{
  int32_T b_A_sizes[2];
  int32_T loop_ub;
  int32_T i3;
  int32_T b_loop_ub;
  int32_T i4;
  real32_T b_A_data[81];
  int32_T b_B_sizes[2];
  real32_T b_B_data[81];
  int8_T unnamed_idx_0;
  int32_T c_B_sizes[2];
  real32_T c_B_data[81];
  b_A_sizes[0] = B_sizes[1];
  b_A_sizes[1] = B_sizes[0];
  loop_ub = B_sizes[0] - 1;
  for (i3 = 0; i3 <= loop_ub; i3++) {
    b_loop_ub = B_sizes[1] - 1;
    for (i4 = 0; i4 <= b_loop_ub; i4++) {
      b_A_data[i4 + b_A_sizes[0] * i3] = B_data[i3 + B_sizes[0] * i4];
    }
  }

  b_B_sizes[0] = A_sizes[1];
  b_B_sizes[1] = 9;
  for (i3 = 0; i3 < 9; i3++) {
    loop_ub = A_sizes[1] - 1;
    for (i4 = 0; i4 <= loop_ub; i4++) {
      b_B_data[i4 + b_B_sizes[0] * i3] = A_data[i3 + A_sizes[0] * i4];
    }
  }

  if ((b_A_sizes[0] == 0) || (b_A_sizes[1] == 0) || (b_B_sizes[0] == 0)) {
    unnamed_idx_0 = (int8_T)b_A_sizes[1];
    b_B_sizes[0] = (int32_T)unnamed_idx_0;
    b_B_sizes[1] = 9;
    loop_ub = unnamed_idx_0 * 9 - 1;
    for (i3 = 0; i3 <= loop_ub; i3++) {
      b_B_data[i3] = 0.0F;
    }
  } else if (b_A_sizes[0] == b_A_sizes[1]) {
    eml_lusolve(b_A_data, b_A_sizes, b_B_data, b_B_sizes);
  } else {
    c_B_sizes[0] = b_B_sizes[0];
    c_B_sizes[1] = 9;
    loop_ub = b_B_sizes[0] * 9 - 1;
    for (i3 = 0; i3 <= loop_ub; i3++) {
      c_B_data[i3] = b_B_data[i3];
    }

    eml_qrsolve(b_A_data, b_A_sizes, c_B_data, c_B_sizes, b_B_data, b_B_sizes);
  }

  y_sizes[0] = 9;
  y_sizes[1] = b_B_sizes[0];
  loop_ub = b_B_sizes[0] - 1;
  for (i3 = 0; i3 <= loop_ub; i3++) {
    for (i4 = 0; i4 < 9; i4++) {
      y_data[i4 + 9 * i3] = b_B_data[i3 + b_B_sizes[0] * i4];
    }
  }
}

/* End of code generation (mrdivide.c) */
