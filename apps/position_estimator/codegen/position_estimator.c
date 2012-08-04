/*
 * position_estimator.c
 *
 * Code generation for function 'position_estimator'
 *
 * C source code generated on: Fri Jun  8 13:31:21 2012
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "position_estimator.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void position_estimator(const real32_T u[2], const real32_T z[3], const real32_T
  xapo[6], const real32_T Papo[36], const real32_T gps_covariance[3], uint8_T
  predict_only, real32_T xapo1[6], real32_T Papo1[36])
{
  real32_T fv0[6];
  real32_T fv1[6];
  real32_T I[36];
  real32_T xapri[6];
  int32_T i;
  int32_T r1;
  static const real32_T fv2[36] = { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.004F,
    1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.004F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.004F, 1.0F };

  static const real32_T fv3[12] = { 0.0F, 0.0F, 0.1744F, 87.2F, 0.0F, 0.0F,
    -0.1744F, -87.2F, 0.0F, 0.0F, 0.0F, 0.0F };

  int32_T r2;
  real32_T Papri[36];
  real32_T maxval;
  static const real32_T fv4[36] = { 1.0F, 0.004F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.004F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.004F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 1.0F };

  static const real32_T fv5[36] = { 1.0E-7F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0E-7F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0E-7F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 1.0F };

  real32_T K[18];
  static const int8_T iv0[18] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0 };

  real32_T fv6[9];
  static const int8_T iv1[18] = { 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    1, 0 };

  real32_T b_gps_covariance[9];
  real32_T A[9];
  real32_T B[18];
  int32_T r3;
  real32_T a21;
  real32_T Y[18];
  real32_T b_z[3];
  int8_T b_I[36];

  /* if predit_onli == 1: no update step: use this when no new gps data is available */
  /* %initialization */
  /* use model F=m*a x''=F/m */
  /*  250Hz---> dT = 0.004s */
  /* u=[phi;theta] */
  /* x=[px;vx;py;vy]; */
  /* %------------------------------------------ */
  /* %------------------------------------------------ */
  /* R_t=[1,-r*dT,q*dT;r*dT,1,-p*dT;-q*dT,p*dT,1]; */
  /* process Covariance Matrix */
  /* measurement Covariance Matrix */
  /* %prediction */
  for (i = 0; i < 6; i++) {
    fv0[i] = 0.0F;
    for (r1 = 0; r1 < 6; r1++) {
      fv0[i] += fv2[i + 6 * r1] * xapo[r1];
    }

    fv1[i] = 0.0F;
    for (r1 = 0; r1 < 2; r1++) {
      fv1[i] += fv3[i + 6 * r1] * u[r1];
    }

    xapri[i] = fv0[i] + fv1[i];
    for (r1 = 0; r1 < 6; r1++) {
      I[i + 6 * r1] = 0.0F;
      for (r2 = 0; r2 < 6; r2++) {
        I[i + 6 * r1] += fv2[i + 6 * r2] * Papo[r2 + 6 * r1];
      }
    }
  }

  for (i = 0; i < 6; i++) {
    for (r1 = 0; r1 < 6; r1++) {
      maxval = 0.0F;
      for (r2 = 0; r2 < 6; r2++) {
        maxval += I[i + 6 * r2] * fv4[r2 + 6 * r1];
      }

      Papri[i + 6 * r1] = maxval + fv5[i + 6 * r1];
    }
  }

  if (1 != predict_only) {
    /* update */
    for (i = 0; i < 3; i++) {
      for (r1 = 0; r1 < 6; r1++) {
        K[i + 3 * r1] = 0.0F;
        for (r2 = 0; r2 < 6; r2++) {
          K[i + 3 * r1] += (real32_T)iv0[i + 3 * r2] * Papri[r2 + 6 * r1];
        }
      }
    }

    for (i = 0; i < 3; i++) {
      for (r1 = 0; r1 < 3; r1++) {
        fv6[i + 3 * r1] = 0.0F;
        for (r2 = 0; r2 < 6; r2++) {
          fv6[i + 3 * r1] += K[r1 + 3 * r2] * (real32_T)iv1[r2 + 6 * i];
        }
      }
    }

    b_gps_covariance[0] = gps_covariance[0];
    b_gps_covariance[1] = 0.0F;
    b_gps_covariance[2] = 0.0F;
    b_gps_covariance[3] = 0.0F;
    b_gps_covariance[4] = gps_covariance[1];
    b_gps_covariance[5] = 0.0F;
    b_gps_covariance[6] = 0.0F;
    b_gps_covariance[7] = 0.0F;
    b_gps_covariance[8] = gps_covariance[2];
    for (i = 0; i < 3; i++) {
      for (r1 = 0; r1 < 3; r1++) {
        A[r1 + 3 * i] = fv6[r1 + 3 * i] + b_gps_covariance[r1 + 3 * i];
      }

      for (r1 = 0; r1 < 6; r1++) {
        B[i + 3 * r1] = 0.0F;
        for (r2 = 0; r2 < 6; r2++) {
          B[i + 3 * r1] += Papri[r1 + 6 * r2] * (real32_T)iv1[r2 + 6 * i];
        }
      }
    }

    r1 = 0;
    r2 = 1;
    r3 = 2;
    maxval = (real32_T)fabs(A[0]);
    a21 = (real32_T)fabs(A[1]);
    if (a21 > maxval) {
      maxval = a21;
      r1 = 1;
      r2 = 0;
    }

    if ((real32_T)fabs(A[2]) > maxval) {
      r1 = 2;
      r2 = 1;
      r3 = 0;
    }

    A[r2] /= A[r1];
    A[r3] /= A[r1];
    A[3 + r2] -= A[r2] * A[3 + r1];
    A[3 + r3] -= A[r3] * A[3 + r1];
    A[6 + r2] -= A[r2] * A[6 + r1];
    A[6 + r3] -= A[r3] * A[6 + r1];
    if ((real32_T)fabs(A[3 + r3]) > (real32_T)fabs(A[3 + r2])) {
      i = r2;
      r2 = r3;
      r3 = i;
    }

    A[3 + r3] /= A[3 + r2];
    A[6 + r3] -= A[3 + r3] * A[6 + r2];
    for (i = 0; i < 6; i++) {
      Y[3 * i] = B[r1 + 3 * i];
      Y[1 + 3 * i] = B[r2 + 3 * i] - Y[3 * i] * A[r2];
      Y[2 + 3 * i] = (B[r3 + 3 * i] - Y[3 * i] * A[r3]) - Y[1 + 3 * i] * A[3 +
        r3];
      Y[2 + 3 * i] /= A[6 + r3];
      Y[3 * i] -= Y[2 + 3 * i] * A[6 + r1];
      Y[1 + 3 * i] -= Y[2 + 3 * i] * A[6 + r2];
      Y[1 + 3 * i] /= A[3 + r2];
      Y[3 * i] -= Y[1 + 3 * i] * A[3 + r1];
      Y[3 * i] /= A[r1];
    }

    for (i = 0; i < 3; i++) {
      for (r1 = 0; r1 < 6; r1++) {
        K[r1 + 6 * i] = Y[i + 3 * r1];
      }
    }

    for (i = 0; i < 3; i++) {
      maxval = 0.0F;
      for (r1 = 0; r1 < 6; r1++) {
        maxval += (real32_T)iv0[i + 3 * r1] * xapri[r1];
      }

      b_z[i] = z[i] - maxval;
    }

    for (i = 0; i < 6; i++) {
      maxval = 0.0F;
      for (r1 = 0; r1 < 3; r1++) {
        maxval += K[i + 6 * r1] * b_z[r1];
      }

      xapo1[i] = xapri[i] + maxval;
    }

    for (i = 0; i < 36; i++) {
      b_I[i] = 0;
    }

    for (i = 0; i < 6; i++) {
      b_I[i + 6 * i] = 1;
    }

    for (i = 0; i < 6; i++) {
      for (r1 = 0; r1 < 6; r1++) {
        maxval = 0.0F;
        for (r2 = 0; r2 < 3; r2++) {
          maxval += K[i + 6 * r2] * (real32_T)iv0[r2 + 3 * r1];
        }

        I[i + 6 * r1] = (real32_T)b_I[i + 6 * r1] - maxval;
      }
    }

    for (i = 0; i < 6; i++) {
      for (r1 = 0; r1 < 6; r1++) {
        Papo1[i + 6 * r1] = 0.0F;
        for (r2 = 0; r2 < 6; r2++) {
          Papo1[i + 6 * r1] += I[i + 6 * r2] * Papri[r2 + 6 * r1];
        }
      }
    }
  } else {
    memcpy((void *)&Papo1[0], (void *)&Papri[0], 36U * sizeof(real32_T));
    for (i = 0; i < 6; i++) {
      xapo1[i] = xapri[i];
    }
  }
}

/* End of code generation (position_estimator.c) */
