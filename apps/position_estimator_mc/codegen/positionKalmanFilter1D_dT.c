/*
 * positionKalmanFilter1D_dT.c
 *
 * Code generation for function 'positionKalmanFilter1D_dT'
 *
 * C source code generated on: Fri Nov 30 17:37:33 2012
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "positionKalmanFilter1D_dT.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void positionKalmanFilter1D_dT(real32_T dT, const real32_T x_aposteriori_k[3],
  const real32_T P_aposteriori_k[9], real32_T u, real32_T z, uint8_T gps_update,
  const real32_T Q[9], real_T R, real32_T thresh, real32_T decay, real32_T
  x_aposteriori[3], real32_T P_aposteriori[9])
{
  real32_T A[9];
  int32_T i;
  static const int8_T iv0[3] = { 0, 0, 1 };

  real32_T K[3];
  real32_T f0;
  int32_T i0;
  real32_T b_A[9];
  int32_T i1;
  real32_T P_apriori[9];
  static const int8_T iv1[3] = { 1, 0, 0 };

  real32_T fv0[3];
  real32_T y;
  static const int8_T iv2[3] = { 1, 0, 0 };

  real32_T S;
  int8_T I[9];

  /* dynamics */
  A[0] = 1.0F;
  A[3] = dT;
  A[6] = -0.5F * dT * dT;
  A[1] = 0.0F;
  A[4] = 1.0F;
  A[7] = -dT;
  for (i = 0; i < 3; i++) {
    A[2 + 3 * i] = (real32_T)iv0[i];
  }

  /* prediction */
  K[0] = 0.5F * dT * dT;
  K[1] = dT;
  K[2] = 0.0F;
  for (i = 0; i < 3; i++) {
    f0 = 0.0F;
    for (i0 = 0; i0 < 3; i0++) {
      f0 += A[i + 3 * i0] * x_aposteriori_k[i0];
    }

    x_aposteriori[i] = f0 + K[i] * u;
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A[i + 3 * i0] = 0.0F;
      for (i1 = 0; i1 < 3; i1++) {
        b_A[i + 3 * i0] += A[i + 3 * i1] * P_aposteriori_k[i1 + 3 * i0];
      }
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      f0 = 0.0F;
      for (i1 = 0; i1 < 3; i1++) {
        f0 += b_A[i + 3 * i1] * A[i0 + 3 * i1];
      }

      P_apriori[i + 3 * i0] = f0 + Q[i + 3 * i0];
    }
  }

  if ((real32_T)fabs(u) < thresh) {
    x_aposteriori[1] *= decay;
  }

  /* update */
  if (gps_update == 1) {
    f0 = 0.0F;
    for (i = 0; i < 3; i++) {
      f0 += (real32_T)iv1[i] * x_aposteriori[i];
      fv0[i] = 0.0F;
      for (i0 = 0; i0 < 3; i0++) {
        fv0[i] += (real32_T)iv1[i0] * P_apriori[i0 + 3 * i];
      }
    }

    y = z - f0;
    f0 = 0.0F;
    for (i = 0; i < 3; i++) {
      f0 += fv0[i] * (real32_T)iv2[i];
    }

    S = f0 + (real32_T)R;
    for (i = 0; i < 3; i++) {
      f0 = 0.0F;
      for (i0 = 0; i0 < 3; i0++) {
        f0 += P_apriori[i + 3 * i0] * (real32_T)iv2[i0];
      }

      K[i] = f0 / S;
    }

    for (i = 0; i < 3; i++) {
      x_aposteriori[i] += K[i] * y;
    }

    for (i = 0; i < 9; i++) {
      I[i] = 0;
    }

    for (i = 0; i < 3; i++) {
      I[i + 3 * i] = 1;
    }

    for (i = 0; i < 3; i++) {
      for (i0 = 0; i0 < 3; i0++) {
        A[i0 + 3 * i] = (real32_T)I[i0 + 3 * i] - K[i0] * (real32_T)iv1[i];
      }
    }

    for (i = 0; i < 3; i++) {
      for (i0 = 0; i0 < 3; i0++) {
        P_aposteriori[i + 3 * i0] = 0.0F;
        for (i1 = 0; i1 < 3; i1++) {
          P_aposteriori[i + 3 * i0] += A[i + 3 * i1] * P_apriori[i1 + 3 * i0];
        }
      }
    }
  } else {
    for (i = 0; i < 9; i++) {
      P_aposteriori[i] = P_apriori[i];
    }
  }
}

/* End of code generation (positionKalmanFilter1D_dT.c) */
