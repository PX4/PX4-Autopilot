/*
 * positionKalmanFilter1D.c
 *
 * Code generation for function 'positionKalmanFilter1D'
 *
 * C source code generated on: Fri Nov 30 14:26:11 2012
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "positionKalmanFilter1D.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void positionKalmanFilter1D(const real32_T A[9], const real32_T B[3], const
  real32_T C[3], const real32_T x_aposteriori_k[3], const real32_T
  P_aposteriori_k[9], real32_T u, real32_T z, uint8_T gps_update, const real32_T
  Q[9], real32_T R, real32_T thresh, real32_T decay, real32_T x_aposteriori[3],
  real32_T P_aposteriori[9])
{
  int32_T i0;
  real32_T f0;
  int32_T k;
  real32_T b_A[9];
  int32_T i1;
  real32_T P_apriori[9];
  real32_T y;
  real32_T K[3];
  real32_T S;
  int8_T I[9];

  /* prediction */
  for (i0 = 0; i0 < 3; i0++) {
    f0 = 0.0F;
    for (k = 0; k < 3; k++) {
      f0 += A[i0 + 3 * k] * x_aposteriori_k[k];
    }

    x_aposteriori[i0] = f0 + B[i0] * u;
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (k = 0; k < 3; k++) {
      b_A[i0 + 3 * k] = 0.0F;
      for (i1 = 0; i1 < 3; i1++) {
        b_A[i0 + 3 * k] += A[i0 + 3 * i1] * P_aposteriori_k[i1 + 3 * k];
      }
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (k = 0; k < 3; k++) {
      f0 = 0.0F;
      for (i1 = 0; i1 < 3; i1++) {
        f0 += b_A[i0 + 3 * i1] * A[k + 3 * i1];
      }

      P_apriori[i0 + 3 * k] = f0 + Q[i0 + 3 * k];
    }
  }

  if ((real32_T)fabs(u) < thresh) {
    x_aposteriori[1] *= decay;
  }

  /* update */
  if (gps_update == 1) {
    y = 0.0F;
    for (k = 0; k < 3; k++) {
      y += C[k] * x_aposteriori[k];
      K[k] = 0.0F;
      for (i0 = 0; i0 < 3; i0++) {
        K[k] += C[i0] * P_apriori[i0 + 3 * k];
      }
    }

    y = z - y;
    S = 0.0F;
    for (k = 0; k < 3; k++) {
      S += K[k] * C[k];
    }

    S += R;
    for (i0 = 0; i0 < 3; i0++) {
      f0 = 0.0F;
      for (k = 0; k < 3; k++) {
        f0 += P_apriori[i0 + 3 * k] * C[k];
      }

      K[i0] = f0 / S;
    }

    for (i0 = 0; i0 < 3; i0++) {
      x_aposteriori[i0] += K[i0] * y;
    }

    for (i0 = 0; i0 < 9; i0++) {
      I[i0] = 0;
    }

    for (k = 0; k < 3; k++) {
      I[k + 3 * k] = 1;
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (k = 0; k < 3; k++) {
        b_A[k + 3 * i0] = (real32_T)I[k + 3 * i0] - K[k] * C[i0];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (k = 0; k < 3; k++) {
        P_aposteriori[i0 + 3 * k] = 0.0F;
        for (i1 = 0; i1 < 3; i1++) {
          P_aposteriori[i0 + 3 * k] += b_A[i0 + 3 * i1] * P_apriori[i1 + 3 * k];
        }
      }
    }
  } else {
    for (i0 = 0; i0 < 9; i0++) {
      P_aposteriori[i0] = P_apriori[i0];
    }
  }
}

/* End of code generation (positionKalmanFilter1D.c) */
