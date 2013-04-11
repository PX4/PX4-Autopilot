/*
 * frontFlowKalmanFilter.c
 *
 * Code generation for function 'frontFlowKalmanFilter'
 *
 * C source code generated on: Tue Apr  9 09:52:43 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "frontFlowKalmanFilter.h"
#include "wallEstimationFilter.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void frontFlowKalmanFilter(real32_T dt, real32_T k1, real32_T k2, const real32_T
  flow_aposteriori_k[40], const real32_T speed_aposteriori_k[4], const int16_T
  flow_left_new[10], const int16_T flow_right_new[10], const real32_T speed_new
  [2], int8_T updated, real32_T flow_aposteriori[40], real32_T
  speed_aposteriori[4])
{
  real32_T A[4];
  int32_T i0;
  real32_T K[2];
  int16_T flow[20];
  int16_T flow_updated[20];
  real32_T speed[2];
  int32_T i;
  real32_T y;
  real32_T b_y[2];
  int32_T i1;
  real32_T f0;

  /*  now only a lowpass but possibility for faster estimation for low */
  /*  frequence front flow. */
  A[0] = 1.0F;
  A[2] = dt;
  for (i0 = 0; i0 < 2; i0++) {
    A[1 + (i0 << 1)] = (real32_T)i0;
  }

  /*  we got only new value */
  K[0] = k1;
  K[1] = k2;
  memset(&flow_aposteriori[0], 0, 40U * sizeof(real32_T));
  for (i0 = 0; i0 < 4; i0++) {
    speed_aposteriori[i0] = 0.0F;
  }

  for (i0 = 0; i0 < 20; i0++) {
    flow[i0] = 0;
    flow_updated[i0] = 0;
  }

  for (i0 = 0; i0 < 2; i0++) {
    speed[i0] = 0.0F;
  }

  if (updated != 0) {
    for (i0 = 0; i0 < 5; i0++) {
      flow[i0] = flow_left_new[i0];
    }

    for (i0 = 0; i0 < 10; i0++) {
      flow[i0 + 5] = 0;
    }

    for (i0 = 0; i0 < 5; i0++) {
      flow[i0 + 15] = flow_right_new[5 + i0];
      flow_updated[i0] = flow_left_new[5 + i0];
    }

    for (i0 = 0; i0 < 10; i0++) {
      flow_updated[i0 + 5] = 0;
    }

    for (i0 = 0; i0 < 5; i0++) {
      flow_updated[i0 + 15] = flow_right_new[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      speed[i0] = speed_new[i0];
    }
  }

  for (i = 0; i < 20; i++) {
    if (flow_updated[i] != 0) {
      y = 0.0F;
      for (i0 = 0; i0 < 2; i0++) {
        b_y[i0] = 0.0F;
        for (i1 = 0; i1 < 2; i1++) {
          b_y[i0] += (1.0F - (real32_T)i1) * A[i1 + (i0 << 1)];
        }

        y += b_y[i0] * flow_aposteriori_k[i0 + (i << 1)];
      }

      y = (real32_T)flow[i] - y;
      for (i0 = 0; i0 < 2; i0++) {
        f0 = 0.0F;
        for (i1 = 0; i1 < 2; i1++) {
          f0 += A[i0 + (i1 << 1)] * flow_aposteriori_k[i1 + (i << 1)];
        }

        flow_aposteriori[i0 + (i << 1)] = f0 + K[i0] * y;
      }
    } else {
      for (i0 = 0; i0 < 2; i0++) {
        flow_aposteriori[i0 + (i << 1)] = 0.0F;
        for (i1 = 0; i1 < 2; i1++) {
          flow_aposteriori[i0 + (i << 1)] += A[i0 + (i1 << 1)] *
            flow_aposteriori_k[i1 + (i << 1)];
        }
      }
    }
  }

  for (i = 0; i < 2; i++) {
    if (updated != 0) {
      y = 0.0F;
      for (i0 = 0; i0 < 2; i0++) {
        b_y[i0] = 0.0F;
        for (i1 = 0; i1 < 2; i1++) {
          b_y[i0] += (1.0F - (real32_T)i1) * A[i1 + (i0 << 1)];
        }

        y += b_y[i0] * speed_aposteriori_k[i0 + (i << 1)];
      }

      y = speed[i] - y;
      for (i0 = 0; i0 < 2; i0++) {
        f0 = 0.0F;
        for (i1 = 0; i1 < 2; i1++) {
          f0 += A[i0 + (i1 << 1)] * speed_aposteriori_k[i1 + (i << 1)];
        }

        speed_aposteriori[i0 + (i << 1)] = f0 + K[i0] * y;
      }
    } else {
      for (i0 = 0; i0 < 2; i0++) {
        speed_aposteriori[i0 + (i << 1)] = 0.0F;
        for (i1 = 0; i1 < 2; i1++) {
          speed_aposteriori[i0 + (i << 1)] += A[i0 + (i1 << 1)] *
            speed_aposteriori_k[i1 + (i << 1)];
        }
      }
    }
  }
}

/* End of code generation (frontFlowKalmanFilter.c) */
