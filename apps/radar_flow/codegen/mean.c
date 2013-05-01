/*
 * mean.c
 *
 * Code generation for function 'mean'
 *
 * C source code generated on: Tue Apr 30 10:03:54 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "frontFlowKalmanFilter.h"
#include "wallEstimationFilter.h"
#include "mean.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
real32_T mean(const real32_T x_data[10], const int32_T x_size[1])
{
  real32_T y;
  int32_T k;
  if (x_size[0] == 0) {
    y = 0.0F;
  } else {
    y = x_data[0];
    for (k = 2; k <= x_size[0]; k++) {
      y += x_data[k - 1];
    }
  }

  y /= (real32_T)x_size[0];
  return y;
}

/* End of code generation (mean.c) */
