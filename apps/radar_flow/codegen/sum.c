/*
 * sum.c
 *
 * Code generation for function 'sum'
 *
 * C source code generated on: Thu Mar 14 15:02:19 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "flowNavigation.h"
#include "frontFlowKalmanFilter.h"
#include "wallEstimationFilter.h"
#include "wallEstimator.h"
#include "sum.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
real32_T b_sum(const real32_T x_data[10], const int32_T x_size[1])
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

  return y;
}

real32_T sum(const real32_T x[10])
{
  real32_T y;
  int32_T k;
  y = x[0];
  for (k = 0; k < 9; k++) {
    y += x[k + 1];
  }

  return y;
}

/* End of code generation (sum.c) */
