/*
 * sum.c
 *
 * Code generation for function 'sum'
 *
 * C source code generated on: Sun May 19 21:16:10 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "radarControl.h"
#include "sum.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
real_T sum(const int16_T x[3])
{
  real_T y;
  int32_T k;
  y = (real_T)x[0];
  for (k = 0; k < 2; k++) {
    y += (real_T)x[k + 1];
  }

  return y;
}

/* End of code generation (sum.c) */
