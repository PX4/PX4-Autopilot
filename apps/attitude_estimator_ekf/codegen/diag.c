/*
 * diag.c
 *
 * Code generation for function 'diag'
 *
 * C source code generated on: Fri Sep 21 13:56:43 2012
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "attitudeKalmanfilter.h"
#include "diag.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */

/*
 *
 */
void diag(const real32_T v[3], real32_T d[9])
{
  int32_T j;
  for (j = 0; j < 9; j++) {
    d[j] = 0.0F;
  }

  for (j = 0; j < 3; j++) {
    d[j + 3 * j] = v[j];
  }
}

/* End of code generation (diag.c) */
