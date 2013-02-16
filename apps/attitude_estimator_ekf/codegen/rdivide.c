/*
 * rdivide.c
 *
 * Code generation for function 'rdivide'
 *
 * C source code generated on: Sat Jan 19 15:25:29 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "attitudeKalmanfilter.h"
#include "rdivide.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */

/*
 *
 */
void rdivide(const real32_T x[3], real32_T y, real32_T z[3])
{
  int32_T i;
  for (i = 0; i < 3; i++) {
    z[i] = x[i] / y;
  }
}

/* End of code generation (rdivide.c) */
