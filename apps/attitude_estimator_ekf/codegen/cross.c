/*
 * cross.c
 *
 * Code generation for function 'cross'
 *
 * C source code generated on: Tue Oct 16 15:27:58 2012
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "attitudeKalmanfilter.h"
#include "cross.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */

/*
 *
 */
void cross(const real32_T a[3], const real32_T b[3], real32_T c[3])
{
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

/* End of code generation (cross.c) */
