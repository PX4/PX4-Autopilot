/*
 * eye.c
 *
 * Code generation for function 'eye'
 *
 * C source code generated on: Tue Oct 16 15:27:58 2012
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "attitudeKalmanfilter.h"
#include "eye.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */

/*
 *
 */
void b_eye(real_T I[144])
{
  int32_T i;
  memset(&I[0], 0, 144U * sizeof(real_T));
  for (i = 0; i < 12; i++) {
    I[i + 12 * i] = 1.0;
  }
}

/*
 *
 */
void eye(real_T I[9])
{
  int32_T i;
  memset(&I[0], 0, 9U * sizeof(real_T));
  for (i = 0; i < 3; i++) {
    I[i + 3 * i] = 1.0;
  }
}

/* End of code generation (eye.c) */
