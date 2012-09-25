/*
 * eye.c
 *
 * Code generation for function 'eye'
 *
 * C source code generated on: Fri Sep 21 13:56:43 2012
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
void b_eye(real_T I[81])
{
  int32_T i;
  memset((void *)&I[0], 0, 81U * sizeof(real_T));
  for (i = 0; i < 9; i++) {
    I[i + 9 * i] = 1.0;
  }
}

/*
 *
 */
void eye(real_T I[9])
{
  int32_T i;
  memset((void *)&I[0], 0, 9U * sizeof(real_T));
  for (i = 0; i < 3; i++) {
    I[i + 3 * i] = 1.0;
  }
}

/* End of code generation (eye.c) */
