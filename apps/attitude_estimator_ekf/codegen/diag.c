/*
 * diag.c
 *
 * Code generation for function 'diag'
 *
 * C source code generated on: Tue Oct 16 15:27:58 2012
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
void b_diag(const real32_T v[9], real32_T d[81])
{
  int32_T j;
  memset(&d[0], 0, 81U * sizeof(real32_T));
  for (j = 0; j < 9; j++) {
    d[j + 9 * j] = v[j];
  }
}

/*
 *
 */
void c_diag(const real32_T v[3], real32_T d[9])
{
  int32_T j;
  for (j = 0; j < 9; j++) {
    d[j] = 0.0F;
  }

  for (j = 0; j < 3; j++) {
    d[j + 3 * j] = v[j];
  }
}

/*
 *
 */
void d_diag(const real32_T v[6], real32_T d[36])
{
  int32_T j;
  memset(&d[0], 0, 36U * sizeof(real32_T));
  for (j = 0; j < 6; j++) {
    d[j + 6 * j] = v[j];
  }
}

/*
 *
 */
void diag(const real32_T v[12], real32_T d[144])
{
  int32_T j;
  memset(&d[0], 0, 144U * sizeof(real32_T));
  for (j = 0; j < 12; j++) {
    d[j + 12 * j] = v[j];
  }
}

/* End of code generation (diag.c) */
