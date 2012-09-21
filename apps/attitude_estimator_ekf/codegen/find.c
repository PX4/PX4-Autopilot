/*
 * find.c
 *
 * Code generation for function 'find'
 *
 * C source code generated on: Fri Sep 21 13:56:43 2012
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "attitudeKalmanfilter.h"
#include "find.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */

/*
 *
 */
void find(const int8_T x[9], real_T i_data[9], int32_T i_sizes[1])
{
  int32_T idx;
  int32_T ii;
  boolean_T exitg1;
  boolean_T guard1 = FALSE;
  int32_T i2;
  int8_T b_i_data[9];
  idx = 0;
  i_sizes[0] = 9;
  ii = 1;
  exitg1 = 0U;
  while ((exitg1 == 0U) && (ii <= 9)) {
    guard1 = FALSE;
    if (x[ii - 1] != 0) {
      idx++;
      i_data[idx - 1] = (real_T)ii;
      if (idx >= 9) {
        exitg1 = 1U;
      } else {
        guard1 = TRUE;
      }
    } else {
      guard1 = TRUE;
    }

    if (guard1 == TRUE) {
      ii++;
    }
  }

  if (1 > idx) {
    idx = 0;
  }

  ii = idx - 1;
  for (i2 = 0; i2 <= ii; i2++) {
    b_i_data[i2] = (int8_T)i_data[i2];
  }

  i_sizes[0] = idx;
  ii = idx - 1;
  for (i2 = 0; i2 <= ii; i2++) {
    i_data[i2] = (real_T)b_i_data[i2];
  }
}

/* End of code generation (find.c) */
