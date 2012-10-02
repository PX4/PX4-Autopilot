/*
 * norm.c
 *
 * Code generation for function 'norm'
 *
 * C source code generated on: Mon Oct 01 19:38:49 2012
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "attitudeKalmanfilter.h"
#include "norm.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */

/*
 *
 */
real32_T norm(const real32_T x[3])
{
  real32_T y;
  real32_T scale;
  boolean_T firstNonZero;
  int32_T k;
  real32_T absxk;
  real32_T t;
  y = 0.0F;
  scale = 0.0F;
  firstNonZero = TRUE;
  for (k = 0; k < 3; k++) {
    if (x[k] != 0.0F) {
      absxk = (real32_T)fabsf(x[k]);
      if (firstNonZero) {
        scale = absxk;
        y = 1.0F;
        firstNonZero = FALSE;
      } else if (scale < absxk) {
        t = scale / absxk;
        y = 1.0F + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }
  }

  return scale * (real32_T)sqrtf(y);
}

/* End of code generation (norm.c) */
