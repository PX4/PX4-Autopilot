/*
 * power.c
 *
 * Code generation for function 'power'
 *
 * C source code generated on: Mon Oct 01 19:38:49 2012
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "attitudeKalmanfilter.h"
#include "power.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static real32_T rt_powf_snf(real32_T u0, real32_T u1);

/* Function Definitions */
static real32_T rt_powf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  real32_T f0;
  real32_T f1;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = ((real32_T)rtNaN);
  } else {
    f0 = (real32_T)fabsf(u0);
    f1 = (real32_T)fabsf(u1);
    if (rtIsInfF(u1)) {
      if (f0 == 1.0F) {
        y = ((real32_T)rtNaN);
      } else if (f0 > 1.0F) {
        if (u1 > 0.0F) {
          y = ((real32_T)rtInf);
        } else {
          y = 0.0F;
        }
      } else if (u1 > 0.0F) {
        y = 0.0F;
      } else {
        y = ((real32_T)rtInf);
      }
    } else if (f1 == 0.0F) {
      y = 1.0F;
    } else if (f1 == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
      y = (real32_T)sqrtf(u0);
    } else if ((u0 < 0.0F) && (u1 > (real32_T)floorf(u1))) {
      y = ((real32_T)rtNaN);
    } else {
      y = (real32_T)powf(u0, u1);
    }
  }

  return y;
}

/*
 *
 */
void power(const real32_T a[12], real_T b, real32_T y[12])
{
  int32_T k;
  for (k = 0; k < 12; k++) {
    y[k] = rt_powf_snf(a[k], (real32_T)b);
  }
}

/* End of code generation (power.c) */
