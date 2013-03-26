/*
 * power.c
 *
 * Code generation for function 'power'
 *
 * C source code generated on: Thu Mar 14 15:02:19 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "flowNavigation.h"
#include "frontFlowKalmanFilter.h"
#include "wallEstimationFilter.h"
#include "wallEstimator.h"
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
  real32_T f1;
  real32_T f2;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = ((real32_T)rtNaN);
  } else {
    f1 = (real32_T)fabs(u0);
    f2 = (real32_T)fabs(u1);
    if (rtIsInfF(u1)) {
      if (f1 == 1.0F) {
        y = ((real32_T)rtNaN);
      } else if (f1 > 1.0F) {
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
    } else if (f2 == 0.0F) {
      y = 1.0F;
    } else if (f2 == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
      y = (real32_T)sqrt(u0);
    } else if ((u0 < 0.0F) && (u1 > (real32_T)floor(u1))) {
      y = ((real32_T)rtNaN);
    } else {
      y = (real32_T)pow(u0, u1);
    }
  }

  return y;
}

void b_power(const real32_T a_data[10], const int32_T a_size[1], real32_T
             y_data[10], int32_T y_size[1])
{
  int32_T k;
  y_size[0] = (int8_T)a_size[0];
  for (k = 0; k < (int8_T)a_size[0]; k++) {
    y_data[k] = rt_powf_snf(a_data[k], 2.0F);
  }
}

void power(const real32_T a[10], real32_T y[10])
{
  int32_T k;
  for (k = 0; k < 10; k++) {
    y[k] = rt_powf_snf(a[k], 2.0F);
  }
}

/* End of code generation (power.c) */
