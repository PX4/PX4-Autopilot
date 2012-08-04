/*
 * position_estimator.h
 *
 * Code generation for function 'position_estimator'
 *
 * C source code generated on: Fri Jun  8 13:31:21 2012
 *
 */

#ifndef __POSITION_ESTIMATOR_H__
#define __POSITION_ESTIMATOR_H__
/* Include files */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rtwtypes.h"
#include "position_estimator_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void position_estimator(const real32_T u[2], const real32_T z[3], const real32_T xapo[6], const real32_T Papo[36], const real32_T gps_covariance[3], uint8_T predict_only, real32_T xapo1[6], real32_T Papo1[36]);
#endif
/* End of code generation (position_estimator.h) */
