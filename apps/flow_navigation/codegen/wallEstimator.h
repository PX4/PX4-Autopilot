/*
 * wallEstimator.h
 *
 * Code generation for function 'wallEstimator'
 *
 * C source code generated on: Fri Mar  8 13:08:40 2013
 *
 */

#ifndef __WALLESTIMATOR_H__
#define __WALLESTIMATOR_H__
/* Include files */
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"

#include "rtwtypes.h"
#include "flowNavigation_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void wallEstimator(const real32_T flow_left[10], const real32_T flow_right[10], real32_T front_distance, int16_T quality, const real32_T speed[2], const real32_T thresholds[3], real32_T *dist_left, real32_T *dist_right);
#endif
/* End of code generation (wallEstimator.h) */
