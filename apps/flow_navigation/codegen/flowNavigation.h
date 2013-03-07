/*
 * flowNavigation.h
 *
 * Code generation for function 'flowNavigation'
 *
 * C source code generated on: Thu Mar  7 14:09:14 2013
 *
 */

#ifndef __FLOWNAVIGATION_H__
#define __FLOWNAVIGATION_H__
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
extern void flowNavigation(const real32_T left[10], const real32_T right[10], real32_T front_distance, int16_T quality, real32_T speed_x, real32_T speed_y, real_T *y_setpoint, real_T *x_setpoint);
#endif
/* End of code generation (flowNavigation.h) */
