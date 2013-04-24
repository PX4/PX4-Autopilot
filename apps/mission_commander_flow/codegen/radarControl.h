/*
 * radarControl.h
 *
 * Code generation for function 'radarControl'
 *
 * C source code generated on: Wed Apr 24 15:06:04 2013
 *
 */

#ifndef __RADARCONTROL_H__
#define __RADARCONTROL_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rt_nonfinite.h"

#include "rtwtypes.h"
#include "radarControl_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void radarControl(const int16_T radar[32], real32_T front_distance, const real32_T settings[9], real32_T *yaw_control, real32_T *x_control, real32_T *y_control);
#endif
/* End of code generation (radarControl.h) */
