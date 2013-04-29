/*
 * radarControl.h
 *
 * Code generation for function 'radarControl'
 *
 * C source code generated on: Mon Apr 29 11:05:59 2013
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
extern boolean_T radarControl(int16_T radar[32], int16_T front_distance, int16_T front_situation[17], const real32_T settings[9], real32_T *yaw_control, real32_T *x_control, real32_T *y_control);
#endif
/* End of code generation (radarControl.h) */
