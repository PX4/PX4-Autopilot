/*
 * radarControl.h
 *
 * Code generation for function 'radarControl'
 *
 * C source code generated on: Tue Apr 30 22:00:18 2013
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
extern boolean_T radarControl(int16_T radar[32], real32_T front_distance, real32_T front_situation[4], const real32_T settings[9], real32_T x_update, real32_T y_update, real32_T yaw_update, real32_T *x_control, real32_T *y_control, real32_T *yaw_control);
#endif
/* End of code generation (radarControl.h) */
