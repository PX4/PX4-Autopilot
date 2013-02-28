/*
 * frontFlowKalmanFilter.h
 *
 * Code generation for function 'frontFlowKalmanFilter'
 *
 * C source code generated on: Thu Feb 28 10:58:05 2013
 *
 */

#ifndef __FRONTFLOWKALMANFILTER_H__
#define __FRONTFLOWKALMANFILTER_H__
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
extern void frontFlowKalmanFilter(real32_T dt, real32_T k1, real32_T k2, const real32_T flow_aposteriori_k[40], const real32_T speed_aposteriori_k[4], const int16_T flow_left_new[10], const int16_T flow_right_new[10], const real32_T speed_new[2], int8_T updated, real32_T flow_aposteriori[40], real32_T speed_aposteriori[4]);
#endif
/* End of code generation (frontFlowKalmanFilter.h) */
