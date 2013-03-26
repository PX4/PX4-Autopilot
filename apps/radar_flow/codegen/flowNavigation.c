/*
 * flowNavigation.c
 *
 * Code generation for function 'flowNavigation'
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

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void flowNavigation(const real32_T left[10], const real32_T right[10], real32_T
                    front_distance, int16_T quality, real32_T speed_x, real32_T
                    speed_y, real32_T *y_setpoint, real32_T *x_setpoint)
{
  /* UNTITLED Summary of this function goes here */
  /*    Detailed explanation goes here */
  *y_setpoint = 0.0F;
  *x_setpoint = 0.0F;
}

/* End of code generation (flowNavigation.c) */
