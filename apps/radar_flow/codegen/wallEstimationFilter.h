/*
 * wallEstimationFilter.h
 *
 * Code generation for function 'wallEstimationFilter'
 *
 * C source code generated on: Wed Apr 24 14:59:21 2013
 *
 */

#ifndef __WALLESTIMATIONFILTER_H__
#define __WALLESTIMATIONFILTER_H__
/* Include files */
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"

#include "rtwtypes.h"
#include "wallEstimationFilter_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void wallEstimationFilter(const real32_T radar_filtered_k[32], const real32_T radar_weights_k[32], const real32_T flow_left[10], const real32_T flow_right[10], real32_T front_distance, const real32_T speed[2], const real32_T position_update[2], real32_T attitude_update, const real32_T settings[8], boolean_T use_sonar, real32_T radar[32], real32_T radar_filtered[32], real32_T radar_weights[32]);
#endif
/* End of code generation (wallEstimationFilter.h) */
