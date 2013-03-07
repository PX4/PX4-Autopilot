/*
 * wallEstimationFilter.h
 *
 * Code generation for function 'wallEstimationFilter'
 *
 * C source code generated on: Thu Mar  7 14:09:14 2013
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
#include "flowNavigation_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void wallEstimationFilter(const real32_T radar_filtered_k[32], const real32_T radar_weights_k[32], const real32_T flow_left[10], const real32_T flow_right[10], real32_T front_distance, uint16_T quality, const real32_T speed[2], const real32_T thresholds[3], real32_T radar[32], real32_T radar_filtered[32], real32_T radar_weights[32]);
#endif
/* End of code generation (wallEstimationFilter.h) */
