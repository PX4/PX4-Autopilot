/*
 * attitudeKalmanfilter.h
 *
 * Code generation for function 'attitudeKalmanfilter'
 *
 * C source code generated on: Fri Sep 21 13:56:43 2012
 *
 */

#ifndef __ATTITUDEKALMANFILTER_H__
#define __ATTITUDEKALMANFILTER_H__
/* Include files */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"

#include "rtwtypes.h"
#include "attitudeKalmanfilter_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void attitudeKalmanfilter(real32_T dt, const int8_T updVect[9], const real32_T z_k_data[9], const int32_T z_k_sizes[1], const real32_T u[4], const real32_T x_aposteriori_k[9], const real32_T P_aposteriori_k[81], const real32_T knownConst[20], real32_T eulerAngles[3], real32_T Rot_matrix[9], real32_T x_aposteriori[9], real32_T P_aposteriori[81]);
#endif
/* End of code generation (attitudeKalmanfilter.h) */
