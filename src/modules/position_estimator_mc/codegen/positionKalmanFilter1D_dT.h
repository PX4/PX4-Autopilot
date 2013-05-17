/*
 * positionKalmanFilter1D_dT.h
 *
 * Code generation for function 'positionKalmanFilter1D_dT'
 *
 * C source code generated on: Fri Nov 30 17:37:33 2012
 *
 */

#ifndef __POSITIONKALMANFILTER1D_DT_H__
#define __POSITIONKALMANFILTER1D_DT_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#include "rtwtypes.h"
#include "positionKalmanFilter1D_dT_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void positionKalmanFilter1D_dT(real32_T dT, const real32_T x_aposteriori_k[3], const real32_T P_aposteriori_k[9], real32_T u, real32_T z, uint8_T gps_update, const real32_T Q[9], real_T R, real32_T thresh, real32_T decay, real32_T x_aposteriori[3], real32_T P_aposteriori[9]);
#endif
/* End of code generation (positionKalmanFilter1D_dT.h) */
