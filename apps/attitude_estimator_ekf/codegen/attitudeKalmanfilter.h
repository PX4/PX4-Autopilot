/*
 * attitudeKalmanfilter.h
 *
 * Code generation for function 'attitudeKalmanfilter'
 *
 * C source code generated on: Wed Jul 11 08:38:35 2012
 *
 */

#ifndef __ATTITUDEKALMANFILTER_H__
#define __ATTITUDEKALMANFILTER_H__
/* Include files */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rtwtypes.h"
#include "attitudeKalmanfilter_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void attitudeKalmanfilter(real32_T dt, const real32_T z_k[9], const real32_T x_aposteriori_k[12], const real32_T P_aposteriori_k[144], const real32_T knownConst[7], real32_T Rot_matrix[9], real32_T x_aposteriori[12], real32_T P_aposteriori[144]);
#endif
/* End of code generation (attitudeKalmanfilter.h) */
