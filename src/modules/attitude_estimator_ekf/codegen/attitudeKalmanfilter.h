/*
 * attitudeKalmanfilter.h
 *
 * Code generation for function 'attitudeKalmanfilter'
 *
 * C source code generated on: Sat Jan 19 15:25:29 2013
 *
 */

#ifndef __ATTITUDEKALMANFILTER_H__
#define __ATTITUDEKALMANFILTER_H__
/* Include files */
#include <math.h>
#include <stddef.h>
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
extern void attitudeKalmanfilter(const uint8_T updateVect[3], real32_T dt, const real32_T z[9], const real32_T x_aposteriori_k[12], const real32_T P_aposteriori_k[144], const real32_T q[12], real32_T r[9], real32_T eulerAngles[3], real32_T Rot_matrix[9], real32_T x_aposteriori[12], real32_T P_aposteriori[144]);
#endif
/* End of code generation (attitudeKalmanfilter.h) */
