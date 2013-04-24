/*
 * kalman_dlqe3.h
 *
 * Code generation for function 'kalman_dlqe3'
 *
 * C source code generated on: Tue Feb 19 15:26:32 2013
 *
 */

#ifndef __KALMAN_DLQE3_H__
#define __KALMAN_DLQE3_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"

#include "rtwtypes.h"
#include "kalman_dlqe3_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void kalman_dlqe3(real32_T dt, real32_T k1, real32_T k2, real32_T k3, const real32_T x_aposteriori_k[3], real32_T z, real32_T posUpdate, real32_T addNoise, real32_T sigma, real32_T x_aposteriori[3]);
#endif
/* End of code generation (kalman_dlqe3.h) */
