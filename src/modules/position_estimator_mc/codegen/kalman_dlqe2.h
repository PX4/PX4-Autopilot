/*
 * kalman_dlqe2.h
 *
 * Code generation for function 'kalman_dlqe2'
 *
 * C source code generated on: Thu Feb 14 12:52:29 2013
 *
 */

#ifndef __KALMAN_DLQE2_H__
#define __KALMAN_DLQE2_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rt_nonfinite.h"

#include "rtwtypes.h"
#include "kalman_dlqe2_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void kalman_dlqe2(real32_T dt, real32_T k1, real32_T k2, real32_T k3, const real32_T x_aposteriori_k[3], real32_T z, real32_T x_aposteriori[3]);
#endif
/* End of code generation (kalman_dlqe2.h) */
