/*
 * kalman_dlqe1.h
 *
 * Code generation for function 'kalman_dlqe1'
 *
 * C source code generated on: Wed Feb 13 20:34:32 2013
 *
 */

#ifndef __KALMAN_DLQE1_H__
#define __KALMAN_DLQE1_H__
/* Include files */
#include <stddef.h>
#include <stdlib.h>

#include "rtwtypes.h"
#include "kalman_dlqe1_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void kalman_dlqe1(const real32_T A[9], const real32_T C[3], const real32_T K[3], const real32_T x_aposteriori_k[3], real32_T z, real32_T x_aposteriori[3]);
#endif
/* End of code generation (kalman_dlqe1.h) */
