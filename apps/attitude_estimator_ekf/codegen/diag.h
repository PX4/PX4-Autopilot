/*
 * diag.h
 *
 * Code generation for function 'diag'
 *
 * C source code generated on: Tue Oct 16 15:27:58 2012
 *
 */

#ifndef __DIAG_H__
#define __DIAG_H__
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
extern void b_diag(const real32_T v[9], real32_T d[81]);
extern void c_diag(const real32_T v[3], real32_T d[9]);
extern void d_diag(const real32_T v[6], real32_T d[36]);
extern void diag(const real32_T v[12], real32_T d[144]);
#endif
/* End of code generation (diag.h) */
