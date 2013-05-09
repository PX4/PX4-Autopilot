/*
 * mrdivide.h
 *
 * Code generation for function 'mrdivide'
 *
 * C source code generated on: Sat Jan 19 15:25:29 2013
 *
 */

#ifndef __MRDIVIDE_H__
#define __MRDIVIDE_H__
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
extern void b_mrdivide(const real32_T A[36], const real32_T B[9], real32_T y[36]);
extern void c_mrdivide(const real32_T A[72], const real32_T B[36], real32_T y[72]);
extern void mrdivide(const real32_T A[108], const real32_T B[81], real32_T y[108]);
#endif
/* End of code generation (mrdivide.h) */
