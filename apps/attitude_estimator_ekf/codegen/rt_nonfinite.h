/*
 * rt_nonfinite.h
 *
 * Code generation for function 'attitudeKalmanfilter'
 *
 * C source code generated on: Tue Oct 16 15:27:58 2012
 *
 */

#ifndef __RT_NONFINITE_H__
#define __RT_NONFINITE_H__

#if defined(_MSC_VER) && (_MSC_VER <= 1200)
#include <float.h>
#endif
#include <stddef.h>
#include "rtwtypes.h"

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
extern void rt_InitInfAndNaN(size_t realSize);
extern boolean_T rtIsInf(real_T value);
extern boolean_T rtIsInfF(real32_T value);
extern boolean_T rtIsNaN(real_T value);
extern boolean_T rtIsNaNF(real32_T value);

typedef struct {
  struct {
    uint32_T wordH;
    uint32_T wordL;
  } words;
} BigEndianIEEEDouble;

typedef struct {
  struct {
    uint32_T wordL;
    uint32_T wordH;
  } words;
} LittleEndianIEEEDouble;

typedef struct {
  union {
    real32_T wordLreal;
    uint32_T wordLuint;
  } wordL;
} IEEESingle;

#endif
/* End of code generation (rt_nonfinite.h) */
