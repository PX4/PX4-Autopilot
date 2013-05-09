/*
 * kalman_dlqe3_initialize.c
 *
 * Code generation for function 'kalman_dlqe3_initialize'
 *
 * C source code generated on: Tue Feb 19 15:26:31 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "kalman_dlqe3.h"
#include "kalman_dlqe3_initialize.h"
#include "kalman_dlqe3_data.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void kalman_dlqe3_initialize(void)
{
  int32_T i;
  static const uint32_T uv0[2] = { 362436069U, 0U };

  rt_InitInfAndNaN(8U);
  state_not_empty = FALSE;
  b_state = 1144108930U;
  b_method = 7U;
  method = 0U;
  for (i = 0; i < 2; i++) {
    c_state[i] = 362436069U + 158852560U * (uint32_T)i;
    state[i] = uv0[i];
  }

  if (state[1] == 0U) {
    state[1] = 521288629U;
  }
}

/* End of code generation (kalman_dlqe3_initialize.c) */
