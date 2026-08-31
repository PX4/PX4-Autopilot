/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 *
 * File: Subsystem.c
 *
 * Code generated for Simulink model 'Subsystem'.
 *
 * Model version                  : 1.12
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Tue Aug 25 17:28:53 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Subsystem.h"
#include <math.h>
#include "Subsystem_private.h"
#include "rtwtypes.h"
#include <string.h>
#include "rt_nonfinite.h"
#include "rt_defines.h"

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Model step function */
void Subsystem_step(RT_MODEL_Subsystem_T *const Subsystem_M, real_T
                    Subsystem_U_accelerometer[3], real_T Subsystem_U_Gyroscope[3],
                    real_T Subsystem_U_Tau, real_T *Subsystem_Y_Roll_filtered,
                    real_T *Subsystem_Y_Pitch_filtered)
{
  DW_Subsystem_T *Subsystem_DW = Subsystem_M->dwork;
  real_T UnitDelay_DSTATE_tmp;
  real_T rtb_theta_dot_tmp;
  real_T rtb_theta_dot_tmp_0;

  /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
   *  UnitDelay: '<S1>/Unit Delay'
   *  UnitDelay: '<S1>/Unit Delay1'
   */
  rtb_theta_dot_tmp = sin(Subsystem_DW->UnitDelay_DSTATE);
  rtb_theta_dot_tmp_0 = cos(Subsystem_DW->UnitDelay_DSTATE);
  UnitDelay_DSTATE_tmp = tan(Subsystem_DW->UnitDelay1_DSTATE);

  /* Sum: '<S1>/Add' incorporates:
   *  Gain: '<S1>/Gain1'
   *  Inport: '<Root>/Gyroscope'
   *  MATLAB Function: '<S1>/MATLAB Function'
   *  UnitDelay: '<S1>/Unit Delay'
   */
  Subsystem_DW->UnitDelay_DSTATE += ((rtb_theta_dot_tmp * UnitDelay_DSTATE_tmp *
    Subsystem_U_Gyroscope[1] + Subsystem_U_Gyroscope[0]) + rtb_theta_dot_tmp_0 *
    UnitDelay_DSTATE_tmp * Subsystem_U_Gyroscope[2]) * 0.004;

  /* Outport: '<Root>/Roll_filtered' incorporates:
   *  Constant: '<S1>/Constant'
   *  Inport: '<Root>/Tau'
   *  Inport: '<Root>/accelerometer'
   *  Product: '<S1>/Product'
   *  Product: '<S1>/Product1'
   *  Sum: '<S1>/Add2'
   *  Sum: '<S1>/Sum1'
   *  Trigonometry: '<S1>/Atan2'
   *  UnitDelay: '<S1>/Unit Delay'
   */
  *Subsystem_Y_Roll_filtered = (1.0 - Subsystem_U_Tau) * rt_atan2d_snf
    (Subsystem_U_accelerometer[1], Subsystem_U_accelerometer[2]) +
    Subsystem_DW->UnitDelay_DSTATE * Subsystem_U_Tau;

  /* Sum: '<S1>/Add1' incorporates:
   *  Gain: '<S1>/Gain2'
   *  Inport: '<Root>/Gyroscope'
   *  MATLAB Function: '<S1>/MATLAB Function'
   *  UnitDelay: '<S1>/Unit Delay1'
   */
  Subsystem_DW->UnitDelay1_DSTATE += (rtb_theta_dot_tmp_0 *
    Subsystem_U_Gyroscope[1] - rtb_theta_dot_tmp * Subsystem_U_Gyroscope[2]) *
    0.004;

  /* Outport: '<Root>/Pitch_filtered' incorporates:
   *  Constant: '<S1>/Constant1'
   *  Gain: '<S1>/Gain'
   *  Inport: '<Root>/Tau'
   *  Inport: '<Root>/accelerometer'
   *  Math: '<S1>/Square'
   *  Math: '<S1>/Square1'
   *  Product: '<S1>/Product2'
   *  Product: '<S1>/Product3'
   *  Sqrt: '<S1>/Sqrt'
   *  Sum: '<S1>/Add3'
   *  Sum: '<S1>/Sum'
   *  Sum: '<S1>/Sum2'
   *  Trigonometry: '<S1>/Atan1'
   *  UnitDelay: '<S1>/Unit Delay1'
   */
  *Subsystem_Y_Pitch_filtered = rt_atan2d_snf(-Subsystem_U_accelerometer[0],
    sqrt(Subsystem_U_accelerometer[1] * Subsystem_U_accelerometer[1] +
         Subsystem_U_accelerometer[2] * Subsystem_U_accelerometer[2])) * (1.0 -
    Subsystem_U_Tau) + Subsystem_DW->UnitDelay1_DSTATE * Subsystem_U_Tau;
}

/* Model initialize function */
void Subsystem_initialize(RT_MODEL_Subsystem_T *const Subsystem_M, real_T
  Subsystem_U_accelerometer[3], real_T Subsystem_U_Gyroscope[3], real_T
  *Subsystem_U_Tau, real_T *Subsystem_Y_Roll_filtered, real_T
  *Subsystem_Y_Pitch_filtered)
{
  DW_Subsystem_T *Subsystem_DW = Subsystem_M->dwork;

  /* Registration code */

  /* states (dwork) */
  (void) memset((void *)Subsystem_DW, 0,
                sizeof(DW_Subsystem_T));

  /* external inputs */
  (void)memset(&Subsystem_U_accelerometer[0], 0, 3U * sizeof(real_T));
  (void)memset(&Subsystem_U_Gyroscope[0], 0, 3U * sizeof(real_T));
  *Subsystem_U_Tau = 0.0;

  /* external outputs */
  *Subsystem_Y_Roll_filtered = 0.0;
  *Subsystem_Y_Pitch_filtered = 0.0;
}

/* Model terminate function */
void Subsystem_terminate(RT_MODEL_Subsystem_T *const Subsystem_M)
{
  /* (no terminate code required) */
  UNUSED_PARAMETER(Subsystem_M);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
