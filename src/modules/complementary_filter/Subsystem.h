/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 *
 * File: Subsystem.h
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

#ifndef Subsystem_h_
#define Subsystem_h_
#ifndef Subsystem_COMMON_INCLUDES_
#define Subsystem_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* Subsystem_COMMON_INCLUDES_ */

#include "Subsystem_types.h"
#include "rtGetNaN.h"
#include <string.h>
#include "rt_defines.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay_DSTATE;             /* '<S1>/Unit Delay' */
  real_T UnitDelay1_DSTATE;            /* '<S1>/Unit Delay1' */
} DW_Subsystem_T;

/* Real-time Model Data Structure */
struct tag_RTM_Subsystem_T {
  const char_T * volatile errorStatus;
  DW_Subsystem_T *dwork;
};

/* Model entry point functions */
extern void Subsystem_initialize(RT_MODEL_Subsystem_T *const Subsystem_M, real_T
  Subsystem_U_accelerometer[3], real_T Subsystem_U_Gyroscope[3], real_T
  *Subsystem_U_Tau, real_T *Subsystem_Y_Roll_filtered, real_T
  *Subsystem_Y_Pitch_filtered);
extern void Subsystem_step(RT_MODEL_Subsystem_T *const Subsystem_M, real_T
  Subsystem_U_accelerometer[3], real_T Subsystem_U_Gyroscope[3], real_T
  Subsystem_U_Tau, real_T *Subsystem_Y_Roll_filtered, real_T
  *Subsystem_Y_Pitch_filtered);
extern void Subsystem_terminate(RT_MODEL_Subsystem_T *const Subsystem_M);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Scope' : Unused code path elimination
 * Block '<S1>/Scope1' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('Complementry_filter_model/Subsystem')    - opens subsystem Complementry_filter_model/Subsystem
 * hilite_system('Complementry_filter_model/Subsystem/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Complementry_filter_model'
 * '<S1>'   : 'Complementry_filter_model/Subsystem'
 * '<S2>'   : 'Complementry_filter_model/Subsystem/MATLAB Function'
 */
#endif                                 /* Subsystem_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
