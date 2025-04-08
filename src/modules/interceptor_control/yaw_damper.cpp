//
// File: yaw_damper.cpp
//
// Code generated for Simulink model 'yaw_damper'.
//
// Model version                  : 1.1
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Mon Dec 23 22:42:51 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "yaw_damper.h"
#include "rtwtypes_yaw.h"
#include "yaw_damper_private.h"

real32_T look1_iflf_binlxpw_yd(real32_T u0, const real32_T bp0[], const real32_T
  table[], uint32_T maxIndex)
{
  real32_T frac;
  real32_T yL_0d0;
  uint32_T iLeft;

  // Column-major Lookup 1-D
  // Search method: 'binary'
  // Use previous index: 'off'
  // Interpolation method: 'Linear point-slope'
  // Extrapolation method: 'Linear'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Extrapolation method: 'Linear'
  // Use previous index: 'off'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    uint32_T bpIdx;
    uint32_T iRght;

    // Binary Search
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  // Column-major Interpolation 1-D
  // Interpolation method: 'Linear point-slope'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Overflow mode: 'portable wrapping'

  yL_0d0 = table[iLeft];
  return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

// Model step function
void yaw_damper::step(real32_T arg_yaw_rate_command, real32_T arg_yaw_rate_body,
                      real32_T arg_speed_mgnitude, real32_T
                      &arg_rudder_deflection)
{
  // Outport: '<Root>/outport' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
  //   Gain: '<S1>/Gain'
  //   Inport: '<Root>/speed_mgnitude'
  //   Inport: '<Root>/yaw_rate_body'
  //   Lookup_n-D: '<Root>/Kp'
  //   Product: '<Root>/Product1'
  //   Sum: '<Root>/Sum'

  arg_rudder_deflection = (yaw_damper_DW.DiscreteTimeIntegrator_DSTATE -
    arg_yaw_rate_body * look1_iflf_binlxpw_yd(arg_speed_mgnitude,
    yaw_damper_P.Kp_bp01Data, yaw_damper_P.Kp_tableData, 10U)) *
    yaw_damper_P.Gain_Gain;

  // Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' incorporates:
  //   Inport: '<Root>/speed_mgnitude'
  //   Inport: '<Root>/yaw_rate_body'
  //   Inport: '<Root>/yaw_rate_command'
  //   Lookup_n-D: '<Root>/Ki'
  //   Product: '<Root>/Product2'
  //   Sum: '<Root>/Sum1'

  yaw_damper_DW.DiscreteTimeIntegrator_DSTATE += (arg_yaw_rate_command -
    arg_yaw_rate_body) * look1_iflf_binlxpw_yd(arg_speed_mgnitude,
    yaw_damper_P.Ki_bp01Data, yaw_damper_P.Ki_tableData, 10U) *
    yaw_damper_P.DiscreteTimeIntegrator_gainval;
  if (yaw_damper_DW.DiscreteTimeIntegrator_DSTATE >=
      yaw_damper_P.DiscreteTimeIntegrator_UpperSat) {
    yaw_damper_DW.DiscreteTimeIntegrator_DSTATE =
      yaw_damper_P.DiscreteTimeIntegrator_UpperSat;
  } else if (yaw_damper_DW.DiscreteTimeIntegrator_DSTATE <=
             yaw_damper_P.DiscreteTimeIntegrator_LowerSat) {
    yaw_damper_DW.DiscreteTimeIntegrator_DSTATE =
      yaw_damper_P.DiscreteTimeIntegrator_LowerSat;
  }

  // End of Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
}

// Model initialize function
void yaw_damper::initialize()
{
  // InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
  yaw_damper_DW.DiscreteTimeIntegrator_DSTATE =
    yaw_damper_P.DiscreteTimeIntegrator_IC;
}

// Model terminate function
void yaw_damper::terminate()
{
  // (no terminate code required)
}

// Constructor
yaw_damper::yaw_damper() :
  yaw_damper_DW(),
  yaw_damper_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
yaw_damper::~yaw_damper() = default;

// Real-Time Model get method
yaw_damper::RT_MODEL_yaw_damper_T * yaw_damper::getRTM()
{
  return (&yaw_damper_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
