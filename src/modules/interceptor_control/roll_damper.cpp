//
// File: roll_damper.cpp
//
// Code generated for Simulink model 'roll_damper'.
//
// Model version                  : 1.1
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Mon Dec 23 22:53:04 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "roll_damper.h"
#include "rtwtypes_roll.h"
#include "roll_damper_private.h"

real32_T look1_iflf_binlxpw_rd(real32_T u0, const real32_T bp0[], const real32_T
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
void roll_damper::step(real32_T arg_roll_rate_command, real32_T arg_roll_rate,
  real32_T arg_speed_magnitude, real32_T &arg_aileron_deflection)
{
  // Outport: '<Root>/aileron_deflection' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
  //   Gain: '<Root>/Gain'
  //   Gain: '<S1>/Gain'
  //   Inport: '<Root>/roll_rate'
  //   Inport: '<Root>/speed_magnitude'
  //   Lookup_n-D: '<Root>/Kp_roll_damper'
  //   Product: '<Root>/Product1'
  //   Sum: '<Root>/Sum'

  arg_aileron_deflection = (roll_damper_DW.DiscreteTimeIntegrator_DSTATE -
    arg_roll_rate * look1_iflf_binlxpw_rd(arg_speed_magnitude,
    roll_damper_P.Kp_roll_damper_bp01Data,
    roll_damper_P.Kp_roll_damper_tableData, 10U)) * roll_damper_P.Gain_Gain *
    roll_damper_P.Gain_Gain_b;

  // Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' incorporates:
  //   Inport: '<Root>/roll_rate'
  //   Inport: '<Root>/roll_rate_command'
  //   Inport: '<Root>/speed_magnitude'
  //   Lookup_n-D: '<Root>/Ki_roll_damper'
  //   Product: '<Root>/Product2'
  //   Sum: '<Root>/Sum1'

  roll_damper_DW.DiscreteTimeIntegrator_DSTATE += (arg_roll_rate_command -
    arg_roll_rate) * look1_iflf_binlxpw_rd(arg_speed_magnitude,
    roll_damper_P.Ki_roll_damper_bp01Data,
    roll_damper_P.Ki_roll_damper_tableData, 10U) *
    roll_damper_P.DiscreteTimeIntegrator_gainval;
  if (roll_damper_DW.DiscreteTimeIntegrator_DSTATE >=
      roll_damper_P.DiscreteTimeIntegrator_UpperSat) {
    roll_damper_DW.DiscreteTimeIntegrator_DSTATE =
      roll_damper_P.DiscreteTimeIntegrator_UpperSat;
  } else if (roll_damper_DW.DiscreteTimeIntegrator_DSTATE <=
             roll_damper_P.DiscreteTimeIntegrator_LowerSat) {
    roll_damper_DW.DiscreteTimeIntegrator_DSTATE =
      roll_damper_P.DiscreteTimeIntegrator_LowerSat;
  }

  // End of Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
}

// Model initialize function
void roll_damper::initialize()
{
  // InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
  roll_damper_DW.DiscreteTimeIntegrator_DSTATE =
    roll_damper_P.DiscreteTimeIntegrator_IC;
}

// Model terminate function
void roll_damper::terminate()
{
  // (no terminate code required)
}

// Constructor
roll_damper::roll_damper() :
  roll_damper_DW(),
  roll_damper_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
roll_damper::~roll_damper() = default;

// Real-Time Model get method
roll_damper::RT_MODEL_roll_damper_T * roll_damper::getRTM()
{
  return (&roll_damper_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
