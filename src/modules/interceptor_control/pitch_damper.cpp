//
// File: pitch_damper.cpp
//
// Code generated for Simulink model 'pitch_damper'.
//
// Model version                  : 1.3
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Wed Dec 18 01:41:49 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "pitch_damper.h"
#include "rtwtypes_pitch.h"

// Model step function
void pitch_damper::step(real32_T arg_pitch_rate_command, real32_T arg_pitch_rate,
  real32_T &arg_elevator_deflection) const
{
  // Outport: '<Root>/elevator_deflection' incorporates:
  //   Gain: '<Root>/Gain'
  //   Gain: '<S1>/Gain'
  //   Inport: '<Root>/pitch_rate'
  //   Inport: '<Root>/pitch_rate_command'
  //   Sum: '<Root>/Sum1'

  arg_elevator_deflection = (arg_pitch_rate_command - arg_pitch_rate) *
    pitch_damper_P.Gain_Gain * pitch_damper_P.Gain_Gain_l;
}

// Model initialize function
void pitch_damper::initialize()
{
  // (no initialization code required)
}

// Model terminate function
void pitch_damper::terminate()
{
  // (no terminate code required)
}

// Constructor
pitch_damper::pitch_damper() :
  pitch_damper_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
pitch_damper::~pitch_damper() = default;

// Real-Time Model get method
pitch_damper::RT_MODEL_pitch_damper_T * pitch_damper::getRTM()
{
  return (&pitch_damper_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
