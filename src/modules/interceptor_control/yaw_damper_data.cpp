//
// File: yaw_damper_data.cpp
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

// Block parameters (default storage)
yaw_damper::P_yaw_damper_T yaw_damper::yaw_damper_P{
  // Computed Parameter: DiscreteTimeIntegrator_gainval
  //  Referenced by: '<Root>/Discrete-Time Integrator'

  0.0025F,

  // Computed Parameter: DiscreteTimeIntegrator_IC
  //  Referenced by: '<Root>/Discrete-Time Integrator'

  0.0F,

  // Computed Parameter: DiscreteTimeIntegrator_UpperSat
  //  Referenced by: '<Root>/Discrete-Time Integrator'

  0.0872664601F,

  // Computed Parameter: DiscreteTimeIntegrator_LowerSat
  //  Referenced by: '<Root>/Discrete-Time Integrator'

  -0.0872664601F,

  // Computed Parameter: Kp_tableData
  //  Referenced by: '<Root>/Kp'

  { 0.5F, 0.8F, 0.2F, 0.1F, 0.1F, 0.035F, 0.025F, 0.015F, 0.0125F, 0.0115F,
    0.0105F },

  // Computed Parameter: Kp_bp01Data
  //  Referenced by: '<Root>/Kp'

  { 25.0F, 50.0F, 100.0F, 160.0F, 170.0F, 200.0F, 250.0F, 275.0F, 290.0F, 350.0F,
    450.0F },

  // Computed Parameter: Gain_Gain
  //  Referenced by: '<S1>/Gain'

  57.2957802F,

  // Computed Parameter: Ki_tableData
  //  Referenced by: '<Root>/Ki'

  { 0.0F, 0.0F, 1.1F, 0.65F, 0.55F, 0.4F, 0.2F, 0.1F, 0.1F, 0.1F, 0.1F },

  // Computed Parameter: Ki_bp01Data
  //  Referenced by: '<Root>/Ki'

  { 25.0F, 50.0F, 100.0F, 150.0F, 160.0F, 170.0F, 200.0F, 250.0F, 300.0F, 400.0F,
    450.0F }
};

//
// File trailer for generated code.
//
// [EOF]
//
