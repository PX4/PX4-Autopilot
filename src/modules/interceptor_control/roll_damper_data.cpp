//
// File: roll_damper_data.cpp
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

// Block parameters (default storage)
roll_damper::P_roll_damper_T roll_damper::roll_damper_P{
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

  // Computed Parameter: Kp_roll_damper_tableData
  //  Referenced by: '<Root>/Kp_roll_damper'

  { 0.5F, 0.2F, 0.025F, 0.015F, 0.015F, 0.0085F, 0.0065F, 0.0045F, 0.0025F,
    0.0015F, 0.00105F },

  // Computed Parameter: Kp_roll_damper_bp01Data
  //  Referenced by: '<Root>/Kp_roll_damper'

  { 25.0F, 50.0F, 100.0F, 160.0F, 170.0F, 200.0F, 250.0F, 275.0F, 290.0F, 350.0F,
    450.0F },

  // Computed Parameter: Gain_Gain
  //  Referenced by: '<Root>/Gain'

  -1.0F,

  // Computed Parameter: Gain_Gain_b
  //  Referenced by: '<S1>/Gain'

  57.2957802F,

  // Computed Parameter: Ki_roll_damper_tableData
  //  Referenced by: '<Root>/Ki_roll_damper'

  { 0.0F, 0.0F, 0.2F, 0.1F, 0.08F, 0.062F, 0.031F, 0.026F, 0.014F, 0.01F, 0.008F
  },

  // Computed Parameter: Ki_roll_damper_bp01Data
  //  Referenced by: '<Root>/Ki_roll_damper'

  { 25.0F, 50.0F, 100.0F, 150.0F, 160.0F, 170.0F, 200.0F, 250.0F, 300.0F, 400.0F,
    450.0F }
};

//
// File trailer for generated code.
//
// [EOF]
//
