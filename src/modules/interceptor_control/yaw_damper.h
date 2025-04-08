//
// File: yaw_damper.h
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
#ifndef RTW_HEADER_yaw_damper_h_
#define RTW_HEADER_yaw_damper_h_
#include "rtwtypes_yaw.h"
#include "yaw_damper_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Class declaration for model yaw_damper
class yaw_damper final
{
  // public data and function members
 public:
  // Block states (default storage) for system '<Root>'
  struct DW_yaw_damper_T {
    real32_T DiscreteTimeIntegrator_DSTATE;// '<Root>/Discrete-Time Integrator'
  };

  // Parameters (default storage)
  struct P_yaw_damper_T {
    real32_T DiscreteTimeIntegrator_gainval;
                           // Computed Parameter: DiscreteTimeIntegrator_gainval
                              //  Referenced by: '<Root>/Discrete-Time Integrator'

    real32_T DiscreteTimeIntegrator_IC;
                                // Computed Parameter: DiscreteTimeIntegrator_IC
                                   //  Referenced by: '<Root>/Discrete-Time Integrator'

    real32_T DiscreteTimeIntegrator_UpperSat;
                          // Computed Parameter: DiscreteTimeIntegrator_UpperSat
                             //  Referenced by: '<Root>/Discrete-Time Integrator'

    real32_T DiscreteTimeIntegrator_LowerSat;
                          // Computed Parameter: DiscreteTimeIntegrator_LowerSat
                             //  Referenced by: '<Root>/Discrete-Time Integrator'

    real32_T Kp_tableData[11];         // Computed Parameter: Kp_tableData
                                          //  Referenced by: '<Root>/Kp'

    real32_T Kp_bp01Data[11];          // Computed Parameter: Kp_bp01Data
                                          //  Referenced by: '<Root>/Kp'

    real32_T Gain_Gain;                // Computed Parameter: Gain_Gain
                                          //  Referenced by: '<S1>/Gain'

    real32_T Ki_tableData[11];         // Computed Parameter: Ki_tableData
                                          //  Referenced by: '<Root>/Ki'

    real32_T Ki_bp01Data[11];          // Computed Parameter: Ki_bp01Data
                                          //  Referenced by: '<Root>/Ki'

  };

  // Real-time Model Data Structure
  struct RT_MODEL_yaw_damper_T {
    const char_T * volatile errorStatus;
  };

  // Copy Constructor
  yaw_damper(yaw_damper const&) = delete;

  // Assignment Operator
  yaw_damper& operator= (yaw_damper const&) & = delete;

  // Move Constructor
  yaw_damper(yaw_damper &&) = delete;

  // Move Assignment Operator
  yaw_damper& operator= (yaw_damper &&) = delete;

  // Real-Time Model get method
  yaw_damper::RT_MODEL_yaw_damper_T * getRTM();

  // Tunable parameters
  P_yaw_damper_T yaw_damper_P;

  // model initialize function
  void initialize();

  // model step function
  void step(real32_T arg_yaw_rate_command, real32_T arg_yaw_rate_body, real32_T
            arg_speed_mgnitude, real32_T &arg_rudder_deflection);

  // model terminate function
  static void terminate();

  // Constructor
  yaw_damper();

  // Destructor
  ~yaw_damper();

  // private data and function members
 private:
  // Block states
  DW_yaw_damper_T yaw_damper_DW;

  // Real-Time Model
  RT_MODEL_yaw_damper_T yaw_damper_M;
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'yaw_damper'
//  '<S1>'   : 'yaw_damper/Radians to Degrees'

#endif                                 // RTW_HEADER_yaw_damper_h_

//
// File trailer for generated code.
//
// [EOF]
//
