#include <systemlib/param/param.h>

// currently tuned for easystar from arkhangar in HIL
//https://github.com/arktools/arkhangar

// 16 is max name length

// gyro low pass filter
PARAM_DEFINE_FLOAT(FWB_P_LP, 300.0f); // roll rate low pass cut freq
PARAM_DEFINE_FLOAT(FWB_Q_LP, 300.0f); // pitch rate low pass cut freq
PARAM_DEFINE_FLOAT(FWB_R_LP, 300.0f); // yaw rate low pass cut freq

// yaw washout
PARAM_DEFINE_FLOAT(FWB_R_HP, 1.0f); // yaw rate high pass

// stabilization mode
PARAM_DEFINE_FLOAT(FWB_P2AIL, 0.3f); // roll rate 2 aileron
PARAM_DEFINE_FLOAT(FWB_Q2ELV, 0.1f); // pitch rate 2 elevator
PARAM_DEFINE_FLOAT(FWB_R2RDR, 0.1f); // yaw rate 2 rudder

//  psi -> phi -> p
PARAM_DEFINE_FLOAT(FWB_PSI2PHI, 0.5f);      // heading 2 roll
PARAM_DEFINE_FLOAT(FWB_PHI2P, 1.0f);        // roll to roll rate
PARAM_DEFINE_FLOAT(FWB_PHI_LIM_MAX, 0.3f);  // roll limit, 28 deg

// velocity -> theta
PARAM_DEFINE_FLOAT(FWB_V2THE_P, 1.0f);      // velocity to pitch angle PID, prop gain
PARAM_DEFINE_FLOAT(FWB_V2THE_I, 0.0f);      // integral gain
PARAM_DEFINE_FLOAT(FWB_V2THE_D, 0.0f);      // derivative gain
PARAM_DEFINE_FLOAT(FWB_V2THE_D_LP, 0.0f);   // derivative low-pass
PARAM_DEFINE_FLOAT(FWB_V2THE_I_MAX, 0.0f);  // integrator wind up guard
PARAM_DEFINE_FLOAT(FWB_THE_MIN, -0.5f);     // the max commanded pitch angle
PARAM_DEFINE_FLOAT(FWB_THE_MAX, 0.5f);      // the min commanded pitch angle


// theta -> q
PARAM_DEFINE_FLOAT(FWB_THE2Q_P, 1.0f);      // pitch angle to pitch-rate PID
PARAM_DEFINE_FLOAT(FWB_THE2Q_I, 0.0f);
PARAM_DEFINE_FLOAT(FWB_THE2Q_D, 0.0f);
PARAM_DEFINE_FLOAT(FWB_THE2Q_D_LP, 0.0f);
PARAM_DEFINE_FLOAT(FWB_THE2Q_I_MAX, 0.0f);

//  h -> thr
PARAM_DEFINE_FLOAT(FWB_H2THR_P, 0.01f);     // altitude to throttle PID
PARAM_DEFINE_FLOAT(FWB_H2THR_I, 0.0f);
PARAM_DEFINE_FLOAT(FWB_H2THR_D, 0.0f);
PARAM_DEFINE_FLOAT(FWB_H2THR_D_LP, 0.0f);
PARAM_DEFINE_FLOAT(FWB_H2THR_I_MAX, 0.0f);

// crosstrack
PARAM_DEFINE_FLOAT(FWB_XT2YAW_MAX, 1.57f);  // cross-track to yaw angle limit 90 deg
PARAM_DEFINE_FLOAT(FWB_XT2YAW, 0.005f);     // cross-track to yaw angle gain

// speed command
PARAM_DEFINE_FLOAT(FWB_V_MIN, 10.0f);       // minimum commanded velocity
PARAM_DEFINE_FLOAT(FWB_V_CMD, 12.0f);       // commanded velocity
PARAM_DEFINE_FLOAT(FWB_V_MAX, 16.0f);       // maximum commanded velocity

// rate of climb
// this is what rate of climb is commanded (in m/s)
// when the pitch stick is fully defelcted in simple mode
PARAM_DEFINE_FLOAT(FWB_CR_MAX, 1.0f);

//  climb rate -> thr
PARAM_DEFINE_FLOAT(FWB_CR2THR_P, 0.01f);   // rate of climb to throttle PID
PARAM_DEFINE_FLOAT(FWB_CR2THR_I, 0.0f);
PARAM_DEFINE_FLOAT(FWB_CR2THR_D, 0.0f);
PARAM_DEFINE_FLOAT(FWB_CR2THR_D_LP, 0.0f);
PARAM_DEFINE_FLOAT(FWB_CR2THR_I_MAX, 0.0f);

PARAM_DEFINE_FLOAT(FWB_TRIM_THR, 0.8f);    // trim throttle (0,1)
PARAM_DEFINE_FLOAT(FWB_TRIM_V, 12.0f);     // trim velocity, m/s
