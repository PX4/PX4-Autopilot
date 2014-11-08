#include <systemlib/param/param.h>
#include <math.h>

// 16 is max name length
PARAM_DEFINE_FLOAT(SEG_YAW2R, 1.0f); // yaw error to yaw rate
PARAM_DEFINE_FLOAT(SEG_R2V, 0.2f); // yaw rate error to voltage
PARAM_DEFINE_FLOAT(SEG_TH2V, 10.0f); // pitch error to voltage
PARAM_DEFINE_FLOAT(SEG_Q2V, 0.5f); // pitch rate error to voltage

// position error to velocity command
PARAM_DEFINE_FLOAT(SEG_X2VEL_P, 0.1f); // proportional gain
PARAM_DEFINE_FLOAT(SEG_X2VEL_I, 0.0f); // integrator gain
PARAM_DEFINE_FLOAT(SEG_X2VEL_I_MAX, 0.0f); // max integrator windup

// for dynamics
PARAM_DEFINE_FLOAT(SEG_MGL, 6.98f); // max gravity torque (m*g*l)
PARAM_DEFINE_FLOAT(SEG_J, 0.470f); // total moment of inertia (J + 2*J_m)
PARAM_DEFINE_FLOAT(SEG_K_EMF, 0.0289f); // motor emf constant
PARAM_DEFINE_FLOAT(SEG_K_DAMP, 0.0504f); // motor damping constant
PARAM_DEFINE_FLOAT(SEG_WN_THETA, 3.87f); // desired natural freq for theta
PARAM_DEFINE_FLOAT(SEG_ZETA_THETA, 0.7f); // desired damping for theta

// velocity error to pitch command
PARAM_DEFINE_FLOAT(SEG_VEL2TH_P, 0.2f); // proportional gain
PARAM_DEFINE_FLOAT(SEG_VEL2TH_I, 0.2f); // integrator gain
PARAM_DEFINE_FLOAT(SEG_VEL2TH_I_MAX, 0.5f); // max integrator windup

PARAM_DEFINE_FLOAT(SEG_TH_LIM_MAX, 0.1f); // pitch limit
PARAM_DEFINE_FLOAT(SEG_VEL_LIM_MAX, 0.2f); // velocity limit
PARAM_DEFINE_FLOAT(SEG_TH_STOP, 0.2f); // turn off motors when over

// system id
PARAM_DEFINE_FLOAT(SEG_SYSID_ENABLE, 0.0f); // wave amplitude, deg pitch
PARAM_DEFINE_FLOAT(SEG_SYSID_AMP, 0.5f); // wave amplitude, deg pitch
PARAM_DEFINE_FLOAT(SEG_SYSID_FREQ, 0.1f); // wave frquency, Hz

// encoder position estimator
PARAM_DEFINE_FLOAT(ENCP_RWHEEL, 0.1f); // radius of wheel
PARAM_DEFINE_FLOAT(ENCP_PPR, 3200.0f); // encoder pulses per revolution of wheel
