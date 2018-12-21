/*
 C code fragment for function that enables the yaw uncertainty to be increased following a yaw reset.
 The variables _state.quat_nominal(0) -> _state.quat_nominal(3) are the attitude quaternions
 The variable daYawVar is the variance of the yaw angle uncertainty in rad**2
 See DeriveYawResetEquations.m for the derivation
*/

// Intermediate variables
float SG[3];
SG[0] = sq(_state.quat_nominal(0)) - sq(_state.quat_nominal(1)) - sq(_state.quat_nominal(2)) + sq(_state.quat_nominal(3));
SG[1] = 2*_state.quat_nominal(0)*_state.quat_nominal(2) - 2*_state.quat_nominal(1)*_state.quat_nominal(3);
SG[2] = 2*_state.quat_nominal(0)*_state.quat_nominal(1) + 2*_state.quat_nominal(2)*_state.quat_nominal(3);

float SQ[4];
SQ[0] = 0.5f * ((_state.quat_nominal(1)*SG[0]) - (_state.quat_nominal(0)*SG[2]) + (_state.quat_nominal(3)*SG[1]));
SQ[1] = 0.5f * ((_state.quat_nominal(0)*SG[1]) - (_state.quat_nominal(2)*SG[0]) + (_state.quat_nominal(3)*SG[2]));
SQ[2] = 0.5f * ((_state.quat_nominal(3)*SG[0]) - (_state.quat_nominal(1)*SG[1]) + (_state.quat_nominal(2)*SG[2]));
SQ[3] = 0.5f * ((_state.quat_nominal(0)*SG[0]) + (_state.quat_nominal(1)*SG[2]) + (_state.quat_nominal(2)*SG[1]));

// Variance of yaw angle uncertainty (rad**2)
const float daYawVar = TBD;

// Add covariances for additonal yaw uncertainty to exisiting covariances.
// This assumes that the additional yaw error is uncorrrelated
P[0][0] += daYawVar*sq(SQ[2]);
P[0][1] += daYawVar*SQ[1]*SQ[2];
P[1][1] += daYawVar*sq(SQ[1]);
P[0][2] += daYawVar*SQ[0]*SQ[2];
P[1][2] += daYawVar*SQ[0]*SQ[1];
P[2][2] += daYawVar*sq(SQ[0]);
P[0][3] += daYawVar*SQ[2]*SQ[3];
P[1][3] += daYawVar*SQ[1]*SQ[3];
P[2][3] += daYawVar*SQ[0]*SQ[3];
P[3][3] += daYawVar*sq(SQ[3]);
