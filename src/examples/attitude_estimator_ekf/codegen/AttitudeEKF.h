/*
 * AttitudeEKF.h
 *
 * Code generation for function 'AttitudeEKF'
 *
 * C source code generated on: Thu Aug 21 11:17:28 2014
 *
 */

#ifndef __ATTITUDEEKF_H__
#define __ATTITUDEEKF_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "rtwtypes.h"
#include "AttitudeEKF_types.h"

/* Function Declarations */
extern void AttitudeEKF(unsigned char approx_prediction, unsigned char use_inertia_matrix, const unsigned char zFlag[3], float dt, const float z[9], float q_rotSpeed, float q_rotAcc, float q_acc, float q_mag, float r_gyro, float r_accel, float r_mag, const float J[9], float xa_apo[12], float Pa_apo[144], float Rot_matrix[9], float eulerAngles[3], float debugOutput[4]);
extern void AttitudeEKF_initialize(void);
extern void AttitudeEKF_terminate(void);
#endif
/* End of code generation (AttitudeEKF.h) */
