/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Tobias Naegeli <naegelit@student.ethz.ch>
 *           Lorenz Meier <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file attitude_estimator_ekf_params.c
 *
 * Parameters for EKF filter
 */

#include "attitude_estimator_ekf_params.h"
#include <math.h>
#include <px4_defines.h>

/* Extended Kalman Filter covariances */


/**
 * Body angular rate process noise
 *
 * @group Attitude EKF estimator
 */
PARAM_DEFINE_FLOAT(EKF_ATT_V3_Q0, 1e-4f);

/**
 * Body angular acceleration process noise
 *
 * @group Attitude EKF estimator
 */
PARAM_DEFINE_FLOAT(EKF_ATT_V3_Q1, 0.08f);

/**
 * Acceleration process noise
 *
 * @group Attitude EKF estimator
 */
PARAM_DEFINE_FLOAT(EKF_ATT_V3_Q2, 0.009f);

/**
 * Magnet field vector process noise
 *
 * @group Attitude EKF estimator
 */
PARAM_DEFINE_FLOAT(EKF_ATT_V3_Q3, 0.005f);

/**
 * Gyro measurement noise
 *
 * @group Attitude EKF estimator
 */
PARAM_DEFINE_FLOAT(EKF_ATT_V4_R0, 0.0008f);

/**
 * Accel measurement noise
 *
 * @group Attitude EKF estimator
 */
PARAM_DEFINE_FLOAT(EKF_ATT_V4_R1, 10000.0f);

/**
 * Mag measurement noise
 *
 * @group Attitude EKF estimator
 */
PARAM_DEFINE_FLOAT(EKF_ATT_V4_R2, 100.0f);

/**
 * Moment of inertia matrix diagonal entry (1, 1)
 *
 * @group Attitude EKF estimator
 * @unit kg*m^2
 */
PARAM_DEFINE_FLOAT(ATT_J11, 0.0018);

/**
 * Moment of inertia matrix diagonal entry (2, 2)
 *
 * @group Attitude EKF estimator
 * @unit kg*m^2
 */
PARAM_DEFINE_FLOAT(ATT_J22, 0.0018);

/**
 * Moment of inertia matrix diagonal entry (3, 3)
 *
 * @group Attitude EKF estimator
 * @unit kg*m^2
 */
PARAM_DEFINE_FLOAT(ATT_J33, 0.0037);

/**
 * Moment of inertia enabled in estimator
 *
 * If set to != 0 the moment of inertia will be used in the estimator
 *
 * @group Attitude EKF estimator
 * @boolean
 */
PARAM_DEFINE_INT32(ATT_J_EN, 0);
