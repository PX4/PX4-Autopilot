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

/* Extended Kalman Filter covariances */

/* gyro process noise */
PARAM_DEFINE_FLOAT(EKF_ATT_V3_Q0, 1e-4f);
PARAM_DEFINE_FLOAT(EKF_ATT_V3_Q1, 0.08f);
PARAM_DEFINE_FLOAT(EKF_ATT_V3_Q2, 0.009f);
/* gyro offsets process noise */
PARAM_DEFINE_FLOAT(EKF_ATT_V3_Q3, 0.005f);
PARAM_DEFINE_FLOAT(EKF_ATT_V3_Q4, 0.0f);

/* gyro measurement noise */
PARAM_DEFINE_FLOAT(EKF_ATT_V4_R0, 0.0008f);
/* accel measurement noise */
PARAM_DEFINE_FLOAT(EKF_ATT_V4_R1, 10000.0f);
/* mag measurement noise */
PARAM_DEFINE_FLOAT(EKF_ATT_V4_R2, 100.0f);
/* offset estimation - UNUSED */
PARAM_DEFINE_FLOAT(EKF_ATT_V4_R3, 0.0f);

/* offsets in roll, pitch and yaw of sensor plane and body */
PARAM_DEFINE_FLOAT(ATT_ROLL_OFF3, 0.0f);
PARAM_DEFINE_FLOAT(ATT_PITCH_OFF3, 0.0f);
PARAM_DEFINE_FLOAT(ATT_YAW_OFF3, 0.0f);

int parameters_init(struct attitude_estimator_ekf_param_handles *h)
{
	/* PID parameters */
	h->q0 	=	param_find("EKF_ATT_V3_Q0");
	h->q1 	=	param_find("EKF_ATT_V3_Q1");
	h->q2 	=	param_find("EKF_ATT_V3_Q2");
	h->q3 	=	param_find("EKF_ATT_V3_Q3");
	h->q4 	=	param_find("EKF_ATT_V3_Q4");

	h->r0 	=	param_find("EKF_ATT_V4_R0");
	h->r1 	=	param_find("EKF_ATT_V4_R1");
	h->r2 	=	param_find("EKF_ATT_V4_R2");
	h->r3 	=	param_find("EKF_ATT_V4_R3");

	h->roll_off  =	param_find("ATT_ROLL_OFF3");
	h->pitch_off =	param_find("ATT_PITCH_OFF3");
	h->yaw_off   =	param_find("ATT_YAW_OFF3");

	return OK;
}

int parameters_update(const struct attitude_estimator_ekf_param_handles *h, struct attitude_estimator_ekf_params *p)
{
	param_get(h->q0, &(p->q[0]));
	param_get(h->q1, &(p->q[1]));
	param_get(h->q2, &(p->q[2]));
	param_get(h->q3, &(p->q[3]));
	param_get(h->q4, &(p->q[4]));

	param_get(h->r0, &(p->r[0]));
	param_get(h->r1, &(p->r[1]));
	param_get(h->r2, &(p->r[2]));
	param_get(h->r3, &(p->r[3]));

	param_get(h->roll_off, &(p->roll_off));
	param_get(h->pitch_off, &(p->pitch_off));
	param_get(h->yaw_off, &(p->yaw_off));

	return OK;
}
