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
PARAM_DEFINE_FLOAT(EKF_ATT_Q0, 1e1f);
PARAM_DEFINE_FLOAT(EKF_ATT_Q1, 1e1f);
PARAM_DEFINE_FLOAT(EKF_ATT_Q2, 1e1f);
/* gyro offsets process noise */
PARAM_DEFINE_FLOAT(EKF_ATT_Q3, 1e-4f);
PARAM_DEFINE_FLOAT(EKF_ATT_Q4, 1e-4f);
PARAM_DEFINE_FLOAT(EKF_ATT_Q5, 1e-4f);
/* accelerometer process noise */
PARAM_DEFINE_FLOAT(EKF_ATT_Q6, 1e-1f);
PARAM_DEFINE_FLOAT(EKF_ATT_Q7, 1e-1f);
PARAM_DEFINE_FLOAT(EKF_ATT_Q8, 1e-1f);
/* magnetometer process noise */
PARAM_DEFINE_FLOAT(EKF_ATT_Q9, 1e-1f);
PARAM_DEFINE_FLOAT(EKF_ATT_Q10, 1e-1f);
PARAM_DEFINE_FLOAT(EKF_ATT_Q11, 1e-1f);

/* gyro measurement noise */
PARAM_DEFINE_FLOAT(EKF_ATT_R0, 0.01f);
PARAM_DEFINE_FLOAT(EKF_ATT_R1, 0.01f);
PARAM_DEFINE_FLOAT(EKF_ATT_R2, 0.01f);
/* accelerometer measurement noise */
PARAM_DEFINE_FLOAT(EKF_ATT_R3, 1e1f);
PARAM_DEFINE_FLOAT(EKF_ATT_R4, 1e1f);
PARAM_DEFINE_FLOAT(EKF_ATT_R5, 1e1f);
/* magnetometer measurement noise */
PARAM_DEFINE_FLOAT(EKF_ATT_R6, 1e-1f);
PARAM_DEFINE_FLOAT(EKF_ATT_R7, 1e-1f);
PARAM_DEFINE_FLOAT(EKF_ATT_R8, 1e-1f);

int parameters_init(struct attitude_estimator_ekf_param_handles *h)
{
	/* PID parameters */
	h->q0 	=	param_find("EKF_ATT_Q0");
	h->q1 	=	param_find("EKF_ATT_Q1");
	h->q2 	=	param_find("EKF_ATT_Q2");
	h->q3 	=	param_find("EKF_ATT_Q3");
	h->q4 	=	param_find("EKF_ATT_Q4");
	h->q5 	=	param_find("EKF_ATT_Q5");
	h->q6 	=	param_find("EKF_ATT_Q6");
	h->q7 	=	param_find("EKF_ATT_Q7");
	h->q8 	=	param_find("EKF_ATT_Q8");
	h->q9 	=	param_find("EKF_ATT_Q9");
	h->q10 	=	param_find("EKF_ATT_Q10");
	h->q11 	=	param_find("EKF_ATT_Q11");

	h->r0 	=	param_find("EKF_ATT_R0");
	h->r1 	=	param_find("EKF_ATT_R1");
	h->r2 	=	param_find("EKF_ATT_R2");
	h->r3 	=	param_find("EKF_ATT_R3");
	h->r4 	=	param_find("EKF_ATT_R4");
	h->r5 	=	param_find("EKF_ATT_R5");
	h->r6 	=	param_find("EKF_ATT_R6");
	h->r7 	=	param_find("EKF_ATT_R7");
	h->r8 	=	param_find("EKF_ATT_R8");

	return OK;
}

int parameters_update(const struct attitude_estimator_ekf_param_handles *h, struct attitude_estimator_ekf_params *p)
{
	param_get(h->q0, &(p->q[0]));
	param_get(h->q1, &(p->q[1]));
	param_get(h->q2, &(p->q[2]));
	param_get(h->q3, &(p->q[3]));
	param_get(h->q4, &(p->q[4]));
	param_get(h->q5, &(p->q[5]));
	param_get(h->q6, &(p->q[6]));
	param_get(h->q7, &(p->q[7]));
	param_get(h->q8, &(p->q[8]));
	param_get(h->q9, &(p->q[9]));
	param_get(h->q10, &(p->q[10]));
	param_get(h->q11, &(p->q[11]));

	param_get(h->r0, &(p->r[0]));
	param_get(h->r1, &(p->r[1]));
	param_get(h->r2, &(p->r[2]));
	param_get(h->r3, &(p->r[3]));
	param_get(h->r4, &(p->r[4]));
	param_get(h->r5, &(p->r[5]));
	param_get(h->r6, &(p->r[6]));
	param_get(h->r7, &(p->r[7]));
	param_get(h->r8, &(p->r[8]));

	return OK;
}
