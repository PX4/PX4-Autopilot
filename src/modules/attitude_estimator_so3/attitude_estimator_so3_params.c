/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Hyon Lim <limhyon@gmail.com>
 *           Anton Babushkin <anton.babushkin@me.com>
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
 * @file attitude_estimator_so3_params.c
 *
 * Parameters for nonlinear complementary filters on the SO(3).
 */

#include "attitude_estimator_so3_params.h"

/* This is filter gain for nonlinear SO3 complementary filter */
/* NOTE : How to tune the gain? First of all, stick with this default gain. And let the quad in stable place.
   Log the steady state reponse of filter. If it is too slow, increase SO3_COMP_KP.
   If you are flying from ground to high altitude in short amount of time, please increase SO3_COMP_KI which
   will compensate gyro bias which depends on temperature and vibration of your vehicle */
PARAM_DEFINE_FLOAT(SO3_COMP_KP, 1.0f); //! This parameter will give you about 15 seconds convergence time.
                                       //! You can set this gain higher if you want more fast response.
                                       //! But note that higher gain will give you also higher overshoot.
PARAM_DEFINE_FLOAT(SO3_COMP_KI, 0.05f); //! This gain will incorporate slow time-varying bias (e.g., temperature change)
					//! This gain is depend on your vehicle status.

/* offsets in roll, pitch and yaw of sensor plane and body */
PARAM_DEFINE_FLOAT(SO3_ROLL_OFFS, 0.0f);
PARAM_DEFINE_FLOAT(SO3_PITCH_OFFS, 0.0f);
PARAM_DEFINE_FLOAT(SO3_YAW_OFFS, 0.0f);

int parameters_init(struct attitude_estimator_so3_param_handles *h)
{
	/* Filter gain parameters */
	h->Kp = 	param_find("SO3_COMP_KP");
	h->Ki = 	param_find("SO3_COMP_KI");

	/* Attitude offset (WARNING: Do not change if you do not know what exactly this variable wil lchange) */
	h->roll_off  =	param_find("SO3_ROLL_OFFS");
	h->pitch_off =	param_find("SO3_PITCH_OFFS");
	h->yaw_off   =	param_find("SO3_YAW_OFFS");

	return OK;
}

int parameters_update(const struct attitude_estimator_so3_param_handles *h, struct attitude_estimator_so3_params *p)
{
	/* Update filter gain */
	param_get(h->Kp, &(p->Kp));
	param_get(h->Ki, &(p->Ki));

	/* Update attitude offset */
	param_get(h->roll_off, &(p->roll_off));
	param_get(h->pitch_off, &(p->pitch_off));
	param_get(h->yaw_off, &(p->yaw_off));

	return OK;
}
