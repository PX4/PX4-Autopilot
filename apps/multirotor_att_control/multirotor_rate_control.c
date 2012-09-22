/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Tobias Naegeli <naegelit@student.ethz.ch>
 *           @author Lorenz Meier <lm@inf.ethz.ch>
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

/**
 * @file multirotor_rate_control.c
 * Implementation of rate controller
 */

#include "multirotor_rate_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <math.h>
#include <systemlib/pid/pid.h>
#include <systemlib/param/param.h>
#include <arch/board/up_hrt.h>

// PARAM_DEFINE_FLOAT(MC_YAWRATE_P, 0.08f); /* same on Flamewheel */
// PARAM_DEFINE_FLOAT(MC_YAWRATE_I, 0.02f);
// PARAM_DEFINE_FLOAT(MC_YAWRATE_AWU, 0.02f);
// PARAM_DEFINE_FLOAT(MC_YAWRATE_LIM, 0.1f);

PARAM_DEFINE_FLOAT(MC_ATTRATE_P, 0.2f); /* 0.15 F405 Flamewheel */
PARAM_DEFINE_FLOAT(MC_ATTRATE_I, 0.0f);
PARAM_DEFINE_FLOAT(MC_ATTRATE_AWU, 0.05f);
PARAM_DEFINE_FLOAT(MC_ATTRATE_LIM, 8.0f);	/**< roughly < 500 deg/s limit */

struct mc_rate_control_params {

	float yawrate_p;
	float yawrate_i;
	float yawrate_awu;
	float yawrate_lim;

	float attrate_p;
	float attrate_i;
	float attrate_awu;
	float attrate_lim;

	float rate_lim;
};

struct mc_rate_control_param_handles {

	param_t yawrate_p;
	param_t yawrate_i;
	param_t yawrate_awu;
	param_t yawrate_lim;

	param_t attrate_p;
	param_t attrate_i;
	param_t attrate_awu;
	param_t attrate_lim;
};

/**
 * Initialize all parameter handles and values
 *
 */
static int parameters_init(struct mc_rate_control_param_handles *h);

/**
 * Update all parameters
 *
 */
static int parameters_update(const struct mc_rate_control_param_handles *h, struct mc_rate_control_params *p);


static int parameters_init(struct mc_rate_control_param_handles *h)
{
	/* PID parameters */
	h->yawrate_p 	=	param_find("MC_YAWRATE_P");
	h->yawrate_i 	=	param_find("MC_YAWRATE_I");
	h->yawrate_awu 	=	param_find("MC_YAWRATE_AWU");
	h->yawrate_lim 	=	param_find("MC_YAWRATE_LIM");

	h->attrate_p 	= 	param_find("MC_ATTRATE_P");
	h->attrate_i 	= 	param_find("MC_ATTRATE_I");
	h->attrate_awu 	= 	param_find("MC_ATTRATE_AWU");
	h->attrate_lim 	= 	param_find("MC_ATTRATE_LIM");

	return OK;
}

static int parameters_update(const struct mc_rate_control_param_handles *h, struct mc_rate_control_params *p)
{
	param_get(h->yawrate_p, &(p->yawrate_p));
	param_get(h->yawrate_i, &(p->yawrate_i));
	param_get(h->yawrate_awu, &(p->yawrate_awu));
	param_get(h->yawrate_lim, &(p->yawrate_lim));

	param_get(h->attrate_p, &(p->attrate_p));
	param_get(h->attrate_i, &(p->attrate_i));
	param_get(h->attrate_awu, &(p->attrate_awu));
	param_get(h->attrate_lim, &(p->attrate_lim));

	return OK;
}

void multirotor_control_rates(const struct vehicle_rates_setpoint_s *rate_sp,
	const float rates[], struct actuator_controls_s *actuators)
{
	static uint64_t last_run = 0;
	const float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
	last_run = hrt_absolute_time();

	static int motor_skip_counter = 0;

	// static PID_t yaw_pos_controller;
	static PID_t yaw_speed_controller;
	static PID_t pitch_controller;
	static PID_t roll_controller;

	static struct mc_rate_control_params p;
	static struct mc_rate_control_param_handles h;

	static bool initialized = false;

	/* initialize the pid controllers when the function is called for the first time */
	if (initialized == false) {
		parameters_init(&h);
		parameters_update(&h, &p);

		pid_init(&yaw_speed_controller, p.yawrate_p, 0, p.yawrate_i, p.yawrate_awu,
			PID_MODE_DERIVATIV_SET);
		pid_init(&pitch_controller, p.attrate_p, p.attrate_i, 0, p.attrate_awu,
			PID_MODE_DERIVATIV_SET);
		pid_init(&roll_controller, p.attrate_p, p.attrate_i, 0, p.attrate_awu,
			PID_MODE_DERIVATIV_SET);

		initialized = true;
	}

	/* load new parameters with lower rate */
	if (motor_skip_counter % 250 == 0) {
		/* update parameters from storage */
		parameters_update(&h, &p);
		/* apply parameters */
		pid_set_parameters(&yaw_speed_controller, p.yawrate_p, p.yawrate_i, 0, p.yawrate_awu);
		pid_set_parameters(&pitch_controller, p.attrate_p, p.attrate_i, 0, p.attrate_awu);
		pid_set_parameters(&roll_controller, p.attrate_p, p.attrate_i, 0, p.attrate_awu);
	}

	/* calculate current control outputs */
	
	/* control pitch (forward) output */
	float pitch_control = pid_calculate(&pitch_controller, rate_sp->pitch,
					rates[1], 0.0f, deltaT);
	/* control roll (left/right) output */
	float roll_control = pid_calculate(&roll_controller, rate_sp->roll,
					rates[0], 0.0f, deltaT);
	/* control yaw rate */
	float yaw_rate_control = pid_calculate(&yaw_speed_controller, rate_sp->yaw, rates[2], 0.0f, deltaT);

	/*
	 * compensate the vertical loss of thrust
	 * when thrust plane has an angle.
	 * start with a factor of 1.0 (no change)
	 */
	float zcompensation = 1.0f;

	// if (fabsf(att->roll) > 0.3f) {
	// 	zcompensation *= 1.04675160154f;

	// } else {
	// 	zcompensation *= 1.0f / cosf(att->roll);
	// }

	// if (fabsf(att->pitch) > 0.3f) {
	// 	zcompensation *= 1.04675160154f;

	// } else {
	// 	zcompensation *= 1.0f / cosf(att->pitch);
	// }

	float motor_thrust = 0.0f;

	motor_thrust = rate_sp->thrust;

	/* compensate thrust vector for roll / pitch contributions */
	motor_thrust *= zcompensation;

	/* limit yaw rate output */
	if (yaw_rate_control > p.yawrate_lim) {
		yaw_rate_control = p.yawrate_lim;
		yaw_speed_controller.saturated = 1;
	}

	if (yaw_rate_control < -p.yawrate_lim) {
		yaw_rate_control = -p.yawrate_lim;
		yaw_speed_controller.saturated = 1;
	}

	if (pitch_control > p.attrate_lim) {
		pitch_control = p.attrate_lim;
		pitch_controller.saturated = 1;
	}

	if (pitch_control < -p.attrate_lim) {
		pitch_control = -p.attrate_lim;
		pitch_controller.saturated = 1;
	}


	if (roll_control > p.attrate_lim) {
		roll_control = p.attrate_lim;
		roll_controller.saturated = 1;
	}

	if (roll_control < -p.attrate_lim) {
		roll_control = -p.attrate_lim;
		roll_controller.saturated = 1;
	}

	actuators->control[0] = roll_control;
	actuators->control[1] = pitch_control;
	actuators->control[2] = yaw_rate_control;
	actuators->control[3] = motor_thrust;

	motor_skip_counter++;
}
