/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Thomas Gubler <thomasgubler@student.ethz.ch>
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
 * @file fixedwing_att_control_rate.c
 * Implementation of a fixed wing attitude controller.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 */
#include <fixedwing_att_control_rate.h>

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/geo/geo.h>
#include <systemlib/systemlib.h>

/*
 * Controller parameters, accessible via MAVLink
 *
 */
// Roll control parameters
PARAM_DEFINE_FLOAT(FW_ROLLR_P, 0.9f);
PARAM_DEFINE_FLOAT(FW_ROLLR_I, 0.2f);
PARAM_DEFINE_FLOAT(FW_ROLLR_AWU, 0.9f);
PARAM_DEFINE_FLOAT(FW_ROLLR_LIM, 0.7f);   // Roll rate limit in radians/sec, applies to the roll controller
PARAM_DEFINE_FLOAT(FW_ROLL_P, 4.0f);
PARAM_DEFINE_FLOAT(FW_PITCH_RCOMP, 0.1f);

//Pitch control parameters
PARAM_DEFINE_FLOAT(FW_PITCHR_P, 0.8f);
PARAM_DEFINE_FLOAT(FW_PITCHR_I, 0.2f);
PARAM_DEFINE_FLOAT(FW_PITCHR_AWU, 0.8f);
PARAM_DEFINE_FLOAT(FW_PITCHR_LIM, 0.35f);   // Pitch rate limit in radians/sec, applies to the pitch controller
PARAM_DEFINE_FLOAT(FW_PITCH_P, 8.0f);

//Yaw control parameters					//XXX TODO this is copy paste, asign correct values
PARAM_DEFINE_FLOAT(FW_YAWR_P, 0.3f);
PARAM_DEFINE_FLOAT(FW_YAWR_I, 0.0f);
PARAM_DEFINE_FLOAT(FW_YAWR_AWU, 0.0f);
PARAM_DEFINE_FLOAT(FW_YAWR_LIM, 0.35f);   // Yaw rate limit in radians/sec

/* feedforward compensation */
PARAM_DEFINE_FLOAT(FW_PITCH_THR_P, 0.1f);	/**< throttle to pitch coupling feedforward */

struct fw_rate_control_params {
	float rollrate_p;
	float rollrate_i;
	float rollrate_awu;
	float pitchrate_p;
	float pitchrate_i;
	float pitchrate_awu;
	float yawrate_p;
	float yawrate_i;
	float yawrate_awu;
	float pitch_thr_ff;
};

struct fw_rate_control_param_handles {
	param_t rollrate_p;
	param_t rollrate_i;
	param_t rollrate_awu;
	param_t pitchrate_p;
	param_t pitchrate_i;
	param_t pitchrate_awu;
	param_t yawrate_p;
	param_t yawrate_i;
	param_t yawrate_awu;
	param_t pitch_thr_ff;
};



/* Internal Prototypes */
static int parameters_init(struct fw_rate_control_param_handles *h);
static int parameters_update(const struct fw_rate_control_param_handles *h, struct fw_rate_control_params *p);

static int parameters_init(struct fw_rate_control_param_handles *h)
{
	/* PID parameters */
	h->rollrate_p 	 =	param_find("FW_ROLLR_P");   //TODO define rate params for fixed wing
	h->rollrate_i 	 =	param_find("FW_ROLLR_I");
	h->rollrate_awu  =	param_find("FW_ROLLR_AWU");

	h->pitchrate_p 	 =	param_find("FW_PITCHR_P");
	h->pitchrate_i 	 =	param_find("FW_PITCHR_I");
	h->pitchrate_awu =	param_find("FW_PITCHR_AWU");

	h->yawrate_p 	 =	param_find("FW_YAWR_P");
	h->yawrate_i 	 =	param_find("FW_YAWR_I");
	h->yawrate_awu   =	param_find("FW_YAWR_AWU");
	h->pitch_thr_ff  =	param_find("FW_PITCH_THR_P");

	return OK;
}

static int parameters_update(const struct fw_rate_control_param_handles *h, struct fw_rate_control_params *p)
{
	param_get(h->rollrate_p, &(p->rollrate_p));
	param_get(h->rollrate_i, &(p->rollrate_i));
	param_get(h->rollrate_awu, &(p->rollrate_awu));
	param_get(h->pitchrate_p, &(p->pitchrate_p));
	param_get(h->pitchrate_i, &(p->pitchrate_i));
	param_get(h->pitchrate_awu, &(p->pitchrate_awu));
	param_get(h->yawrate_p, &(p->yawrate_p));
	param_get(h->yawrate_i, &(p->yawrate_i));
	param_get(h->yawrate_awu, &(p->yawrate_awu));
	param_get(h->pitch_thr_ff, &(p->pitch_thr_ff));

	return OK;
}

int fixedwing_att_control_rates(const struct vehicle_rates_setpoint_s *rate_sp,
				const float rates[],
				struct actuator_controls_s *actuators)
{
	static int counter = 0;
	static bool initialized = false;

	static struct fw_rate_control_params p;
	static struct fw_rate_control_param_handles h;

	static PID_t roll_rate_controller;
	static PID_t pitch_rate_controller;
	static PID_t yaw_rate_controller;

	static uint64_t last_run = 0;
	const float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
	last_run = hrt_absolute_time();

	if (!initialized) {
		parameters_init(&h);
		parameters_update(&h, &p);
		pid_init(&roll_rate_controller, p.rollrate_p, p.rollrate_i, 0, p.rollrate_awu, 1, PID_MODE_DERIVATIV_NONE); // set D part to 0 because the controller layout is with a PI rate controller
		pid_init(&pitch_rate_controller, p.pitchrate_p, p.pitchrate_i, 0, p.pitchrate_awu, 1, PID_MODE_DERIVATIV_NONE); // set D part to 0 because the contpitcher layout is with a PI rate contpitcher
		pid_init(&yaw_rate_controller, p.yawrate_p, p.yawrate_i, 0, p.yawrate_awu, 1, PID_MODE_DERIVATIV_NONE); // set D part to 0 because the contpitcher layout is with a PI rate contpitcher
		initialized = true;
	}

	/* load new parameters with lower rate */
	if (counter % 100 == 0) {
		/* update parameters from storage */
		parameters_update(&h, &p);
		pid_set_parameters(&roll_rate_controller, p.rollrate_p, p.rollrate_i, 0, p.rollrate_awu, 1);
		pid_set_parameters(&pitch_rate_controller, p.pitchrate_p, p.pitchrate_i, 0, p.pitchrate_awu, 1);
		pid_set_parameters(&yaw_rate_controller, p.yawrate_p, p.yawrate_i, 0, p.yawrate_awu, 1);
	}


	/* roll rate (PI) */
	actuators->control[0] = pid_calculate(&roll_rate_controller, rate_sp->roll, rates[0], 0.0f, deltaT);
	/* pitch rate (PI) */
	actuators->control[1] = -pid_calculate(&pitch_rate_controller, rate_sp->pitch, rates[1], 0.0f, deltaT);
	/* yaw rate (PI) */
	actuators->control[2] = pid_calculate(&yaw_rate_controller, rate_sp->yaw, rates[2], 0.0f, deltaT);

	counter++;

	return 0;
}



