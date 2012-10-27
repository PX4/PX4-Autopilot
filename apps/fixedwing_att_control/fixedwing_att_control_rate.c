/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Thomas Gubler <thomasgubler@student.ethz.ch>
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





struct fw_rate_control_params {
	float rollrate_p;
	float rollrate_i;
	float rollrate_awu;
	float rollrate_lim;
	float pitchrate_p;
	float pitchrate_i;
	float pitchrate_awu;
	float pitchrate_lim;
	float yawrate_p;
	float yawrate_i;
	float yawrate_awu;
	float yawrate_lim;

};

struct fw_rate_control_param_handles {
	float rollrate_p;
	float rollrate_i;
	float rollrate_awu;
	float rollrate_lim;
	float pitchrate_p;
	float pitchrate_i;
	float pitchrate_awu;
	float pitchrate_lim;
	float yawrate_p;
	float yawrate_i;
	float yawrate_awu;
	float yawrate_lim;
};



/* Internal Prototypes */
static int parameters_init(struct fw_rate_control_param_handles *h);
static int parameters_update(const struct fw_rate_control_param_handles *h, struct fw_rate_control_params *p);

static int parameters_init(struct fw_rate_control_param_handles *h)
{
	/* PID parameters */
	h->rollrate_p 	=	param_find("FW_ROLLRATE_P");   //TODO define rate params for fixed wing
	h->rollrate_i 	=	param_find("FW_ROLLRATE_I");
	h->rollrate_awu =	param_find("FW_ROLLRATE_AWU");
	h->rollrate_lim =	param_find("FW_ROLLRATE_LIM");
	h->pitchrate_p 	=	param_find("FW_PITCHRATE_P");
	h->pitchrate_i 	=	param_find("FW_PITCHRATE_I");
	h->pitchrate_awu =	param_find("FW_PITCHRATE_AWU");
	h->pitchrate_lim =	param_find("FW_PITCHRATE_LIM");
	h->yawrate_p 	=	param_find("FW_YAWRATE_P");
	h->yawrate_i 	=	param_find("FW_YAWRATE_I");
	h->yawrate_awu =	param_find("FW_YAWRATE_AWU");
	h->yawrate_lim =	param_find("FW_YAWRATE_LIM");


//	if(h->attrate_i == PARAM_INVALID)
//		printf("FATAL MC_ATTRATE_I does not exist\n");

	return OK;
}

static int parameters_update(const struct fw_rate_control_param_handles *h, struct fw_rate_control_params *p)
{
	param_get(h->rollrate_p, &(p->rollrate_p));
	param_get(h->rollrate_i, &(p->rollrate_i));
	param_get(h->rollrate_awu, &(p->rollrate_awu));
	param_get(h->rollrate_lim, &(p->rollrate_lim));
	param_get(h->pitchrate_p, &(p->pitchrate_p));
	param_get(h->pitchrate_i, &(p->pitchrate_i));
	param_get(h->pitchrate_awu, &(p->pitchrate_awu));
	param_get(h->pitchrate_lim, &(p->pitchrate_lim));
	param_get(h->yawrate_p, &(p->yawrate_p));
	param_get(h->yawrate_i, &(p->yawrate_i));
	param_get(h->yawrate_awu, &(p->yawrate_awu));
	param_get(h->yawrate_lim, &(p->yawrate_lim));

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

	if(!initialized)
	{
		parameters_init(&h);
		parameters_update(&h, &p);
		pid_init(&roll_rate_controller, p.rollrate_p, p.rollrate_i, 0, p.rollrate_awu, p.rollrate_lim, PID_MODE_DERIVATIV_SET); // set D part to 0 because the controller layout is with a PI rate controller
		pid_init(&pitch_rate_controller, p.pitchrate_p, p.pitchrate_i, 0, p.pitchrate_awu, p.pitchrate_lim, PID_MODE_DERIVATIV_SET); // set D part to 0 because the contpitcher layout is with a PI rate contpitcher
		pid_init(&yaw_rate_controller, p.yawrate_p, p.yawrate_i, 0, p.yawrate_awu, p.yawrate_lim, PID_MODE_DERIVATIV_SET); // set D part to 0 because the contpitcher layout is with a PI rate contpitcher
		initialized = true;
	}

	/* load new parameters with lower rate */
	if (counter % 2500 == 0) {
		/* update parameters from storage */
		pid_set_parameters(&roll_rate_controller, p.rollrate_p, p.rollrate_i, 0, p.rollrate_awu, p.rollrate_lim);
		pid_set_parameters(&pitch_rate_controller, p.pitchrate_p, p.pitchrate_i, 0, p.pitchrate_awu, p.pitchrate_lim);
		pid_set_parameters(&yaw_rate_controller, p.yawrate_p, p.yawrate_i, 0, p.yawrate_awu, p.yawrate_lim);
		parameters_update(&h, &p);
	}


	/* Roll Rate (PI) */
	actuators->control[0] = pid_calculate(&roll_rate_controller, rate_sp->roll, rates[0], 0, deltaT);

	//XXX TODO disabled for now
	actuators->control[1] = 0;//pid_calculate(&pitch_rate_controller, rate_sp->pitch, rates[1], 0, deltaT);
	actuators->control[2] = 0;//pid_calculate(&yaw_rate_controller, rate_sp->yaw, rates[2], 0, deltaT);

	counter++;

	return 0;
}



