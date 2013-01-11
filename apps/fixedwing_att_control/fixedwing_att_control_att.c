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
#include <fixedwing_att_control_att.h>

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




struct fw_att_control_params {
	float roll_p;
	float rollrate_lim;
	float pitch_p;
	float pitchrate_lim;
	float yawrate_lim;
	float pitch_roll_compensation_p;
};

struct fw_pos_control_param_handles {
	param_t roll_p;
	param_t rollrate_lim;
	param_t pitch_p;
	param_t pitchrate_lim;
	param_t yawrate_lim;
	param_t pitch_roll_compensation_p;
};



/* Internal Prototypes */
static int parameters_init(struct fw_pos_control_param_handles *h);
static int parameters_update(const struct fw_pos_control_param_handles *h, struct fw_att_control_params *p);

static int parameters_init(struct fw_pos_control_param_handles *h)
{
	/* PID parameters */
	h->roll_p 		=	param_find("FW_ROLL_P");
	h->rollrate_lim =	param_find("FW_ROLLR_LIM");
	h->pitch_p 		=	param_find("FW_PITCH_P");
	h->pitchrate_lim =	param_find("FW_PITCHR_LIM");
	h->yawrate_lim =	param_find("FW_YAWR_LIM");
	h->pitch_roll_compensation_p = param_find("FW_PITCH_RCOMP");

	return OK;
}

static int parameters_update(const struct fw_pos_control_param_handles *h, struct fw_att_control_params *p)
{
	param_get(h->roll_p, &(p->roll_p));
	param_get(h->rollrate_lim, &(p->rollrate_lim));
	param_get(h->pitch_p, &(p->pitch_p));
	param_get(h->pitchrate_lim, &(p->pitchrate_lim));
	param_get(h->yawrate_lim, &(p->yawrate_lim));
	param_get(h->pitch_roll_compensation_p, &(p->pitch_roll_compensation_p));

	return OK;
}

int fixedwing_att_control_attitude(const struct vehicle_attitude_setpoint_s *att_sp,
				   const struct vehicle_attitude_s *att,
				   const float speed_body[],
				   struct vehicle_rates_setpoint_s *rates_sp)
{
	static int counter = 0;
	static bool initialized = false;

	static struct fw_att_control_params p;
	static struct fw_pos_control_param_handles h;

	static PID_t roll_controller;
	static PID_t pitch_controller;


	if (!initialized) {
		parameters_init(&h);
		parameters_update(&h, &p);
		pid_init(&roll_controller, p.roll_p, 0, 0, 0, p.rollrate_lim, PID_MODE_DERIVATIV_NONE); //P Controller
		pid_init(&pitch_controller, p.pitch_p, 0, 0, 0, p.pitchrate_lim, PID_MODE_DERIVATIV_NONE); //P Controller
		initialized = true;
	}

	/* load new parameters with lower rate */
	if (counter % 100 == 0) {
		/* update parameters from storage */
		parameters_update(&h, &p);
		pid_set_parameters(&roll_controller, p.roll_p, 0, 0, 0, p.rollrate_lim);
		pid_set_parameters(&pitch_controller, p.pitch_p, 0, 0, 0, p.pitchrate_lim);
	}

	/* Roll (P) */
	rates_sp->roll = pid_calculate(&roll_controller, att_sp->roll_body, att->roll, 0, 0);


	/* Pitch (P) */

	/* compensate feedforward for loss of lift due to non-horizontal angle of wing */
	float pitch_sp_rollcompensation = p.pitch_roll_compensation_p * fabsf(sinf(att_sp->roll_body));
	/* set pitch plus feedforward roll compensation */
	rates_sp->pitch = pid_calculate(&pitch_controller,
					att_sp->pitch_body + pitch_sp_rollcompensation,
					att->pitch, 0, 0);

	/* Yaw (from coordinated turn constraint or lateral force) */
	rates_sp->yaw = (att->rollspeed * rates_sp->roll + 9.81f * sinf(att->roll) * cosf(att->pitch) + speed_body[0] * rates_sp->pitch * sinf(att->roll))
			/ (speed_body[0] * cosf(att->roll) * cosf(att->pitch) + speed_body[2] * sinf(att->pitch));

//	printf("rates_sp->yaw %.4f \n", (double)rates_sp->yaw);

	counter++;

	return 0;
}



