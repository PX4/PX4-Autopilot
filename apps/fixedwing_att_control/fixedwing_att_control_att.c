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
#include <arch/board/up_hrt.h>
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
	param_t roll_p;
};



/* Internal Prototypes */
static int parameters_init(struct fw_att_control_params *h);
static int parameters_update(const struct fw_att_control_params *h, struct fw_att_control_params *p);

static int parameters_init(struct fw_att_control_params *h)
{
	/* PID parameters */
	h->roll_p 	=	param_find("FW_ROLL_POS_P");   //TODO define rate params for fixed wing


//	if(h->attrate_i == PARAM_INVALID)
//		printf("FATAL MC_ATTRATE_I does not exist\n");

	return OK;
}

static int parameters_update(const struct fw_att_control_params *h, struct fw_att_control_params *p)
{
	param_get(h->roll_p, &(p->roll_p));

	return OK;
}

int fixedwing_att_control_attitude(const struct vehicle_attitude_setpoint_s *att_sp,
		const struct vehicle_attitude_s *att,
		struct vehicle_rates_setpoint_s *rates_sp)
{
	static int counter = 0;
	static bool initialized = false;

	static struct fw_att_control_params p;
	static struct fw_att_control_params h;

	static uint64_t last_run = 0;
	const float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
	last_run = hrt_absolute_time();

	if(!initialized)
	{
		parameters_init(&h);
		parameters_update(&h, &p);
		initialized = true;
	}

	/* load new parameters with lower rate */
	if (counter % 2500 == 0) {
		/* update parameters from storage */
		parameters_update(&h, &p);
	}

	/* Roll (P) */
	float roll_error = att_sp->roll_tait_bryan - att->roll;
	rates_sp->roll = p.roll_p * roll_error;

	counter++;

	return 0;
}



