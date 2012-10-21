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
 * @file fixedwing_control2.c
 * Implementation of a fixed wing attitude controller.
 */

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
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/geo/geo.h>
#include <systemlib/systemlib.h>


struct fw_rate_control_params {

	param_t yawrate_p;
	param_t yawrate_i;
	param_t yawrate_d;
	param_t yawrate_awu;
	param_t yawrate_lim;

	param_t attrate_p;
	param_t attrate_i;
	param_t attrate_d;
	param_t attrate_awu;
	param_t attrate_lim;
};



/* Internal Prototypes */
static int parameters_init(struct fw_rate_control_params *h);
static int parameters_update(const struct fw_rate_control_params *h, struct fw_rate_control_params *p);

static int parameters_init(struct fw_rate_control_params *h)
{
	/* PID parameters */
	h->yawrate_p 	=	param_find("MC_YAWRATE_P");   //TODO define rate params for fixed wing
	h->yawrate_i 	=	param_find("MC_YAWRATE_I");
	h->yawrate_d 	=	param_find("MC_YAWRATE_D");
	h->yawrate_awu 	=	param_find("MC_YAWRATE_AWU");
	h->yawrate_lim 	=	param_find("MC_YAWRATE_LIM");

	h->attrate_p 	= 	param_find("MC_ATTRATE_P");
	h->attrate_i 	= 	param_find("MC_ATTRATE_I");
	h->attrate_d 	= 	param_find("MC_ATTRATE_D");
	h->attrate_awu 	= 	param_find("MC_ATTRATE_AWU");
	h->attrate_lim 	= 	param_find("MC_ATTRATE_LIM");

	return OK;
}

static int parameters_update(const struct fw_rate_control_params *h, struct fw_rate_control_params *p)
{
	param_get(h->yawrate_p, &(p->yawrate_p));
	param_get(h->yawrate_i, &(p->yawrate_i));
	param_get(h->yawrate_d, &(p->yawrate_d));
	param_get(h->yawrate_awu, &(p->yawrate_awu));
	param_get(h->yawrate_lim, &(p->yawrate_lim));

	param_get(h->attrate_p, &(p->attrate_p));
	param_get(h->attrate_i, &(p->attrate_i));
	param_get(h->attrate_d, &(p->attrate_d));
	param_get(h->attrate_awu, &(p->attrate_awu));
	param_get(h->attrate_lim, &(p->attrate_lim));

	return OK;
}


int fixedwing_control2_rates(const struct vehicle_rates_setpoint_s *rate_sp,
		const float rates[],
		struct actuator_controls_s *actuators)
{
	static int counter = 0;
	static bool initialized = false;

	static struct fw_rate_control_params p;
	static struct fw_rate_control_params h;

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
		printf("p.yawrate_p: %8.4f\n", (double)p.yawrate_p);
	}

	/* Roll Rate */
	float roll_error = rate_sp->roll - rates[0];
	actuators->control[0] =p.attrate_p*roll_error;


	actuators->control[1] = 0;
	actuators->control[2] = 0;

	counter++;

	return 0;
}



