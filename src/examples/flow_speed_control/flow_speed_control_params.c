/****************************************************************************
 *
 *   Copyright (C) 2008-2013 PX4 Development Team. All rights reserved.
 *   Author: Samuel Zihlmann <samuezih@ee.ethz.ch>
 *   		 Lorenz Meier <lm@inf.ethz.ch>
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
 * @file flow_speed_control_params.c
 * 
 */

#include "flow_speed_control_params.h"

/* controller parameters */
PARAM_DEFINE_FLOAT(FSC_S_P, 0.1f);
PARAM_DEFINE_FLOAT(FSC_L_PITCH, 0.4f);
PARAM_DEFINE_FLOAT(FSC_L_ROLL, 0.4f);

int parameters_init(struct flow_speed_control_param_handles *h)
{
	/* PID parameters */
	h->speed_p	 			=	param_find("FSC_S_P");
	h->limit_pitch 			=	param_find("FSC_L_PITCH");
	h->limit_roll 			=	param_find("FSC_L_ROLL");
	h->trim_roll 			=	param_find("TRIM_ROLL");
	h->trim_pitch 			=	param_find("TRIM_PITCH");


	return OK;
}

int parameters_update(const struct flow_speed_control_param_handles *h, struct flow_speed_control_params *p)
{
	param_get(h->speed_p, &(p->speed_p));
	param_get(h->limit_pitch, &(p->limit_pitch));
	param_get(h->limit_roll, &(p->limit_roll));
	param_get(h->trim_roll, &(p->trim_roll));
	param_get(h->trim_pitch, &(p->trim_pitch));

	return OK;
}
