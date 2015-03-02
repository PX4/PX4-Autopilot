/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file params.c
 *
 * Parameters for fixedwing demo
 */

#include "params.h"

/* controller parameters, use max. 15 characters for param name! */

/**
 *
 */
PARAM_DEFINE_FLOAT(EXFW_HDNG_P, 0.1f);

/**
 *
 */
PARAM_DEFINE_FLOAT(EXFW_ROLL_P, 0.2f);

/**
 *
 */
PARAM_DEFINE_FLOAT(EXFW_PITCH_P, 0.2f);

int parameters_init(struct param_handles *h)
{
	/* PID parameters */
	h->hdng_p 	=	param_find("EXFW_HDNG_P");
	h->roll_p 	=	param_find("EXFW_ROLL_P");
	h->pitch_p 	=	param_find("EXFW_PITCH_P");

	return OK;
}

int parameters_update(const struct param_handles *h, struct params *p)
{
	param_get(h->hdng_p, &(p->hdng_p));
	param_get(h->roll_p, &(p->roll_p));
	param_get(h->pitch_p, &(p->pitch_p));

	return OK;
}
