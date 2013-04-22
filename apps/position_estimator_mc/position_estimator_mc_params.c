/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: 	Damian Aregger <daregger@student.ethz.ch>
 *   			Tobias Naegeli <naegelit@student.ethz.ch>
* 				Lorenz Meier <lm@inf.ethz.ch>
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
 * @file position_estimator_mc_params.c
 * 
 * Parameters for position_estimator_mc
 */

#include "position_estimator_mc_params.h"

/* Kalman Filter covariances */
/* gps measurement noise standard deviation */
PARAM_DEFINE_FLOAT(POS_EST_ADDN, 1.0f);
PARAM_DEFINE_FLOAT(POS_EST_SIGMA, 0.0f);
PARAM_DEFINE_FLOAT(POS_EST_R, 1.0f);
PARAM_DEFINE_INT32(POS_EST_BARO, 0.0f);

int parameters_init(struct position_estimator_mc_param_handles *h)
{
	h->addNoise	= param_find("POS_EST_ADDN");
	h->sigma = param_find("POS_EST_SIGMA");
	h->r = param_find("POS_EST_R");
	h->baro_param_handle = param_find("POS_EST_BARO");
	return OK;
}

int parameters_update(const struct position_estimator_mc_param_handles *h, struct position_estimator_mc_params *p)
{
	param_get(h->addNoise, &(p->addNoise));
	param_get(h->sigma, &(p->sigma));
	param_get(h->r, &(p->R));
	param_get(h->baro_param_handle, &(p->baro));
	return OK;
}
