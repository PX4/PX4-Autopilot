/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Tobias Naegeli <naegelit@student.ethz.ch>
 *           Lorenz Meier <lm@inf.ethz.ch>
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
 * @file multirotor_att_control_params.h
 * 
 * Parameters for attitude control
 */

#ifndef MULTIROTOR_ATT_CONTROL_PARAMS_H_
#define MULTIROTOR_ATT_CONTROL_PARAMS_H_

#include <systemlib/param/param.h>

struct multirotor_att_control_params {
	float failsafe_throttle;

	/* attitude control */
	float yaw_p;
	float yaw_i;
	float yaw_d;
//	float yaw_awu;
//	float yaw_lim;

	float yaw_intmax;

	float att_p;
	float att_i;
	float att_d;
//	float att_awu;
//	float att_lim;

//	float att_xoff;
//	float att_yoff;


	/* rate control */
	float yawrate_p;
	float yawrate_d;
	float yawrate_i;
//	float yawrate_awu;
//	float yawrate_lim;

	float attrate_p;
	float attrate_d;
	float attrate_i;
//	float attrate_awu;
//	float attrate_lim;

//	float rate_lim;
};

struct multirotor_att_control_param_handles {
	param_t failsafe_throttle;

	/* attitude control */
	param_t yaw_p;
	param_t yaw_i;
	param_t yaw_d;
//	param_t yaw_awu;
//	param_t yaw_lim;

	param_t yaw_intmax;

	param_t att_p;
	param_t att_i;
	param_t att_d;
//	param_t att_awu;
//	param_t att_lim;

//	param_t att_xoff;
//	param_t att_yoff;

	/* rate control */
	param_t yawrate_p;
	param_t yawrate_i;
	param_t yawrate_d;
//	param_t yawrate_awu;
//	param_t yawrate_lim;

	param_t attrate_p;
	param_t attrate_i;
	param_t attrate_d;
//	param_t attrate_awu;
//	param_t attrate_lim;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct multirotor_att_control_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct multirotor_att_control_param_handles *h, struct multirotor_att_control_params *p);

#endif /* MULTIROTOR_ATT_CONTROL_PARAMS_H_ */
