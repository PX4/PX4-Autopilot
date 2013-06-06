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
 * @file multirotor_att_control_params.c
 * 
 * Parameters for attitude control
 */

#include "multirotor_att_control_params.h"

/* controller parameters */
PARAM_DEFINE_FLOAT(MC_RCLOSS_THR, 0.0f); // This defines the throttle when the RC signal is lost.

PARAM_DEFINE_FLOAT(MC_YAWPOS_P, 0.3f);
PARAM_DEFINE_FLOAT(MC_YAWPOS_I, 0.15f);
PARAM_DEFINE_FLOAT(MC_YAWPOS_D, 0.0f);
//PARAM_DEFINE_FLOAT(MC_YAWPOS_AWU, 1.0f);
//PARAM_DEFINE_FLOAT(MC_YAWPOS_LIM, 3.0f);

PARAM_DEFINE_FLOAT(MC_YAWPOS_I_MAX, 1000.0f);

PARAM_DEFINE_FLOAT(MC_ATT_P, 0.2f);
PARAM_DEFINE_FLOAT(MC_ATT_I, 0.0f);
PARAM_DEFINE_FLOAT(MC_ATT_D, 0.05f);
//PARAM_DEFINE_FLOAT(MC_ATT_AWU, 0.05f);
//PARAM_DEFINE_FLOAT(MC_ATT_LIM, 0.4f);

//PARAM_DEFINE_FLOAT(MC_ATT_XOFF, 0.0f);
//PARAM_DEFINE_FLOAT(MC_ATT_YOFF, 0.0f);

PARAM_DEFINE_FLOAT(MC_YAWRATE_P, 0.0f); /* same on Flamewheel */
PARAM_DEFINE_FLOAT(MC_YAWRATE_D, 0.0f);
PARAM_DEFINE_FLOAT(MC_YAWRATE_I, 0.0f);
//PARAM_DEFINE_FLOAT(MC_YAWRATE_AWU, 0.0f);
//PARAM_DEFINE_FLOAT(MC_YAWRATE_LIM, 1.0f);

PARAM_DEFINE_FLOAT(MC_ATTRATE_P, 0.0f); /* 0.15 F405 Flamewheel */
PARAM_DEFINE_FLOAT(MC_ATTRATE_D, 0.0f);
PARAM_DEFINE_FLOAT(MC_ATTRATE_I, 0.0f);
//PARAM_DEFINE_FLOAT(MC_ATTRATE_AWU, 0.05f);
//PARAM_DEFINE_FLOAT(MC_ATTRATE_LIM, 1.0f);	/**< roughly < 500 deg/s limit */




int parameters_init(struct multirotor_att_control_param_handles *h)
{
	h->failsafe_throttle	=	param_find("MC_RCLOSS_THR");

	/* attitude control */
	h->yaw_p 				=	param_find("MC_YAWPOS_P");
	h->yaw_i 				=	param_find("MC_YAWPOS_I");
	h->yaw_d 				=	param_find("MC_YAWPOS_D");
//	h->yaw_awu 				=	param_find("MC_YAWPOS_AWU");
//	h->yaw_lim 				=	param_find("MC_YAWPOS_LIM");

	h->yaw_intmax 			=	param_find("MC_YAWPOS_I_MAX");

	h->att_p 				= 	param_find("MC_ATT_P");
	h->att_i 				= 	param_find("MC_ATT_I");
	h->att_d 				= 	param_find("MC_ATT_D");
//	h->att_awu 				= 	param_find("MC_ATT_AWU");
//	h->att_lim			 	= 	param_find("MC_ATT_LIM");

//	h->att_xoff 			= 	param_find("MC_ATT_XOFF");
//	h->att_yoff			 	= 	param_find("MC_ATT_YOFF");


	/* rate control */
	h->yawrate_p 			=	param_find("MC_YAWRATE_P");
	h->yawrate_i 			=	param_find("MC_YAWRATE_I");
	h->yawrate_d 			=	param_find("MC_YAWRATE_D");
//	h->yawrate_awu 			=	param_find("MC_YAWRATE_AWU");
//	h->yawrate_lim 			=	param_find("MC_YAWRATE_LIM");

	h->attrate_p 			= 	param_find("MC_ATTRATE_P");
	h->attrate_i 			= 	param_find("MC_ATTRATE_I");
	h->attrate_d 			= 	param_find("MC_ATTRATE_D");
//	h->attrate_awu 			= 	param_find("MC_ATTRATE_AWU");
//	h->attrate_lim 			= 	param_find("MC_ATTRATE_LIM");

	return OK;
}

int parameters_update(const struct multirotor_att_control_param_handles *h, struct multirotor_att_control_params *p)
{
	param_get(h->failsafe_throttle, &(p->failsafe_throttle));

	/* attitude control */
	param_get(h->yaw_p, &(p->yaw_p));
	param_get(h->yaw_i, &(p->yaw_i));
	param_get(h->yaw_d, &(p->yaw_d));
//	param_get(h->yaw_awu, &(p->yaw_awu));
//	param_get(h->yaw_lim, &(p->yaw_lim));

	param_get(h->yaw_intmax, &(p->yaw_intmax));

	param_get(h->att_p, &(p->att_p));
	param_get(h->att_i, &(p->att_i));
	param_get(h->att_d, &(p->att_d));
//	param_get(h->att_awu, &(p->att_awu));
//	param_get(h->att_lim, &(p->att_lim));

//	param_get(h->att_xoff, &(p->att_xoff));
//	param_get(h->att_yoff, &(p->att_yoff));


	/* rate control */
	param_get(h->yawrate_p, &(p->yawrate_p));
	param_get(h->yawrate_i, &(p->yawrate_i));
	param_get(h->yawrate_d, &(p->yawrate_d));
//	param_get(h->yawrate_awu, &(p->yawrate_awu));
//	param_get(h->yawrate_lim, &(p->yawrate_lim));

	param_get(h->attrate_p, &(p->attrate_p));
	param_get(h->attrate_i, &(p->attrate_i));
	param_get(h->attrate_d, &(p->attrate_d));
//	param_get(h->attrate_awu, &(p->attrate_awu));
//	param_get(h->attrate_lim, &(p->attrate_lim));

	return OK;
}
