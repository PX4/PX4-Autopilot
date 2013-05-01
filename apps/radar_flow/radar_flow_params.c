/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Samuel Zihlmann <samuezih@ee.ethz.ch>
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
 * @file radar_flow_params.c
 */

#include "radar_flow_params.h"

/* radar parameters */
PARAM_DEFINE_FLOAT(RF_POS_SP_X, 0.0f);
PARAM_DEFINE_FLOAT(RF_POS_SP_Y, 0.0f);
PARAM_DEFINE_FLOAT(RF_BEEP_F, 0.0f);
PARAM_DEFINE_FLOAT(RF_BEEP_B, 0.0f);
PARAM_DEFINE_FLOAT(RF_KAL_K1, 0.0235f);
PARAM_DEFINE_FLOAT(RF_KAL_K2, 0.0140f);
PARAM_DEFINE_FLOAT(RF_FRONT_A, 0.1f);
PARAM_DEFINE_INT32(RF_SONAR, 0);
PARAM_DEFINE_INT32(RF_POS_UPDATE, 0);
PARAM_DEFINE_FLOAT(RF_S0, 0.05f);
PARAM_DEFINE_FLOAT(RF_S1, 0.05f);
PARAM_DEFINE_FLOAT(RF_S2, 2.0f);
PARAM_DEFINE_FLOAT(RF_S3, 0.01f);
PARAM_DEFINE_FLOAT(RF_S4, 0.3f);
PARAM_DEFINE_FLOAT(RF_S5, 0.05f);
PARAM_DEFINE_FLOAT(RF_S6, 0.01f);
PARAM_DEFINE_FLOAT(RF_S7, 0.001f);
PARAM_DEFINE_INT32(RF_DEBUG, 0);


int parameters_init(struct radar_flow_param_handles *h)
{
	h->pos_sp_x	 			=	param_find("RF_POS_SP_X");
	h->pos_sp_y				=	param_find("RF_POS_SP_Y");
	h->beep_front_sonar		=	param_find("RF_BEEP_F");
	h->beep_bottom_sonar	=	param_find("RF_BEEP_B");
	h->kalman_k1	 		=	param_find("RF_KAL_K1");
	h->kalman_k2			=	param_find("RF_KAL_K2");
	h->front_lp_alpha		=	param_find("RF_FRONT_A");
	h->with_sonar			=	param_find("RF_SONAR");
	h->with_pos_update		=	param_find("RF_POS_UPDATE");
	h->s0					=	param_find("RF_S0");
	h->s1					=	param_find("RF_S1");
	h->s2					=	param_find("RF_S2");
	h->s3					=	param_find("RF_S3");
	h->s4					=	param_find("RF_S4");
	h->s5					=	param_find("RF_S5");
	h->s6					=	param_find("RF_S6");
	h->s7					=	param_find("RF_S7");
	h->debug				=	param_find("RF_DEBUG");

	return OK;
}

int parameters_update(const struct radar_flow_param_handles *h, struct radar_flow_params *p)
{
	param_get(h->pos_sp_x, &(p->pos_sp_x));
	param_get(h->pos_sp_y, &(p->pos_sp_y));
	param_get(h->beep_front_sonar, &(p->beep_front_sonar));
	param_get(h->beep_bottom_sonar, &(p->beep_bottom_sonar));
	param_get(h->kalman_k1, &(p->kalman_k1));
	param_get(h->kalman_k2, &(p->kalman_k2));
	param_get(h->front_lp_alpha, &(p->front_lp_alpha));
	param_get(h->with_sonar, &(p->with_sonar));
	param_get(h->with_pos_update, &(p->with_pos_update));
	param_get(h->s0, &(p->s0));
	param_get(h->s1, &(p->s1));
	param_get(h->s2, &(p->s2));
	param_get(h->s3, &(p->s3));
	param_get(h->s4, &(p->s4));
	param_get(h->s5, &(p->s5));
	param_get(h->s6, &(p->s6));
	param_get(h->s7, &(p->s7));
	param_get(h->debug, &(p->debug));

	return OK;
}
