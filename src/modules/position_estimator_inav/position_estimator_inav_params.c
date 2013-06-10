/****************************************************************************
 *
 *   Copyright (C) 2013 Anton Babushkin. All rights reserved.
 *   Author: 	Anton Babushkin	<rk3dov@gmail.com>
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
 * @file position_estimator_inav_params.c
 *
 * Parameters for position_estimator_inav
 */

#include "position_estimator_inav_params.h"

PARAM_DEFINE_INT32(INAV_USE_GPS, 1);
PARAM_DEFINE_FLOAT(INAV_W_ALT_BARO, 1.0f);
PARAM_DEFINE_FLOAT(INAV_W_ALT_ACC, 50.0f);
PARAM_DEFINE_FLOAT(INAV_W_POS_GPS_P, 1.0f);
PARAM_DEFINE_FLOAT(INAV_W_POS_GPS_V, 1.0f);
PARAM_DEFINE_FLOAT(INAV_W_POS_ACC, 50.0f);

int parameters_init(struct position_estimator_inav_param_handles *h)
{
	h->use_gps = param_find("INAV_USE_GPS");
	h->w_alt_baro = param_find("INAV_W_ALT_BARO");
	h->w_alt_acc = param_find("INAV_W_ALT_ACC");
	h->w_pos_gps_p = param_find("INAV_W_POS_GPS_P");
	h->w_pos_gps_v = param_find("INAV_W_POS_GPS_V");
	h->w_pos_acc = param_find("INAV_W_POS_ACC");

	return OK;
}

int parameters_update(const struct position_estimator_inav_param_handles *h, struct position_estimator_inav_params *p)
{
	param_get(h->use_gps, &(p->use_gps));
	param_get(h->w_alt_baro, &(p->w_alt_baro));
	param_get(h->w_alt_acc, &(p->w_alt_acc));
	param_get(h->w_pos_gps_p, &(p->w_pos_gps_p));
	param_get(h->w_pos_gps_v, &(p->w_pos_gps_v));
	param_get(h->w_pos_acc, &(p->w_pos_acc));

	return OK;
}
