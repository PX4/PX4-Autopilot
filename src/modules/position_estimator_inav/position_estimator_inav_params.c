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

PARAM_DEFINE_FLOAT(INAV_W_Z_BARO, 0.5f);
PARAM_DEFINE_FLOAT(INAV_W_Z_GPS_P, 0.005f);
PARAM_DEFINE_FLOAT(INAV_W_Z_SONAR, 3.0f);
PARAM_DEFINE_FLOAT(INAV_W_XY_GPS_P, 1.0f);
PARAM_DEFINE_FLOAT(INAV_W_XY_GPS_V, 2.0f);
PARAM_DEFINE_FLOAT(INAV_W_XY_FLOW, 5.0f);
PARAM_DEFINE_FLOAT(INAV_W_GPS_FLOW, 0.1f);
PARAM_DEFINE_FLOAT(INAV_W_ACC_BIAS, 0.05f);
PARAM_DEFINE_FLOAT(INAV_FLOW_K, 0.15f);
PARAM_DEFINE_FLOAT(INAV_FLOW_Q_MIN, 0.5f);
PARAM_DEFINE_FLOAT(INAV_SONAR_FILT, 0.05f);
PARAM_DEFINE_FLOAT(INAV_SONAR_ERR, 0.5f);
PARAM_DEFINE_FLOAT(INAV_LAND_T, 3.0f);
PARAM_DEFINE_FLOAT(INAV_LAND_DISP, 0.7f);
PARAM_DEFINE_FLOAT(INAV_LAND_THR, 0.2f);
PARAM_DEFINE_FLOAT(INAV_DELAY_GPS, 0.2f);

int parameters_init(struct position_estimator_inav_param_handles *h)
{
	h->w_z_baro = param_find("INAV_W_Z_BARO");
	h->w_z_gps_p = param_find("INAV_W_Z_GPS_P");
	h->w_z_sonar = param_find("INAV_W_Z_SONAR");
	h->w_xy_gps_p = param_find("INAV_W_XY_GPS_P");
	h->w_xy_gps_v = param_find("INAV_W_XY_GPS_V");
	h->w_xy_flow = param_find("INAV_W_XY_FLOW");
	h->w_gps_flow = param_find("INAV_W_GPS_FLOW");
	h->w_acc_bias = param_find("INAV_W_ACC_BIAS");
	h->flow_k = param_find("INAV_FLOW_K");
	h->flow_q_min = param_find("INAV_FLOW_Q_MIN");
	h->sonar_filt = param_find("INAV_SONAR_FILT");
	h->sonar_err = param_find("INAV_SONAR_ERR");
	h->land_t = param_find("INAV_LAND_T");
	h->land_disp = param_find("INAV_LAND_DISP");
	h->land_thr = param_find("INAV_LAND_THR");
	h->delay_gps = param_find("INAV_DELAY_GPS");

	return OK;
}

int parameters_update(const struct position_estimator_inav_param_handles *h, struct position_estimator_inav_params *p)
{
	param_get(h->w_z_baro, &(p->w_z_baro));
	param_get(h->w_z_gps_p, &(p->w_z_gps_p));
	param_get(h->w_z_sonar, &(p->w_z_sonar));
	param_get(h->w_xy_gps_p, &(p->w_xy_gps_p));
	param_get(h->w_xy_gps_v, &(p->w_xy_gps_v));
	param_get(h->w_xy_flow, &(p->w_xy_flow));
	param_get(h->w_gps_flow, &(p->w_gps_flow));
	param_get(h->w_acc_bias, &(p->w_acc_bias));
	param_get(h->flow_k, &(p->flow_k));
	param_get(h->flow_q_min, &(p->flow_q_min));
	param_get(h->sonar_filt, &(p->sonar_filt));
	param_get(h->sonar_err, &(p->sonar_err));
	param_get(h->land_t, &(p->land_t));
	param_get(h->land_disp, &(p->land_disp));
	param_get(h->land_thr, &(p->land_thr));
	param_get(h->delay_gps, &(p->delay_gps));

	return OK;
}
