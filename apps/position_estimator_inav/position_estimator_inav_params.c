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

/* Kalman Filter covariances */
/* gps measurement noise standard deviation */
PARAM_DEFINE_INT32(INAV_USE_GPS, 0);

PARAM_DEFINE_FLOAT(INAV_K_ALT_00, 0.0f);
PARAM_DEFINE_FLOAT(INAV_K_ALT_01, 0.0f);
PARAM_DEFINE_FLOAT(INAV_K_ALT_10, 0.0f);
PARAM_DEFINE_FLOAT(INAV_K_ALT_11, 0.0f);
PARAM_DEFINE_FLOAT(INAV_K_ALT_20, 0.0f);
PARAM_DEFINE_FLOAT(INAV_K_ALT_21, 0.0f);

PARAM_DEFINE_INT32(INAV_ACC_OFFS_0, 0);
PARAM_DEFINE_INT32(INAV_ACC_OFFS_1, 0);
PARAM_DEFINE_INT32(INAV_ACC_OFFS_2, 0);

PARAM_DEFINE_FLOAT(INAV_ACC_T_00, 0.0021f);
PARAM_DEFINE_FLOAT(INAV_ACC_T_01, 0.0f);
PARAM_DEFINE_FLOAT(INAV_ACC_T_02, 0.0f);
PARAM_DEFINE_FLOAT(INAV_ACC_T_10, 0.0f);
PARAM_DEFINE_FLOAT(INAV_ACC_T_11, 0.0021f);
PARAM_DEFINE_FLOAT(INAV_ACC_T_12, 0.0f);
PARAM_DEFINE_FLOAT(INAV_ACC_T_20, 0.0f);
PARAM_DEFINE_FLOAT(INAV_ACC_T_21, 0.0f);
PARAM_DEFINE_FLOAT(INAV_ACC_T_22, 0.0021f);

int parameters_init(struct position_estimator_inav_param_handles *h) {
	h->use_gps = param_find("INAV_USE_GPS");

	h->k_alt_00 = param_find("INAV_K_ALT_00");
	h->k_alt_01 = param_find("INAV_K_ALT_01");
	h->k_alt_10 = param_find("INAV_K_ALT_10");
	h->k_alt_11 = param_find("INAV_K_ALT_11");
	h->k_alt_20 = param_find("INAV_K_ALT_20");
	h->k_alt_21 = param_find("INAV_K_ALT_21");

	h->acc_offs_0 = param_find("INAV_ACC_OFFS_0");
	h->acc_offs_1 = param_find("INAV_ACC_OFFS_1");
	h->acc_offs_2 = param_find("INAV_ACC_OFFS_2");

	h->acc_t_00 = param_find("INAV_ACC_T_00");
	h->acc_t_01 = param_find("INAV_ACC_T_01");
	h->acc_t_02 = param_find("INAV_ACC_T_02");
	h->acc_t_10 = param_find("INAV_ACC_T_10");
	h->acc_t_11 = param_find("INAV_ACC_T_11");
	h->acc_t_12 = param_find("INAV_ACC_T_12");
	h->acc_t_20 = param_find("INAV_ACC_T_20");
	h->acc_t_21 = param_find("INAV_ACC_T_21");
	h->acc_t_22 = param_find("INAV_ACC_T_22");
	return OK;
}

int parameters_update(const struct position_estimator_inav_param_handles *h, struct position_estimator_inav_params *p) {
	param_get(h->use_gps, &(p->use_gps));

	param_get(h->k_alt_00, &(p->k[0][0]));
	param_get(h->k_alt_01, &(p->k[0][1]));
	param_get(h->k_alt_10, &(p->k[1][0]));
	param_get(h->k_alt_11, &(p->k[1][1]));
	param_get(h->k_alt_20, &(p->k[2][0]));
	param_get(h->k_alt_21, &(p->k[2][1]));

	/* read int32 and cast to int16 */
	int32_t t;
	param_get(h->acc_offs_0, &t);
	p->acc_offs[0] = t;
	param_get(h->acc_offs_1, &t);
	p->acc_offs[1] = t;
	param_get(h->acc_offs_2, &t);
	p->acc_offs[2] = t;

	param_get(h->acc_t_00, &(p->acc_T[0][0]));
	param_get(h->acc_t_01, &(p->acc_T[0][1]));
	param_get(h->acc_t_02, &(p->acc_T[0][2]));
	param_get(h->acc_t_10, &(p->acc_T[1][0]));
	param_get(h->acc_t_11, &(p->acc_T[1][1]));
	param_get(h->acc_t_12, &(p->acc_T[1][2]));
	param_get(h->acc_t_20, &(p->acc_T[2][0]));
	param_get(h->acc_t_21, &(p->acc_T[2][1]));
	param_get(h->acc_t_22, &(p->acc_T[2][2]));
	return OK;
}
