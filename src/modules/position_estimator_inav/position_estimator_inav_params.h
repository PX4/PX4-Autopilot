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
 * @file position_estimator_inav_params.h
 * 
 * Parameters for Position Estimator
 */

#include <systemlib/param/param.h>

struct position_estimator_inav_params {
	int use_gps;
	float k_alt[3][2];
	float k_pos[3][3];
};

struct position_estimator_inav_param_handles {
	param_t use_gps;

	param_t k_alt_00;
	param_t k_alt_01;
	param_t k_alt_10;
	param_t k_alt_11;
	param_t k_alt_20;
	param_t k_alt_21;

	param_t k_pos_00;
	param_t k_pos_01;
	param_t k_pos_02;
	param_t k_pos_10;
	param_t k_pos_11;
	param_t k_pos_12;
	param_t k_pos_20;
	param_t k_pos_21;
	param_t k_pos_22;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct position_estimator_inav_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct position_estimator_inav_param_handles *h, struct position_estimator_inav_params *p);
