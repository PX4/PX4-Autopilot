/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @author Anton Babushkin <rk3dov@gmail.com>
 *
 * Parameters for position_estimator_inav
 */

#include "position_estimator_inav_params.h"

/**
 * Z axis weight for barometer
 *
 * Weight (cutoff frequency) for barometer altitude measurements.
 *
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_Z_BARO, 0.5f);

/**
 * Z axis weight for GPS
 *
 * Weight (cutoff frequency) for GPS altitude measurements. GPS altitude data is very noisy and should be used only as slow correction for baro offset.
 *
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_Z_GPS_P, 0.005f);

/**
 * Z velocity weight for GPS
 *
 * Weight (cutoff frequency) for GPS altitude velocity measurements.
 *
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_Z_GPS_V, 0.0f);

/**
 * Z axis weight for vision
 *
 * Weight (cutoff frequency) for vision altitude measurements. vision altitude data is very noisy and should be used only as slow correction for baro offset.
 *
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_Z_VIS_P, 5.0f);

/**
 * Z axis weight for sonar
 *
 * Weight (cutoff frequency) for sonar measurements.
 *
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_Z_SONAR, 3.0f);

/**
 * XY axis weight for GPS position
 *
 * Weight (cutoff frequency) for GPS position measurements.
 *
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_XY_GPS_P, 1.0f);

/**
 * XY axis weight for GPS velocity
 *
 * Weight (cutoff frequency) for GPS velocity measurements.
 *
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_XY_GPS_V, 2.0f);

/**
 * XY axis weight for vision position
 *
 * Weight (cutoff frequency) for vision position measurements.
 *
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_XY_VIS_P, 7.0f);

/**
 * XY axis weight for vision velocity
 *
 * Weight (cutoff frequency) for vision velocity measurements.
 *
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_XY_VIS_V, 0.0f);

/**
 * XY axis weight for optical flow
 *
 * Weight (cutoff frequency) for optical flow (velocity) measurements.
 *
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_XY_FLOW, 5.0f);

/**
 * XY axis weight for resetting velocity
 *
 * When velocity sources lost slowly decrease estimated horizontal velocity with this weight.
 *
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_XY_RES_V, 0.5f);

/**
 * XY axis weight factor for GPS when optical flow available
 *
 * When optical flow data available, multiply GPS weights (for position and velocity) by this factor.
 *
 * @min 0.0
 * @max 1.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_GPS_FLOW, 0.1f);

/**
 * Accelerometer bias estimation weight
 *
 * Weight (cutoff frequency) for accelerometer bias estimation. 0 to disable.
 *
 * @min 0.0
 * @max 0.1
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_ACC_BIAS, 0.05f);

/**
 * Optical flow scale factor
 *
 * Factor to convert raw optical flow (in pixels) to radians [rad/px].
 *
 * @min 0.0
 * @max 1.0
 * @unit rad/px
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_FLOW_K, 0.15f);

/**
 * Minimal acceptable optical flow quality
 *
 * 0 - lowest quality, 1 - best quality.
 *
 * @min 0.0
 * @max 1.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_FLOW_Q_MIN, 0.5f);

/**
 * Weight for sonar filter
 *
 * Sonar filter detects spikes on sonar measurements and used to detect new surface level.
 *
 * @min 0.0
 * @max 1.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_SONAR_FILT, 0.05f);

/**
 * Sonar maximal error for new surface
 *
 * If sonar measurement error is larger than this value it skiped (spike) or accepted as new surface level (if offset is stable).
 *
 * @min 0.0
 * @max 1.0
 * @unit m
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_SONAR_ERR, 0.5f);

/**
 * Land detector time
 *
 * Vehicle assumed landed if no altitude changes happened during this time on low throttle.
 *
 * @min 0.0
 * @max 10.0
 * @unit s
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_LAND_T, 3.0f);

/**
 * Land detector altitude dispersion threshold
 *
 * Dispersion threshold for triggering land detector.
 *
 * @min 0.0
 * @max 10.0
 * @unit m
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_LAND_DISP, 0.7f);

/**
 * Land detector throttle threshold
 *
 * Value should be lower than minimal hovering thrust. Half of it is good choice.
 *
 * @min 0.0
 * @max 1.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_LAND_THR, 0.2f);

/**
 * GPS delay
 *
 * GPS delay compensation
 *
 * @min 0.0
 * @max 1.0
 * @unit s
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_DELAY_GPS, 0.2f);

/**
 * Disable vision input
 *
 * Set to the appropriate key (328754) to disable vision input.
 *
 * @min 0
 * @max 1
 * @group Position Estimator INAV
 */
PARAM_DEFINE_INT32(CBRK_NO_VISION, 0);

/**
 * INAV enabled
 *
 * If set to 1, use INAV for position estimation.
 * Else the system uses the combined attitude / position
 * filter framework.
 *
 * @min 0
 * @max 1
 * @group Position Estimator INAV
 */
PARAM_DEFINE_INT32(INAV_ENABLED, 1);

int parameters_init(struct position_estimator_inav_param_handles *h)
{
	h->w_z_baro = param_find("INAV_W_Z_BARO");
	h->w_z_gps_p = param_find("INAV_W_Z_GPS_P");
	h->w_z_gps_v = param_find("INAV_W_Z_GPS_V");
	h->w_z_vision_p = param_find("INAV_W_Z_VIS_P");
	h->w_z_sonar = param_find("INAV_W_Z_SONAR");
	h->w_xy_gps_p = param_find("INAV_W_XY_GPS_P");
	h->w_xy_gps_v = param_find("INAV_W_XY_GPS_V");
	h->w_xy_vision_p = param_find("INAV_W_XY_VIS_P");
	h->w_xy_vision_v = param_find("INAV_W_XY_VIS_V");
	h->w_xy_flow = param_find("INAV_W_XY_FLOW");
	h->w_xy_res_v = param_find("INAV_W_XY_RES_V");
	h->w_gps_flow = param_find("INAV_W_GPS_FLOW");
	h->w_acc_bias = param_find("INAV_W_ACC_BIAS");
	h->flow_k = param_find("INAV_FLOW_K");
	h->flow_q_min = param_find("INAV_FLOW_Q_MIN");
	h->sonar_filt = param_find("INAV_SONAR_FILT");
	h->sonar_err = param_find("INAV_SONAR_ERR");
	h->land_t = param_find("INAV_LAND_T");
	h->land_disp = param_find("INAV_LAND_DISP");
	h->land_thr = param_find("INAV_LAND_THR");
	h->no_vision = param_find("CBRK_NO_VISION");
	h->delay_gps = param_find("INAV_DELAY_GPS");

	return OK;
}

int parameters_update(const struct position_estimator_inav_param_handles *h, struct position_estimator_inav_params *p)
{
	param_get(h->w_z_baro, &(p->w_z_baro));
	param_get(h->w_z_gps_p, &(p->w_z_gps_p));
	param_get(h->w_z_vision_p, &(p->w_z_vision_p));
	param_get(h->w_z_sonar, &(p->w_z_sonar));
	param_get(h->w_xy_gps_p, &(p->w_xy_gps_p));
	param_get(h->w_xy_gps_v, &(p->w_xy_gps_v));
	param_get(h->w_xy_vision_p, &(p->w_xy_vision_p));
	param_get(h->w_xy_vision_v, &(p->w_xy_vision_v));
	param_get(h->w_xy_flow, &(p->w_xy_flow));
	param_get(h->w_xy_res_v, &(p->w_xy_res_v));
	param_get(h->w_gps_flow, &(p->w_gps_flow));
	param_get(h->w_acc_bias, &(p->w_acc_bias));
	param_get(h->flow_k, &(p->flow_k));
	param_get(h->flow_q_min, &(p->flow_q_min));
	param_get(h->sonar_filt, &(p->sonar_filt));
	param_get(h->sonar_err, &(p->sonar_err));
	param_get(h->land_t, &(p->land_t));
	param_get(h->land_disp, &(p->land_disp));
	param_get(h->land_thr, &(p->land_thr));
	param_get(h->no_vision, &(p->no_vision));
	param_get(h->delay_gps, &(p->delay_gps));

	return OK;
}
