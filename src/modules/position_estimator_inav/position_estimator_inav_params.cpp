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
 * Z axis weight for lidar
 *
 * Weight (cutoff frequency) for lidar measurements.
 *
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_Z_LIDAR, 3.0f);

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
 * Weight for mocap system
 *
 * Weight (cutoff frequency) for mocap position measurements.
 *
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */

PARAM_DEFINE_FLOAT(INAV_W_MOC_P, 10.0f);

/**
 * XY axis weight for optical flow
 *
 * Weight (cutoff frequency) for optical flow (velocity) measurements.
 *
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_XY_FLOW, 0.8f);

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
 * Factor to scale optical flow
 *
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_FLOW_K, 1.35f);

/**
 * Minimal acceptable optical flow quality
 *
 * 0 - lowest quality, 1 - best quality.
 *
 * @min 0.0
 * @max 1.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_FLOW_Q_MIN, 0.3f);

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
PARAM_DEFINE_FLOAT(INAV_LIDAR_ERR, 0.2f);

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
 * Flow module offset (center of rotation) in X direction
 *
 * Yaw X flow compensation
 *
 * @min -1.0
 * @max 1.0
 * @unit m
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_FLOW_DIST_X, 0.0f);

/**
 * Flow module offset (center of rotation) in Y direction
 *
 * Yaw Y flow compensation
 *
 * @min -1.0
 * @max 1.0
 * @unit m
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_FLOW_DIST_Y, 0.0f);

/**
 * Disable mocap (set 0 if using fake gps)
 *
 * Disable mocap
 *
 * @boolean
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_DISAB_MOCAP, 0);

/**
 * Enable LIDAR for altitude estimation
 *
 * Enable LIDAR for altitude estimation
 *
 * @boolean
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_LIDAR_EST, 0);

/**
 * LIDAR calibration offset
 *
 * LIDAR calibration offset. Value will be added to the measured distance
 *
 * @min -20
 * @max 20
 * @unit m
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_LIDAR_OFF, 0.0f);

/**
 * Disable vision input
 *
 * Set to the appropriate key (328754) to disable vision input.
 *
 * @reboot_required true
 * @min 0
 * @max 328754
 * @group Position Estimator INAV
 */
PARAM_DEFINE_INT32(CBRK_NO_VISION, 0);

int inav_parameters_init(struct position_estimator_inav_param_handles *h)
{
	h->w_z_baro = param_find("INAV_W_Z_BARO");
	h->w_z_gps_p = param_find("INAV_W_Z_GPS_P");
	h->w_z_gps_v = param_find("INAV_W_Z_GPS_V");
	h->w_z_vision_p = param_find("INAV_W_Z_VIS_P");
	h->w_z_lidar = param_find("INAV_W_Z_LIDAR");
	h->w_xy_gps_p = param_find("INAV_W_XY_GPS_P");
	h->w_xy_gps_v = param_find("INAV_W_XY_GPS_V");
	h->w_xy_vision_p = param_find("INAV_W_XY_VIS_P");
	h->w_xy_vision_v = param_find("INAV_W_XY_VIS_V");
	h->w_mocap_p = param_find("INAV_W_MOC_P");
	h->w_xy_flow = param_find("INAV_W_XY_FLOW");
	h->w_xy_res_v = param_find("INAV_W_XY_RES_V");
	h->w_gps_flow = param_find("INAV_W_GPS_FLOW");
	h->w_acc_bias = param_find("INAV_W_ACC_BIAS");
	h->flow_k = param_find("INAV_FLOW_K");
	h->flow_q_min = param_find("INAV_FLOW_Q_MIN");
	h->lidar_err = param_find("INAV_LIDAR_ERR");
	h->land_t = param_find("INAV_LAND_T");
	h->land_disp = param_find("INAV_LAND_DISP");
	h->land_thr = param_find("INAV_LAND_THR");
	h->no_vision = param_find("CBRK_NO_VISION");
	h->delay_gps = param_find("INAV_DELAY_GPS");
	h->flow_module_offset_x = param_find("INAV_FLOW_DIST_X");
	h->flow_module_offset_y = param_find("INAV_FLOW_DIST_Y");
	h->disable_mocap = param_find("INAV_DISAB_MOCAP");
	h->enable_lidar_alt_est = param_find("INAV_LIDAR_EST");
	h->lidar_calibration_offset = param_find("INAV_LIDAR_OFF");
	h->att_ext_hdg_m = param_find("ATT_EXT_HDG_M");

	return 0;
}

int inav_parameters_update(const struct position_estimator_inav_param_handles *h,
			   struct position_estimator_inav_params *p)
{
	param_get(h->w_z_baro, &(p->w_z_baro));
	param_get(h->w_z_gps_p, &(p->w_z_gps_p));
	param_get(h->w_z_gps_v, &(p->w_z_gps_v));
	param_get(h->w_z_vision_p, &(p->w_z_vision_p));
	param_get(h->w_z_lidar, &(p->w_z_lidar));
	param_get(h->w_xy_gps_p, &(p->w_xy_gps_p));
	param_get(h->w_xy_gps_v, &(p->w_xy_gps_v));
	param_get(h->w_xy_vision_p, &(p->w_xy_vision_p));
	param_get(h->w_xy_vision_v, &(p->w_xy_vision_v));
	param_get(h->w_mocap_p, &(p->w_mocap_p));
	param_get(h->w_xy_flow, &(p->w_xy_flow));
	param_get(h->w_xy_res_v, &(p->w_xy_res_v));
	param_get(h->w_gps_flow, &(p->w_gps_flow));
	param_get(h->w_acc_bias, &(p->w_acc_bias));
	param_get(h->flow_k, &(p->flow_k));
	param_get(h->flow_q_min, &(p->flow_q_min));
	param_get(h->lidar_err, &(p->lidar_err));
	param_get(h->land_t, &(p->land_t));
	param_get(h->land_disp, &(p->land_disp));
	param_get(h->land_thr, &(p->land_thr));
	param_get(h->no_vision, &(p->no_vision));
	param_get(h->delay_gps, &(p->delay_gps));
	param_get(h->flow_module_offset_x, &(p->flow_module_offset_x));
	param_get(h->flow_module_offset_y, &(p->flow_module_offset_y));
	param_get(h->disable_mocap, &(p->disable_mocap));
	param_get(h->enable_lidar_alt_est, &(p->enable_lidar_alt_est));
	param_get(h->lidar_calibration_offset, &(p->lidar_calibration_offset));
	param_get(h->att_ext_hdg_m, &(p->att_ext_hdg_m));

	return 0;
}
