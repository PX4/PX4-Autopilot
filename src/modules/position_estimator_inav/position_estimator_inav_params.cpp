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
 * 气压计高度测量的权重(截止频率)
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
 * GPS高度测量的权重(截止频率)，GPS高度数据噪声很大，应该仅用于气压计偏移的缓慢校正
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
 * GPS垂直速度测量的权重(截止频率)
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
 * 视觉高度测量的的权重(截止频率)，视觉高度数据噪声很大
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
 * lidar测量的权重(截止频率)
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
 * GPS位置测量的权重(截止频率)
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
 * GPS速度测量的权重(截止频率)
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
 * 视觉位置测量的权重(截止频率)
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
 * 视觉速度测量的权重(截止频率)
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
 * 动作捕捉装置位置测量的权重(截止频率)
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
 * 光流(速度)测量的权重(截止频率)
 * 
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_XY_FLOW, 0.8f);

/**
 * XY axis weight for resetting velocity 
 * 用于重置速度的XY轴权重
 *
 * When velocity sources lost slowly decrease estimated horizontal velocity with this weight.
 * 当速度源丢失，用这个权重缓慢减少估计的水平速度
 *
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_XY_RES_V, 0.5f);

/**
 * XY axis weight factor for GPS when optical flow available
 * 当光流可用时GPS的XY轴上的权重因子
 *
 * When optical flow data available, multiply GPS weights (for position and velocity) by this factor.
 * 当光流数据可用时，将GPS的权重(用于位置和速度)乘以这个因子
 * 
 * @min 0.0
 * @max 1.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_W_GPS_FLOW, 0.1f);

/**
 * Accelerometer bias estimation weight
 * 加速度计偏差估计权重
 *
 * Weight (cutoff frequency) for accelerometer bias estimation. 0 to disable.
 * 加速度计偏差估计的权重(截止频率)。将此值设置为0以禁用。
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
 * 光流比例因子
 * 
 * @min 0.0
 * @max 10.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_FLOW_K, 1.35f);

/**
 * Minimal acceptable optical flow quality
 * 可接受的光流数据质量的最小值
 *
 * 0 - lowest quality, 1 - best quality.
 * 0 - 最低质量，1 - 最高质量
 *
 * @min 0.0
 * @max 1.0
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_FLOW_Q_MIN, 0.3f);

/**
 * Sonar maximal error for new surface
 * 对于一个新的表面的最大声呐误差
 *
 * If sonar measurement error is larger than this value it skiped (spike) or accepted as new surface level (if offset is stable).
 * 如果声呐的测量误差比这个值大，将跳过或者被接受为一个新的表面层(如果偏移很稳定)
 *
 * @min 0.0
 * @max 1.0
 * @unit m
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_LIDAR_ERR, 0.2f);

/**
 * Land detector time 
 * 着陆检测时间
 *
 * Vehicle assumed landed if no altitude changes happened during this time on low throttle.
 * 油门杆拉低时，如果在这段时间内没有高度改变发生，飞行器将进行着陆操作
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
 * 触发着陆检测的分散阈值
 *
 * @min 0.0
 * @max 10.0
 * @unit m
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_LAND_DISP, 0.7f);

/**
 * Land detector throttle threshold
 * 着陆检测的油门阈值
 *
 * Value should be lower than minimal hovering thrust. Half of it is good choice.
 * 这个值应该比飞行器的最小悬停推力小。取其一半为佳
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
 * GPS延迟补偿
 *
 * @min 0.0
 * @max 1.0
 * @unit s
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_DELAY_GPS, 0.2f);

/**
 * Flow module offset (center of rotation) in X direction
 * 光流模块X方向上的偏移(离旋转中心)
 *
 * Yaw X flow compensation
 * 
 *
 * @min -1.0
 * @max 1.0
 * @unit m
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_FLOW_DIST_X, 0.0f);

/**
 * Flow module offset (center of rotation) in Y direction
 * 光流模块Y方向上的偏移(离旋转中心)
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
 * Mo-cap 动作捕捉
 *
 * Set to 0 if using fake GPS
 * 如果使用了假GPS定位，将此值置0
 * 
 * @value 0 Mo-cap enabled 值为0时，使能动作捕捉装置
 * @value 1 Mo-cap disabled
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_DISAB_MOCAP, 0);

/**
 * LIDAR for altitude estimation
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
 * LIDAR for altitude estimation
 * 用于高度估计的LIDAR
 *
 * @boolean
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_LIDAR_EST, 1);

/**
 * LIDAR calibration offset 
 * 激光雷达偏移校准
 *
 * LIDAR calibration offset. Value will be added to the measured distance
 * 激光雷达偏移校准。此值与测量得到的距离相加
 *
 * @min -20
 * @max 20
 * @unit m
 * @group Position Estimator INAV
 */
PARAM_DEFINE_FLOAT(INAV_LIDAR_OFF, 0.0f);

/**
 * Disable vision input
 * 禁用视觉输入
 *
 * Set to the appropriate key (328754) to disable vision input.
 * 将此值设置为一个合适的值(328754)以禁用视觉输入
 *
 * @reboot_required true 需要重启
 * @min 0
 * @max 328754
 * @group Position Estimator INAV
 */
PARAM_DEFINE_INT32(CBRK_NO_VISION, 0);

int inav_parameters_init(struct position_estimator_inav_param_handles *h)
{
	h->w_z_baro = param_find("INAV_W_Z_BARO"); // 初始化参数句柄 handle
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
{    // 把h的值给P
	param_get(h->w_z_baro, &(p->w_z_baro));     //更新函数
	param_get(h->w_z_gps_p, &(p->w_z_gps_p));// 使用内存拷贝的方式
	param_get(h->w_z_gps_v, &(p->w_z_gps_v)); //将参数句柄中的值赋给参数变量（h赋值给p）
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
