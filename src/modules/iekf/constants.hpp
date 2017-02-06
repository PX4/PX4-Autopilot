/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#pragma once

#include<mavlink/mavlink_bridge_header.h>

const double deg2rad = M_PI / 180;
const double rad2deg = 180 / M_PI;
const float deg2radf = M_PI_F / 180;
const float rad2degf = 180 / M_PI_F;

/**
 * Note that structs are used instead of enums
 * to allow use in arrays without casting
 * and to keep size small
 */

/**
 * State enum
 */
enum : uint8_t {
	X_q_nb_0 = 0,
	X_q_nb_1,
	X_q_nb_2,
	X_q_nb_3,
	X_angvel_bX,
	X_angvel_bY,
	X_angvel_bZ,
	X_vel_N,
	X_vel_E,
	X_vel_D,
	X_gyro_bias_bX,
	X_gyro_bias_bY,
	X_gyro_bias_bZ,
	X_accel_bias_bX,
	X_accel_bias_bY,
	X_accel_bias_bZ,
	X_pos_N,
	X_pos_E,
	X_asl,
	X_terrain_asl,
	X_baro_bias,
	//X_wind_N,
	//X_wind_E,
	//X_wind_D,
	X_n,
};

// mavlink sensor type for states
const uint8_t XSensors[X_n] = {
	// q
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	// angular velocity
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	// velocity
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	// acceleration
	//MAV_SENSOR_TYPE_NONE,
	//MAV_SENSOR_TYPE_NONE,
	//MAV_SENSOR_TYPE_NONE,
	// gyro bias
	MAV_SENSOR_TYPE_GYRO,
	MAV_SENSOR_TYPE_GYRO,
	MAV_SENSOR_TYPE_GYRO,
	// accel bias
	MAV_SENSOR_TYPE_ACCEL,
	MAV_SENSOR_TYPE_ACCEL,
	MAV_SENSOR_TYPE_ACCEL,
	// pos
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	// asl
	MAV_SENSOR_TYPE_NONE,
	// terrain asl
	MAV_SENSOR_TYPE_NONE,
	// baro bias
	MAV_SENSOR_TYPE_BARO,
};

// mavlink fields types for states
const uint8_t  XIDs[X_n] = {
	// q
	MAV_FIELD_Q0,
	MAV_FIELD_Q1,
	MAV_FIELD_Q2,
	MAV_FIELD_Q3,
	// angular velocity
	// XXX is actually in nav frame, need to add to mavlink
	MAV_FIELD_ANGVEL_X,
	MAV_FIELD_ANGVEL_Y,
	MAV_FIELD_ANGVEL_Z,
	// vel
	MAV_FIELD_VEL_N,
	MAV_FIELD_VEL_E,
	MAV_FIELD_VEL_D,
	// gyro bias
	MAV_FIELD_BIAS_X,
	MAV_FIELD_BIAS_Y,
	MAV_FIELD_BIAS_Z,
	// accel bias
	MAV_FIELD_BIAS_X,
	MAV_FIELD_BIAS_Y,
	MAV_FIELD_BIAS_Z,
	// pos
	MAV_FIELD_POS_N,
	MAV_FIELD_POS_E,
	// asl
	MAV_FIELD_ASL,
	// terrain asl
	MAV_FIELD_TERRAIN_ASL,
	// baro bias
	MAV_FIELD_BIAS_D,
};

/**
 * Error state enum
 * used for linearization
 *
 * Note gyro bias in navigation frame
 */
enum : uint8_t {
	Xe_rot_N = 0,
	Xe_rot_E,
	Xe_rot_D,
	Xe_angvel_N,
	Xe_angvel_E,
	Xe_angvel_D,
	Xe_vel_N,
	Xe_vel_E,
	Xe_vel_D,
	Xe_gyro_bias_N,
	Xe_gyro_bias_E,
	Xe_gyro_bias_D,
	Xe_accel_bias_N,
	Xe_accel_bias_E,
	Xe_accel_bias_D,
	Xe_pos_N,
	Xe_pos_E,
	Xe_asl,
	Xe_terrain_asl,
	Xe_baro_bias,
	//Xe_wind_N,
	//Xe_wind_E,
	//Xe_wind_D,
	Xe_n,
};

// mavlink sensor type for error states
const uint8_t XeSensors[Xe_n] = {
	// rot
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	// angular velocity
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	// vel
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	// acc
	//MAV_SENSOR_TYPE_NONE,
	//MAV_SENSOR_TYPE_NONE,
	//MAV_SENSOR_TYPE_NONE,
	// gyro bias
	MAV_SENSOR_TYPE_GYRO,
	MAV_SENSOR_TYPE_GYRO,
	MAV_SENSOR_TYPE_GYRO,
	// accel bias
	MAV_SENSOR_TYPE_ACCEL,
	MAV_SENSOR_TYPE_ACCEL,
	MAV_SENSOR_TYPE_ACCEL,
	// pos
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	// asl
	MAV_SENSOR_TYPE_NONE,
	// terrain
	MAV_SENSOR_TYPE_NONE,
	// baro
	MAV_SENSOR_TYPE_BARO,
};

// mavlink fields types for error states
const uint8_t XeIDs[Xe_n] = {
	// rot
	MAV_FIELD_ROLL,
	MAV_FIELD_PITCH,
	MAV_FIELD_YAW,
	// angular velocity
	// XXX is actually in nav frame, need to add to mavlink
	MAV_FIELD_ANGVEL_X,
	MAV_FIELD_ANGVEL_Y,
	MAV_FIELD_ANGVEL_Z,
	// vel
	MAV_FIELD_VEL_N,
	MAV_FIELD_VEL_E,
	MAV_FIELD_VEL_D,
	// gyro bias
	MAV_FIELD_BIAS_X,
	MAV_FIELD_BIAS_Y,
	MAV_FIELD_BIAS_Z,
	// accel bias
	MAV_FIELD_BIAS_X,
	MAV_FIELD_BIAS_Y,
	MAV_FIELD_BIAS_Z,
	// pos
	MAV_FIELD_POS_N,
	MAV_FIELD_POS_E,
	// asl
	MAV_FIELD_ASL,
	// terrain
	MAV_FIELD_TERRAIN_ASL,
	// baro
	MAV_FIELD_BIAS_D,
};

/**
 * Input enum
 */
enum : uint8_t {
	U_omega_nb_bX = 0,
	U_omega_nb_bY,
	U_omega_nb_bZ,
	U_accel_bX,
	U_accel_bY,
	U_accel_bZ,
	U_n,
};

/**
 * Accel measurement enum
 */
enum : uint8_t {
	Y_accel_acc_N = 0,
	Y_accel_acc_E,
	Y_accel_acc_D,
	Y_accel_n,
};

/**
 * Land enum
 */
enum : uint8_t {
	Y_land_vel_N = 0,
	Y_land_vel_E,
	Y_land_vel_D,
	Y_land_agl,
	Y_land_n,
};

/**
 * GPS measurement
 */
enum : uint8_t {
	Y_gps_pos_N = 0,
	Y_gps_pos_E,
	Y_gps_asl,
	Y_gps_vel_N,
	Y_gps_vel_E,
	Y_gps_vel_D,
	Y_gps_n,
};

/**
 * Baro measurement
 */
enum : uint8_t {
	Y_baro_asl = 0,
	Y_baro_n,
};

/**
 * Magnetometer measurement
 */
enum : uint8_t {
	Y_mag_hdg = 0,
	Y_mag_n,
};

/**
 * Gyroscope measurement
 */
enum : uint8_t {
	Y_gyro_angvel_N = 0,
	Y_gyro_angvel_E,
	Y_gyro_angvel_D,
	Y_gyro_n,
};

/**
 * Airspeed measurement
 */
enum : uint8_t {
	Y_airspeed_airspeed = 0,
	Y_airspeed_n,
};

/**
 * Optical flow measurement
 */
enum : uint8_t {
	Y_flow_flowX = 0,
	Y_flow_flowY,
	Y_flow_n,
};

/**
 * Distance down measurement
 */
enum : uint8_t {
	Y_distance_down_d = 0,
	Y_distance_down_n,
};

/**
 * Innovation enum
 */
enum : uint8_t {
	// flow
	Innov_FLOW_flow_X = 0,
	Innov_FLOW_flow_Y,
	// gyro
	Innov_GYRO_angvel_N,
	Innov_GYRO_angvel_E,
	Innov_GYRO_angvel_D,
	// gps
	Innov_GPS_vel_N,
	Innov_GPS_vel_E,
	Innov_GPS_vel_D,
	Innov_GPS_pos_N,
	Innov_GPS_pos_E,
	Innov_GPS_asl,
	// baro
	Innov_BARO_asl,
	// accel
	Innov_ACCEL_acc_N,
	Innov_ACCEL_acc_E,
	Innov_ACCEL_acc_D,
	// mag
	Innov_MAG_mag_hdg,
	// lidar
	Innov_LIDAR_dist_bottom,
	// sonar
	Innov_SONAR_dist_bottom,
	// land
	Innov_LAND_vel_N,
	Innov_LAND_vel_E,
	Innov_LAND_vel_D,
	Innov_LAND_agl,
	// airspeed
	Innov_PITOT_airspeed,
	// vision
	//Innov_VISION_vel_N,
	//Innov_VISION_vel_E,
	//Innov_VISION_vel_D,
	//Innov_VISION_pos_N,
	//Innov_VISION_pos_E,
	//Innov_VISION_pos_D,
	//Innov_VISION_rot_N,
	//Innov_VISION_rot_E,
	//Innov_VISION_rot_D,
	// mocap
	//Innov_MOCAP_vel_N,
	//Innov_MOCAP_vel_E,
	//Innov_MOCAP_vel_D,
	//Innov_MOCAP_pos_N,
	//Innov_MOCAP_pos_E,
	//Innov_MOCAP_pos_D,
	//Innov_MOCAP_rot_N,
	//Innov_MOCAP_rot_E,
	//Innov_MOCAP_rot_D,
	Innov_n,
};

// innovation sensors
const uint8_t InnovSensors[Innov_n] = {
	// flow
	MAV_SENSOR_TYPE_FLOW,
	MAV_SENSOR_TYPE_FLOW,
	// gyro
	MAV_SENSOR_TYPE_GYRO,
	MAV_SENSOR_TYPE_GYRO,
	MAV_SENSOR_TYPE_GYRO,
	// gps
	MAV_SENSOR_TYPE_GPS,
	MAV_SENSOR_TYPE_GPS,
	MAV_SENSOR_TYPE_GPS,
	MAV_SENSOR_TYPE_GPS,
	MAV_SENSOR_TYPE_GPS,
	MAV_SENSOR_TYPE_GPS,
	// baro
	MAV_SENSOR_TYPE_BARO,
	// accel
	MAV_SENSOR_TYPE_ACCEL,
	MAV_SENSOR_TYPE_ACCEL,
	MAV_SENSOR_TYPE_ACCEL,
	// mag
	MAV_SENSOR_TYPE_MAG,
	// lidar
	MAV_SENSOR_TYPE_LIDAR,
	// sonar
	MAV_SENSOR_TYPE_SONAR,
	// land
	MAV_SENSOR_TYPE_LAND,
	MAV_SENSOR_TYPE_LAND,
	MAV_SENSOR_TYPE_LAND,
	MAV_SENSOR_TYPE_LAND,
	// airspeed
	MAV_SENSOR_TYPE_PITOT,
	// vision
	//MAV_SENSOR_TYPE_VISION,
	//MAV_SENSOR_TYPE_VISION,
	//MAV_SENSOR_TYPE_VISION,
	//MAV_SENSOR_TYPE_VISION,
	//MAV_SENSOR_TYPE_VISION,
	//MAV_SENSOR_TYPE_VISION,
	//MAV_SENSOR_TYPE_VISION,
	//MAV_SENSOR_TYPE_VISION,
	//MAV_SENSOR_TYPE_VISION,
	// mocap
	//MAV_SENSOR_TYPE_MOCAP,
	//MAV_SENSOR_TYPE_MOCAP,
	//MAV_SENSOR_TYPE_MOCAP,
	//MAV_SENSOR_TYPE_MOCAP,
	//MAV_SENSOR_TYPE_MOCAP,
	//MAV_SENSOR_TYPE_MOCAP,
	//MAV_SENSOR_TYPE_MOCAP,
	//MAV_SENSOR_TYPE_MOCAP,
	//MAV_SENSOR_TYPE_MOCAP,
};

// innovation fields
const uint8_t InnovIDs[Innov_n] = {
	// flow
	MAV_FIELD_FLOW_X,
	MAV_FIELD_FLOW_Y,
	// gyro // XXX actually in nav frame, but  need to add to mavlink
	MAV_FIELD_ANGVEL_X,
	MAV_FIELD_ANGVEL_Y,
	MAV_FIELD_ANGVEL_Z,
	// gps
	MAV_FIELD_VEL_N,
	MAV_FIELD_VEL_E,
	MAV_FIELD_VEL_D,
	MAV_FIELD_POS_N,
	MAV_FIELD_POS_E,
	MAV_FIELD_ASL,
	// baro
	MAV_FIELD_ASL,
	// accel
	MAV_FIELD_ACC_N,
	MAV_FIELD_ACC_E,
	MAV_FIELD_ACC_D,
	// mag
	MAV_FIELD_MAG_HDG,
	// lidar
	MAV_FIELD_DIST_BOTTOM,
	// sonar
	MAV_FIELD_DIST_BOTTOM,
	// land
	MAV_FIELD_VEL_N,
	MAV_FIELD_VEL_E,
	MAV_FIELD_VEL_D,
	MAV_FIELD_AGL,
	// airspeed
	MAV_FIELD_AIRSPEED,
	// vision
	//MAV_FIELD_VEL_N,
	//MAV_FIELD_VEL_E,
	//MAV_FIELD_VEL_D,
	//MAV_FIELD_POS_N,
	//MAV_FIELD_POS_E,
	//MAV_FIELD_POS_D,
	//MAV_FIELD_ROLL,
	//MAV_FIELD_PITCH,
	//MAV_FIELD_YAW,
	// mocap
	//MAV_FIELD_VEL_N,
	//MAV_FIELD_VEL_E,
	//MAV_FIELD_VEL_D,
	//MAV_FIELD_POS_N,
	//MAV_FIELD_POS_E,
	//MAV_FIELD_POS_D,
	//MAV_FIELD_ROLL,
	//MAV_FIELD_PITCH,
	//MAV_FIELD_YAW,
};

static const float BETA_TABLE[] = {
	0,
	8.82050518214,
	12.094592431,
	13.9876612368,
	16.0875642296,
	17.8797700658,
	19.6465647819,
	21.3802576894,
	23.0806434845,
	24.6673803845,
	26.1487953661,
	27.6350821245,
	29.6565383703,
	31.2211113844,
	32.7673547211,
	34.2967756977,
	35.6906782236,
	37.0724753352,
	38.4549693067,
	39.836592699,
};

// TODO these should all be rosparams eventually

//const float terrain_sigma_asl = 0; // (m/s) / sqrt(s)
const float g = 9.81f;
// don't predict if rotation speed > than this
const float gyro_saturation_thresh = 720 * deg2radf; // rad/s
// don't predict if accel norm > than this
const float accel_saturation_thresh = 3 * g; // m/s^2

//const float wind_correlation_time = 1000; // s
