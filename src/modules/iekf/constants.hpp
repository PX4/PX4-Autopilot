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
struct X {
	static const uint8_t q_nb_0 = 0;
	static const uint8_t q_nb_1 = 1;
	static const uint8_t q_nb_2 = 2;
	static const uint8_t q_nb_3 = 3;
	static const uint8_t vel_N = 4;
	static const uint8_t vel_E = 5;
	static const uint8_t vel_D = 6;
	static const uint8_t gyro_bias_bX = 7;
	static const uint8_t gyro_bias_bY = 8;
	static const uint8_t gyro_bias_bZ = 9;
	static const uint8_t accel_bias_bX = 10;
	static const uint8_t accel_bias_bY = 11;
	static const uint8_t accel_bias_bZ = 12;
	static const uint8_t pos_N = 13;
	static const uint8_t pos_E = 14;
	static const uint8_t asl = 15;
	static const uint8_t terrain_asl = 16;
	static const uint8_t baro_bias = 17;
	//static const uint8_t wind_N = 18;
	//static const uint8_t wind_E = 19;
	//static const uint8_t wind_D = 20;
	static const uint8_t n = 18;
};

// mavlink sensor type for states
const uint8_t XSensors[X::n] = {
	// q
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	// vel
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
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
const uint8_t  XIDs[X::n] = {
	// q
	MAV_FIELD_Q0,
	MAV_FIELD_Q1,
	MAV_FIELD_Q2,
	MAV_FIELD_Q3,
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
struct Xe {
	static const uint8_t rot_N = 0;
	static const uint8_t rot_E = 1;
	static const uint8_t rot_D = 2;
	static const uint8_t vel_N = 3;
	static const uint8_t vel_E = 4;
	static const uint8_t vel_D = 5;
	static const uint8_t gyro_bias_N = 6;
	static const uint8_t gyro_bias_E = 7;
	static const uint8_t gyro_bias_D = 8;
	static const uint8_t accel_bias_N = 9;
	static const uint8_t accel_bias_E = 10;
	static const uint8_t accel_bias_D = 11;
	static const uint8_t pos_N = 12;
	static const uint8_t pos_E = 13;
	static const uint8_t asl = 14;
	static const uint8_t terrain_asl = 15;
	static const uint8_t baro_bias = 16;
	//static const uint8_t wind_N = 17;
	//static const uint8_t wind_E = 18;
	//static const uint8_t wind_D = 19;
	static const uint8_t n = 17;
};

// mavlink sensor type for error states
const uint8_t XeSensors[Xe::n] = {
	// rot
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	// vel
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
	MAV_SENSOR_TYPE_NONE,
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
const uint8_t XeIDs[Xe::n] = {
	// rot
	MAV_FIELD_ROLL,
	MAV_FIELD_PITCH,
	MAV_FIELD_YAW,
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
struct U {
	static const uint8_t omega_nb_bX = 0;
	static const uint8_t omega_nb_bY = 1;
	static const uint8_t omega_nb_bZ = 2;
	static const uint8_t accel_bX = 3;
	static const uint8_t accel_bY = 4;
	static const uint8_t accel_bZ = 5;
	static const uint8_t n = 6;
};

/**
 * Accel measurement enum
 */
struct Y_accel {
	static const uint8_t accel_bX = 0;
	static const uint8_t accel_bY = 1;
	static const uint8_t accel_bZ = 2;
	static const uint8_t n = 3;
};

/**
 * Land enum
 */
struct Y_land {
	static const uint8_t vel_N = 0;
	static const uint8_t vel_E = 1;
	static const uint8_t vel_D = 2;
	static const uint8_t agl = 3;
	static const uint8_t n = 4;
};

/**
 * GPS measurement
 */
struct Y_gps {
	static const uint8_t pos_N = 0;
	static const uint8_t pos_E = 1;
	static const uint8_t asl = 2;
	static const uint8_t vel_N = 3;
	static const uint8_t vel_E = 4;
	static const uint8_t vel_D = 5;
	static const uint8_t n = 6;
};

/**
 * Baro measurement
 */
struct Y_baro {
	static const uint8_t asl = 0;
	static const uint8_t n = 1;
};

/**
 * Magnetometer measurement
 *
 * The filter treats the error
 * in the navigation frame
 * (north, east, down) even though the
 * field is measured in the body
 * frame.
 */
struct Y_mag {
	static const uint8_t hdg = 0;
	//static const uint8_t mag_E = 1;
	//static const uint8_t mag_D = 2;
	static const uint8_t n = 1;
};

/**
 * Airspeed measurement
 */
struct Y_airspeed {
	static const uint8_t airspeed = 0;
	static const uint8_t n = 1;
};

/**
 * Optical flow measurement
 */
struct Y_flow {
	static const uint8_t flowX = 0;
	static const uint8_t flowY = 1;
	static const uint8_t n = 2;
};

/**
 * Distance down measurement
 */
struct Y_distance_down {
	static const uint8_t d = 0;
	static const uint8_t n = 1;
};

/**
 * Innovation enum
 */
struct Innov {
	// flow
	static const uint8_t FLOW_flow_X = 0;
	static const uint8_t FLOW_flow_Y = 1;
	// gps
	static const uint8_t GPS_vel_N = 2;
	static const uint8_t GPS_vel_E = 3;
	static const uint8_t GPS_vel_D = 4;
	static const uint8_t GPS_pos_N = 5;
	static const uint8_t GPS_pos_E = 6;
	static const uint8_t GPS_asl = 7;
	// baro
	static const uint8_t BARO_asl = 8;
	// accel
	static const uint8_t ACCEL_acc_X = 9;
	static const uint8_t ACCEL_acc_Y = 10;
	static const uint8_t ACCEL_acc_Z = 11;
	// mag
	static const uint8_t MAG_mag_hdg = 12;
	// lidar
	static const uint8_t LIDAR_dist_bottom = 13;
	// sonar
	static const uint8_t SONAR_dist_bottom = 14;
	// land
	static const uint8_t LAND_vel_N = 15;
	static const uint8_t LAND_vel_E = 16;
	static const uint8_t LAND_vel_D = 17;
	static const uint8_t LAND_agl = 18;
	// airspeed
	static const uint8_t PITOT_airspeed = 19;
	// vision
	static const uint8_t VISION_vel_N = 20;
	static const uint8_t VISION_vel_E = 21;
	static const uint8_t VISION_vel_D = 22;
	static const uint8_t VISION_pos_N = 23;
	static const uint8_t VISION_pos_E = 24;
	static const uint8_t VISION_pos_D = 25;
	static const uint8_t VISION_rot_N = 26;
	static const uint8_t VISION_rot_E = 27;
	static const uint8_t VISION_rot_D = 28;
	// mocap
	static const uint8_t MOCAP_vel_N = 29;
	static const uint8_t MOCAP_vel_E = 30;
	static const uint8_t MOCAP_vel_D = 31;
	static const uint8_t MOCAP_pos_N = 32;
	static const uint8_t MOCAP_pos_E = 33;
	static const uint8_t MOCAP_pos_D = 34;
	static const uint8_t MOCAP_rot_N = 35;
	static const uint8_t MOCAP_rot_E = 36;
	static const uint8_t MOCAP_rot_D = 37;
	static const uint8_t n = 38;
};

// innovation sensors
const uint8_t InnovSensors[Innov::n] = {
	// flow
	MAV_SENSOR_TYPE_FLOW,
	MAV_SENSOR_TYPE_FLOW,
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
	MAV_SENSOR_TYPE_VISION,
	MAV_SENSOR_TYPE_VISION,
	MAV_SENSOR_TYPE_VISION,
	MAV_SENSOR_TYPE_VISION,
	MAV_SENSOR_TYPE_VISION,
	MAV_SENSOR_TYPE_VISION,
	MAV_SENSOR_TYPE_VISION,
	MAV_SENSOR_TYPE_VISION,
	MAV_SENSOR_TYPE_VISION,
	// mocap
	MAV_SENSOR_TYPE_MOCAP,
	MAV_SENSOR_TYPE_MOCAP,
	MAV_SENSOR_TYPE_MOCAP,
	MAV_SENSOR_TYPE_MOCAP,
	MAV_SENSOR_TYPE_MOCAP,
	MAV_SENSOR_TYPE_MOCAP,
	MAV_SENSOR_TYPE_MOCAP,
	MAV_SENSOR_TYPE_MOCAP,
	MAV_SENSOR_TYPE_MOCAP,
};

// innovation fields
const uint8_t InnovIDs[Innov::n] = {
	// flow
	MAV_FIELD_FLOW_X,
	MAV_FIELD_FLOW_Y,
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
	MAV_FIELD_ACC_X,
	MAV_FIELD_ACC_Y,
	MAV_FIELD_ACC_Z,
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
	MAV_FIELD_VEL_N,
	MAV_FIELD_VEL_E,
	MAV_FIELD_VEL_D,
	MAV_FIELD_POS_N,
	MAV_FIELD_POS_E,
	MAV_FIELD_POS_D,
	MAV_FIELD_ROLL,
	MAV_FIELD_PITCH,
	MAV_FIELD_YAW,
	// mocap
	MAV_FIELD_VEL_N,
	MAV_FIELD_VEL_E,
	MAV_FIELD_VEL_D,
	MAV_FIELD_POS_N,
	MAV_FIELD_POS_E,
	MAV_FIELD_POS_D,
	MAV_FIELD_ROLL,
	MAV_FIELD_PITCH,
	MAV_FIELD_YAW,
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
