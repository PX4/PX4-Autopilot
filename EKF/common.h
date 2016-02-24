/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

#include "Matrix.hpp"

/**
 * @file common.h
 * Definition of base class for attitude estimators
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 *
 */

namespace estimator
{
struct gps_message {
	uint64_t time_usec;
	int32_t lat;                // Latitude in 1E-7 degrees
	int32_t lon;                // Longitude in 1E-7 degrees
	int32_t alt;                // Altitude in 1E-3 meters (millimeters) above MSL
	uint8_t fix_type;           // 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time
	float eph;                  // GPS horizontal position accuracy in m
	float epv;                  // GPS vertical position accuracy in m
	float sacc;                 // GPS speed accuracy in m/s
	uint64_t time_usec_vel;     // Timestamp for velocity informations
	float vel_m_s;              // GPS ground speed (m/s)
	float vel_ned[3];           // GPS ground speed NED
	bool vel_ned_valid;         // GPS ground speed is valid
	uint8_t nsats;              // number of satellites used
	float gdop;                 // geometric dilution of precision
};

typedef matrix::Vector<float, 2> Vector2f;
typedef matrix::Vector<float, 3> Vector3f;
typedef matrix::Quaternion<float> Quaternion;
typedef matrix::Matrix<float, 3, 3> Matrix3f;

struct outputSample {
	Quaternion  quat_nominal;	// nominal quaternion describing vehicle attitude
	Vector3f    vel;	// NED velocity estimate in earth frame in m/s
	Vector3f    pos;	// NED position estimate in earth frame in m/s
	uint64_t 	time_us;	// timestamp in microseconds
};

struct imuSample {
	Vector3f    delta_ang;	// delta angle in body frame (integrated gyro measurements)
	Vector3f    delta_vel;	// delta velocity in body frame (integrated accelerometer measurements)
	float       delta_ang_dt;	// delta angle integration period in seconds
	float       delta_vel_dt;	// delta velocity integration period in seconds
	uint64_t    time_us;	// timestamp in microseconds
};

struct gpsSample {
	Vector2f    pos;	// NE earth frame gps horizontal position measurement in m
	float       hgt;	// gps height measurement in m
	Vector3f    vel;	// NED earth frame gps velocity measurement in m/s
	uint64_t    time_us;	// timestamp in microseconds
};

struct magSample {
	Vector3f    mag;	// NED magnetometer body frame measurements
	uint64_t    time_us;	// timestamp in microseconds
};

struct baroSample {
	float       hgt;	// barometer height above sea level measurement in m
	uint64_t    time_us;	// timestamp in microseconds
};

struct rangeSample {
	float       rng;	// range (distance to ground) measurement in m
	uint64_t    time_us;	// timestamp in microseconds
};

struct airspeedSample {
	float       airspeed;	// airspeed measurement in m/s
	uint64_t    time_us;	// timestamp in microseconds
};

struct flowSample {
	Vector2f    flowRadXY;
	Vector2f    flowRadXYcomp;
	uint64_t    time_us;
};

struct parameters {
	float mag_delay_ms;         // magnetometer measurement delay relative to the IMU
	float baro_delay_ms;        // barometer height measurement delay relative to the IMU
	float gps_delay_ms;         // GPS measurement delay relative to the IMU
	float airspeed_delay_ms;    // airspeed measurement delay relative to the IMU

	// input noise
	float gyro_noise;           // IMU angular rate noise used for covariance prediction
	float accel_noise;          // IMU acceleration noise use for covariance prediction

	// process noise
	float gyro_bias_p_noise;    // process noise for IMU delta angle bias prediction
	float accel_bias_p_noise;   // process noise for IMU delta velocity bias prediction
	float gyro_scale_p_noise;   // process noise for gyro scale factor prediction
	float mag_p_noise;          // process noise for magnetic field prediction
	float wind_vel_p_noise;     // process noise for wind velocity prediction

	float gps_vel_noise;        // observation noise for gps velocity fusion
	float gps_pos_noise;        // observation noise for gps position fusion
	float pos_noaid_noise;      // observation noise for non-aiding position fusion
	float baro_noise;           // observation noise for barometric height fusion
	float baro_innov_gate;      // barometric height innovation consistency gate size in standard deviations
	float posNE_innov_gate;     // GPS horizontal position innovation consistency gate size in standard deviations
	float vel_innov_gate;       // GPS velocity innovation consistency gate size in standard deviations

	float mag_heading_noise;    // measurement noise used for simple heading fusion
	float mag_noise;            // measurement noise used for 3-axis magnetoemeter fusion
	float mag_declination_deg;  // magnetic declination in degrees
	float heading_innov_gate;   // heading fusion innovation consistency gate size in standard deviations
	float mag_innov_gate;       // magnetometer fusion innovation consistency gate size in standard deviations
	int mag_declination_source; // bitmask used to control the handling of declination data
	int mag_fusion_type;        // integer used to specify the type of magnetometer fusion used

	// these parameters control the strictness of GPS quality checks used to determine uf the GPS is
	// good enough to set a local origin and commence aiding
	int gps_check_mask;     // bitmask used to control which GPS quality checks are used
	float req_hacc;         // maximum acceptable horizontal position error
	float req_vacc;         // maximum acceptable vertical position error
	float req_sacc;         // maximum acceptable speed error
	int req_nsats;          // minimum acceptable satellite count
	float req_gdop;         // maximum acceptable geometric dilution of precision
	float req_hdrift;       // maximum acceptable horizontal drift speed
	float req_vdrift;       // maximum acceptable vertical drift speed
	
    // Initialize parameter values.  Initialization must be accomplished in the constructor to allow C99 compiler compatibility.
	parameters()
	{
		mag_delay_ms = 0.0f;
		baro_delay_ms = 0.0f;
		gps_delay_ms = 200.0f;
		airspeed_delay_ms = 200.0f;

		// input noise
		gyro_noise = 1.0e-3f;
		accel_noise = 2.5e-1f;

		// process noise
		gyro_bias_p_noise = 7.0e-5f;
		accel_bias_p_noise = 1.0e-4f;
		gyro_scale_p_noise = 3.0e-3f;
		mag_p_noise = 2.5e-2f;
		wind_vel_p_noise = 1.0e-1f;

		gps_vel_noise = 5.0e-1f;
		gps_pos_noise = 1.0f;
		pos_noaid_noise = 10.0f;
		baro_noise = 3.0f;
		baro_innov_gate = 3.0f;
		posNE_innov_gate = 3.0f;
		vel_innov_gate = 3.0f;

		mag_heading_noise = 1.7e-1f;
		mag_noise = 5.0e-2f;
		mag_declination_deg = 0.0f;
		heading_innov_gate = 3.0f;
		mag_innov_gate = 3.0f;

		mag_declination_source = 7;
		mag_fusion_type = 0;

		gps_check_mask = 21;
		req_hacc = 5.0f;
		req_vacc = 8.0f;
		req_sacc = 1.0f;
		req_nsats = 6;
		req_gdop = 2.0f;
		req_hdrift = 0.3f;
		req_vdrift = 0.5f;
	}
};

// Bit locations for mag_declination_source
#define MASK_USE_GEO_DECL   (1<<0)  // set to true to use the declination from the geo library when the GPS position becomes available, set to false to always use the EKF2_MAG_DECL value
#define MASK_SAVE_GEO_DECL  (1<<1)  // set to true to set the EKF2_MAG_DECL parameter to the value returned by the geo library
#define MASK_FUSE_DECL      (1<<2)  // set to true if the declination is always fused as an observation to constrain drift when 3-axis fusion is performed

// Integer definitions for mag_fusion_type
#define MAG_FUSE_TYPE_AUTO      0   // The selection of either heading or 3D magnetometer fusion will be automatic
#define MAG_FUSE_TYPE_HEADING   1   // Simple yaw angle fusion will always be used. This is less accurate, but less affected by earth field distortions. It should not be used for pitch angles outside the range from -60 to +60 deg
#define MAG_FUSE_TYPE_3D        2   // Magnetometer 3-axis fusion will always be used. This is more accurate, but more affected by localised earth field distortions
#define MAG_FUSE_TYPE_2D        3   // A 2D fusion that uses the horizontal projection of the magnetic fields measurement will alays be used. This is less accurate, but less affected by earth field distortions.

struct stateSample {
	Vector3f    ang_error;	// attitude axis angle error (error state formulation)
	Vector3f    vel;	// NED velocity in earth frame in m/s
	Vector3f    pos;	// NED position in earth frame in m
	Vector3f    gyro_bias;	// gyro bias estimate in rad/s
	Vector3f    gyro_scale;	// gyro scale estimate
	float       accel_z_bias;	// accelerometer z axis bias estimate
	Vector3f    mag_I;	// NED earth magnetic field in gauss
	Vector3f    mag_B;	// magnetometer bias estimate in body frame in gauss
	Vector2f    wind_vel;	// wind velocity in m/s
	Quaternion  quat_nominal;	// nominal quaternion describing vehicle attitude
};

struct fault_status_t {
	bool bad_mag_x: 1; // true if the fusion of the magnetometer X-axis has encountered a numerical error
	bool bad_mag_y: 1; // true if the fusion of the magnetometer Y-axis has encountered a numerical error
	bool bad_mag_z: 1; // true if the fusion of the magnetometer Z-axis has encountered a numerical error
	bool bad_mag_hdg: 1; // true if the fusion of the magnetic heading has encountered a numerical error
	bool bad_mag_decl: 1; // true if the fusion of the magnetic declination has encountered a numerical error
	bool bad_airspeed: 1; // true if fusion of the airspeed has encountered a numerical error
	bool bad_sideslip: 1; // true if fusion of the synthetic sideslip constraint has encountered a numerical error
	bool bad_optflow_X: 1; // true if fusion of the optical flow X axis has encountered a numerical error
	bool bad_optflow_Y: 1; // true if fusion of the optical flow Y axis has encountered a numerical error
};

// publish the status of various GPS quality checks
union gps_check_fail_status_u {
	struct {
		uint16_t fix    : 1; // 0 - true if the fix type is insufficient (no 3D solution)
		uint16_t nsats  : 1; // 1 - true if number of satellites used is insufficient
		uint16_t gdop   : 1; // 2 - true if geometric dilution of precision is insufficient
		uint16_t hacc   : 1; // 3 - true if reported horizontal accuracy is insufficient
		uint16_t vacc   : 1; // 4 - true if reported vertical accuracy is insufficient
		uint16_t sacc   : 1; // 5 - true if reported speed accuracy is insufficient
		uint16_t hdrift : 1; // 6 - true if horizontal drift is excessive (can only be used when stationary on ground)
		uint16_t vdrift : 1; // 7 - true if vertical drift is excessive (can only be used when stationary on ground)
		uint16_t hspeed : 1; // 8 - true if horizontal speed is excessive (can only be used when stationary on ground)
		uint16_t vspeed : 1; // 9 - true if vertical speed error is excessive
	} flags;
	uint16_t value;
};

// bitmask containing filter control status
union filter_control_status_u {
	struct {
		uint8_t tilt_align  : 1; // 0 - true if the filter tilt alignment is complete
		uint8_t yaw_align   : 1; // 1 - true if the filter yaw alignment is complete
		uint8_t gps         : 1; // 2 - true if GPS measurements are being fused
		uint8_t opt_flow    : 1; // 3 - true if optical flow measurements are being fused
		uint8_t mag_hdg     : 1; // 4 - true if a simple magnetic yaw heading is being fused
		uint8_t mag_2D      : 1; // 5 - true if the horizontal projection of magnetometer data is being fused
		uint8_t mag_3D      : 1; // 6 - true if 3-axis magnetometer measurement are being fused
		uint8_t mag_dec     : 1; // 7 - true if synthetic magnetic declination measurements are being fused
		uint8_t in_air      : 1; // 8 - true when the vehicle is airborne
		uint8_t armed       : 1; // 9 - true when the vehicle motors are armed
		uint8_t wind        : 1; // 10 - true when wind velocity is being estimated
	} flags;
	uint16_t value;
};
}
