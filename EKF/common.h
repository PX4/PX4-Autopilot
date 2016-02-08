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

/**
 * @file estimator_base.h
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
	Quaternion  quat_nominal;
	Vector3f    vel;
	Vector3f    pos;
	uint64_t 	time_us;
};

struct imuSample {
	Vector3f    delta_ang;
	Vector3f    delta_vel;
	float       delta_ang_dt;
	float       delta_vel_dt;
	uint64_t    time_us;
};

struct gpsSample {
	Vector2f    pos;
	float       hgt;
	Vector3f    vel;
	uint64_t    time_us;
};

struct magSample {
	Vector3f    mag;
	uint64_t    time_us;
};

struct baroSample {
	float       hgt;
	uint64_t    time_us;
};

struct rangeSample {
	float       rng;
	uint64_t    time_us;
};

struct airspeedSample {
	float       airspeed;
	uint64_t    time_us;
};

struct flowSample {
	Vector2f    flowRadXY;
	Vector2f    flowRadXYcomp;
	uint64_t    time_us;
};

struct parameters {
	float mag_delay_ms = 0.0f;          // magnetometer measurement delay relative to the IMU
	float baro_delay_ms = 0.0f;         // barometer height measurement delay relative to the IMU
	float gps_delay_ms = 200.0f;        // GPS measurement delay relative to the IMU
	float airspeed_delay_ms = 200.0f;   // airspeed measurement delay relative to the IMU

	// input noise
	float gyro_noise = 1.0e-3f;         // IMU angular rate noise used for covariance prediction
	float accel_noise = 2.5e-1f;        // IMU acceleration noise use for covariance prediction

	// process noise
	float gyro_bias_p_noise = 7.0e-5f;  // process noise for IMU delta angle bias prediction
	float accel_bias_p_noise = 1.0e-4f; // process noise for IMU delta velocity bias prediction
	float gyro_scale_p_noise = 3.0e-3f; // process noise for gyro scale factor prediction
	float mag_p_noise = 2.5e-2f;        // process noise for magnetic field prediction
	float wind_vel_p_noise = 1.0e-1f;   // process noise for wind velocity prediction

	float gps_vel_noise = 5.0e-1f;      // observation noise for gps velocity fusion
	float gps_pos_noise = 1.0f;         // observation noise for gps position fusion
	float pos_noaid_noise = 10.0f;      // observation noise for non-aiding position fusion
	float baro_noise = 3.0f;            // observation noise for barometric height fusion
	float baro_innov_gate = 3.0f;       // barometric height innovation consistency gate size in standard deviations
	float posNE_innov_gate = 3.0f;      // GPS horizontal position innovation consistency gate size in standard deviations
	float vel_innov_gate = 3.0f;        // GPS velocity innovation consistency gate size in standard deviations

	float mag_heading_noise = 1.7e-1f;  // measurement noise used for simple heading fusion
	float mag_noise = 5.0e-2f;          // measurement noise used for 3-axis magnetoemeter fusion
	float mag_declination_deg = 0.0f;   // magnetic declination in degrees
	float heading_innov_gate = 3.0f;    // heading fusion innovation consistency gate size in standard deviations
	float mag_innov_gate = 3.0f;        // magnetometer fusion innovation consistency gate size in standard deviations

	// these parameters control the strictness of GPS quality checks used to determine uf the GPS is
	// good enough to set a local origin and commence aiding
	int gps_check_mask = 21;    // bitmask used to control which GPS quality checks are used
	float req_hacc = 5.0f;      // maximum acceptable horizontal position error
	float req_vacc = 8.0f;      // maximum acceptable vertical position error
	float req_sacc = 1.0f;      // maximum acceptable speed error
	int req_nsats = 6;          // minimum acceptable satellite count
	float req_gdop = 2.0f;      // maximum acceptable geometric dilution of precision
	float req_hdrift = 0.3f;    // maximum acceptable horizontal drift speed
	float req_vdrift = 0.5f;    // maximum acceptable vertical drift speed
};

struct stateSample {
	Vector3f    ang_error;
	Vector3f    vel;
	Vector3f    pos;
	Vector3f    gyro_bias;
	Vector3f    gyro_scale;
	float       accel_z_bias;
	Vector3f    mag_I;
	Vector3f    mag_B;
	Vector2f    wind_vel;
	Quaternion  quat_nominal;
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
		uint8_t angle_align : 1; // 0 - true if the filter angular alignment is complete
		uint8_t gps         : 1; // 1 - true if GPS measurements are being fused
		uint8_t opt_flow    : 1; // 2 - true if optical flow measurements are being fused
		uint8_t mag_hdg     : 1; // 3 - true if a simple magnetic heading is being fused
		uint8_t mag_3D      : 1; // 4 - true if 3-axis magnetometer measurement are being fused
		uint8_t mag_dec     : 1; // 5 - true if synthetic magnetic declination measurements are being fused
		uint8_t in_air      : 1; // 6 - true when the vehicle is airborne
		uint8_t armed       : 1; // 7 - true when the vehicle motors are armed
	} flags;
	uint16_t value;
};
}
