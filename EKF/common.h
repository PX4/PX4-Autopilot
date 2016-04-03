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

struct flow_message {
	uint8_t quality;			// Quality of Flow data
	Vector2f flowdata;			// Flow data received
	Vector2f gyrodata;			// Gyro data from flow sensor
	uint32_t dt;				// integration time of flow samples
};

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
	float	    hacc;	// 1-std horizontal position error m
	float	    vacc;	// 1-std vertical position error m
	float       sacc;	// 1-std speed error m/s
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
	float       true_airspeed;	// true airspeed measurement in m/s
	uint64_t    time_us;	// timestamp in microseconds
};

struct flowSample {
	uint8_t  quality; // quality indicator between 0 and 255
	Vector2f flowRadXY; // measured delta angle of the image about the X and Y body axes (rad), RH rotaton is positive
	Vector2f flowRadXYcomp;	// measured delta angle of the image about the X and Y body axes after removal of body rotation (rad), RH rotation is positive
	Vector2f gyroXY; // measured delta angle of the inertial frame about the X and Y body axes obtained from rate gyro measurements (rad), RH rotation is positive
	float    dt; // amount of integration time (sec)
	uint64_t time_us; // timestamp in microseconds of the integration period mid-point
};

// Integer definitions for vdist_sensor_type
#define VDIST_SENSOR_BARO  0	// Use baro height
#define VDIST_SENSOR_GPS   1	// Use GPS height
#define VDIST_SENSOR_RANGE 2	// Use range finder height

// Bit locations for mag_declination_source
#define MASK_USE_GEO_DECL   (1<<0)  // set to true to use the declination from the geo library when the GPS position becomes available, set to false to always use the EKF2_MAG_DECL value
#define MASK_SAVE_GEO_DECL  (1<<1)  // set to true to set the EKF2_MAG_DECL parameter to the value returned by the geo library
#define MASK_FUSE_DECL      (1<<2)  // set to true if the declination is always fused as an observation to constrain drift when 3-axis fusion is performed

// Bit locations for fusion_mode
#define MASK_USE_GPS    (1<<0)  // set to true to use GPS data
#define MASK_USE_OF     (1<<1)  // set to true to use optical flow data

// Integer definitions for mag_fusion_type
#define MAG_FUSE_TYPE_AUTO      0   // The selection of either heading or 3D magnetometer fusion will be automatic
#define MAG_FUSE_TYPE_HEADING   1   // Simple yaw angle fusion will always be used. This is less accurate, but less affected by earth field distortions. It should not be used for pitch angles outside the range from -60 to +60 deg
#define MAG_FUSE_TYPE_3D        2   // Magnetometer 3-axis fusion will always be used. This is more accurate, but more affected by localised earth field distortions
#define MAG_FUSE_TYPE_2D        3   // A 2D fusion that uses the horizontal projection of the magnetic fields measurement will alays be used. This is less accurate, but less affected by earth field distortions.

// Maximum sensor intervals in usec
#define GPS_MAX_INTERVAL	5e5
#define BARO_MAX_INTERVAL	2e5
#define RNG_MAX_INTERVAL	2e5

struct parameters {
	// measurement source control
	int fusion_mode;		// bitmasked integer that selects which of the GPS and optical flow aiding sources will be used
	int vdist_sensor_type;		// selects the primary source for height data

	// measurement time delays
	float mag_delay_ms;		// magnetometer measurement delay relative to the IMU (msec)
	float baro_delay_ms;		// barometer height measurement delay relative to the IMU (msec)
	float gps_delay_ms;		// GPS measurement delay relative to the IMU (msec)
	float airspeed_delay_ms;	// airspeed measurement delay relative to the IMU (msec)
	float flow_delay_ms;		// optical flow measurement delay relative to the IMU (msec) - this is to the middle of the optical flow integration interval
	float range_delay_ms;		// range finder measurement delay relative to the IMU (msec)

	// input noise
	float gyro_noise;		// IMU angular rate noise used for covariance prediction (rad/sec)
	float accel_noise;		// IMU acceleration noise use for covariance prediction (m/sec/sec)

	// process noise
	float gyro_bias_p_noise;	// process noise for IMU delta angle bias prediction (rad/sec)
	float accel_bias_p_noise;	// process noise for IMU delta velocity bias prediction (m/sec/sec)
	float gyro_scale_p_noise;	// process noise for gyro scale factor prediction (N/A)
	float mage_p_noise;		// process noise for earth magnetic field prediction (Guass/sec)
	float magb_p_noise;		// process noise for body magnetic field prediction (Guass/sec)
	float wind_vel_p_noise;		// process noise for wind velocity prediction (m/sec/sec)
	float terrain_p_noise;		// process noise for terrain offset (m/sec)
	float terrain_gradient;		// gradient of terrain used to estimate process noise due to changing position (m/m)

	// position and velocity fusion
	float gps_vel_noise;		// observation noise for gps velocity fusion (m/sec)
	float gps_pos_noise;		// observation noise for gps position fusion (m)
	float pos_noaid_noise;		// observation noise for non-aiding position fusion (m)
	float baro_noise;		// observation noise for barometric height fusion (m)
	float baro_innov_gate;		// barometric height innovation consistency gate size (STD)
	float posNE_innov_gate;		// GPS horizontal position innovation consistency gate size (STD)
	float vel_innov_gate;		// GPS velocity innovation consistency gate size (STD)
	float hgt_reset_lim;		// The maximum 1-sigma uncertainty in height that can be tolerated before the height state is reset (m)

	// magnetometer fusion
	float mag_heading_noise;	// measurement noise used for simple heading fusion (rad)
	float mag_noise;		// measurement noise used for 3-axis magnetoemeter fusion (Gauss)
	float mag_declination_deg;	// magnetic declination (degrees)
	float heading_innov_gate;	// heading fusion innovation consistency gate size (STD)
	float mag_innov_gate;		// magnetometer fusion innovation consistency gate size (STD)
	int mag_declination_source;	// bitmask used to control the handling of declination data
	int mag_fusion_type;		// integer used to specify the type of magnetometer fusion used

	// airspeed fusion
	float tas_innov_gate;		// True Airspeed Innovation consistency gate size in standard deciation [WHAT SHALL THIS VALUE BE?]
  	float eas_noise;			// EAS measurement noise standard deviation used for airspeed fusion [m/s]

	// range finder fusion
	float range_noise;		// observation noise for range finder measurements (m)
	float range_innov_gate;		// range finder fusion innovation consistency gate size (STD)
	float rng_gnd_clearance;	// minimum valid value for range when on ground (m)

	// optical flow fusion
	float flow_noise;		// observation noise for optical flow LOS rate measurements (rad/sec)
	float flow_noise_qual_min;	// observation noise for optical flow LOS rate measurements when flow sensor quality is at the minimum useable (rad/sec)
	int flow_qual_min;		// minimum acceptable quality integer from  the flow sensor
	float flow_innov_gate;		// optical flow fusion innovation consistency gate size (STD)
	float flow_rate_max;		// maximum valid optical flow rate (rad/sec)

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
		// measurement source control
		fusion_mode = MASK_USE_GPS;
		vdist_sensor_type = VDIST_SENSOR_BARO;

		// measurement time delays
		mag_delay_ms = 0.0f;
		baro_delay_ms = 0.0f;
		gps_delay_ms = 200.0f;
		airspeed_delay_ms = 200.0f;
		flow_delay_ms = 60.0f;
		range_delay_ms = 200.0f;

		// input noise
		gyro_noise = 6.0e-2f;
		accel_noise = 2.5e-1f;

		// process noise
		gyro_bias_p_noise = 2.5e-6f;
		accel_bias_p_noise = 3.0e-5f;
		gyro_scale_p_noise = 3.0e-4f;
		mage_p_noise = 2.5e-3f;
		magb_p_noise = 5.0e-4f;
		wind_vel_p_noise = 1.0e-1f;
		terrain_p_noise = 5.0f;
		terrain_gradient = 0.5f;

		// position and velocity fusion
		gps_vel_noise = 5.0e-1f;
		gps_pos_noise = 0.5f;
		pos_noaid_noise = 10.0f;
		baro_noise = 2.0f;
		baro_innov_gate = 5.0f;
		posNE_innov_gate = 5.0f;
		vel_innov_gate = 5.0f;
		hgt_reset_lim = 5.0f;

		// magnetometer fusion
		mag_heading_noise = 3.0e-1f;
		mag_noise = 5.0e-2f;
		mag_declination_deg = 0.0f;
		heading_innov_gate = 2.6f;
		mag_innov_gate = 3.0f;
		mag_declination_source = 3;
		mag_fusion_type = 0;

		// airspeed fusion
		tas_innov_gate = 3.0f; // [CHECK THIS VALUE]		
  		eas_noise = 1.4f;			

		// range finder fusion
		range_noise = 0.1f;
		range_innov_gate = 5.0f;
		rng_gnd_clearance = 0.1f;

		// optical flow fusion
		flow_noise = 0.15f;
		flow_noise_qual_min = 0.5f;
		flow_qual_min = 1;
		flow_innov_gate = 3.0f;
		flow_rate_max = 2.5f;

		// GPS quality checks
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
		uint16_t tilt_align  : 1; // 0 - true if the filter tilt alignment is complete
		uint16_t yaw_align   : 1; // 1 - true if the filter yaw alignment is complete
		uint16_t gps         : 1; // 2 - true if GPS measurements are being fused
		uint16_t opt_flow    : 1; // 3 - true if optical flow measurements are being fused
		uint16_t mag_hdg     : 1; // 4 - true if a simple magnetic yaw heading is being fused
		uint16_t mag_2D      : 1; // 5 - true if the horizontal projection of magnetometer data is being fused
		uint16_t mag_3D      : 1; // 6 - true if 3-axis magnetometer measurement are being fused
		uint16_t mag_dec     : 1; // 7 - true if synthetic magnetic declination measurements are being fused
		uint16_t in_air      : 1; // 8 - true when the vehicle is airborne
		uint16_t armed       : 1; // 9 - true when the vehicle motors are armed
		uint16_t wind        : 1; // 10 - true when wind velocity is being estimated
		uint16_t baro_hgt    : 1; // 11 - true when baro height is being fused as a primary height reference
		uint16_t rng_hgt     : 1; // 12 - true when range finder height is being fused as a primary height reference
		uint16_t gps_hgt     : 1; // 15 - true when range finder height is being fused as a primary height reference
	} flags;
	uint16_t value;
};

}
