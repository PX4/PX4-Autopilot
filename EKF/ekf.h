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
 * @file ekf.h
 * Class for core functions for ekf attitude and position estimator.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "estimator_interface.h"

class Ekf : public EstimatorInterface
{
public:

	Ekf();
	~Ekf();

	// initialise variables to sane values (also interface class)
	bool init(uint64_t timestamp);

	// should be called every time new data is pushed into the filter
	bool update();

	// gets the innovations of velocity and position measurements
	// 0-2 vel, 3-5 pos
	void get_vel_pos_innov(float vel_pos_innov[6]);

	// gets the innovations of the earth magnetic field measurements
	void get_mag_innov(float mag_innov[3]);

	// gets the innovations of the heading measurement
	void get_heading_innov(float *heading_innov);

	// gets the innovation variances of velocity and position measurements
	// 0-2 vel, 3-5 pos
	void get_vel_pos_innov_var(float vel_pos_innov_var[6]);

	// gets the innovation variances of the earth magnetic field measurements
	void get_mag_innov_var(float mag_innov_var[3]);

	// gets the innovation variance of the heading measurement
	void get_heading_innov_var(float *heading_innov_var);

	// get the state vector at the delayed time horizon
	void get_state_delayed(float *state);

	// get the diagonal elements of the covariance matrix
	void get_covariances(float *covariances);

	// ask estimator for sensor data collection decision and do any preprocessing if required, returns true if not defined
	bool collect_gps(uint64_t time_usec, struct gps_message *gps);
	bool collect_imu(imuSample &imu);

	// this is the current status of the filter control modes
	filter_control_status_u _control_status = {};

	// this is the previous status of the filter control modes - used to detect mode transitions
	filter_control_status_u _control_status_prev = {};

	// get the ekf WGS-84 origin position and height and the system time it was last set
	void get_ekf_origin(uint64_t *origin_time, map_projection_reference_s *origin_pos, float *origin_alt);

private:

	static const uint8_t _k_num_states = 24;
	static constexpr float _k_earth_rate = 0.000072921f;

	stateSample _state;		// state struct of the ekf running at the delayed time horizon

	bool _filter_initialised;
	bool _earth_rate_initialised;

	bool _fuse_height;	// baro height data should be fused
	bool _fuse_pos;		// gps position data should be fused
	bool _fuse_hor_vel;		// gps horizontal velocity measurement should be fused
	bool _fuse_vert_vel;	// gps vertical velocity measurement should be fused

	uint64_t _time_last_fake_gps;	// last time in us at which we have faked gps measurement for static mode

	uint64_t _time_last_pos_fuse;   // time the last fusion of horizotal position measurements was performed (usec)
	uint64_t _time_last_vel_fuse;   // time the last fusion of velocity measurements was performed (usec)
	uint64_t _time_last_hgt_fuse;   // time the last fusion of height measurements was performed (usec)
	uint64_t _time_last_of_fuse;    // time the last fusion of optical flow measurements were performed (usec)
	Vector2f _last_known_posNE;     // last known local NE position vector (m)
	float _last_disarmed_posD;      // vertical position recorded at arming (m)

	Vector3f _earth_rate_NED;	// earth rotation vector (NED) in rad/s

	matrix::Dcm<float> _R_prev;	// transformation matrix from earth frame to body frame of previous ekf step

	float P[_k_num_states][_k_num_states];	// state covariance matrix

	float _vel_pos_innov[6];	// innovations: 0-2 vel,  3-5 pos
	float _mag_innov[3];		// earth magnetic field innovations
	float _heading_innov;		// heading measurement innovation

	float _vel_pos_innov_var[6]; // innovation variances: 0-2 vel, 3-5 pos
	float _mag_innov_var[3]; // earth magnetic field innovation variance
	float _heading_innov_var; // heading measurement innovation variance

	float _mag_declination = 0.0f; // magnetic declination used by reset and fusion functions (rad)

	// complementary filter states
	Vector3f _delta_angle_corr;	// delta angle correction vector
	Vector3f _delta_vel_corr;	// delta velocity correction vector
	Vector3f _vel_corr;			// velocity correction vector
	imuSample _imu_down_sampled;	// down sampled imu data (sensor rate -> filter update rate)
	Quaternion _q_down_sampled;		// down sampled quaternion (tracking delta angles between ekf update steps)

	// variables used for the GPS quality checks
	float _gpsDriftVelN = 0.0f;     // GPS north position derivative (m/s)
	float _gpsDriftVelE = 0.0f;     // GPS east position derivative (m/s)
	float _gps_drift_velD = 0.0f;     // GPS down position derivative (m/s)
	float _gps_velD_diff_filt = 0.0f;   // GPS filtered Down velocity (m/s)
	float _gps_velN_filt = 0.0f;  // GPS filtered North velocity (m/s)
	float _gps_velE_filt = 0.0f;   // GPS filtered East velocity (m/s)
	uint64_t _last_gps_fail_us = 0;   // last system time in usec that the GPS failed it's checks

	// Variables used to publish the WGS-84 location of the EKF local NED origin
	uint64_t _last_gps_origin_time_us = 0;              // time the origin was last set (uSec)
	float _gps_alt_ref = 0.0f;                          // WGS-84 height (m)

	// Variables used to initialise the filter states
	uint8_t _baro_counter = 0;      // number of baro samples averaged
	float _baro_sum = 0.0f;         // summed baro measurement
	uint8_t _mag_counter = 0;       // number of magnetometer samples averaged
	Vector3f _mag_sum = {};         // summed magnetometer measurement
	Vector3f _delVel_sum = {};      // summed delta velocity
	float _baro_at_alignment;       // baro offset relative to alignment position

	gps_check_fail_status_u _gps_check_fail_status;

	// update the real time complementary filter states. This includes the prediction
	// and the correction step
	void calculateOutputStates();

	// initialise filter states of both the delayed ekf and the real time complementary filter
	bool initialiseFilter(void);

	// initialise ekf covariance matrix
	void initialiseCovariance();

	// predict ekf state
	void predictState();

	// predict ekf covariance
	void predictCovariance();

	// ekf sequential fusion of magnetometer measurements
	void fuseMag();

	// fuse magnetometer heading measurement
	void fuseHeading();

	// fuse magnetometer declination measurement
	void fuseDeclination();

	// fuse airspeed measurement
	void fuseAirspeed();

	// fuse range measurements
	void fuseRange();

	// fuse velocity and position measurements (also barometer height)
	void fuseVelPosHeight();

	// reset velocity states of the ekf
	void resetVelocity();

	// reset the heading and magnetic field states using the declination and magnetometer measurements
	// return true if successful
	bool resetMagHeading(Vector3f &mag_init);

	// calculate the magnetic declination to be used by the alignment and fusion processing
	void calcMagDeclination();

	// reset position states of the ekf (only vertical position)
	void resetPosition();

	// reset height state of the ekf
	void resetHeight();

	void makeCovSymetrical();

	// limit the diagonal of the covariance matrix
	void limitCov();

	// make ekf covariance matrix symmetric
	void makeSymmetrical();

	// constrain the ekf states
	void constrainStates();

	// generic function which will perform a fusion step given a kalman gain K
	// and a scalar innovation value
	void fuse(float *K, float innovation);

	// calculate the earth rotation vector from a given latitude
	void calcEarthRateNED(Vector3f &omega, double lat_rad) const;

	// return true id the GPS quality is good enough to set an origin and start aiding
	bool gps_is_good(struct gps_message *gps);

	// Control the filter fusion modes
	void controlFusionModes();

	// Determine if we are airborne or motors are armed
	void calculateVehicleStatus();

	// return the square of two foating point numbers - used in autocoded sections
	inline float sq(float var)
	{
		return var * var;
	}

	// zero the specified range of rows in the state covariance matricx
	void zeroRows(float (&cov_mat)[_k_num_states][_k_num_states], uint8_t first, uint8_t last);

	// zero the specified range of columns in the state covariance matricx
	void zeroCols(float (&cov_mat)[_k_num_states][_k_num_states], uint8_t first, uint8_t last);
};
