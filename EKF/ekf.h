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

#pragma once

#include "estimator_interface.h"

class Ekf : public EstimatorInterface
{
public:

	Ekf() = default;
	virtual ~Ekf() = default;

	// initialise variables to sane values (also interface class)
	bool init(uint64_t timestamp) override;

	// set the internal states and status to their default value
	void reset() override;

	bool initialiseTilt();

	// should be called every time new data is pushed into the filter
	bool update() override;

	void getGpsVelPosInnov(float hvel[2], float &vvel, float hpos[2], float &vpos) override;

	void getGpsVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos) override;

	void getGpsVelPosInnovRatio(float& hvel, float &vvel, float& hpos, float &vpos) override;

	void getEvVelPosInnov(float hvel[2], float& vvel, float hpos[2], float& vpos) override;

	void getEvVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos) override;

	void getEvVelPosInnovRatio(float& hvel, float &vvel, float& hpos, float &vpos) override;

	void getBaroHgtInnov(float &baro_hgt_innov) override;

	void getBaroHgtInnovVar(float &baro_hgt_innov_var) override;

	void getBaroHgtInnovRatio(float &baro_hgt_innov_ratio) override;

	void getRngHgtInnov(float &rng_hgt_innov) override;

	void getRngHgtInnovVar(float &rng_hgt_innov_var) override;

	void getRngHgtInnovRatio(float &rng_hgt_innov_ratio) override;

	void getAuxVelInnov(float aux_vel_innov[2]) override;

	void getAuxVelInnovVar(float aux_vel_innov[2]) override;

	void getAuxVelInnovRatio(float &aux_vel_innov_ratio) override;

	void getFlowInnov(float flow_innov[2]) override;

	void getFlowInnovVar(float flow_innov_var[2]) override;

	void getFlowInnovRatio(float &flow_innov_ratio) override;

	void getHeadingInnov(float &heading_innov) override;

	void getHeadingInnovVar(float &heading_innov_var) override;

	void getHeadingInnovRatio(float &heading_innov_ratio) override;

	void getMagInnov(float mag_innov[3]) override;

	void getMagInnovVar(float mag_innov_var[3]) override;

	void getMagInnovRatio(float &mag_innov_ratio) override;

	void getDragInnov(float drag_innov[2]) override;

	void getDragInnovVar(float drag_innov_var[2]) override;

	void getDragInnovRatio(float drag_innov_ratio[2]) override;

	void getAirspeedInnov(float &airspeed_innov) override;

	void getAirspeedInnovVar(float &airspeed_innov_var) override;

	void getAirspeedInnovRatio(float &airspeed_innov_ratio) override;

	void getBetaInnov(float &beta_innov) override;

	void getBetaInnovVar(float &beta_innov_var) override;

	void getBetaInnovRatio(float &beta_innov_ratio) override;

	void getHaglInnov(float &hagl_innov) override;

	void getHaglInnovVar(float &hagl_innov_var) override;

	void getHaglInnovRatio(float &hagl_innov_ratio) override;

	// get the state vector at the delayed time horizon
	void get_state_delayed(float *state) override;

	// get the wind velocity in m/s
	void get_wind_velocity(float *wind) override;

	// get the wind velocity var
	void get_wind_velocity_var(float *wind_var) override;

	// get the true airspeed in m/s
	void get_true_airspeed(float *tas) override;

	// get the full covariance matrix
	matrix::SquareMatrix<float, 24> covariances() const { return P; }

	// get the diagonal elements of the covariance matrix
	matrix::Vector<float, 24> covariances_diagonal() const { return P.diag(); }

	// get the orientation (quaterion) covariances
	matrix::SquareMatrix<float, 4> orientation_covariances() const { return P.slice<4, 4>(0, 0); }

	// get the linear velocity covariances
	matrix::SquareMatrix<float, 3> velocity_covariances() const { return P.slice<3, 3>(4, 4); }

	// get the position covariances
	matrix::SquareMatrix<float, 3> position_covariances() const { return P.slice<3, 3>(7, 7); }

	// ask estimator for sensor data collection decision and do any preprocessing if required, returns true if not defined
	bool collect_gps(const gps_message &gps) override;

	// get the ekf WGS-84 origin position and height and the system time it was last set
	// return true if the origin is valid
	bool get_ekf_origin(uint64_t *origin_time, map_projection_reference_s *origin_pos, float *origin_alt) override;

	// get the 1-sigma horizontal and vertical position uncertainty of the ekf WGS-84 position
	void get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv) override;

	// get the 1-sigma horizontal and vertical position uncertainty of the ekf local position
	void get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv) override;

	// get the 1-sigma horizontal and vertical velocity uncertainty
	void get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv) override;

	// get the vehicle control limits required by the estimator to keep within sensor limitations
	void get_ekf_ctrl_limits(float *vxy_max, float *vz_max, float *hagl_min, float *hagl_max) override;

	/*
	Reset all IMU bias states and covariances to initial alignment values.
	Use when the IMU sensor has changed.
	Returns true if reset performed, false if rejected due to less than 10 seconds lapsed since last reset.
	*/
	bool reset_imu_bias() override;

	void get_vel_var(Vector3f &vel_var) override;

	void get_pos_var(Vector3f &pos_var) override;

	// return an array containing the output predictor angular, velocity and position tracking
	// error magnitudes (rad), (m/sec), (m)
	void get_output_tracking_error(float error[3]) override;

	/*
	Returns  following IMU vibration metrics in the following array locations
	0 : Gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
	1 : Gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
	2 : Accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)
	*/
	void get_imu_vibe_metrics(float vibe[3]) override;

	/*
	First argument returns GPS drift  metrics in the following array locations
	0 : Horizontal position drift rate (m/s)
	1 : Vertical position drift rate (m/s)
	2 : Filtered horizontal velocity (m/s)
	Second argument returns true when IMU movement is blocking the drift calculation
	Function returns true if the metrics have been updated and not returned previously by this function
	*/
	bool get_gps_drift_metrics(float drift[3], bool *blocked) override;

	// return true if the global position estimate is valid
	bool global_position_is_valid() override;

	// check if the EKF is dead reckoning horizontal velocity using inertial data only
	void update_deadreckoning_status();

	bool isTerrainEstimateValid() const override;

	void updateTerrainValidity();

	// get the estimated terrain vertical position relative to the NED origin
	void getTerrainVertPos(float *ret) override;

	// get the terrain variance
	float get_terrain_var() const { return _terrain_var; }

	// get the accelerometer bias in m/s/s
	void get_accel_bias(float bias[3]) override;

	// get the gyroscope bias in rad/s
	void get_gyro_bias(float bias[3]) override;

	// get GPS check status
	void get_gps_check_status(uint16_t *val) override;

	// return the amount the local vertical position changed in the last reset and the number of reset events
	void get_posD_reset(float *delta, uint8_t *counter) override {*delta = _state_reset_status.posD_change; *counter = _state_reset_status.posD_counter;}

	// return the amount the local vertical velocity changed in the last reset and the number of reset events
	void get_velD_reset(float *delta, uint8_t *counter) override {*delta = _state_reset_status.velD_change; *counter = _state_reset_status.velD_counter;}

	// return the amount the local horizontal position changed in the last reset and the number of reset events
	void get_posNE_reset(float delta[2], uint8_t *counter) override
	{
		_state_reset_status.posNE_change.copyTo(delta);
		*counter = _state_reset_status.posNE_counter;
	}

	// return the amount the local horizontal velocity changed in the last reset and the number of reset events
	void get_velNE_reset(float delta[2], uint8_t *counter) override
	{
		_state_reset_status.velNE_change.copyTo(delta);
		*counter = _state_reset_status.velNE_counter;
	}

	// return the amount the quaternion has changed in the last reset and the number of reset events
	void get_quat_reset(float delta_quat[4], uint8_t *counter) override
	{
		_state_reset_status.quat_change.copyTo(delta_quat);
		*counter = _state_reset_status.quat_counter;
	}

	// get EKF innovation consistency check status information comprising of:
	// status - a bitmask integer containing the pass/fail status for each EKF measurement innovation consistency check
	// Innovation Test Ratios - these are the ratio of the innovation to the acceptance threshold.
	// A value > 1 indicates that the sensor measurement has exceeded the maximum acceptable level and has been rejected by the EKF
	// Where a measurement type is a vector quantity, eg magnetometer, GPS position, etc, the maximum value is returned.
	void get_innovation_test_status(uint16_t &status, float &mag, float &vel, float &pos, float &hgt, float &tas, float &hagl, float &beta) override;

	// return a bitmask integer that describes which state estimates can be used for flight control
	void get_ekf_soln_status(uint16_t *status) override;

	// return the quaternion defining the rotation from the External Vision to the EKF reference frame
	void get_ev2ekf_quaternion(float *quat) override;

	// use the latest IMU data at the current time horizon.
	Quatf calculate_quaternion() const;

	// set minimum continuous period without GPS fail required to mark a healthy GPS status
	void set_min_required_gps_health_time(uint32_t time_us) { _min_gps_health_time_us = time_us; }

private:

	static constexpr uint8_t _k_num_states{24};		///< number of EKF states

	struct {
		uint8_t velNE_counter;	///< number of horizontal position reset events (allow to wrap if count exceeds 255)
		uint8_t velD_counter;	///< number of vertical velocity reset events (allow to wrap if count exceeds 255)
		uint8_t posNE_counter;	///< number of horizontal position reset events (allow to wrap if count exceeds 255)
		uint8_t posD_counter;	///< number of vertical position reset events (allow to wrap if count exceeds 255)
		uint8_t quat_counter;	///< number of quaternion reset events (allow to wrap if count exceeds 255)
		Vector2f velNE_change;  ///< North East velocity change due to last reset (m)
		float velD_change;	///< Down velocity change due to last reset (m/sec)
		Vector2f posNE_change;	///< North, East position change due to last reset (m)
		float posD_change;	///< Down position change due to last reset (m)
		Quatf quat_change;	///< quaternion delta due to last reset - multiply pre-reset quaternion by this to get post-reset quaternion
	} _state_reset_status{};	///< reset event monitoring structure containing velocity, position, height and yaw reset information

	float _dt_ekf_avg{FILTER_UPDATE_PERIOD_S}; ///< average update rate of the ekf
	float _dt_update{0.01f}; ///< delta time since last ekf update. This time can be used for filters which run at the same rate as the Ekf::update() function. (sec)

	stateSample _state{};		///< state struct of the ekf running at the delayed time horizon

	bool _filter_initialised{false};	///< true when the EKF sttes and covariances been initialised

	bool _fuse_height{false};	///< true when baro height data should be fused
	bool _fuse_pos{false};		///< true when gps position data should be fused
	bool _fuse_hor_vel{false};	///< true when gps horizontal velocity measurement should be fused
	bool _fuse_vert_vel{false};	///< true when gps vertical velocity measurement should be fused
	bool _fuse_hor_vel_aux{false};	///< true when auxiliary horizontal velocity measurement should be fused

	// variables used when position data is being fused using a relative position odometry model
	bool _fuse_hpos_as_odom{false};		///< true when the NE position data is being fused using an odometry assumption
	Vector3f _pos_meas_prev;		///< previous value of NED position measurement fused using odometry assumption (m)
	Vector2f _hpos_pred_prev;		///< previous value of NE position state used by odometry fusion (m)
	bool _hpos_prev_available{false};	///< true when previous values of the estimate and measurement are available for use
	Dcmf _R_ev_to_ekf;			///< transformation matrix that rotates observations from the EV to the EKF navigation frame, initialized with Identity

	// booleans true when fresh sensor data is available at the fusion time horizon
	bool _gps_data_ready{false};	///< true when new GPS data has fallen behind the fusion time horizon and is available to be fused
	bool _mag_data_ready{false};	///< true when new magnetometer data has fallen behind the fusion time horizon and is available to be fused
	bool _baro_data_ready{false};	///< true when new baro height data has fallen behind the fusion time horizon and is available to be fused
	bool _range_data_ready{false};	///< true when new range finder data has fallen behind the fusion time horizon and is available to be fused
	bool _flow_data_ready{false};	///< true when the leading edge of the optical flow integration period has fallen behind the fusion time horizon
	bool _ev_data_ready{false};	///< true when new external vision system data has fallen behind the fusion time horizon and is available to be fused
	bool _tas_data_ready{false};	///< true when new true airspeed data has fallen behind the fusion time horizon and is available to be fused
	bool _flow_for_terrain_data_ready{false}; /// same flag as "_flow_data_ready" but used for separate terrain estimator

	uint64_t _time_last_aiding{0};	///< amount of time we have been doing inertial only deadreckoning (uSec)
	bool _using_synthetic_position{false};	///< true if we are using a synthetic position to constrain drift

	uint64_t _time_last_hor_pos_fuse{0};	///< time the last fusion of horizontal position measurements was performed (uSec)
	uint64_t _time_last_hgt_fuse{0};	///< time the last fusion of vertical position measurements was performed (uSec)
	uint64_t _time_last_hor_vel_fuse{0};	///< time the last fusion of horizontal velocity measurements was performed (uSec)
	uint64_t _time_last_ver_vel_fuse{0};	///< time the last fusion of verticalvelocity measurements was performed (uSec)
	uint64_t _time_last_delpos_fuse{0};	///< time the last fusion of incremental horizontal position measurements was performed (uSec)
	uint64_t _time_last_of_fuse{0};		///< time the last fusion of optical flow measurements were performed (uSec)
	uint64_t _time_last_arsp_fuse{0};	///< time the last fusion of airspeed measurements were performed (uSec)
	uint64_t _time_last_beta_fuse{0};	///< time the last fusion of synthetic sideslip measurements were performed (uSec)
	uint64_t _time_last_rng_ready{0};	///< time the last range finder measurement was ready (uSec)
	uint64_t _time_last_mag{0};		///< measurement time of last magnetomter sample (uSec)
	uint64_t _time_last_fake_pos{0};	///< last time we faked position measurements to constrain tilt errors during operation without external aiding (uSec)

	Vector2f _last_known_posNE;		///< last known local NE position vector (m)
	float _imu_collection_time_adj{0.0f};	///< the amount of time the IMU collection needs to be advanced to meet the target set by FILTER_UPDATE_PERIOD_MS (sec)

	uint64_t _time_acc_bias_check{0};	///< last time the  accel bias check passed (uSec)
	uint64_t _delta_time_baro_us{0};	///< delta time between two consecutive delayed baro samples from the buffer (uSec)

	uint64_t _last_imu_bias_cov_reset_us{0};	///< time the last reset of IMU delta angle and velocity state covariances was performed (uSec)

	Vector3f _earth_rate_NED;	///< earth rotation vector (NED) in rad/s

	Dcmf _R_to_earth;	///< transformation matrix from body frame to earth frame from last EKF prediction

	// used by magnetometer fusion mode selection
	Vector2f _accel_lpf_NE;			///< Low pass filtered horizontal earth frame acceleration (m/sec**2)
	float _yaw_delta_ef{0.0f};		///< Recent change in yaw angle measured about the earth frame D axis (rad)
	float _yaw_rate_lpf_ef{0.0f};		///< Filtered angular rate about earth frame D axis (rad/sec)
	bool _mag_bias_observable{false};	///< true when there is enough rotation to make magnetometer bias errors observable
	bool _yaw_angle_observable{false};	///< true when there is enough horizontal acceleration to make yaw observable
	uint64_t _time_yaw_started{0};		///< last system time in usec that a yaw rotation manoeuvre was detected
	uint8_t _num_bad_flight_yaw_events{0};	///< number of times a bad heading has been detected in flight and required a yaw reset
	uint64_t _mag_use_not_inhibit_us{0};	///< last system time in usec before magnetometer use was inhibited
	bool _mag_use_inhibit{false};		///< true when magnetometer use is being inhibited
	bool _mag_use_inhibit_prev{false};	///< true when magnetometer use was being inhibited the previous frame
	bool _mag_inhibit_yaw_reset_req{false};	///< true when magnetometer inhibit has been active for long enough to require a yaw reset when conditions improve.
	float _last_static_yaw{0.0f};		///< last yaw angle recorded when on ground motion checks were passing (rad)
	bool _mag_yaw_reset_req{false};		///< true when a reset of the yaw using the magnetometer data has been requested
	bool _mag_decl_cov_reset{false};	///< true after the fuseDeclination() function has been used to modify the earth field covariances after a magnetic field reset event.
	bool _synthetic_mag_z_active{false};	///< true if we are generating synthetic magnetometer Z measurements

	matrix::SquareMatrix<float, _k_num_states> P;	///< state covariance matrix

	Vector3f _delta_vel_bias_var_accum;		///< kahan summation algorithm accumulator for delta velocity bias variance
	Vector3f _delta_angle_bias_var_accum;	///< kahan summation algorithm accumulator for delta angle bias variance


	Vector3f _gps_vel_innov;	///< GPS velocity innovations (m/sec)
	Vector3f _gps_vel_innov_var;	///< GPS velocity innovation variances ((m/sec)**2)

	Vector3f _gps_pos_innov;	///< GPS position innovations (m)
	Vector3f _gps_pos_innov_var;	///< GPS position innovation variances (m**2)

	Vector3f _ev_vel_innov;	///< external vision velocity innovations (m/sec)
	Vector3f _ev_vel_innov_var;	///< external vision velocity innovation variances ((m/sec)**2)

	Vector3f _ev_pos_innov;	///< external vision position innovations (m)
	Vector3f _ev_pos_innov_var;	///< external vision position innovation variances (m**2)

	Vector3f _baro_hgt_innov;		///< baro hgt innovations (m)
	Vector3f _baro_hgt_innov_var;	///< baro hgt innovation variances (m**2)

	Vector3f _rng_hgt_innov;	///< range hgt innovations (m)
	Vector3f _rng_hgt_innov_var;	///< range hgt innovation variances (m**2)

	Vector3f _aux_vel_innov;	///< horizontal auxiliary velocity innovations: (m/sec)
	Vector3f _aux_vel_innov_var;	///< horizontal auxiliary velocity innovation variances: ((m/sec)**2)

	float _heading_innov{0.0f};	///< heading measurement innovation (rad)
	float _heading_innov_var{0.0f};	///< heading measurement innovation variance (rad**2)

	float _mag_innov[3] {};		///< earth magnetic field innovations (Gauss)
	float _mag_innov_var[3] {};	///< earth magnetic field innovation variance (Gauss**2)

	float _drag_innov[2] {};	///< multirotor drag measurement innovation (m/sec**2)
	float _drag_innov_var[2] {};	///< multirotor drag measurement innovation variance ((m/sec**2)**2)

	float _airspeed_innov{0.0f};		///< airspeed measurement innovation (m/sec)
	float _airspeed_innov_var{0.0f};	///< airspeed measurement innovation variance ((m/sec)**2)

	float _beta_innov{0.0f};	///< synthetic sideslip measurement innovation (rad)
	float _beta_innov_var{0.0f};	///< synthetic sideslip measurement innovation variance (rad**2)

	float _hagl_innov{0.0f};		///< innovation of the last height above terrain measurement (m)
	float _hagl_innov_var{0.0f};		///< innovation variance for the last height above terrain measurement (m**2)

	// optical flow processing
	float _flow_innov[2] {};	///< flow measurement innovation (rad/sec)
	float _flow_innov_var[2] {};	///< flow innovation variance ((rad/sec)**2)
	Vector3f _flow_gyro_bias;	///< bias errors in optical flow sensor rate gyro outputs (rad/sec)
	Vector3f _imu_del_ang_of;	///< bias corrected delta angle measurements accumulated across the same time frame as the optical flow rates (rad)
	float _delta_time_of{0.0f};	///< time in sec that _imu_del_ang_of was accumulated over (sec)
	uint64_t _time_bad_motion_us{0};	///< last system time that on-ground motion exceeded limits (uSec)
	uint64_t _time_good_motion_us{0};	///< last system time that on-ground motion was within limits (uSec)
	bool _inhibit_flow_use{false};	///< true when use of optical flow and range finder is being inhibited
	Vector2f _flow_compensated_XY_rad;	///< measured delta angle of the image about the X and Y body axes after removal of body rotation (rad), RH rotation is positive

	// output predictor states
	Vector3f _delta_angle_corr;	///< delta angle correction vector (rad)
	Vector3f _vel_err_integ;	///< integral of velocity tracking error (m)
	Vector3f _pos_err_integ;	///< integral of position tracking error (m.s)
	float _output_tracking_error[3] {}; ///< contains the magnitude of the angle, velocity and position track errors (rad, m/s, m)

	// variables used for the GPS quality checks
	Vector3f _gps_pos_deriv_filt;	///< GPS NED position derivative (m/sec)
	Vector2f _gps_velNE_filt;	///< filtered GPS North and East velocity (m/sec)
	float _gps_velD_diff_filt{0.0f};	///< GPS filtered Down velocity (m/sec)
	uint64_t _last_gps_fail_us{0};		///< last system time in usec that the GPS failed it's checks
	uint64_t _last_gps_pass_us{0};		///< last system time in usec that the GPS passed it's checks
	float _gps_error_norm{1.0f};		///< normalised gps error
	uint32_t _min_gps_health_time_us{10000000}; ///< GPS is marked as healthy only after this amount of time
	bool _gps_checks_passed{false};		///> true when all active GPS checks have passed

	// Variables used to publish the WGS-84 location of the EKF local NED origin
	uint64_t _last_gps_origin_time_us{0};	///< time the origin was last set (uSec)
	float _gps_alt_ref{0.0f};		///< WGS-84 height (m)

	// Variables used to initialise the filter states
	bool _is_first_imu_sample{true};
	uint32_t _baro_counter{0};		///< number of baro samples read during initialisation
	uint32_t _mag_counter{0};		///< number of magnetometer samples read during initialisation
	AlphaFilterVector3f _mag_lpf;		///< filtered magnetometer measurement for instant reset(Gauss)
	AlphaFilterVector3f _accel_lpf;		///< filtered accelerometer measurement for instant reset(Gauss)
	float _hgt_sensor_offset{0.0f};		///< set as necessary if desired to maintain the same height after a height reset (m)
	float _baro_hgt_offset{0.0f};		///< baro height reading at the local NED origin (m)

	// Variables used to control activation of post takeoff functionality
	float _last_on_ground_posD{0.0f};	///< last vertical position when the in_air status was false (m)
	uint64_t _flt_mag_align_start_time{0};	///< time that inflight magnetic field alignment started (uSec)
	uint64_t _time_last_mov_3d_mag_suitable{0};	///< last system time that sufficient movement to use 3-axis magnetometer fusion was detected (uSec)
	float _saved_mag_bf_variance[4] {};	///< magnetic field state variances that have been saved for use at the next initialisation (Gauss**2)
	float _saved_mag_ef_covmat[2][2] {};    ///< NE magnetic field state covariance sub-matrix saved for use at the next initialisation (Gauss**2)
	bool _velpos_reset_request{false};	///< true when a large yaw error has been fixed and a velocity and position state reset is required

	gps_check_fail_status_u _gps_check_fail_status{};

	// variables used to inhibit accel bias learning
	bool _accel_bias_inhibit{false};	///< true when the accel bias learning is being inhibited
	Vector3f _accel_vec_filt;		///< acceleration vector after application of a low pass filter (m/sec**2)
	float _accel_magnitude_filt{0.0f};	///< acceleration magnitude after application of a decaying envelope filter (rad/sec)
	float _ang_rate_magnitude_filt{0.0f};		///< angular rate magnitude after application of a decaying envelope filter (rad/sec)
	Vector3f _prev_dvel_bias_var;		///< saved delta velocity XYZ bias variances (m/sec)**2

	// Terrain height state estimation
	float _terrain_vpos{0.0f};		///< estimated vertical position of the terrain underneath the vehicle in local NED frame (m)
	float _terrain_var{1e4f};		///< variance of terrain position estimate (m**2)
	uint64_t _time_last_hagl_fuse{0};		///< last system time that the hagl measurement failed it's checks (uSec)
	bool _terrain_initialised{false};	///< true when the terrain estimator has been initialized
	float _sin_tilt_rng{0.0f};		///< sine of the range finder tilt rotation about the Y body axis
	float _cos_tilt_rng{0.0f};		///< cosine of the range finder tilt rotation about the Y body axis
	float _R_rng_to_earth_2_2{0.0f};	///< 2,2 element of the rotation matrix from sensor frame to earth frame
	float _dt_last_range_update_filt_us{0.0f};	///< filtered value of the delta time elapsed since the last range measurement came into the filter (uSec)
	bool _hagl_valid{false};		///< true when the height above ground estimate is valid

	// height sensor status
	bool _baro_hgt_faulty{false};		///< true if valid baro data is unavailable for use
	bool _gps_hgt_intermittent{false};	///< true if gps height into the buffer is intermittent
	bool _rng_hgt_valid{false};		///< true if range finder sample retrieved from buffer is valid
	uint64_t _time_bad_rng_signal_quality{0};	///< timestamp at which range finder signal quality was 0 (used for hysteresis)

	// imu fault status
	uint64_t _time_bad_vert_accel{0};	///< last time a bad vertical accel was detected (uSec)
	uint64_t _time_good_vert_accel{0};	///< last time a good vertical accel was detected (uSec)
	bool _bad_vert_accel_detected{false};	///< true when bad vertical accelerometer data has been detected

	// variables used to control range aid functionality
	bool _is_range_aid_suitable{false};	///< true when range finder can be used in flight as the height reference instead of the primary height sensor
	bool _range_aid_mode_selected{false};	///< true when range finder is being used as the height reference instead of the primary height sensor

	// variables used to check range finder validity data
	float _rng_stuck_min_val{0.0f};		///< minimum value for new rng measurement when being stuck
	float _rng_stuck_max_val{0.0f};		///< maximum value for new rng measurement when being stuck

	float _height_rate_lpf{0.0f};

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

	// fuse the first euler angle from either a 321 or 312 rotation sequence as the observation (currently measures yaw using the magnetometer)
	void fuseHeading();

	// fuse the yaw angle obtained from a dual antenna GPS unit
	void fuseGpsAntYaw();

	// reset the quaternions states using the yaw angle obtained from a dual antenna GPS unit
	// return true if the reset was successful
	bool resetGpsAntYaw();

	// fuse magnetometer declination measurement
	// argument passed in is the declination uncertainty in radians
	void fuseDeclination(float decl_sigma);

	// apply sensible limits to the declination and length of the NE mag field states estimates
	void limitDeclination();

	// fuse airspeed measurement
	void fuseAirspeed();

	// fuse synthetic zero sideslip measurement
	void fuseSideslip();

	// fuse body frame drag specific forces for multi-rotor wind estimation
	void fuseDrag();

	// fuse single velocity and position measurement
	void fuseVelPosHeight(const float innov, const float innov_var, const int obs_index);

	// reset velocity states of the ekf
	bool resetVelocity();

	// fuse optical flow line of sight rate measurements
	void fuseOptFlow();

	bool fuseHorizontalVelocity(const Vector3f &innov, const Vector2f &innov_gate,
				 const Vector3f &obs_var, Vector3f &innov_var, Vector2f &test_ratio);

	bool fuseVerticalVelocity(const Vector3f &innov, const Vector2f &innov_gate,
				 const Vector3f &obs_var, Vector3f &innov_var, Vector2f &test_ratio);

	bool fuseHorizontalPosition(const Vector3f &innov, const Vector2f &innov_gate,
				 const Vector3f &obs_var, Vector3f &innov_var, Vector2f &test_ratio);

	bool fuseVerticalPosition(const Vector3f &innov, const Vector2f &innov_gate,
				 const Vector3f &obs_var, Vector3f &innov_var, Vector2f &test_ratio);

	// calculate optical flow body angular rate compensation
	// returns false if bias corrected body rate data is unavailable
	bool calcOptFlowBodyRateComp();

	// initialise the terrain vertical position estimator
	// return true if the initialisation is successful
	bool initHagl();

	// run the terrain estimator
	void runTerrainEstimator();

	// update the terrain vertical position estimate using a height above ground measurement from the range finder
	void fuseHagl();

	// update the terrain vertical position estimate using an optical flow measurement
	void fuseFlowForTerrain();

	// reset the heading and magnetic field states using the declination and magnetometer/external vision measurements
	// return true if successful
	bool resetMagHeading(const Vector3f &mag_init, bool increase_yaw_var = true, bool update_buffer=true);

	// Do a forced re-alignment of the yaw angle to align with the horizontal velocity vector from the GPS.
	// It is used to align the yaw angle after launch or takeoff for fixed wing vehicle.
	bool realignYawGPS();

	// Return the magnetic declination in radians to be used by the alignment and fusion processing
	float getMagDeclination();

	// reset position states of the ekf (only horizontal position)
	bool resetPosition();

	// reset height state of the ekf
	void resetHeight();

	// modify output filter to match the the EKF state at the fusion time horizon
	void alignOutputFilter();

	// update the rotation matrix which transforms EV navigation frame measurements into NED
	void calcExtVisRotMat();

	// limit the diagonal of the covariance matrix
	// force symmetry when the argument is true
	void fixCovarianceErrors(bool force_symmetry);

	// constrain the ekf states
	void constrainStates();

	// generic function which will perform a fusion step given a kalman gain K
	// and a scalar innovation value
	void fuse(float *K, float innovation);

	float compensateBaroForDynamicPressure(float baro_alt_uncompensated) override;

	// calculate the earth rotation vector from a given latitude
	Vector3f calcEarthRateNED(float lat_rad) const;

	// return true id the GPS quality is good enough to set an origin and start aiding
	bool gps_is_good(const gps_message &gps);

	// Control the filter fusion modes
	void controlFusionModes();

	// control fusion of external vision observations
	void controlExternalVisionFusion();

	// control fusion of optical flow observations
	void controlOpticalFlowFusion();

	// control fusion of GPS observations
	void controlGpsFusion();

	// control fusion of magnetometer observations
	void controlMagFusion();
	void updateMagFilter();

	bool canRunMagFusion() const;

	void checkHaglYawResetReq();
	float getTerrainVPos() const;

	void runOnGroundYawReset();
	bool isYawResetAuthorized() const;
	bool canResetMagHeading() const;
	void runInAirYawReset();
	bool canRealignYawUsingGps() const;
	void runVelPosReset();

	void selectMagAuto();
	void check3DMagFusionSuitability();
	void checkYawAngleObservability();
	void checkMagBiasObservability();
	bool isYawAngleObservable() const;
	bool isMagBiasObservable() const;
	bool canUse3DMagFusion() const;

	void checkMagDeclRequired();
	void checkMagInhibition();
	bool shouldInhibitMag() const;
	void checkMagFieldStrength();
	bool isStrongMagneticDisturbance() const;
	bool isMeasuredMatchingGpsMagStrength() const;
	bool isMeasuredMatchingAverageMagStrength() const;
	static bool isMeasuredMatchingExpected(float measured, float expected, float gate);
	void runMagAndMagDeclFusions();
	void run3DMagAndDeclFusions();

	// control fusion of range finder observations
	void controlRangeFinderFusion();

	// control fusion of air data observations
	void controlAirDataFusion();

	// control fusion of synthetic sideslip observations
	void controlBetaFusion();

	// control fusion of multi-rotor drag specific force observations
	void controlDragFusion();

	// control fusion of pressure altitude observations
	void controlBaroFusion();

	// control fusion of fake position observations to constrain drift
	void controlFakePosFusion();

	// control fusion of auxiliary velocity observations
	void controlAuxVelFusion();

	// control for height sensor timeouts, sensor changes and state resets
	void controlHeightSensorTimeouts();

	// control for combined height fusion mode (implemented for switching between baro and range height)
	void controlHeightFusion();

	// determine if flight condition is suitable to use range finder instead of the primary height sensor
	void checkRangeAidSuitability();
	bool isRangeAidSuitable() { return _is_range_aid_suitable; }

	// update _rng_hgt_valid, which indicates if the current range sample has passed validity checks
	void updateRangeDataValidity();

	// check for "stuck" range finder measurements when rng was not valid for certain period
	void updateRangeDataStuck();

	// return the square of two floating point numbers - used in auto coded sections
	static constexpr float sq(float var) { return var * var; }

	// set control flags to use baro height
	void setControlBaroHeight();

	// set control flags to use range height
	void setControlRangeHeight();

	// set control flags to use GPS height
	void setControlGPSHeight();

	// set control flags to use external vision height
	void setControlEVHeight();

	void stopMagFusion();
	void stopMag3DFusion();
	void stopMagHdgFusion();
	void startMagHdgFusion();
	void startMag3DFusion();

	// calculate the measurement variance for the optical flow sensor
	float calcOptFlowMeasVar();

	// rotate quaternion covariances into variances for an equivalent rotation vector
	Vector3f calcRotVecVariances();

	// initialise the quaternion covariances using rotation vector variances
	void initialiseQuatCovariances(Vector3f &rot_vec_var);

	// perform a limited reset of the magnetic field state covariances
	void resetMagRelatedCovariances();

	// perform a limited reset of the wind state covariances
	void resetWindCovariance();

	// perform a reset of the wind states
	void resetWindStates();

	// check that the range finder data is continuous
	void updateRangeDataContinuity();

	bool isRangeDataContinuous() { return _dt_last_range_update_filt_us < 2e6f; }

	// Increase the yaw error variance of the quaternions
	// Argument is additional yaw variance in rad**2
	void increaseQuatYawErrVariance(float yaw_variance);

	// load and save mag field state covariance data for re-use
	void loadMagCovData();
	void saveMagCovData();
	void clearMagCov();
	void zeroMagCov();

	// uncorrelate quaternion states from other states
	void uncorrelateQuatStates();

	// Use Kahan summation algorithm to get the sum of "sum_previous" and "input".
	// This function relies on the caller to be responsible for keeping a copy of
	// "accumulator" and passing this value at the next iteration.
	// Ref: https://en.wikipedia.org/wiki/Kahan_summation_algorithm
	float kahanSummation(float sum_previous, float input, float &accumulator) const;

	// calculate a synthetic value for the magnetometer Z component, given the 3D magnetomter
	// sensor measurement
	float calculate_synthetic_mag_z_measurement(const Vector3f& mag_meas, const Vector3f& mag_earth_predicted);

	bool isTimedOut(uint64_t last_sensor_timestamp, uint64_t timeout_period) const
	{
		return last_sensor_timestamp + timeout_period < _time_last_imu;
	}

	bool isRecent(uint64_t sensor_timestamp, uint64_t acceptance_interval) const
	{
		return sensor_timestamp + acceptance_interval > _time_last_imu;
	}

	void stopGpsFusion();

	void stopGpsPosFusion();

	void stopGpsVelFusion();

	void stopGpsYawFusion();

	void stopEvFusion();

	void stopEvPosFusion();

	void stopEvVelFusion();

	void stopEvYawFusion();

	void stopAuxVelFusion();

	void stopFlowFusion();

	void setVelPosFaultStatus(const int index, const bool status);

};
