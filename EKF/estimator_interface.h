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
 * @file estimator_interface.h
 * Definition of base class for attitude estimators
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#pragma once

#include <ecl.h>
#include "common.h"
#include "RingBuffer.h"
#include "AlphaFilter.hpp"
#include "imu_down_sampler.hpp"

#include <geo/geo.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>

using namespace estimator;

class EstimatorInterface
{

public:
	typedef AlphaFilter<Vector3f> AlphaFilterVector3f;

	EstimatorInterface():_imu_down_sampler(FILTER_UPDATE_PERIOD_S){};
	virtual ~EstimatorInterface() = default;

	virtual bool init(uint64_t timestamp) = 0;
	virtual void reset() = 0;

	virtual bool update() = 0;

	virtual void getGpsVelPosInnov(float hvel[2], float &vvel, float hpos[2], float &vpos) = 0;
	virtual void getGpsVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos) = 0;
	virtual void getGpsVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos) = 0;

	virtual void getEvVelPosInnov(float hvel[2], float &vvel, float hpos[2], float &vpos) = 0;
	virtual void getEvVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos) = 0;
	virtual void getEvVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos) = 0;

	virtual void getBaroHgtInnov(float &baro_hgt_innov) = 0;
	virtual void getBaroHgtInnovVar(float &baro_hgt_innov_var) = 0;
	virtual void getBaroHgtInnovRatio(float &baro_hgt_innov_ratio) = 0;

	virtual void getRngHgtInnov(float &rng_hgt_innov) = 0;
	virtual void getRngHgtInnovVar(float &rng_hgt_innov_var) = 0;
	virtual void getRngHgtInnovRatio(float &rng_hgt_innov_ratio) = 0;

	virtual void getAuxVelInnov(float aux_vel_innov[2]) = 0;
	virtual void getAuxVelInnovVar(float aux_vel_innov[2]) = 0;
	virtual void getAuxVelInnovRatio(float &aux_vel_innov_ratio) = 0;

	virtual void getFlowInnov(float flow_innov[2]) = 0;
	virtual void getFlowInnovVar(float flow_innov_var[2]) = 0;
	virtual void getFlowInnovRatio(float &flow_innov_ratio) = 0;

	virtual void getHeadingInnov(float &heading_innov) = 0;
	virtual void getHeadingInnovVar(float &heading_innov_var) = 0;
	virtual void getHeadingInnovRatio(float &heading_innov_ratio) = 0;

	virtual void getMagInnov(float mag_innov[3]) = 0;
	virtual void getMagInnovVar(float mag_innov_var[3]) = 0;
	virtual void getMagInnovRatio(float &mag_innov_ratio) = 0;

	virtual void getDragInnov(float drag_innov[2]) = 0;
	virtual void getDragInnovVar(float drag_innov_var[2]) = 0;
	virtual void getDragInnovRatio(float drag_innov_ratio[2]) = 0;

	virtual void getAirspeedInnov(float &airspeed_innov) = 0;
	virtual void getAirspeedInnovVar(float &get_airspeed_innov_var) = 0;
	virtual void getAirspeedInnovRatio(float &airspeed_innov_ratio) = 0;

	virtual void getBetaInnov(float &beta_innov) = 0;
	virtual void getBetaInnovVar(float &get_beta_innov_var) = 0;
	virtual void getBetaInnovRatio(float &beta_innov_ratio) = 0;

	virtual void getHaglInnov(float &hagl_innov) = 0;
	virtual void getHaglInnovVar(float &hagl_innov_var) = 0;
	virtual void getHaglInnovRatio(float &hagl_innov_ratio) = 0;


	virtual void get_state_delayed(float *state) = 0;

	virtual void get_wind_velocity(float *wind) = 0;

	virtual void get_wind_velocity_var(float *wind_var) = 0;

	virtual void get_true_airspeed(float *tas) = 0;

	// gets the variances for the NED velocity states
	virtual void get_vel_var(Vector3f &vel_var) = 0;

	// gets the variances for the NED position states
	virtual void get_pos_var(Vector3f &pos_var) = 0;


	// return an array containing the output predictor angular, velocity and position tracking
	// error magnitudes (rad), (m/s), (m)
	virtual void get_output_tracking_error(float error[3]) = 0;

	/*
	Returns  following IMU vibration metrics in the following array locations
	0 : Gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
	1 : Gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
	2 : Accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)
	*/
	virtual void get_imu_vibe_metrics(float vibe[3]) = 0;

	/*
	First argument returns GPS drift  metrics in the following array locations
	0 : Horizontal position drift rate (m/s)
	1 : Vertical position drift rate (m/s)
	2 : Filtered horizontal velocity (m/s)
	Second argument returns true when IMU movement is blocking the drift calculation
	Function returns true if the metrics have been updated and not returned previously by this function
	*/
	virtual bool get_gps_drift_metrics(float drift[3], bool *blocked) = 0;

	// get the ekf WGS-84 origin position and height and the system time it was last set
	// return true if the origin is valid
	virtual bool get_ekf_origin(uint64_t *origin_time, map_projection_reference_s *origin_pos, float *origin_alt) = 0;

	// get the 1-sigma horizontal and vertical position uncertainty of the ekf WGS-84 position
	virtual void get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv) = 0;

	// get the 1-sigma horizontal and vertical position uncertainty of the ekf local position
	virtual void get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv) = 0;

	// get the 1-sigma horizontal and vertical velocity uncertainty
	virtual void get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv) = 0;

	// get the vehicle control limits required by the estimator to keep within sensor limitations
	virtual void get_ekf_ctrl_limits(float *vxy_max, float *vz_max, float *hagl_min, float *hagl_max) = 0;

	// ask estimator for sensor data collection decision and do any preprocessing if required, returns true if not defined
	virtual bool collect_gps(const gps_message &gps) = 0;

	void setIMUData(const imuSample &imu_sample);


	void setMagData(const magSample &mag_sample);

	void setGpsData(const gps_message &gps);

	void setBaroData(const baroSample &baro_sample);

	void setAirspeedData(const airspeedSample &airspeed_sample);

	void setRangeData(const rangeSample& range_sample);

	// if optical flow sensor gyro delta angles are not available, set gyro_xyz vector fields to NaN and the EKF will use its internal delta angle data instead
	void setOpticalFlowData(const flowSample& flow);

	// set external vision position and attitude data
	void setExtVisionData(const extVisionSample& evdata);

	void setAuxVelData(const auxVelSample& auxvel_sample);

	// return a address to the parameters struct
	// in order to give access to the application
	parameters *getParamHandle() {return &_params;}

	// set vehicle landed status data
	void set_in_air_status(bool in_air) {_control_status.flags.in_air = in_air;}

	/*
	Reset all IMU bias states and covariances to initial alignment values.
	Use when the IMU sensor has changed.
	Returns true if reset performed, false if rejected due to less than 10 seconds lapsed since last reset.
	*/
	virtual bool reset_imu_bias() = 0;

	// return true if the attitude is usable
	bool attitude_valid() { return ISFINITE(_output_new.quat_nominal(0)) && _control_status.flags.tilt_align; }

	// get vehicle landed status data
	bool get_in_air_status() {return _control_status.flags.in_air;}

	// get wind estimation status
	bool get_wind_status() { return _control_status.flags.wind; }

	// set vehicle is fixed wing status
	void set_is_fixed_wing(bool is_fixed_wing) {_control_status.flags.fixed_wing = is_fixed_wing;}

	// set flag if synthetic sideslip measurement should be fused
	void set_fuse_beta_flag(bool fuse_beta) {_control_status.flags.fuse_beta = (fuse_beta && _control_status.flags.in_air);}

	// set flag if static pressure rise due to ground effect is expected
	// use _params.gnd_effect_deadzone to adjust for expected rise in static pressure
	// flag will clear after GNDEFFECT_TIMEOUT uSec
	void set_gnd_effect_flag(bool gnd_effect)
	{
		_control_status.flags.gnd_effect = gnd_effect;
		_time_last_gnd_effect_on = _time_last_imu;
	}

	// set air density used by the multi-rotor specific drag force fusion
	void set_air_density(float air_density) {_air_density = air_density;}

	// set sensor limitations reported by the rangefinder
	void set_rangefinder_limits(float min_distance, float max_distance)
	{
		_rng_valid_min_val = min_distance;
		_rng_valid_max_val = max_distance;
	}

	// set sensor limitations reported by the optical flow sensor
	void set_optical_flow_limits(float max_flow_rate, float min_distance, float max_distance)
	{
		_flow_max_rate = max_flow_rate;
		_flow_min_distance = min_distance;
		_flow_max_distance = max_distance;
	}

	// return true if the global position estimate is valid
	virtual bool global_position_is_valid() = 0;

	// the flags considered are opt_flow, gps, ev_vel and ev_pos
	bool isOnlyActiveSourceOfHorizontalAiding(bool aiding_flag) const;

	/*
	 * Check if there are any other active source of horizontal aiding
	 * Warning: does not tell if the selected source is
	 * active, use isOnlyActiveSourceOfHorizontalAiding() for this
	 *
	 * The flags considered are opt_flow, gps, ev_vel and ev_pos
	 *
	 * @param aiding_flag a flag in _control_status.flags
	 * @return true if an other source than aiding_flag is active
	 */
	bool isOtherSourceOfHorizontalAidingThan(bool aiding_flag) const;

	// Return true if at least one source of horizontal aiding is active
	// the flags considered are opt_flow, gps, ev_vel and ev_pos
	bool isHorizontalAidingActive() const;

	int getNumberOfActiveHorizontalAidingSources() const;

	// return true if the EKF is dead reckoning the position using inertial data only
	bool inertial_dead_reckoning() {return _is_dead_reckoning;}

	virtual bool isTerrainEstimateValid() const = 0;
	//[[deprecated("Replaced by isTerrainEstimateValid")]]
	bool get_terrain_valid() { return isTerrainEstimateValid(); }

	// get the estimated terrain vertical position relative to the NED origin
	virtual void getTerrainVertPos(float *ret) = 0;
	//[[deprecated("Replaced by getTerrainVertPos")]]
	void get_terrain_vert_pos(float *ret) { getTerrainVertPos(ret); }

	// return true if the local position estimate is valid
	bool local_position_is_valid();

	const matrix::Quatf &get_quaternion() const { return _output_new.quat_nominal; }

	// return the quaternion defining the rotation from the EKF to the External Vision reference frame
	virtual void get_ev2ekf_quaternion(float *quat) = 0;

	// get the velocity of the body frame origin in local NED earth frame
	void get_velocity(float *vel)
	{
		Vector3f vel_earth = _output_new.vel - _vel_imu_rel_body_ned;

		for (unsigned i = 0; i < 3; i++) {
			vel[i] = vel_earth(i);
		}
	}

	// get the NED velocity derivative in earth frame
	void get_vel_deriv_ned(float *vel_deriv)
	{
		for (unsigned i = 0; i < 3; i++) {
			vel_deriv[i] = _vel_deriv_ned(i);
		}
	}

	// get the derivative of the vertical position of the body frame origin in local NED earth frame
	void get_pos_d_deriv(float *pos_d_deriv)
	{
		float var = _output_vert_new.vel_d - _vel_imu_rel_body_ned(2);
		*pos_d_deriv = var;
	}

	// get the position of the body frame origin in local NED earth frame
	void get_position(float *pos)
	{
		// rotate the position of the IMU relative to the boy origin into earth frame
		Vector3f pos_offset_earth = _R_to_earth_now * _params.imu_pos_body;

		// subtract from the EKF position (which is at the IMU) to get position at the body origin
		for (unsigned i = 0; i < 3; i++) {
			pos[i] = _output_new.pos(i) - pos_offset_earth(i);
		}
	}
	void copy_timestamp(uint64_t *time_us)
	{
		*time_us = _time_last_imu;
	}

	// Get the value of magnetic declination in degrees to be saved for use at the next startup
	// Returns true when the declination can be saved
	// At the next startup, set param.mag_declination_deg to the value saved
	bool get_mag_decl_deg(float *val)
	{
		*val = 0.0f;
		if (_NED_origin_initialised && (_params.mag_declination_source & MASK_SAVE_GEO_DECL)) {
			*val = math::degrees(_mag_declination_gps);
			return true;

		} else {
			return false;
		}
	}

	virtual void get_accel_bias(float bias[3]) = 0;
	virtual void get_gyro_bias(float bias[3]) = 0;

	// get EKF mode status
	void get_control_mode(uint32_t *val)
	{
		*val = _control_status.value;
	}

	// get EKF internal fault status
	void get_filter_fault_status(uint16_t *val)
	{
		*val = _fault_status.value;
	}

	bool isVehicleAtRest() const { return _vehicle_at_rest; }

	// get GPS check status
	virtual void get_gps_check_status(uint16_t *val) = 0;

	// return the amount the local vertical position changed in the last reset and the number of reset events
	virtual void get_posD_reset(float *delta, uint8_t *counter) = 0;

	// return the amount the local vertical velocity changed in the last reset and the number of reset events
	virtual void get_velD_reset(float *delta, uint8_t *counter) = 0;

	// return the amount the local horizontal position changed in the last reset and the number of reset events
	virtual void get_posNE_reset(float delta[2], uint8_t *counter) = 0;

	// return the amount the local horizontal velocity changed in the last reset and the number of reset events
	virtual void get_velNE_reset(float delta[2], uint8_t *counter) = 0;

	// return the amount the quaternion has changed in the last reset and the number of reset events
	virtual void get_quat_reset(float delta_quat[4], uint8_t *counter) = 0;

	// get EKF innovation consistency check status information comprising of:
	// status - a bitmask integer containing the pass/fail status for each EKF measurement innovation consistency check
	// Innovation Test Ratios - these are the ratio of the innovation to the acceptance threshold.
	// A value > 1 indicates that the sensor measurement has exceeded the maximum acceptable level and has been rejected by the EKF
	// Where a measurement type is a vector quantity, eg magnetometer, GPS position, etc, the maximum value is returned.
	virtual void get_innovation_test_status(uint16_t &status, float &mag, float &vel, float &pos, float &hgt, float &tas, float &hagl, float &beta) = 0;

	// return a bitmask integer that describes which state estimates can be used for flight control
	virtual void get_ekf_soln_status(uint16_t *status) = 0;

	// Getter for the average imu update period in s
	float get_dt_imu_avg() const { return _dt_imu_avg; }

	// Getter for the imu sample on the delayed time horizon
	imuSample get_imu_sample_delayed()
	{
		return _imu_sample_delayed;
	}

	// Getter for the baro sample on the delayed time horizon
	baroSample get_baro_sample_delayed()
	{
		return _baro_sample_delayed;
	}

	void print_status();

	static constexpr unsigned FILTER_UPDATE_PERIOD_MS{8};	// ekf prediction period in milliseconds - this should ideally be an integer multiple of the IMU time delta
	static constexpr float FILTER_UPDATE_PERIOD_S{FILTER_UPDATE_PERIOD_MS * 0.001f};

protected:

	parameters _params;		// filter parameters

	ImuDownSampler _imu_down_sampler;

	/*
	 OBS_BUFFER_LENGTH defines how many observations (non-IMU measurements) we can buffer
	 which sets the maximum frequency at which we can process non-IMU measurements. Measurements that
	 arrive too soon after the previous measurement will not be processed.
	 max freq (Hz) = (OBS_BUFFER_LENGTH - 1) / (IMU_BUFFER_LENGTH * FILTER_UPDATE_PERIOD_S)
	 This can be adjusted to match the max sensor data rate plus some margin for jitter.
	*/
	uint8_t _obs_buffer_length{0};

	/*
	IMU_BUFFER_LENGTH defines how many IMU samples we buffer which sets the time delay from current time to the
	EKF fusion time horizon and therefore the maximum sensor time offset relative to the IMU that we can compensate for.
	max sensor time offet (msec) =  IMU_BUFFER_LENGTH * FILTER_UPDATE_PERIOD_MS
	This can be adjusted to a value that is FILTER_UPDATE_PERIOD_MS longer than the maximum observation time delay.
	*/
	uint8_t _imu_buffer_length{0};

	unsigned _min_obs_interval_us{0}; // minimum time interval between observations that will guarantee data is not lost (usec)

	float _dt_imu_avg{0.0f};	// average imu update period in s

	imuSample _imu_sample_delayed{};	// captures the imu sample on the delayed time horizon

	// measurement samples capturing measurements on the delayed time horizon
	magSample _mag_sample_delayed{};
	baroSample _baro_sample_delayed{};
	gpsSample _gps_sample_delayed{};
	rangeSample _range_sample_delayed{};
	airspeedSample _airspeed_sample_delayed{};
	flowSample _flow_sample_delayed{};
	extVisionSample _ev_sample_delayed{};
	dragSample _drag_sample_delayed{};
	dragSample _drag_down_sampled{};	// down sampled drag specific force data (filter prediction rate -> observation rate)
	auxVelSample _auxvel_sample_delayed{};

	// Used by the multi-rotor specific drag force fusion
	uint8_t _drag_sample_count{0};	// number of drag specific force samples assumulated at the filter prediction rate
	float _drag_sample_time_dt{0.0f};	// time integral across all samples used to form _drag_down_sampled (sec)
	float _air_density{CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C};		// air density (kg/m**3)

	// Sensor limitations
	float _rng_valid_min_val{0.0f};	///< minimum distance that the rangefinder can measure (m)
	float _rng_valid_max_val{0.0f};	///< maximum distance that the rangefinder can measure (m)
	float _flow_max_rate{0.0f}; ///< maximum angular flow rate that the optical flow sensor can measure (rad/s)
	float _flow_min_distance{0.0f};	///< minimum distance that the optical flow sensor can operate at (m)
	float _flow_max_distance{0.0f};	///< maximum distance that the optical flow sensor can operate at (m)

	// Output Predictor
	outputSample _output_sample_delayed{};	// filter output on the delayed time horizon
	outputSample _output_new{};		// filter output on the non-delayed time horizon
	outputVert _output_vert_delayed{};	// vertical filter output on the delayed time horizon
	outputVert _output_vert_new{};		// vertical filter output on the non-delayed time horizon
	imuSample _newest_high_rate_imu_sample{};		// imu sample capturing the newest imu data
	Matrix3f _R_to_earth_now;		// rotation matrix from body to earth frame at current time
	Vector3f _vel_imu_rel_body_ned;		// velocity of IMU relative to body origin in NED earth frame
	Vector3f _vel_deriv_ned;		// velocity derivative at the IMU in NED earth frame (m/s/s)

	bool _imu_updated{false};      // true if the ekf should update (completed downsampling process)
	bool _initialised{false};      // true if the ekf interface instance (data buffering) is initialized

	bool _NED_origin_initialised{false};
	bool _gps_speed_valid{false};
	float _gps_origin_eph{0.0f}; // horizontal position uncertainty of the GPS origin
	float _gps_origin_epv{0.0f}; // vertical position uncertainty of the GPS origin
	struct map_projection_reference_s _pos_ref {};   // Contains WGS-84 position latitude and longitude (radians) of the EKF origin
	struct map_projection_reference_s _gps_pos_prev {};   // Contains WGS-84 position latitude and longitude (radians) of the previous GPS message
	float _gps_alt_prev{0.0f};	// height from the previous GPS message (m)
	float _gps_yaw_offset{0.0f};	// Yaw offset angle for dual GPS antennas used for yaw estimation (radians).

	// innovation consistency check monitoring ratios
	float _yaw_test_ratio{};		// yaw innovation consistency check ratio
	float _mag_test_ratio[3] {};		// magnetometer XYZ innovation consistency check ratios
	Vector2f _gps_vel_test_ratio;	// GPS velocity innovation consistency check ratios
	Vector2f _gps_pos_test_ratio;	// GPS position innovation consistency check ratios
	Vector2f _ev_vel_test_ratio;		// EV velocity innovation consistency check ratios
	Vector2f _ev_pos_test_ratio ;		// EV position innovation consistency check ratios
	Vector2f _aux_vel_test_ratio;		// Auxiliray horizontal velocity innovation consistency check ratio
	Vector2f _baro_hgt_test_ratio;	// baro height innovation consistency check ratios
	Vector2f _rng_hgt_test_ratio;	// range finder height innovation consistency check ratios
	float _optflow_test_ratio{};		// Optical flow innovation consistency check ratio
	float _tas_test_ratio{};		// tas innovation consistency check ratio
	float _hagl_test_ratio{};		// height above terrain measurement innovation consistency check ratio
	float _beta_test_ratio{};		// sideslip innovation consistency check ratio
	float _drag_test_ratio[2] {};	// drag innovation consistency check ratio
	innovation_fault_status_u _innov_check_fail_status{};

	bool _is_dead_reckoning{false};		// true if we are no longer fusing measurements that constrain horizontal velocity drift
	bool _deadreckon_time_exceeded{true};	// true if the horizontal nav solution has been deadreckoning for too long and is invalid
	bool _is_wind_dead_reckoning{false};	// true if we are navigationg reliant on wind relative measurements

	// IMU vibration and movement monitoring
	Vector3f _delta_ang_prev;	// delta angle from the previous IMU measurement
	Vector3f _delta_vel_prev;	// delta velocity from the previous IMU measurement
	float _vibe_metrics[3] {};	// IMU vibration metrics
					// [0] Level of coning vibration in the IMU delta angles (rad^2)
					// [1] high frequency vibration level in the IMU delta angle data (rad)
					// [2] high frequency vibration level in the IMU delta velocity data (m/s)
	float _gps_drift_metrics[3] {};	// Array containing GPS drift metrics
					// [0] Horizontal position drift rate (m/s)
					// [1] Vertical position drift rate (m/s)
					// [2] Filtered horizontal velocity (m/s)
	bool _vehicle_at_rest{false};	// true when the vehicle is at rest
	uint64_t _time_last_move_detect_us{0};	// timestamp of last movement detection event in microseconds
	bool _gps_drift_updated{false};	// true when _gps_drift_metrics has been updated and is ready for retrieval

	// data buffer instances
	RingBuffer<imuSample> _imu_buffer;
	RingBuffer<gpsSample> _gps_buffer;
	RingBuffer<magSample> _mag_buffer;
	RingBuffer<baroSample> _baro_buffer;
	RingBuffer<rangeSample> _range_buffer;
	RingBuffer<airspeedSample> _airspeed_buffer;
	RingBuffer<flowSample> 	_flow_buffer;
	RingBuffer<extVisionSample> _ext_vision_buffer;
	RingBuffer<outputSample> _output_buffer;
	RingBuffer<outputVert> _output_vert_buffer;
	RingBuffer<dragSample> _drag_buffer;
	RingBuffer<auxVelSample> _auxvel_buffer;

	// observation buffer final allocation failed
	bool _gps_buffer_fail{false};
	bool _mag_buffer_fail{false};
	bool _baro_buffer_fail{false};
	bool _range_buffer_fail{false};
	bool _airspeed_buffer_fail{false};
	bool _flow_buffer_fail{false};
	bool _ev_buffer_fail{false};
	bool _drag_buffer_fail{false};
	bool _auxvel_buffer_fail{false};

	// timestamps of latest in buffer saved measurement in microseconds
	uint64_t _time_last_imu{0};
	uint64_t _time_last_gps{0};
	uint64_t _time_last_mag{0};
	uint64_t _time_last_baro{0};
	uint64_t _time_last_range{0};
	uint64_t _time_last_airspeed{0};
	uint64_t _time_last_ext_vision{0};
	uint64_t _time_last_optflow{0};
	uint64_t _time_last_auxvel{0};
	//last time the baro ground effect compensation was turned on externally (uSec)
	uint64_t _time_last_gnd_effect_on{0};

	// Used to downsample magnetometer data
	Vector3f _mag_data_sum;
	uint8_t _mag_sample_count {0};
	uint64_t _mag_timestamp_sum {0};

	// Used to down sample barometer data
	float _baro_alt_sum {0.0f};			// summed pressure altitude readings (m)
	uint8_t _baro_sample_count {0};		// number of barometric altitude measurements summed
	uint64_t _baro_timestamp_sum {0};	// summed timestamp to provide the timestamp of the averaged sample

	fault_status_u _fault_status{};

	// allocate data buffers and initialize interface variables
	bool initialise_interface(uint64_t timestamp);

	// free buffer memory
	void unallocate_buffers();

	float _mag_declination_gps{0.0f};         // magnetic declination returned by the geo library using the last valid GPS position (rad)
	float _mag_inclination_gps{0.0f};	  // magnetic inclination returned by the geo library using the last valid GPS position (rad)
	float _mag_strength_gps{0.0f};	          // magnetic strength returned by the geo library using the last valid GPS position (T)

	// this is the current status of the filter control modes
	filter_control_status_u _control_status{};

	// this is the previous status of the filter control modes - used to detect mode transitions
	filter_control_status_u _control_status_prev{};

	// calculate the inverse rotation matrix from a quaternion rotation
	Matrix3f quat_to_invrotmat(const Quatf &quat);

	inline void setDragData();

	inline void computeVibrationMetric();
	inline bool checkIfVehicleAtRest(float dt);

	virtual float compensateBaroForDynamicPressure(const float baro_alt_uncompensated) = 0;

	void printBufferAllocationFailed(const char * buffer_name);
};
