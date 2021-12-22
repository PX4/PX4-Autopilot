/****************************************************************************
 *
 *   Copyright (c) 2015-2020 Estimation and Control Library (ECL). All rights reserved.
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

#ifndef EKF_ESTIMATOR_INTERFACE_H
#define EKF_ESTIMATOR_INTERFACE_H

#if defined(MODULE_NAME)
# define ECL_INFO PX4_DEBUG
# define ECL_WARN PX4_DEBUG
# define ECL_ERR  PX4_DEBUG
#else
# define ECL_INFO(X, ...) printf(X "\n", ##__VA_ARGS__)
# define ECL_WARN(X, ...) fprintf(stderr, X "\n", ##__VA_ARGS__)
# define ECL_ERR(X, ...) fprintf(stderr, X "\n", ##__VA_ARGS__)

#endif

#include "common.h"
#include "RingBuffer.h"
#include "imu_down_sampler.hpp"
#include "sensor_range_finder.hpp"
#include "utils.hpp"

#include <lib/geo/geo.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <mathlib/math/filter/AlphaFilter.hpp>

using namespace estimator;

class EstimatorInterface
{
public:
	// ask estimator for sensor data collection decision and do any preprocessing if required, returns true if not defined
	virtual bool collect_gps(const gps_message &gps) = 0;

	void setIMUData(const imuSample &imu_sample);

	/*
	Returns  following IMU vibration metrics in the following array locations
	0 : Gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
	1 : Gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
	2 : Accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)
	*/
	const Vector3f &getImuVibrationMetrics() const { return _vibe_metrics; }
	void setDeltaAngleConingMetric(float delta_angle_coning_metric) { _vibe_metrics(0) = delta_angle_coning_metric; }
	void setDeltaAngleHighFrequencyVibrationMetric(float delta_angle_vibration_metric) { _vibe_metrics(1) = delta_angle_vibration_metric; }
	void setDeltaVelocityHighFrequencyVibrationMetric(float delta_velocity_vibration_metric) { _vibe_metrics(2) = delta_velocity_vibration_metric; }

	void setMagData(const magSample &mag_sample);

	void setGpsData(const gps_message &gps);

	void setBaroData(const baroSample &baro_sample);

	void setAirspeedData(const airspeedSample &airspeed_sample);

	void setRangeData(const rangeSample &range_sample);

	// if optical flow sensor gyro delta angles are not available, set gyro_xyz vector fields to NaN and the EKF will use its internal delta angle data instead
	void setOpticalFlowData(const flowSample &flow);

	// set external vision position and attitude data
	void setExtVisionData(const extVisionSample &evdata);

	void setAuxVelData(const auxVelSample &auxvel_sample);

	// return a address to the parameters struct
	// in order to give access to the application
	parameters *getParamHandle() { return &_params; }

	// set vehicle landed status data
	void set_in_air_status(bool in_air)
	{
		if (!in_air) {
			_time_last_on_ground_us = _time_last_imu;

		} else {
			_time_last_in_air = _time_last_imu;
		}

		_control_status.flags.in_air = in_air;
	}

	// return true if the attitude is usable
	bool attitude_valid() const { return PX4_ISFINITE(_output_new.quat_nominal(0)) && _control_status.flags.tilt_align; }

	// get vehicle landed status data
	bool get_in_air_status() const { return _control_status.flags.in_air; }

	// get wind estimation status
	bool get_wind_status() const { return _control_status.flags.wind; }

	// set vehicle is fixed wing status
	void set_is_fixed_wing(bool is_fixed_wing) { _control_status.flags.fixed_wing = is_fixed_wing; }

	// set flag if synthetic sideslip measurement should be fused
	void set_fuse_beta_flag(bool fuse_beta) { _control_status.flags.fuse_beta = (fuse_beta && _control_status.flags.in_air); }

	// set flag if static pressure rise due to ground effect is expected
	// use _params.gnd_effect_deadzone to adjust for expected rise in static pressure
	// flag will clear after GNDEFFECT_TIMEOUT uSec
	void set_gnd_effect_flag(bool gnd_effect)
	{
		_control_status.flags.gnd_effect = gnd_effect;
		_time_last_gnd_effect_on = _time_last_imu;
	}

	// set air density used by the multi-rotor specific drag force fusion
	void set_air_density(float air_density) { _air_density = air_density; }

	// set sensor limitations reported by the rangefinder
	void set_rangefinder_limits(float min_distance, float max_distance)
	{
		_range_sensor.setLimits(min_distance, max_distance);
	}

	// set sensor limitations reported by the optical flow sensor
	void set_optical_flow_limits(float max_flow_rate, float min_distance, float max_distance)
	{
		_flow_max_rate = max_flow_rate;
		_flow_min_distance = min_distance;
		_flow_max_distance = max_distance;
	}

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
	bool inertial_dead_reckoning() const { return _is_dead_reckoning; }

	const matrix::Quatf &getQuaternion() const { return _output_new.quat_nominal; }

	// get the velocity of the body frame origin in local NED earth frame
	Vector3f getVelocity() const { return _output_new.vel - _vel_imu_rel_body_ned; }

	// get the velocity derivative in earth frame
	const Vector3f &getVelocityDerivative() const { return _vel_deriv; }

	// get the derivative of the vertical position of the body frame origin in local NED earth frame
	float getVerticalPositionDerivative() const { return _output_vert_new.vert_vel - _vel_imu_rel_body_ned(2); }

	// get the position of the body frame origin in local earth frame
	Vector3f getPosition() const
	{
		// rotate the position of the IMU relative to the boy origin into earth frame
		const Vector3f pos_offset_earth = _R_to_earth_now * _params.imu_pos_body;
		// subtract from the EKF position (which is at the IMU) to get position at the body origin
		return _output_new.pos - pos_offset_earth;
	}

	// Get the value of magnetic declination in degrees to be saved for use at the next startup
	// Returns true when the declination can be saved
	// At the next startup, set param.mag_declination_deg to the value saved
	bool get_mag_decl_deg(float *val) const
	{
		if (_NED_origin_initialised && (_params.mag_declination_source & MASK_SAVE_GEO_DECL)) {
			*val = math::degrees(_mag_declination_gps);
			return true;

		} else {
			return false;
		}
	}

	// get EKF mode status
	const filter_control_status_u &control_status() const { return _control_status; }
	const decltype(filter_control_status_u::flags) &control_status_flags() const { return _control_status.flags; }

	const filter_control_status_u &control_status_prev() const { return _control_status_prev; }
	const decltype(filter_control_status_u::flags) &control_status_prev_flags() const { return _control_status_prev.flags; }

	// get EKF internal fault status
	const fault_status_u &fault_status() const { return _fault_status; }
	const decltype(fault_status_u::flags) &fault_status_flags() const { return _fault_status.flags; }

	const innovation_fault_status_u &innov_check_fail_status() const { return _innov_check_fail_status; }
	const decltype(innovation_fault_status_u::flags) &innov_check_fail_status_flags() const { return _innov_check_fail_status.flags; }

	const warning_event_status_u &warning_event_status() const { return _warning_events; }
	const decltype(warning_event_status_u::flags) &warning_event_flags() const { return _warning_events.flags; }
	void clear_warning_events() { _warning_events.value = 0; }

	const information_event_status_u &information_event_status() const { return _information_events; }
	const decltype(information_event_status_u::flags) &information_event_flags() const { return _information_events.flags; }
	void clear_information_events() { _information_events.value = 0; }

	bool isVehicleAtRest() const { return _control_status.flags.vehicle_at_rest; }

	// Getter for the average imu update period in s
	float get_dt_imu_avg() const { return _dt_imu_avg; }

	// Getter for the imu sample on the delayed time horizon
	const imuSample &get_imu_sample_delayed() const { return _imu_sample_delayed; }

	// Getter for the baro sample on the delayed time horizon
	const baroSample &get_baro_sample_delayed() const { return _baro_sample_delayed; }

	const bool &global_origin_valid() const { return _NED_origin_initialised; }
	const MapProjection &global_origin() const { return _pos_ref; }

	void print_status();

	static constexpr unsigned FILTER_UPDATE_PERIOD_MS{10};	// ekf prediction period in milliseconds - this should ideally be an integer multiple of the IMU time delta
	static constexpr float FILTER_UPDATE_PERIOD_S{FILTER_UPDATE_PERIOD_MS * 0.001f};

protected:

	EstimatorInterface() = default;
	virtual ~EstimatorInterface() = default;

	virtual bool init(uint64_t timestamp) = 0;

	parameters _params{};		// filter parameters

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

	float _dt_imu_avg{0.0f};	// average imu update period in s

	imuSample _imu_sample_delayed{};	// captures the imu sample on the delayed time horizon

	// measurement samples capturing measurements on the delayed time horizon
	magSample _mag_sample_delayed{};
	baroSample _baro_sample_delayed{};
	gpsSample _gps_sample_delayed{};
	sensor::SensorRangeFinder _range_sensor{};
	airspeedSample _airspeed_sample_delayed{};
	flowSample _flow_sample_delayed{};
	extVisionSample _ev_sample_delayed{};
	dragSample _drag_sample_delayed{};
	dragSample _drag_down_sampled{};	// down sampled drag specific force data (filter prediction rate -> observation rate)
	auxVelSample _auxvel_sample_delayed{};

	float _air_density{CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C};		// air density (kg/m**3)

	// Sensor limitations
	float _flow_max_rate{0.0f}; ///< maximum angular flow rate that the optical flow sensor can measure (rad/s)
	float _flow_min_distance{0.0f};	///< minimum distance that the optical flow sensor can operate at (m)
	float _flow_max_distance{0.0f};	///< maximum distance that the optical flow sensor can operate at (m)

	// Output Predictor
	outputSample _output_new{};		// filter output on the non-delayed time horizon
	outputVert _output_vert_new{};		// vertical filter output on the non-delayed time horizon
	imuSample _newest_high_rate_imu_sample{};		// imu sample capturing the newest imu data
	Matrix3f _R_to_earth_now{};		// rotation matrix from body to earth frame at current time
	Vector3f _vel_imu_rel_body_ned{};		// velocity of IMU relative to body origin in NED earth frame
	Vector3f _vel_deriv{};		// velocity derivative at the IMU in NED earth frame (m/s/s)

	bool _imu_updated{false};      // true if the ekf should update (completed downsampling process)
	bool _initialised{false};      // true if the ekf interface instance (data buffering) is initialized

	bool _NED_origin_initialised{false};
	bool _gps_speed_valid{false};
	float _gps_origin_eph{0.0f}; // horizontal position uncertainty of the GPS origin
	float _gps_origin_epv{0.0f}; // vertical position uncertainty of the GPS origin
	MapProjection _pos_ref{}; // Contains WGS-84 position latitude and longitude of the EKF origin
	MapProjection _gps_pos_prev{}; // Contains WGS-84 position latitude and longitude of the previous GPS message
	float _gps_alt_prev{0.0f};	// height from the previous GPS message (m)
	float _gps_yaw_offset{0.0f};	// Yaw offset angle for dual GPS antennas used for yaw estimation (radians).

	// innovation consistency check monitoring ratios
	float _yaw_test_ratio{};		// yaw innovation consistency check ratio
	AlphaFilter<float>_yaw_signed_test_ratio_lpf{0.1f}; // average signed test ratio used to detect a bias in the state
	Vector3f _mag_test_ratio{};		// magnetometer XYZ innovation consistency check ratios
	Vector2f _gps_vel_test_ratio{};		// GPS velocity innovation consistency check ratios
	Vector2f _gps_pos_test_ratio{};		// GPS position innovation consistency check ratios
	Vector2f _ev_vel_test_ratio{};		// EV velocity innovation consistency check ratios
	Vector2f _ev_pos_test_ratio{};		// EV position innovation consistency check ratios
	Vector2f _aux_vel_test_ratio{};		// Auxiliary horizontal velocity innovation consistency check ratio
	float _baro_hgt_test_ratio{};	// baro height innovation consistency check ratios
	float _rng_hgt_test_ratio{};		// range finder height innovation consistency check ratios
	float _optflow_test_ratio{};		// Optical flow innovation consistency check ratio
	float _tas_test_ratio{};		// tas innovation consistency check ratio
	float _hagl_test_ratio{};		// height above terrain measurement innovation consistency check ratio
	float _beta_test_ratio{};		// sideslip innovation consistency check ratio
	Vector2f _drag_test_ratio{};		// drag innovation consistency check ratio
	innovation_fault_status_u _innov_check_fail_status{};

	bool _is_dead_reckoning{false};		// true if we are no longer fusing measurements that constrain horizontal velocity drift
	bool _deadreckon_time_exceeded{true};	// true if the horizontal nav solution has been deadreckoning for too long and is invalid
	bool _is_wind_dead_reckoning{false};	// true if we are navigationg reliant on wind relative measurements

	float _gps_drift_metrics[3] {};	// Array containing GPS drift metrics
					// [0] Horizontal position drift rate (m/s)
					// [1] Vertical position drift rate (m/s)
					// [2] Filtered horizontal velocity (m/s)
	uint64_t _time_last_move_detect_us{0};	// timestamp of last movement detection event in microseconds
	uint64_t _time_last_on_ground_us{0};	///< last time we were on the ground (uSec)
	uint64_t _time_last_in_air{0};		///< last time we were in air (uSec)
	bool _gps_drift_updated{false};	// true when _gps_drift_metrics has been updated and is ready for retrieval

	// data buffer instances
	RingBuffer<imuSample> _imu_buffer{12};           // buffer length 12 with default parameters
	RingBuffer<outputSample> _output_buffer{12};
	RingBuffer<outputVert> _output_vert_buffer{12};

	RingBuffer<gpsSample> _gps_buffer;
	RingBuffer<magSample> _mag_buffer;
	RingBuffer<baroSample> _baro_buffer;
	RingBuffer<rangeSample> _range_buffer;
	RingBuffer<airspeedSample> _airspeed_buffer;
	RingBuffer<flowSample> 	_flow_buffer;
	RingBuffer<extVisionSample> _ext_vision_buffer;
	RingBuffer<dragSample> _drag_buffer;
	RingBuffer<auxVelSample> _auxvel_buffer;

	// timestamps of latest in buffer saved measurement in microseconds
	uint64_t _time_last_imu{0};
	uint64_t _time_last_gps{0};
	uint64_t _time_last_mag{0}; ///< measurement time of last magnetomter sample (uSec)
	uint64_t _time_last_baro{0};
	uint64_t _time_last_range{0};
	uint64_t _time_last_airspeed{0};
	uint64_t _time_last_ext_vision{0};
	uint64_t _time_last_optflow{0};
	uint64_t _time_last_auxvel{0};
	//last time the baro ground effect compensation was turned on externally (uSec)
	uint64_t _time_last_gnd_effect_on{0};

	fault_status_u _fault_status{};

	// allocate data buffers and initialize interface variables
	bool initialise_interface(uint64_t timestamp);

	float _mag_declination_gps{NAN};         // magnetic declination returned by the geo library using the last valid GPS position (rad)
	float _mag_inclination_gps{NAN};	  // magnetic inclination returned by the geo library using the last valid GPS position (rad)
	float _mag_strength_gps{NAN};	          // magnetic strength returned by the geo library using the last valid GPS position (T)

	// this is the current status of the filter control modes
	filter_control_status_u _control_status{};

	// this is the previous status of the filter control modes - used to detect mode transitions
	filter_control_status_u _control_status_prev{};

	virtual float compensateBaroForDynamicPressure(const float baro_alt_uncompensated) const = 0;

	// these are used to record single frame events for external monitoring and should NOT be used for
	// state logic becasue they will be cleared externally after being read.
	warning_event_status_u _warning_events{};
	information_event_status_u _information_events{};

private:

	inline void setDragData(const imuSample &imu);

	inline void computeVibrationMetric(const imuSample &imu);
	inline bool checkIfVehicleAtRest(float dt, const imuSample &imu);

	void printBufferAllocationFailed(const char *buffer_name);

	ImuDownSampler _imu_down_sampler{FILTER_UPDATE_PERIOD_S};

	unsigned _min_obs_interval_us{0}; // minimum time interval between observations that will guarantee data is not lost (usec)

	// IMU vibration and movement monitoring
	Vector3f _vibe_metrics{};	// IMU vibration metrics
					// [0] Level of coning vibration in the IMU delta angles (rad^2)
					// [1] high frequency vibration level in the IMU delta angle data (rad)
					// [2] high frequency vibration level in the IMU delta velocity data (m/s)

	// Used to down sample barometer data
	uint64_t _baro_timestamp_sum{0};	// summed timestamp to provide the timestamp of the averaged sample
	float _baro_alt_sum{0.0f};			// summed pressure altitude readings (m)
	uint8_t _baro_sample_count{0};		// number of barometric altitude measurements summed

	// Used by the multi-rotor specific drag force fusion
	uint8_t _drag_sample_count{0};	// number of drag specific force samples assumulated at the filter prediction rate
	float _drag_sample_time_dt{0.0f};	// time integral across all samples used to form _drag_down_sampled (sec)

	// Used to downsample magnetometer data
	uint64_t _mag_timestamp_sum{0};
	Vector3f _mag_data_sum{};
	uint8_t _mag_sample_count{0};

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

};
#endif // !EKF_ESTIMATOR_INTERFACE_H
