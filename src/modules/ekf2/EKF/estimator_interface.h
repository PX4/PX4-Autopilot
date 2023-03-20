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
#include <px4_platform_common/log.h>
# define ECL_INFO PX4_DEBUG
# define ECL_WARN PX4_DEBUG
# define ECL_ERR  PX4_DEBUG
# define ECL_DEBUG PX4_DEBUG
#else
# define ECL_INFO(X, ...) printf(X "\n", ##__VA_ARGS__)
# define ECL_WARN(X, ...) fprintf(stderr, X "\n", ##__VA_ARGS__)
# define ECL_ERR(X, ...) fprintf(stderr, X "\n", ##__VA_ARGS__)

# if defined(DEBUG_BUILD)
#  define ECL_DEBUG(X, ...) fprintf(stderr, X "\n", ##__VA_ARGS__)
# else
#  define ECL_DEBUG(X, ...)
#endif

#endif

#include "common.h"
#include "RingBuffer.h"
#include "imu_down_sampler.hpp"
#include "utils.hpp"
#include "output_predictor.h"

#if defined(CONFIG_EKF2_RANGE_FINDER)
# include "range_finder_consistency_check.hpp"
# include "sensor_range_finder.hpp"
#endif // CONFIG_EKF2_RANGE_FINDER

#include <lib/geo/geo.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <mathlib/math/filter/AlphaFilter.hpp>

using namespace estimator;

class EstimatorInterface
{
public:
	// ask estimator for sensor data collection decision and do any preprocessing if required, returns true if not defined
	virtual bool collect_gps(const gpsMessage &gps) = 0;

	void setIMUData(const imuSample &imu_sample);

	void setMagData(const magSample &mag_sample);

	void setGpsData(const gpsMessage &gps);

	void setBaroData(const baroSample &baro_sample);

#if defined(CONFIG_EKF2_AIRSPEED)
	void setAirspeedData(const airspeedSample &airspeed_sample);
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_RANGE_FINDER)
	void setRangeData(const rangeSample &range_sample);

	// set sensor limitations reported by the rangefinder
	void set_rangefinder_limits(float min_distance, float max_distance)
	{
		_range_sensor.setLimits(min_distance, max_distance);
	}

	const rangeSample &get_rng_sample_delayed() { return *(_range_sensor.getSampleAddress()); }
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// if optical flow sensor gyro delta angles are not available, set gyro_xyz vector fields to NaN and the EKF will use its internal delta angle data instead
	void setOpticalFlowData(const flowSample &flow);

	// set sensor limitations reported by the optical flow sensor
	void set_optical_flow_limits(float max_flow_rate, float min_distance, float max_distance)
	{
		_flow_max_rate = max_flow_rate;
		_flow_min_distance = min_distance;
		_flow_max_distance = max_distance;
	}
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// set external vision position and attitude data
	void setExtVisionData(const extVisionSample &evdata);
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUXVEL)
	void setAuxVelData(const auxVelSample &auxvel_sample);
#endif // CONFIG_EKF2_AUXVEL

	void setSystemFlagData(const systemFlagUpdate &system_flags);

	// return a address to the parameters struct
	// in order to give access to the application
	parameters *getParamHandle() { return &_params; }

	// set vehicle landed status data
	void set_in_air_status(bool in_air)
	{
		if (!in_air) {
			_time_last_on_ground_us = _time_delayed_us;

		} else {
			_time_last_in_air = _time_delayed_us;
		}

		_control_status.flags.in_air = in_air;
	}

	void set_vehicle_at_rest(bool at_rest) { _control_status.flags.vehicle_at_rest = at_rest; }

	// return true if the attitude is usable
	bool attitude_valid() const { return _control_status.flags.tilt_align; }

	// get vehicle landed status data
	bool get_in_air_status() const { return _control_status.flags.in_air; }

	// get wind estimation status
	bool get_wind_status() const { return _control_status.flags.wind; }

	// set vehicle is fixed wing status
	void set_is_fixed_wing(bool is_fixed_wing) { _control_status.flags.fixed_wing = is_fixed_wing; }

	// set flag if static pressure rise due to ground effect is expected
	// use _params.gnd_effect_deadzone to adjust for expected rise in static pressure
	// flag will clear after GNDEFFECT_TIMEOUT uSec
	void set_gnd_effect()
	{
		_control_status.flags.gnd_effect = true;
		_time_last_gnd_effect_on = _time_delayed_us;
	}

	// set air density used by the multi-rotor specific drag force fusion
	void set_air_density(float air_density) { _air_density = air_density; }

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
	bool isVerticalAidingActive() const;

	int getNumberOfActiveHorizontalAidingSources() const;

	bool isOtherSourceOfVerticalPositionAidingThan(bool aiding_flag) const;
	bool isVerticalPositionAidingActive() const;
	bool isOnlyActiveSourceOfVerticalPositionAiding(const bool aiding_flag) const;
	int getNumberOfActiveVerticalPositionAidingSources() const;

	bool isVerticalVelocityAidingActive() const;
	int getNumberOfActiveVerticalVelocityAidingSources() const;

	const matrix::Quatf &getQuaternion() const { return _output_predictor.getQuaternion(); }
	Vector3f getVelocity() const { return _output_predictor.getVelocity(); }
	const Vector3f &getVelocityDerivative() const { return _output_predictor.getVelocityDerivative(); }
	float getVerticalPositionDerivative() const { return _output_predictor.getVerticalPositionDerivative(); }
	Vector3f getPosition() const { return _output_predictor.getPosition(); }
	const Vector3f &getOutputTrackingError() const { return _output_predictor.getOutputTrackingError(); }

	// Get the value of magnetic declination in degrees to be saved for use at the next startup
	// Returns true when the declination can be saved
	// At the next startup, set param.mag_declination_deg to the value saved
	bool get_mag_decl_deg(float *val) const
	{
		if (_NED_origin_initialised && (_params.mag_declination_source & GeoDeclinationMask::SAVE_GEO_DECL)) {
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

	// Getter for the average EKF update period in s
	float get_dt_ekf_avg() const { return _dt_ekf_avg; }

	// Getters for samples on the delayed time horizon
	const imuSample &get_imu_sample_delayed() const { return _imu_buffer.get_oldest(); }
	const uint64_t &time_delayed_us() const { return _time_delayed_us; }

	const gpsSample &get_gps_sample_delayed() const { return _gps_sample_delayed; }

	const bool &global_origin_valid() const { return _NED_origin_initialised; }
	const MapProjection &global_origin() const { return _pos_ref; }

	void print_status();

	float gps_horizontal_position_drift_rate_m_s() const { return _gps_horizontal_position_drift_rate_m_s; }
	float gps_vertical_position_drift_rate_m_s() const { return _gps_vertical_position_drift_rate_m_s; }
	float gps_filtered_horizontal_velocity_m_s() const { return _gps_filtered_horizontal_velocity_m_s; }

	OutputPredictor &output_predictor() { return _output_predictor; };

protected:

	EstimatorInterface() = default;
	virtual ~EstimatorInterface();

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

	float _dt_ekf_avg{0.010f}; ///< average update rate of the ekf in s

	uint64_t _time_delayed_us{0}; // captures the imu sample on the delayed time horizon
	uint64_t _time_latest_us{0}; // imu sample capturing the newest imu data

	OutputPredictor _output_predictor{};

	// measurement samples capturing measurements on the delayed time horizon
	gpsSample _gps_sample_delayed{};


#if defined(CONFIG_EKF2_AIRSPEED)
	airspeedSample _airspeed_sample_delayed{};
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	extVisionSample _ev_sample_prev{};
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_RANGE_FINDER)
	RingBuffer<rangeSample> *_range_buffer{nullptr};
	uint64_t _time_last_range_buffer_push{0};

	sensor::SensorRangeFinder _range_sensor{};
	RangeFinderConsistencyCheck _rng_consistency_check;
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	RingBuffer<flowSample> 	*_flow_buffer{nullptr};

	flowSample _flow_sample_delayed{};

	// Sensor limitations
	float _flow_max_rate{1.0f}; ///< maximum angular flow rate that the optical flow sensor can measure (rad/s)
	float _flow_min_distance{0.0f};	///< minimum distance that the optical flow sensor can operate at (m)
	float _flow_max_distance{10.f};	///< maximum distance that the optical flow sensor can operate at (m)
#endif // CONFIG_EKF2_OPTICAL_FLOW

	float _air_density{CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C};		// air density (kg/m**3)

	bool _imu_updated{false};      // true if the ekf should update (completed downsampling process)
	bool _initialised{false};      // true if the ekf interface instance (data buffering) is initialized

	bool _NED_origin_initialised{false};
	float _gps_origin_eph{0.0f}; // horizontal position uncertainty of the GPS origin
	float _gps_origin_epv{0.0f}; // vertical position uncertainty of the GPS origin
	MapProjection _pos_ref{}; // Contains WGS-84 position latitude and longitude of the EKF origin
	MapProjection _gps_pos_prev{}; // Contains WGS-84 position latitude and longitude of the previous GPS message
	float _gps_alt_prev{0.0f};	// height from the previous GPS message (m)
#if defined(CONFIG_EKF2_GNSS_YAW)
	float _gps_yaw_offset{0.0f};	// Yaw offset angle for dual GPS antennas used for yaw estimation (radians).
	// innovation consistency check monitoring ratios
	AlphaFilter<float> _gnss_yaw_signed_test_ratio_lpf{0.1f}; // average signed test ratio used to detect a bias in the state
	uint64_t _time_last_gps_yaw_buffer_push{0};
#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_DRAG_FUSION)
	RingBuffer<dragSample> *_drag_buffer{nullptr};
	dragSample _drag_down_sampled{};	// down sampled drag specific force data (filter prediction rate -> observation rate)
	Vector2f _drag_test_ratio{};		// drag innovation consistency check ratio
#endif // CONFIG_EKF2_DRAG_FUSION

	innovation_fault_status_u _innov_check_fail_status{};

	bool _horizontal_deadreckon_time_exceeded{true};
	bool _vertical_position_deadreckon_time_exceeded{true};
	bool _vertical_velocity_deadreckon_time_exceeded{true};

	float _gps_horizontal_position_drift_rate_m_s{NAN}; // Horizontal position drift rate (m/s)
	float _gps_vertical_position_drift_rate_m_s{NAN};   // Vertical position drift rate (m/s)
	float _gps_filtered_horizontal_velocity_m_s{NAN};   // Filtered horizontal velocity (m/s)

	uint64_t _time_last_on_ground_us{0};	///< last time we were on the ground (uSec)
	uint64_t _time_last_in_air{0};		///< last time we were in air (uSec)

	// data buffer instances
	static constexpr uint8_t kBufferLengthDefault = 12;
	RingBuffer<imuSample> _imu_buffer{kBufferLengthDefault};

	RingBuffer<gpsSample> *_gps_buffer{nullptr};
	RingBuffer<magSample> *_mag_buffer{nullptr};
	RingBuffer<baroSample> *_baro_buffer{nullptr};

#if defined(CONFIG_EKF2_AIRSPEED)
	RingBuffer<airspeedSample> *_airspeed_buffer{nullptr};
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	RingBuffer<extVisionSample> *_ext_vision_buffer{nullptr};
	uint64_t _time_last_ext_vision_buffer_push{0};
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_AUXVEL)
	RingBuffer<auxVelSample> *_auxvel_buffer{nullptr};
#endif // CONFIG_EKF2_AUXVEL
	RingBuffer<systemFlagUpdate> *_system_flag_buffer{nullptr};

	uint64_t _time_last_gps_buffer_push{0};
	uint64_t _time_last_mag_buffer_push{0};
	uint64_t _time_last_baro_buffer_push{0};

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

	// these are used to record single frame events for external monitoring and should NOT be used for
	// state logic becasue they will be cleared externally after being read.
	warning_event_status_u _warning_events{};
	information_event_status_u _information_events{};

private:

#if defined(CONFIG_EKF2_DRAG_FUSION)
	void setDragData(const imuSample &imu);

	// Used by the multi-rotor specific drag force fusion
	uint8_t _drag_sample_count{0};	// number of drag specific force samples assumulated at the filter prediction rate
	float _drag_sample_time_dt{0.0f};	// time integral across all samples used to form _drag_down_sampled (sec)
#endif // CONFIG_EKF2_DRAG_FUSION

	void printBufferAllocationFailed(const char *buffer_name);

	ImuDownSampler _imu_down_sampler{_params.filter_update_interval_us};

	unsigned _min_obs_interval_us{0}; // minimum time interval between observations that will guarantee data is not lost (usec)
};
#endif // !EKF_ESTIMATOR_INTERFACE_H
