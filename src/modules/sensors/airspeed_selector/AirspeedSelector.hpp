/****************************************************************************
 *
 *   Copyright (c) 2018-2026 PX4 Development Team. All rights reserved.
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

/**
 * @file AirspeedSelector.hpp
 *
 * This class provides a single airspeed_validated topic, containing indicated (IAS),
 * calibrated (CAS), true airspeed (TAS) and the information if the estimation currently
 * is invalid and if based sensor readings or on groundspeed minus windspeed.
 * Supporting the input of multiple "raw" airspeed inputs, this class automatically switches
 * to a valid sensor in case of failure detection. For failure detection as well as for
 * the estimation of a scale factor from IAS to CAS, it runs several wind estimators
 * and also publishes those.
 *
 * @author Silvan Fuhrer <silvan@auterion.com>
 */

#pragma once

#include "AirspeedValidator.hpp"

#include <drivers/drv_hrt.h>
#include <lib/wind_estimator/WindEstimator.hpp>
#include <matrix/math.hpp>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/airspeed/airspeed.h>
#include <lib/systemlib/mavlink_log.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/launch_detection_status.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/airspeed_wind.h>
#include <uORB/topics/flight_phase_estimation.h>

class AirspeedSelector : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	AirspeedSelector();
	~AirspeedSelector() override;

	void Start();
	void Stop();

private:
	static constexpr uint32_t SCHEDULE_INTERVAL{100_ms};	/**< The schedule interval in usec (10 Hz) */
	static constexpr float _kThrottleFilterTimeConstant{0.5f};

	void Run() override;

	static constexpr int MAX_NUM_AIRSPEED_SENSORS = 3; /**< Support max 3 airspeed sensors */
	enum class AirspeedSource : int {
		DISABLED = -1,
		GROUND_MINUS_WIND,
		SENSOR_1,
		SENSOR_2,
		SENSOR_3,
		SYNTHETIC
	};

	uORB::Publication<airspeed_validated_s> _airspeed_validated_pub {ORB_ID(airspeed_validated)};			/**< airspeed validated topic*/
	uORB::PublicationMulti<airspeed_wind_s> _wind_est_pub[MAX_NUM_AIRSPEED_SENSORS + 1] {{ORB_ID(airspeed_wind)}, {ORB_ID(airspeed_wind)}, {ORB_ID(airspeed_wind)}, {ORB_ID(airspeed_wind)}}; /**< wind estimate topic (for each airspeed validator + purely sideslip fusion) */
	orb_advert_t 	_mavlink_log_pub {nullptr}; 						/**< mavlink log topic*/

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _tecs_status_sub{ORB_ID(tecs_status)};
	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _position_setpoint_sub{ORB_ID(position_setpoint)};
	uORB::Subscription _launch_detection_status_sub{ORB_ID(launch_detection_status)};
	uORB::SubscriptionMultiArray<airspeed_s, MAX_NUM_AIRSPEED_SENSORS> _airspeed_subs{ORB_ID::airspeed};
	uORB::SubscriptionData<flight_phase_estimation_s> _flight_phase_estimation_sub{ORB_ID(flight_phase_estimation)};


	tecs_status_s _tecs_status {};
	estimator_status_s _estimator_status {};
	vehicle_acceleration_s _accel {};
	vehicle_air_data_s _vehicle_air_data {};
	vehicle_land_detected_s _vehicle_land_detected {};
	vehicle_local_position_s _vehicle_local_position {};
	vehicle_status_s _vehicle_status {};
	position_setpoint_s _position_setpoint {};

	WindEstimator	_wind_estimator_sideslip; /**< wind estimator instance only fusing sideslip */
	airspeed_wind_s _wind_estimate_sideslip {}; /**< wind estimate message for wind estimator instance only fusing sideslip */

	int32_t _number_of_airspeed_sensors{0}; /**<  number of airspeed sensors in use (detected during initialization)*/
	int32_t _prev_number_of_airspeed_sensors{0}; /**<  number of airspeed sensors in previous loop (to detect a new added sensor)*/
	AirspeedValidator _airspeed_validator[MAX_NUM_AIRSPEED_SENSORS] {}; /**< airspeedValidator instances (one for each sensor) */

	matrix::Quatf _q_att;
	hrt_abstime _time_now_usec{0};
	AirspeedSource _valid_airspeed_src{AirspeedSource::DISABLED};
	AirspeedSource _prev_airspeed_src{AirspeedSource::DISABLED};
	bool _initialized{false}; /**< module initialized*/
	bool _gnss_lpos_valid{false}; /**< local position (from GNSS) valid */
	bool _in_takeoff_situation{true}; /**< in takeoff situation (defined as not yet stall speed reached) */
	float _ground_minus_wind_TAS{NAN}; /**< true airspeed from groundspeed minus windspeed */
	float _ground_minus_wind_CAS{NAN}; /**< calibrated airspeed from groundspeed minus windspeed */
	bool _armed_prev{false};

	hrt_abstime _time_last_airspeed_update[MAX_NUM_AIRSPEED_SENSORS] {};

	perf_counter_t _perf_elapsed{};

	float _param_airspeed_scale[MAX_NUM_AIRSPEED_SENSORS] {}; /** array to save the airspeed scale params in */

	enum CheckTypeBits {
		CHECK_TYPE_ONLY_DATA_MISSING_BIT = (1 << 0),
		CHECK_TYPE_DATA_STUCK_BIT = (1 << 1),
		CHECK_TYPE_INNOVATION_BIT = (1 << 2),
		CHECK_TYPE_LOAD_FACTOR_BIT = (1 << 3),
		CHECK_TYPE_FIRST_PRINCIPLE_BIT = (1 << 4)
	};


	param_t _param_handle_pitch_sp_offset{PARAM_INVALID};
	float _param_pitch_sp_offset{0.0f};
	param_t _param_handle_fw_thr_max{PARAM_INVALID};
	float _param_fw_thr_max{0.0f};

	AlphaFilter<float> _throttle_filtered{_kThrottleFilterTimeConstant};
	uint64_t _t_last_throttle_fw{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::ASPD_WIND_NSD>) _param_aspd_wind_nsd,
		(ParamFloat<px4::params::ASPD_SCALE_NSD>) _param_aspd_scale_nsd,
		(ParamFloat<px4::params::ASPD_TAS_NOISE>) _param_west_tas_noise,
		(ParamFloat<px4::params::ASPD_BETA_NOISE>) _param_west_beta_noise,
		(ParamInt<px4::params::ASPD_TAS_GATE>) _param_west_tas_gate,
		(ParamInt<px4::params::ASPD_BETA_GATE>) _param_west_beta_gate,
		(ParamInt<px4::params::ASPD_SCALE_APPLY>) _param_aspd_scale_apply,
		(ParamFloat<px4::params::ASPD_SCALE_1>) _param_airspeed_scale_1,
		(ParamFloat<px4::params::ASPD_SCALE_2>) _param_airspeed_scale_2,
		(ParamFloat<px4::params::ASPD_SCALE_3>) _param_airspeed_scale_3,
		(ParamInt<px4::params::ASPD_PRIMARY>) _param_airspeed_primary_index,
		(ParamInt<px4::params::ASPD_DO_CHECKS>) _param_airspeed_checks_on,
		(ParamInt<px4::params::ASPD_FALLBACK>) _param_airspeed_fallback,

		(ParamFloat<px4::params::ASPD_FS_INNOV>) _tas_innov_threshold, /**< innovation check threshold */
		(ParamFloat<px4::params::ASPD_FS_INTEG>) _tas_innov_integ_threshold, /**< innovation check integrator threshold */
		(ParamFloat<px4::params::ASPD_FS_T_STOP>) _checks_fail_delay, /**< delay to declare airspeed invalid */
		(ParamFloat<px4::params::ASPD_FS_T_START>) _checks_clear_delay, /**<  delay to declare airspeed valid again */

		(ParamFloat<px4::params::ASPD_WERR_THR>) _param_wind_sigma_max_synth_tas,
		(ParamFloat<px4::params::ASPD_FP_T_WINDOW>) _aspd_fp_t_window,

		// external parameters
		(ParamFloat<px4::params::FW_AIRSPD_STALL>) _param_fw_airspd_stall,
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,
		(ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max,
		(ParamFloat<px4::params::FW_THR_ASPD_MIN>) _param_fw_thr_aspd_min,
		(ParamFloat<px4::params::FW_THR_TRIM>) _param_fw_thr_trim,
		(ParamFloat<px4::params::FW_THR_ASPD_MAX>) _param_fw_thr_aspd_max
	)

	void init(); 	/**< initialization of the airspeed validator instances */
	void check_for_connected_airspeed_sensors(); /**< check for airspeed sensors (airspeed topics) and get _number_of_airspeed_sensors */
	void update_params(); /**< update parameters */
	void poll_topics(); /**< poll all topics required beside airspeed (e.g. current temperature) */
	void update_wind_estimator_sideslip(); /**< update the wind estimator instance only fusing sideslip */
	void update_ground_minus_wind_airspeed(); /**< update airspeed estimate based on groundspeed minus windspeed */
	void select_airspeed_and_publish(); /**< select airspeed sensor (or groundspeed-windspeed) */
	float get_synthetic_airspeed(float throttle);
	void update_throttle_filter(hrt_abstime t_now);
};
