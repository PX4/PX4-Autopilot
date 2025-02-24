/****************************************************************************
 *
 *   Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
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

#include "AirspeedValidator.hpp"

#include <drivers/drv_hrt.h>
#include <lib/wind_estimator/WindEstimator.hpp>
#include <matrix/math.hpp>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/airspeed/airspeed.h>
#include <lib/systemlib/mavlink_log.h>

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
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/airspeed_wind.h>

using namespace time_literals;

static constexpr uint32_t SCHEDULE_INTERVAL{100_ms};	/**< The schedule interval in usec (10 Hz) */

using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

class AirspeedModule : public ModuleBase<AirspeedModule>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:

	AirspeedModule();

	~AirspeedModule() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

private:

	void Run() override;

	static constexpr int MAX_NUM_AIRSPEED_SENSORS = 3; /**< Support max 3 airspeed sensors */
	enum airspeed_index {
		DISABLED_INDEX = -1,
		GROUND_MINUS_WIND_INDEX,
		FIRST_SENSOR_INDEX,
		SECOND_SENSOR_INDEX,
		THIRD_SENSOR_INDEX
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
	uORB::Subscription _vtol_vehicle_status_sub{ORB_ID(vtol_vehicle_status)};
	uORB::Subscription _position_setpoint_sub{ORB_ID(position_setpoint)};
	uORB::Subscription _launch_detection_status_sub{ORB_ID(launch_detection_status)};
	uORB::SubscriptionMultiArray<airspeed_s, MAX_NUM_AIRSPEED_SENSORS> _airspeed_subs{ORB_ID::airspeed};


	tecs_status_s _tecs_status {};
	estimator_status_s _estimator_status {};
	vehicle_acceleration_s _accel {};
	vehicle_air_data_s _vehicle_air_data {};
	vehicle_land_detected_s _vehicle_land_detected {};
	vehicle_local_position_s _vehicle_local_position {};
	vehicle_status_s _vehicle_status {};
	vtol_vehicle_status_s _vtol_vehicle_status {};
	position_setpoint_s _position_setpoint {};

	WindEstimator	_wind_estimator_sideslip; /**< wind estimator instance only fusing sideslip */
	airspeed_wind_s _wind_estimate_sideslip {}; /**< wind estimate message for wind estimator instance only fusing sideslip */

	int32_t _number_of_airspeed_sensors{0}; /**<  number of airspeed sensors in use (detected during initialization)*/
	int32_t _prev_number_of_airspeed_sensors{0}; /**<  number of airspeed sensors in previous loop (to detect a new added sensor)*/
	AirspeedValidator _airspeed_validator[MAX_NUM_AIRSPEED_SENSORS] {}; /**< airspeedValidator instances (one for each sensor) */

	matrix::Quatf _q_att;
	hrt_abstime _time_now_usec{0};
	int _valid_airspeed_index{-2}; /**< index of currently chosen (valid) airspeed sensor */
	int _prev_airspeed_index{-2}; /**< previously chosen airspeed sensor index */
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
		(ParamInt<px4::params::ASPD_FALLBACK_GW>) _param_airspeed_fallback_gw,

		(ParamFloat<px4::params::ASPD_FS_INNOV>) _tas_innov_threshold, /**< innovation check threshold */
		(ParamFloat<px4::params::ASPD_FS_INTEG>) _tas_innov_integ_threshold, /**< innovation check integrator threshold */
		(ParamFloat<px4::params::ASPD_FS_T_STOP>) _checks_fail_delay, /**< delay to declare airspeed invalid */
		(ParamFloat<px4::params::ASPD_FS_T_START>) _checks_clear_delay, /**<  delay to declare airspeed valid again */

		(ParamFloat<px4::params::ASPD_WERR_THR>) _param_wind_sigma_max_synth_tas,
		(ParamFloat<px4::params::ASPD_FP_T_WINDOW>) _aspd_fp_t_window,

		// external parameters
		(ParamFloat<px4::params::FW_AIRSPD_STALL>) _param_fw_airspd_stall,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim
	)

	void 		init(); 	/**< initialization of the airspeed validator instances */
	void 		check_for_connected_airspeed_sensors(); /**< check for airspeed sensors (airspeed topics) and get _number_of_airspeed_sensors */
	void		update_params(); /**< update parameters */
	void 		poll_topics(); /**< poll all topics required beside airspeed (e.g. current temperature) */
	void 		update_wind_estimator_sideslip(); /**< update the wind estimator instance only fusing sideslip */
	void		update_ground_minus_wind_airspeed(); /**< update airspeed estimate based on groundspeed minus windspeed */
	void 		select_airspeed_and_publish(); /**< select airspeed sensor (or groundspeed-windspeed) */

};

AirspeedModule::AirspeedModule():
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_param_handle_pitch_sp_offset = param_find("FW_PSP_OFF");
	_param_handle_fw_thr_max = param_find("FW_THR_MAX");
	// initialise parameters
	update_params();

	_perf_elapsed = perf_alloc(PC_ELAPSED, MODULE_NAME": elapsed");
	_airspeed_validated_pub.advertise();
	_wind_est_pub[0].advertise();
	_wind_est_pub[1].advertise();
}

AirspeedModule::~AirspeedModule()
{
	ScheduleClear();

	perf_free(_perf_elapsed);
}

int
AirspeedModule::task_spawn(int argc, char *argv[])
{
	AirspeedModule *dev = new AirspeedModule();

	// check if the trampoline is called for the first time
	if (!dev) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(dev);

	dev->ScheduleOnInterval(SCHEDULE_INTERVAL, 10000);
	_task_id = task_id_is_work_queue;
	return PX4_OK;

}
void
AirspeedModule::init()
{
	check_for_connected_airspeed_sensors();

	// Set the default sensor
	if (_param_airspeed_primary_index.get() > _number_of_airspeed_sensors) {
		// constrain the index to the number of sensors connected
		_valid_airspeed_index = math::min(_param_airspeed_primary_index.get(), _number_of_airspeed_sensors);

		if (_number_of_airspeed_sensors == 0) {
			mavlink_log_info(&_mavlink_log_pub, "No airspeed sensor detected. Switch to non-airspeed mode.\t");
			events::send(events::ID("airspeed_selector_switch"), events::Log::Info,
				     "No airspeed sensor detected, switching to non-airspeed mode");

		} else {
			mavlink_log_info(&_mavlink_log_pub, "Primary airspeed index bigger than number connected sensors. Take last sensor.\t");
			events::send(events::ID("airspeed_selector_prim_too_high"), events::Log::Info,
				     "Primary airspeed index bigger than number connected sensors, taking last sensor");
		}

	} else {
		// set index to the one provided in the parameter ASPD_PRIMARY
		_valid_airspeed_index =	_param_airspeed_primary_index.get();
	}

	_prev_airspeed_index = _valid_airspeed_index; // used to detect a sensor switching
}

void
AirspeedModule::check_for_connected_airspeed_sensors()
{
	// check for new connected airspeed sensor
	int detected_airspeed_sensors = 0;

	if (_param_airspeed_primary_index.get() > 0) {

		for (int i = 0; i < _airspeed_subs.size(); i++) {
			if (!_airspeed_subs[i].advertised()) {
				break;
			}

			detected_airspeed_sensors = i + 1;
		}

	} else {
		// user has selected groundspeed-windspeed as primary source, or disabled airspeed
		detected_airspeed_sensors = 0;
	}

	_number_of_airspeed_sensors = detected_airspeed_sensors;
}


void
AirspeedModule::Run()
{
	_time_now_usec = hrt_absolute_time(); // hrt time of the current cycle

	// do not run the airspeed selector until 2s after system boot,
	// as data from airspeed sensor	and estimator may not be valid yet
	if (_time_now_usec < 2_s) {
		return;
	}

	perf_begin(_perf_elapsed);

	if (!_initialized) {
		init(); // initialize airspeed validator instances

		for (int i = 0; i < MAX_NUM_AIRSPEED_SENSORS; i++) {
			_airspeed_validator[i].set_CAS_scale_validated(_param_airspeed_scale[i]);
			_airspeed_validator[i].set_scale_init(_param_airspeed_scale[i]);
		}

		_initialized = true;
	}

	parameter_update_s update;

	if (_parameter_update_sub.update(&update)) {
		update_params();
	}

	const bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

	// check for new connected airspeed sensors as long as we're disarmed
	if (!armed) {
		check_for_connected_airspeed_sensors();
	}

	poll_topics();
	update_wind_estimator_sideslip();
	update_ground_minus_wind_airspeed();

	if (_number_of_airspeed_sensors > 0) {

		// disable checks if not a fixed-wing or the vehicle is landing/landed, as then airspeed can fall below stall speed
		// and wind estimate isn't accurate anymore. Even better would be to have a reliable "ground_contact" detection
		// for fixed-wing landings.
		const bool in_air_fixed_wing = !_vehicle_land_detected.landed &&
					       _position_setpoint.type != position_setpoint_s::SETPOINT_TYPE_LAND &&
					       _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

		const matrix::Vector3f vI(_vehicle_local_position.vx, _vehicle_local_position.vy, _vehicle_local_position.vz);

		// Prepare data for airspeed_validator
		struct airspeed_validator_update_data input_data = {};
		input_data.timestamp = _time_now_usec;
		input_data.ground_velocity = vI;
		input_data.gnss_valid = _gnss_lpos_valid;
		input_data.lpos_evh = _vehicle_local_position.evh;
		input_data.lpos_evv = _vehicle_local_position.evv;
		input_data.q_att = _q_att;
		input_data.air_pressure_pa = _vehicle_air_data.baro_pressure_pa;
		input_data.accel_z = _accel.xyz[2];
		input_data.vel_test_ratio = _estimator_status.vel_test_ratio;
		input_data.hdg_test_ratio = _estimator_status.hdg_test_ratio;
		input_data.tecs_timestamp = _tecs_status.timestamp;
		input_data.fixed_wing_tecs_throttle = _tecs_status.throttle_sp;
		input_data.fixed_wing_tecs_throttle_trim = _tecs_status.throttle_trim;

		// iterate through all airspeed sensors, poll new data from them and update their validators
		for (int i = 0; i < _number_of_airspeed_sensors; i++) {

			// poll raw airspeed topic of the i-th sensor
			airspeed_s airspeed_raw;

			if (_airspeed_subs[i].update(&airspeed_raw)) {

				input_data.airspeed_indicated_raw = airspeed_raw.indicated_airspeed_m_s;
				input_data.airspeed_true_raw = airspeed_raw.true_airspeed_m_s;
				input_data.airspeed_timestamp = airspeed_raw.timestamp;
				input_data.air_temperature_celsius = _vehicle_air_data.ambient_temperature;

				if (_in_takeoff_situation) {
					// set flag to false if either speed is above stall speed,
					// or launch detection + land detection indicate flying
					const bool speed_above_stall = airspeed_raw.indicated_airspeed_m_s > _param_fw_airspd_stall.get();
					airspeed_raw.indicated_airspeed_m_s > _param_fw_airspd_stall.get()
					|| (PX4_ISFINITE(_ground_minus_wind_CAS) && _ground_minus_wind_CAS > _param_fw_airspd_stall.get());

					launch_detection_status_s launch_detection_status{};
					_launch_detection_status_sub.copy(&launch_detection_status);
					const bool launch_detection_flying = launch_detection_status.launch_detection_state ==
									     launch_detection_status_s::STATE_FLYING
									     && !_vehicle_land_detected.landed;
					_in_takeoff_situation = !(speed_above_stall || launch_detection_flying);

				} else {
					// reset takeoff_situation to true when not in air and not in fixed-wing mode
					_in_takeoff_situation = !in_air_fixed_wing;
				}

				input_data.in_fixed_wing_flight = (in_air_fixed_wing && !_in_takeoff_situation);

				// push input data into airspeed validator
				_airspeed_validator[i].update_airspeed_validator(input_data);

				_time_last_airspeed_update[i] = _time_now_usec;

			} else if (_time_now_usec - _time_last_airspeed_update[i] > 1_s) {
				// declare airspeed invalid if more then 1s since last raw airspeed update
				_airspeed_validator[i].reset_airspeed_to_invalid(_time_now_usec);

			}

			// save estimated airspeed scale after disarm if airspeed is valid and scale has changed
			if (!armed && _armed_prev) {
				if (_param_aspd_scale_apply.get() > 0 && _airspeed_validator[i].get_airspeed_valid()
				    && fabsf(_airspeed_validator[i].get_CAS_scale_validated() - _param_airspeed_scale[i]) > FLT_EPSILON) {

					mavlink_log_info(&_mavlink_log_pub, "Airspeed sensor Nr. %d ASPD_SCALE updated: %.4f --> %.4f", i + 1,
							 (double)_param_airspeed_scale[i],
							 (double)_airspeed_validator[i].get_CAS_scale_validated());

					switch (i) {
					case 0:
						_param_airspeed_scale_1.set(_airspeed_validator[i].get_CAS_scale_validated());
						_param_airspeed_scale_1.commit_no_notification();
						break;

					case 1:
						_param_airspeed_scale_2.set(_airspeed_validator[i].get_CAS_scale_validated());
						_param_airspeed_scale_2.commit_no_notification();
						break;

					case 2:
						_param_airspeed_scale_3.set(_airspeed_validator[i].get_CAS_scale_validated());
						_param_airspeed_scale_3.commit_no_notification();
						break;
					}

				}

				_airspeed_validator[i].set_scale_init(_param_airspeed_scale[i]);
			}
		}
	}

	select_airspeed_and_publish();

	_armed_prev = armed;

	perf_end(_perf_elapsed);

	if (should_exit()) {
		exit_and_cleanup();
	}
}

void AirspeedModule::update_params()
{
	updateParams();

	if (_param_handle_pitch_sp_offset != PARAM_INVALID) {
		param_get(_param_handle_pitch_sp_offset, &_param_pitch_sp_offset);
	}

	if (_param_handle_fw_thr_max != PARAM_INVALID) {
		param_get(_param_handle_fw_thr_max, &_param_fw_thr_max);
	}

	_param_airspeed_scale[0] = _param_airspeed_scale_1.get();
	_param_airspeed_scale[1] = _param_airspeed_scale_2.get();
	_param_airspeed_scale[2] = _param_airspeed_scale_3.get();

	_wind_estimator_sideslip.set_wind_process_noise_spectral_density(_param_aspd_wind_nsd.get());
	_wind_estimator_sideslip.set_tas_scale_process_noise_spectral_density(_param_aspd_scale_nsd.get());
	_wind_estimator_sideslip.set_tas_noise(_param_west_tas_noise.get());
	_wind_estimator_sideslip.set_beta_noise(_param_west_beta_noise.get());
	_wind_estimator_sideslip.set_tas_gate(_param_west_tas_gate.get());
	_wind_estimator_sideslip.set_beta_gate(_param_west_beta_gate.get());

	for (int i = 0; i < MAX_NUM_AIRSPEED_SENSORS; i++) {
		_airspeed_validator[i].set_wind_estimator_wind_process_noise_spectral_density(_param_aspd_wind_nsd.get());
		_airspeed_validator[i].set_wind_estimator_tas_scale_process_noise_spectral_density(_param_aspd_scale_nsd.get());
		_airspeed_validator[i].set_wind_estimator_tas_noise(_param_west_tas_noise.get());
		_airspeed_validator[i].set_wind_estimator_beta_noise(_param_west_beta_noise.get());
		_airspeed_validator[i].set_wind_estimator_tas_gate(_param_west_tas_gate.get());
		_airspeed_validator[i].set_wind_estimator_beta_gate(_param_west_beta_gate.get());

		_airspeed_validator[i].set_tas_scale_apply(_param_aspd_scale_apply.get());
		_airspeed_validator[i].set_wind_estimator_tas_scale_init(_param_airspeed_scale[i]);

		_airspeed_validator[i].set_tas_innov_threshold(_tas_innov_threshold.get());
		_airspeed_validator[i].set_tas_innov_integ_threshold(_tas_innov_integ_threshold.get());
		_airspeed_validator[i].set_checks_fail_delay(_checks_fail_delay.get());
		_airspeed_validator[i].set_checks_clear_delay(_checks_clear_delay.get());
		_airspeed_validator[i].set_airspeed_stall(_param_fw_airspd_stall.get());

		_airspeed_validator[i].set_enable_data_stuck_check(_param_airspeed_checks_on.get() &
				CheckTypeBits::CHECK_TYPE_DATA_STUCK_BIT);
		_airspeed_validator[i].set_enable_innovation_check(_param_airspeed_checks_on.get() &
				CheckTypeBits::CHECK_TYPE_INNOVATION_BIT);
		_airspeed_validator[i].set_enable_load_factor_check(_param_airspeed_checks_on.get() &
				CheckTypeBits::CHECK_TYPE_LOAD_FACTOR_BIT);
		_airspeed_validator[i].set_enable_first_principle_check(_param_airspeed_checks_on.get() &
				CheckTypeBits::CHECK_TYPE_FIRST_PRINCIPLE_BIT);
		_airspeed_validator[i].set_psp_off_param(math::radians(_param_pitch_sp_offset));
		_airspeed_validator[i].set_throttle_max_param(_param_fw_thr_max);
		_airspeed_validator[i].set_fp_t_window(_aspd_fp_t_window.get());
	}
}

void AirspeedModule::poll_topics()
{
	// use primary estimator_status
	if (_estimator_selector_status_sub.updated()) {
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			if (estimator_selector_status.primary_instance != _estimator_status_sub.get_instance()) {
				_estimator_status_sub.ChangeInstance(estimator_selector_status.primary_instance);
			}
		}
	}

	_estimator_status_sub.update(&_estimator_status);
	_vehicle_acceleration_sub.update(&_accel);
	_vehicle_air_data_sub.update(&_vehicle_air_data);
	_vehicle_land_detected_sub.update(&_vehicle_land_detected);
	_vehicle_status_sub.update(&_vehicle_status);
	_vtol_vehicle_status_sub.update(&_vtol_vehicle_status);
	_vehicle_local_position_sub.update(&_vehicle_local_position);
	_position_setpoint_sub.update(&_position_setpoint);

	_tecs_status_sub.update(&_tecs_status);

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude;
		_vehicle_attitude_sub.update(&vehicle_attitude);

		if (_vehicle_status.is_vtol_tailsitter && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {

			// if the vehicle is a tailsitter we have to rotate the attitude by 90° to get to the airspeed frame
			_q_att = Quatf(vehicle_attitude.q) * Quatf(matrix::Eulerf(0.f, M_PI_2_F, 0.f));

		} else {
			_q_att = Quatf(vehicle_attitude.q);
		}
	}

	_gnss_lpos_valid = (_time_now_usec - _vehicle_local_position.timestamp < 1_s)
			   && (_vehicle_local_position.timestamp > 0)
			   && _vehicle_local_position.v_xy_valid
			   && _estimator_status.control_mode_flags & (1 << estimator_status_s::CS_GPS);
}

void AirspeedModule::update_wind_estimator_sideslip()
{
	// update wind and airspeed estimator
	_wind_estimator_sideslip.update(_time_now_usec);

	if (_gnss_lpos_valid
	    && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
	    && !_vehicle_land_detected.landed) {
		Vector3f vI(_vehicle_local_position.vx, _vehicle_local_position.vy, _vehicle_local_position.vz);

		const float hor_vel_variance =  _vehicle_local_position.evh * _vehicle_local_position.evh;
		_wind_estimator_sideslip.fuse_beta(_time_now_usec, vI, hor_vel_variance, _q_att);
	}

	_wind_estimate_sideslip.timestamp = _time_now_usec;
	_wind_estimate_sideslip.windspeed_north = _wind_estimator_sideslip.get_wind()(0);
	_wind_estimate_sideslip.windspeed_east = _wind_estimator_sideslip.get_wind()(1);
	_wind_estimate_sideslip.variance_north = _wind_estimator_sideslip.get_wind_var()(0);
	_wind_estimate_sideslip.variance_east = _wind_estimator_sideslip.get_wind_var()(1);
	_wind_estimate_sideslip.tas_innov = NAN;
	_wind_estimate_sideslip.tas_innov_var = NAN;
	_wind_estimate_sideslip.beta_innov = _wind_estimator_sideslip.get_beta_innov();
	_wind_estimate_sideslip.beta_innov_var = _wind_estimator_sideslip.get_beta_innov_var();
	_wind_estimate_sideslip.tas_scale_raw = NAN;
	_wind_estimate_sideslip.tas_scale_raw_var = NAN;
	_wind_estimate_sideslip.tas_scale_validated = NAN;
	_wind_estimate_sideslip.source = airspeed_wind_s::SOURCE_AS_BETA_ONLY;
}

void AirspeedModule::update_ground_minus_wind_airspeed()
{
	const float wind_uncertainty = sqrtf(_wind_estimate_sideslip.variance_north + _wind_estimate_sideslip.variance_east);

	if (_wind_estimator_sideslip.is_estimate_valid() && wind_uncertainty < _param_wind_sigma_max_synth_tas.get()) {
		// calculate airspeed estimate based on groundspeed-windspeed
		const float TAS_north = _vehicle_local_position.vx - _wind_estimate_sideslip.windspeed_north;
		const float TAS_east = _vehicle_local_position.vy - _wind_estimate_sideslip.windspeed_east;
		const float TAS_down = _vehicle_local_position.vz; // no wind estimate in z
		_ground_minus_wind_TAS = sqrtf(TAS_north * TAS_north + TAS_east * TAS_east + TAS_down * TAS_down);
		_ground_minus_wind_CAS = calc_calibrated_from_true_airspeed(_ground_minus_wind_TAS, _vehicle_air_data.rho);

	} else {
		_ground_minus_wind_TAS = _ground_minus_wind_CAS = NAN;
	}
}


void AirspeedModule::select_airspeed_and_publish()
{

	// we need to re-evaluate the sensors if we're currently not on a phyisical sensor or the current sensor got invalid
	bool airspeed_sensor_switching_necessary = false;

	if (_prev_airspeed_index < airspeed_index::FIRST_SENSOR_INDEX) {
		airspeed_sensor_switching_necessary = true;

	} else {
		airspeed_sensor_switching_necessary = !_airspeed_validator[_prev_airspeed_index - 1].get_airspeed_valid();
	}

	const bool airspeed_sensor_switching_allowed = _number_of_airspeed_sensors > 0 &&
			_param_airspeed_primary_index.get() > airspeed_index::GROUND_MINUS_WIND_INDEX && _param_airspeed_checks_on.get();

	const bool airspeed_sensor_added = _prev_number_of_airspeed_sensors < _number_of_airspeed_sensors;

	if (airspeed_sensor_switching_necessary && (airspeed_sensor_switching_allowed || airspeed_sensor_added)) {

		_valid_airspeed_index = airspeed_index::DISABLED_INDEX;

		// loop through all sensors and take the first valid one
		for (int i = 0; i < _number_of_airspeed_sensors; i++) {
			if (_airspeed_validator[i].get_airspeed_valid()) {
				_valid_airspeed_index = i + 1;
				break;
			}
		}
	}

	// check if airspeed based on ground-wind speed is valid and can be published
	if (_valid_airspeed_index < airspeed_index::FIRST_SENSOR_INDEX
	    || _param_airspeed_primary_index.get() == airspeed_index::GROUND_MINUS_WIND_INDEX) {

		// _gnss_lpos_valid determines if ground-wind estimate is valid
		if (_gnss_lpos_valid &&
		    (_param_airspeed_fallback_gw.get() || _param_airspeed_primary_index.get() == airspeed_index::GROUND_MINUS_WIND_INDEX)) {
			_valid_airspeed_index = airspeed_index::GROUND_MINUS_WIND_INDEX;

		} else {
			_valid_airspeed_index = airspeed_index::DISABLED_INDEX;
		}
	}

	// print warning or info, depending of whether airspeed got declared invalid or healthy
	if (_valid_airspeed_index != _prev_airspeed_index &&
	    _number_of_airspeed_sensors > 0) {
		if (_vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED && _prev_airspeed_index > 0) {
			mavlink_log_critical(&_mavlink_log_pub, "Airspeed sensor failure detected. Check connection and reboot.\t");
			events::send(events::ID("airspeed_selector_sensor_failure_disarmed"), events::Log::Critical,
				     "Airspeed sensor failure detected. Check connection and reboot");

		} else if (_prev_airspeed_index > 0) {
			mavlink_log_critical(&_mavlink_log_pub, "Airspeed sensor failure detected. Return to launch (RTL) is advised.\t");
			events::send(events::ID("airspeed_selector_sensor_failure"), events::Log::Critical,
				     "Airspeed sensor failure detected. Return to launch (RTL) is advised");

		} else if (_prev_airspeed_index == 0 && _valid_airspeed_index == -1) {
			mavlink_log_info(&_mavlink_log_pub, "Airspeed estimation invalid\t");
			events::send(events::ID("airspeed_selector_estimation_invalid"), events::Log::Error,
				     "Airspeed estimation invalid");

		} else if (_prev_airspeed_index == -1 && _valid_airspeed_index == 0) {
			mavlink_log_info(&_mavlink_log_pub, "Airspeed estimation valid\t");
			events::send(events::ID("airspeed_selector_estimation_valid"), events::Log::Info,
				     "Airspeed estimation valid");

		} else {
			mavlink_log_info(&_mavlink_log_pub, "Airspeed sensor healthy, start using again (%i, %i)\t", _prev_airspeed_index,
					 _valid_airspeed_index);
			/* EVENT
			 * @description Previously selected sensor index: {1}, current sensor index: {2}.
			 */
			events::send<uint8_t, uint8_t>(events::ID("airspeed_selector_estimation_regain"), events::Log::Info,
						       "Airspeed sensor healthy, start using again", _prev_airspeed_index,
						       _valid_airspeed_index);
		}
	}

	_prev_airspeed_index = _valid_airspeed_index;
	_prev_number_of_airspeed_sensors = _number_of_airspeed_sensors;

	airspeed_validated_s airspeed_validated = {};
	airspeed_validated.timestamp = _time_now_usec;
	airspeed_validated.true_ground_minus_wind_m_s = NAN;
	airspeed_validated.calibrated_ground_minus_wind_m_s = NAN;
	airspeed_validated.indicated_airspeed_m_s = NAN;
	airspeed_validated.calibrated_airspeed_m_s = NAN;
	airspeed_validated.true_airspeed_m_s = NAN;
	airspeed_validated.airspeed_sensor_measurement_valid = false;
	airspeed_validated.selected_airspeed_index = _valid_airspeed_index;

	airspeed_validated.airspeed_derivative_filtered = _airspeed_validator[_valid_airspeed_index -
					      1].get_airspeed_derivative();
	airspeed_validated.throttle_filtered = _airspeed_validator[_valid_airspeed_index - 1].get_throttle_filtered();
	airspeed_validated.pitch_filtered = _airspeed_validator[_valid_airspeed_index - 1].get_pitch_filtered();

	switch (_valid_airspeed_index) {
	case airspeed_index::DISABLED_INDEX:
		break;

	case airspeed_index::GROUND_MINUS_WIND_INDEX:
		airspeed_validated.indicated_airspeed_m_s = _ground_minus_wind_CAS;
		airspeed_validated.calibrated_airspeed_m_s = _ground_minus_wind_CAS;
		airspeed_validated.true_airspeed_m_s = _ground_minus_wind_TAS;
		airspeed_validated.calibrated_ground_minus_wind_m_s = _ground_minus_wind_CAS;
		airspeed_validated.true_ground_minus_wind_m_s = _ground_minus_wind_TAS;

		break;

	default:
		airspeed_validated.indicated_airspeed_m_s = _airspeed_validator[_valid_airspeed_index - 1].get_IAS();
		airspeed_validated.calibrated_airspeed_m_s = _airspeed_validator[_valid_airspeed_index - 1].get_CAS();
		airspeed_validated.true_airspeed_m_s = _airspeed_validator[_valid_airspeed_index - 1].get_TAS();
		airspeed_validated.calibrated_ground_minus_wind_m_s = _ground_minus_wind_CAS;
		airspeed_validated.true_ground_minus_wind_m_s = _ground_minus_wind_TAS;
		airspeed_validated.airspeed_sensor_measurement_valid = true;
		break;
	}

	_airspeed_validated_pub.publish(airspeed_validated);

	_wind_est_pub[0].publish(_wind_estimate_sideslip);

	// publish the wind estimator states from all airspeed validators
	for (int i = 0; i < _number_of_airspeed_sensors; i++) {
		airspeed_wind_s wind_est = _airspeed_validator[i].get_wind_estimator_states(_time_now_usec);

		if (i == 0) {
			wind_est.source = airspeed_wind_s::SOURCE_AS_SENSOR_1;

		} else if (i == 1) {
			wind_est.source = airspeed_wind_s::SOURCE_AS_SENSOR_2;

		} else {
			wind_est.source = airspeed_wind_s::SOURCE_AS_SENSOR_3;
		}

		_wind_est_pub[i + 1].publish(wind_est);
	}

}

int AirspeedModule::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		int ret = AirspeedModule::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int AirspeedModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module provides a single airspeed_validated topic, containing indicated (IAS),
calibrated (CAS), true airspeed (TAS) and the information if the estimation currently
is invalid and if based sensor readings or on groundspeed minus windspeed.
Supporting the input of multiple "raw" airspeed inputs, this module automatically switches
to a valid sensor in case of failure detection. For failure detection as well as for
the estimation of a scale factor from IAS to CAS, it runs several wind estimators
and also publishes those.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("airspeed_estimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int airspeed_selector_main(int argc, char *argv[])
{
	return AirspeedModule::main(argc, argv);
}
