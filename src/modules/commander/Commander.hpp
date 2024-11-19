/****************************************************************************
 *
 *   Copyright (c) 2017-2023 PX4 Development Team. All rights reserved.
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

#pragma once

/*   Helper classes  */
#include "failsafe/failsafe.h"
#include "failure_detector/FailureDetector.hpp"
#include "HealthAndArmingChecks/HealthAndArmingChecks.hpp"
#include "HomePosition.hpp"
#include "ModeManagement.hpp"
#include "MulticopterThrowLaunch/MulticopterThrowLaunch.hpp"
#include "Safety.hpp"
#include "UserModeIntention.hpp"
#include "worker_thread.hpp"

#include <lib/controllib/blocks.hpp>
#include <lib/hysteresis/hysteresis.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

// publications
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/failure_detector_status.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>

// subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/action_request.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/iridiumsbd_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/power_button_state.h>
#include <uORB/topics/rtl_time_estimate.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vtol_vehicle_status.h>

using math::constrain;
using systemlib::Hysteresis;

typedef enum {
	TRANSITION_DENIED = -1,
	TRANSITION_NOT_CHANGED = 0,
	TRANSITION_CHANGED
} transition_result_t;

using arm_disarm_reason_t = events::px4::enums::arm_disarm_reason_t;

using namespace time_literals;

class Commander : public ModuleBase<Commander>, public ModuleParams
{
public:
	Commander();
	~Commander();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Commander *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void enable_hil();

private:
	bool isArmed() const { return (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED); }
	static ModeChangeSource getSourceFromCommand(const vehicle_command_s &cmd);

	void answer_command(const vehicle_command_s &cmd, uint8_t result);

	transition_result_t arm(arm_disarm_reason_t calling_reason, bool run_preflight_checks = true);

	transition_result_t disarm(arm_disarm_reason_t calling_reason, bool forced = false);

	void battery_status_check();

	void control_status_leds(bool changed, const uint8_t battery_warning);

	/**
	 * Checks the status of all available data links and handles switching between different system telemetry states.
	 */
	void dataLinkCheck();

	void manualControlCheck();

	void offboardControlCheck();

	/**
	 * @brief Handle incoming vehicle command relavant to Commander
	 *
	 * It ignores irrelevant vehicle commands defined inside the switch case statement
	 * in the function.
	 *
	 * @param cmd 		Vehicle command to handle
	 */
	bool handle_command(const vehicle_command_s &cmd);

	unsigned handleCommandActuatorTest(const vehicle_command_s &cmd);

	void executeActionRequest(const action_request_s &action_request);

	void printRejectMode(uint8_t nav_state);

	void updateControlMode();

	void send_parachute_command();

	void checkForMissionUpdate();

	void handlePowerButtonState();

	void systemPowerUpdate();

	void landDetectorUpdate();

	void safetyButtonUpdate();

	bool isThrowLaunchInProgress() const;

	void throwLaunchUpdate();

	void vtolStatusUpdate();

	void updateTunes();

	void checkWorkerThread();

	bool getPrearmState() const;

	void handleAutoDisarm();

	bool handleModeIntentionAndFailsafe();

	void updateParameters();

	void checkAndInformReadyForTakeoff();

	void handleCommandsFromModeExecutors();

	void modeManagementUpdate();

	static void onFailsafeNotifyUserTrampoline(void *arg);
	void onFailsafeNotifyUser();

	enum class PrearmedMode {
		DISABLED = 0,
		SAFETY_BUTTON = 1,
		ALWAYS = 2
	};

	enum class RcOverrideBits : int32_t {
		AUTO_MODE_BIT = (1 << 0),
		OFFBOARD_MODE_BIT = (1 << 1),
	};

	/* Decouple update interval and hysteresis counters, all depends on intervals */
	static constexpr uint64_t COMMANDER_MONITORING_INTERVAL{10_ms};

	vehicle_status_s        _vehicle_status{};

	Failsafe		_failsafe_instance{this};
	FailsafeBase		&_failsafe{_failsafe_instance};
	FailureDetector		_failure_detector{this};
	HealthAndArmingChecks	_health_and_arming_checks{this, _vehicle_status};
	MulticopterThrowLaunch  _multicopter_throw_launch{this};
	Safety			_safety{};
	WorkerThread 		_worker_thread{};
	ModeManagement  	_mode_management{
#ifndef CONSTRAINED_FLASH
		_health_and_arming_checks.externalChecks()
#endif
	};
	UserModeIntention	_user_mode_intention {this, _vehicle_status, _health_and_arming_checks, &_mode_management};

	const failsafe_flags_s &_failsafe_flags{_health_and_arming_checks.failsafeFlags()};
	HomePosition 		_home_position{_failsafe_flags};
	config_overrides_s   _config_overrides{};


	Hysteresis _auto_disarm_landed{false};
	Hysteresis _auto_disarm_killed{false};

	hrt_abstime _datalink_last_heartbeat_open_drone_id_system{0};
	hrt_abstime _datalink_last_heartbeat_avoidance_system{0};
	hrt_abstime _datalink_last_heartbeat_gcs{0};
	hrt_abstime _datalink_last_heartbeat_onboard_controller{0};
	hrt_abstime _datalink_last_heartbeat_parachute_system{0};

	hrt_abstime _last_print_mode_reject_time{0};	///< To remember when last notification was sent

	hrt_abstime _high_latency_datalink_timestamp{0};
	hrt_abstime _high_latency_datalink_lost{0};
	hrt_abstime _high_latency_datalink_regained{0};

	hrt_abstime _boot_timestamp{0};
	hrt_abstime _last_disarmed_timestamp{0};
	hrt_abstime _overload_start{0};		///< time when CPU overload started

#if !defined(CONFIG_ARCH_LEDS) && defined(BOARD_HAS_CONTROL_STATUS_LEDS)
	hrt_abstime _led_armed_state_toggle {0};
#endif
	hrt_abstime _led_overload_toggle {0};

	hrt_abstime _last_health_and_arming_check{0};

	uint8_t		_battery_warning{battery_status_s::BATTERY_WARNING_NONE};

	bool _failsafe_user_override_request{false}; ///< override request due to stick movements

	bool _open_drone_id_system_lost{true};
	bool _avoidance_system_lost{false};
	bool _onboard_controller_lost{false};
	bool _parachute_system_lost{true};

	bool _last_overload{false};
	bool _mode_switch_mapped{false};

	bool _is_throttle_above_center{false};
	bool _is_throttle_low{false};

	bool _arm_tune_played{false};
	bool _have_taken_off_since_arming{false};
	bool _status_changed{true};

	vehicle_land_detected_s	_vehicle_land_detected{};

	// commander publications
	actuator_armed_s        _actuator_armed{};
	vehicle_control_mode_s  _vehicle_control_mode{};
	vtol_vehicle_status_s	_vtol_vehicle_status{};

	// Subscriptions
	uORB::Subscription					_action_request_sub{ORB_ID(action_request)};
	uORB::Subscription					_cpuload_sub{ORB_ID(cpuload)};
	uORB::Subscription					_iridiumsbd_status_sub{ORB_ID(iridiumsbd_status)};
	uORB::Subscription					_manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription					_system_power_sub{ORB_ID(system_power)};
	uORB::Subscription					_vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription					_vehicle_command_mode_executor_sub{ORB_ID(vehicle_command_mode_executor)};
	uORB::Subscription					_vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription					_vtol_vehicle_status_sub{ORB_ID(vtol_vehicle_status)};

	uORB::SubscriptionInterval				_parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionMultiArray<telemetry_status_s>	_telemetry_status_subs{ORB_ID::telemetry_status};

#if defined(BOARD_HAS_POWER_CONTROL)
	uORB::Subscription					_power_button_state_sub {ORB_ID(power_button_state)};
#endif // BOARD_HAS_POWER_CONTROL

	uORB::SubscriptionData<mission_result_s>		_mission_result_sub{ORB_ID(mission_result)};
	uORB::SubscriptionData<offboard_control_mode_s>		_offboard_control_mode_sub{ORB_ID(offboard_control_mode)};

	// Publications
	uORB::Publication<actuator_armed_s>			_actuator_armed_pub{ORB_ID(actuator_armed)};
	uORB::Publication<actuator_test_s>			_actuator_test_pub{ORB_ID(actuator_test)};
	uORB::Publication<failure_detector_status_s>		_failure_detector_status_pub{ORB_ID(failure_detector_status)};
	uORB::Publication<vehicle_command_ack_s>		_vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<vehicle_command_s>			_vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_control_mode_s>		_vehicle_control_mode_pub{ORB_ID(vehicle_control_mode)};
	uORB::Publication<vehicle_status_s>			_vehicle_status_pub{ORB_ID(vehicle_status)};

	orb_advert_t _mavlink_log_pub{nullptr};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _preflight_check_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": preflight check")};

	// optional parameters
	param_t _param_mav_type{PARAM_INVALID};
	param_t _param_rc_map_fltmode{PARAM_INVALID};

	DEFINE_PARAMETERS(

		(ParamFloat<px4::params::COM_DISARM_LAND>)  _param_com_disarm_land,
		(ParamFloat<px4::params::COM_DISARM_PRFLT>) _param_com_disarm_prflt,
		(ParamBool<px4::params::COM_DISARM_MAN>)    _param_com_disarm_man,
		(ParamInt<px4::params::COM_DL_LOSS_T>)      _param_com_dl_loss_t,
		(ParamInt<px4::params::COM_HLDL_LOSS_T>)    _param_com_hldl_loss_t,
		(ParamInt<px4::params::COM_HLDL_REG_T>)     _param_com_hldl_reg_t,
		(ParamBool<px4::params::COM_HOME_EN>)       _param_com_home_en,
		(ParamBool<px4::params::COM_HOME_IN_AIR>)   _param_com_home_in_air,
		(ParamInt<px4::params::COM_FLT_PROFILE>)    _param_com_flt_profile,
		(ParamBool<px4::params::COM_FORCE_SAFETY>)  _param_com_force_safety,
		(ParamFloat<px4::params::COM_KILL_DISARM>)  _param_com_kill_disarm,
		(ParamBool<px4::params::COM_MOT_TEST_EN>)   _param_com_mot_test_en,
		(ParamBool<px4::params::COM_OBS_AVOID>)     _param_com_obs_avoid,
		(ParamFloat<px4::params::COM_OBC_LOSS_T>)   _param_com_obc_loss_t,
		(ParamInt<px4::params::COM_PREARM_MODE>)    _param_com_prearm_mode,
		(ParamInt<px4::params::COM_RC_OVERRIDE>)    _param_com_rc_override,
		(ParamInt<px4::params::COM_FLIGHT_UUID>)    _param_com_flight_uuid,
		(ParamInt<px4::params::COM_TAKEOFF_ACT>)    _param_com_takeoff_act,
		(ParamFloat<px4::params::COM_CPU_MAX>)      _param_com_cpu_max
	)
};
