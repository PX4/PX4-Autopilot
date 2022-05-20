/****************************************************************************
 *
 *   Copyright (c) 2017-2022 PX4 Development Team. All rights reserved.
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
#include "Arming/ArmStateMachine/ArmStateMachine.hpp"
#include "Arming/PreFlightCheck/PreFlightCheck.hpp"
#include "failure_detector/FailureDetector.hpp"
#include "Safety.hpp"
#include "state_machine_helper.h"
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
#include <uORB/topics/home_position.h>
#include <uORB/topics/test_motor.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>

// subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/action_request.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/iridiumsbd_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/power_button_state.h>
#include <uORB/topics/rtl_time_estimate.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/wind.h>

using math::constrain;
using systemlib::Hysteresis;

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

	void get_circuit_breaker_params();

private:
	void answer_command(const vehicle_command_s &cmd, uint8_t result);

	transition_result_t arm(arm_disarm_reason_t calling_reason, bool run_preflight_checks = true);
	transition_result_t disarm(arm_disarm_reason_t calling_reason, bool forced = false);

	void battery_status_check();

	bool check_posvel_validity(const bool data_valid, const float data_accuracy, const float required_accuracy,
				   const hrt_abstime &data_timestamp_us, hrt_abstime &last_fail_time_us,
				   const bool was_valid);

	void control_status_leds(bool changed, const uint8_t battery_warning);

	/**
	 * Checks the status of all available data links and handles switching between different system telemetry states.
	 */
	void data_link_check();

	void avoidance_check();

	void esc_status_check();

	void estimator_check();

	bool handle_command(const vehicle_command_s &cmd);

	unsigned handle_command_motor_test(const vehicle_command_s &cmd);
	unsigned handle_command_actuator_test(const vehicle_command_s &cmd);

	void executeActionRequest(const action_request_s &action_request);

	void offboard_control_update();

	void print_reject_mode(uint8_t main_state);

	bool hasMovedFromCurrentHomeLocation();
	bool set_home_position();
	bool set_home_position_alt_only();
	void set_in_air_home_position();
	void fillLocalHomePos(home_position_s &home, const vehicle_local_position_s &lpos) const;
	void fillLocalHomePos(home_position_s &home, float x, float y, float z, float heading) const;
	void fillGlobalHomePos(home_position_s &home, const vehicle_global_position_s &gpos) const;
	void fillGlobalHomePos(home_position_s &home, double lat, double lon, float alt) const;
	void setHomePosValid();
	void updateHomePositionYaw(float yaw);

	void update_control_mode();

	bool shutdown_if_allowed();

	bool stabilization_required();

	void send_parachute_command();

	void checkWindSpeedThresholds();

	void updateParameters();

	DEFINE_PARAMETERS(

		(ParamInt<px4::params::NAV_DLL_ACT>) _param_nav_dll_act,
		(ParamInt<px4::params::COM_DL_LOSS_T>) _param_com_dl_loss_t,

		(ParamInt<px4::params::COM_RC_OVERRIDE>) _param_com_rc_override,

		(ParamInt<px4::params::COM_HLDL_LOSS_T>) _param_com_hldl_loss_t,
		(ParamInt<px4::params::COM_HLDL_REG_T>) _param_com_hldl_reg_t,

		(ParamInt<px4::params::NAV_RCL_ACT>) _param_nav_rcl_act,
		(ParamFloat<px4::params::COM_RCL_ACT_T>) _param_com_rcl_act_t,
		(ParamInt<px4::params::COM_RCL_EXCEPT>) _param_com_rcl_except,

		(ParamBool<px4::params::COM_HOME_EN>) _param_com_home_en,
		(ParamBool<px4::params::COM_HOME_IN_AIR>) _param_com_home_in_air,

		(ParamFloat<px4::params::COM_POS_FS_EPH>) _param_com_pos_fs_eph,
		(ParamFloat<px4::params::COM_POS_FS_EPV>) _param_com_pos_fs_epv, 	/*Not realy used for now*/
		(ParamFloat<px4::params::COM_VEL_FS_EVH>) _param_com_vel_fs_evh,
		(ParamInt<px4::params::COM_POSCTL_NAVL>) _param_com_posctl_navl,	/* failsafe response to loss of navigation accuracy */

		(ParamInt<px4::params::COM_POS_FS_DELAY>) _param_com_pos_fs_delay,

		(ParamInt<px4::params::COM_LOW_BAT_ACT>) _param_com_low_bat_act,
		(ParamFloat<px4::params::COM_BAT_ACT_T>) _param_com_bat_act_t,
		(ParamInt<px4::params::COM_IMB_PROP_ACT>) _param_com_imb_prop_act,
		(ParamFloat<px4::params::COM_DISARM_LAND>) _param_com_disarm_land,
		(ParamFloat<px4::params::COM_DISARM_PRFLT>) _param_com_disarm_preflight,

		(ParamBool<px4::params::COM_OBS_AVOID>) _param_com_obs_avoid,

		(ParamInt<px4::params::COM_FLT_PROFILE>) _param_com_flt_profile,

		(ParamFloat<px4::params::COM_OBC_LOSS_T>) _param_com_obc_loss_t,

		(ParamFloat<px4::params::COM_WIND_WARN>) _param_com_wind_warn,

		// Quadchute
		(ParamInt<px4::params::COM_QC_ACT>) _param_com_qc_act,

		// Offboard
		(ParamFloat<px4::params::COM_OF_LOSS_T>) _param_com_of_loss_t,
		(ParamInt<px4::params::COM_OBL_ACT>) _param_com_obl_act,
		(ParamInt<px4::params::COM_OBL_RC_ACT>) _param_com_obl_rc_act,

		(ParamInt<px4::params::COM_PREARM_MODE>) _param_com_prearm_mode,
		(ParamBool<px4::params::COM_FORCE_SAFETY>) _param_com_force_safety,
		(ParamBool<px4::params::COM_MOT_TEST_EN>) _param_com_mot_test_en,

		(ParamFloat<px4::params::COM_KILL_DISARM>) _param_com_kill_disarm,
		(ParamFloat<px4::params::COM_LKDOWN_TKO>) _param_com_lkdown_tko,

		// Engine failure
		(ParamInt<px4::params::COM_ACT_FAIL_ACT>) _param_com_actuator_failure_act,

		(ParamBool<px4::params::COM_ARM_WO_GPS>) _param_arm_without_gps,
		(ParamBool<px4::params::COM_ARM_MIS_REQ>) _param_arm_mission_required,
		(ParamBool<px4::params::COM_ARM_AUTH_REQ>) _param_arm_auth_required,
		(ParamBool<px4::params::COM_ARM_CHK_ESCS>) _param_escs_checks_required,

		(ParamInt<px4::params::COM_FLIGHT_UUID>) _param_flight_uuid,
		(ParamInt<px4::params::COM_TAKEOFF_ACT>) _param_takeoff_finished_action,

		// Circuit breakers
		(ParamInt<px4::params::CBRK_SUPPLY_CHK>) _param_cbrk_supply_chk,
		(ParamInt<px4::params::CBRK_USB_CHK>) _param_cbrk_usb_chk,
		(ParamInt<px4::params::CBRK_AIRSPD_CHK>) _param_cbrk_airspd_chk,
		(ParamInt<px4::params::CBRK_FLIGHTTERM>) _param_cbrk_flightterm,
		(ParamInt<px4::params::CBRK_VELPOSERR>) _param_cbrk_velposerr,
		(ParamInt<px4::params::CBRK_VTOLARMING>) _param_cbrk_vtolarming,

		(ParamInt<px4::params::COM_FLT_TIME_MAX>) _param_com_flt_time_max,
		(ParamFloat<px4::params::COM_WIND_MAX>) _param_com_wind_max
	)

	// optional parameters
	param_t _param_cp_dist{PARAM_INVALID};
	param_t _param_mav_comp_id{PARAM_INVALID};
	param_t _param_mav_sys_id{PARAM_INVALID};
	param_t _param_mav_type{PARAM_INVALID};
	param_t _param_rc_map_fltmode{PARAM_INVALID};

	enum class PrearmedMode {
		DISABLED = 0,
		SAFETY_BUTTON = 1,
		ALWAYS = 2
	};

	enum class RcOverrideBits : int32_t {
		AUTO_MODE_BIT = (1 << 0),
		OFFBOARD_MODE_BIT = (1 << 1),
	};

	enum class ActuatorFailureActions {
		DISABLED = 0,
		AUTO_LOITER = 1,
		AUTO_LAND = 2,
		AUTO_RTL = 3,
		TERMINATE = 4,
	};

	/* Decouple update interval and hysteresis counters, all depends on intervals */
	static constexpr uint64_t COMMANDER_MONITORING_INTERVAL{10_ms};

	static constexpr uint64_t HOTPLUG_SENS_TIMEOUT{8_s};	/**< wait for hotplug sensors to come online for upto 8 seconds */
	static constexpr uint64_t INAIR_RESTART_HOLDOFF_INTERVAL{500_ms};

	ArmStateMachine _arm_state_machine{};
	PreFlightCheck::arm_requirements_t	_arm_requirements{};

	hrt_abstime	_valid_distance_sensor_time_us{0}; /**< Last time that distance sensor data arrived (usec) */

	hrt_abstime	_last_gpos_fail_time_us{0};	/**< Last time that the global position validity recovery check failed (usec) */
	hrt_abstime	_last_lpos_fail_time_us{0};	/**< Last time that the local position validity recovery check failed (usec) */
	hrt_abstime	_last_lvel_fail_time_us{0};	/**< Last time that the local velocity validity recovery check failed (usec) */

	/* class variables used to check for navigation failure after takeoff */
	hrt_abstime	_time_last_innov_pass{0};	/**< last time velocity and position innovations passed */
	hrt_abstime	_time_last_innov_fail{0};	/**< last time velocity and position innovations failed */
	bool		_nav_test_passed{false};	/**< true if the post takeoff navigation test has passed */
	bool		_nav_test_failed{false};	/**< true if the post takeoff navigation test has failed */

	static constexpr hrt_abstime GPS_VALID_TIME{3_s};
	Hysteresis _vehicle_gps_position_valid{false};
	hrt_abstime _vehicle_gps_position_timestamp_last{0};

	bool		_geofence_loiter_on{false};
	bool		_geofence_rtl_on{false};
	bool		_geofence_land_on{false};
	bool		_geofence_warning_action_on{false};
	bool		_geofence_violated_prev{false};

	bool            _collision_prevention_enabled{false};

	bool		_rtl_time_actions_done{false};

	FailureDetector	_failure_detector;
	bool		_flight_termination_triggered{false};
	bool		_lockdown_triggered{false};
	bool            _imbalanced_propeller_check_triggered{false};


	hrt_abstime	_datalink_last_heartbeat_gcs{0};
	hrt_abstime	_datalink_last_heartbeat_avoidance_system{0};
	hrt_abstime	_datalink_last_heartbeat_onboard_controller{0};
	hrt_abstime	_datalink_last_heartbeat_parachute_system{0};
	bool		_onboard_controller_lost{false};
	bool		_avoidance_system_lost{false};
	bool		_parachute_system_lost{true};

	hrt_abstime	_high_latency_datalink_heartbeat{0};
	hrt_abstime	_high_latency_datalink_lost{0};

	int		_last_esc_online_flags{-1};
	int		_last_esc_failure[esc_status_s::CONNECTED_ESC_MAX] {};
	hrt_abstime	_last_esc_status_updated{0};

	uint8_t		_battery_warning{battery_status_s::BATTERY_WARNING_NONE};
	hrt_abstime	_battery_failsafe_timestamp{0};
	uint8_t		_last_connected_batteries{0};
	uint32_t	_last_battery_custom_fault[battery_status_s::MAX_INSTANCES] {};
	uint16_t	_last_battery_fault[battery_status_s::MAX_INSTANCES] {};
	uint8_t		_last_battery_mode[battery_status_s::MAX_INSTANCES] {};


	Hysteresis	_auto_disarm_landed{false};
	Hysteresis	_auto_disarm_killed{false};
	Hysteresis	_offboard_available{false};

	hrt_abstime	_last_print_mode_reject_time{0};	///< To remember when last notification was sent
	bool            _mode_switch_mapped{false};

	bool		_last_local_altitude_valid{false};
	bool		_last_local_position_valid{false};
	bool		_last_global_position_valid{false};

	bool		_last_overload{false};

	hrt_abstime	_last_valid_manual_control_setpoint{0};

	bool		_is_throttle_above_center{false};
	bool		_is_throttle_low{false};

	hrt_abstime	_boot_timestamp{0};
	hrt_abstime	_last_disarmed_timestamp{0};
	hrt_abstime	_overload_start{0};		///< time when CPU overload started

	hrt_abstime _led_armed_state_toggle{0};
	hrt_abstime _led_overload_toggle{0};

	hrt_abstime _last_termination_message_sent{0};

	uint8_t		_heading_reset_counter{0};

	bool		_status_changed{true};
	bool		_arm_tune_played{false};
	bool		_was_armed{false};
	bool		_failsafe_old{false};	///< check which state machines for changes, clear "changed" flag
	bool		_have_taken_off_since_arming{false};
	bool		_system_power_usb_connected{false};

	geofence_result_s	_geofence_result{};
	vehicle_land_detected_s	_vehicle_land_detected{};
	vtol_vehicle_status_s	_vtol_status{};

	hrt_abstime _last_wind_warning{0};

	// commander publications
	actuator_armed_s        _armed{};
	commander_state_s       _internal_state{};
	vehicle_control_mode_s  _vehicle_control_mode{};
	vehicle_status_s        _status{};
	vehicle_status_flags_s  _status_flags{};

	Safety _safety{};

	WorkerThread _worker_thread;

	// Subscriptions
	uORB::Subscription					_action_request_sub {ORB_ID(action_request)};
	uORB::Subscription					_cpuload_sub{ORB_ID(cpuload)};
	uORB::Subscription					_esc_status_sub{ORB_ID(esc_status)};
	uORB::Subscription					_estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription					_estimator_status_sub{ORB_ID(estimator_status)};
	uORB::Subscription					_geofence_result_sub{ORB_ID(geofence_result)};
	uORB::Subscription					_iridiumsbd_status_sub{ORB_ID(iridiumsbd_status)};
	uORB::Subscription					_vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription					_manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription					_rtl_time_estimate_sub{ORB_ID(rtl_time_estimate)};
	uORB::Subscription					_system_power_sub{ORB_ID(system_power)};
	uORB::Subscription					_vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription					_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription					_vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription					_vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription					_vtol_vehicle_status_sub{ORB_ID(vtol_vehicle_status)};
	uORB::Subscription					_wind_sub{ORB_ID(wind)};

	uORB::SubscriptionInterval				_parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionMultiArray<battery_status_s, battery_status_s::MAX_INSTANCES> _battery_status_subs{ORB_ID::battery_status};
	uORB::SubscriptionMultiArray<distance_sensor_s>         _distance_sensor_subs{ORB_ID::distance_sensor};
	uORB::SubscriptionMultiArray<telemetry_status_s>        _telemetry_status_subs{ORB_ID::telemetry_status};

#if defined(BOARD_HAS_POWER_CONTROL)
	uORB::Subscription					_power_button_state_sub {ORB_ID(power_button_state)};
#endif // BOARD_HAS_POWER_CONTROL

	uORB::SubscriptionData<estimator_status_flags_s>	_estimator_status_flags_sub{ORB_ID(estimator_status_flags)};
	uORB::SubscriptionData<mission_result_s>		_mission_result_sub{ORB_ID(mission_result)};
	uORB::SubscriptionData<offboard_control_mode_s>		_offboard_control_mode_sub{ORB_ID(offboard_control_mode)};
	uORB::SubscriptionData<vehicle_global_position_s>	_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::SubscriptionData<vehicle_local_position_s>	_local_position_sub{ORB_ID(vehicle_local_position)};

	// Publications
	uORB::Publication<actuator_armed_s>			_armed_pub{ORB_ID(actuator_armed)};
	uORB::Publication<commander_state_s>			_commander_state_pub{ORB_ID(commander_state)};
	uORB::Publication<failure_detector_status_s>		_failure_detector_status_pub{ORB_ID(failure_detector_status)};
	uORB::Publication<test_motor_s>				_test_motor_pub{ORB_ID(test_motor)};
	uORB::Publication<actuator_test_s>			_actuator_test_pub{ORB_ID(actuator_test)};
	uORB::Publication<vehicle_control_mode_s>		_control_mode_pub{ORB_ID(vehicle_control_mode)};
	uORB::Publication<vehicle_status_flags_s>		_vehicle_status_flags_pub{ORB_ID(vehicle_status_flags)};
	uORB::Publication<vehicle_status_s>			_status_pub{ORB_ID(vehicle_status)};

	uORB::PublicationData<home_position_s>			_home_pub{ORB_ID(home_position)};

	uORB::Publication<vehicle_command_ack_s>		_command_ack_pub{ORB_ID(vehicle_command_ack)};

	orb_advert_t _mavlink_log_pub{nullptr};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _preflight_check_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": preflight check")};
};
