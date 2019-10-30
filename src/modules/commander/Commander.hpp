/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#ifndef COMMANDER_HPP_
#define COMMANDER_HPP_

#include "state_machine_helper.h"
#include "failure_detector/FailureDetector.hpp"

#include <lib/controllib/blocks.hpp>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <lib/hysteresis/hysteresis.h>

// publications
#include <uORB/Publication.hpp>
#include <uORB/PublicationQueued.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/topics/test_motor.h>

// subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/iridiumsbd_status.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/esc_status.h>

using math::constrain;

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

	void enable_hil();

	// TODO: only temporarily static until low priority thread is removed
	static bool preflight_check(bool report);

	void get_circuit_breaker_params();

private:

	DEFINE_PARAMETERS(

		(ParamInt<px4::params::NAV_DLL_ACT>) _param_nav_dll_act,
		(ParamInt<px4::params::COM_DL_LOSS_T>) _param_com_dl_loss_t,

		(ParamInt<px4::params::COM_HLDL_LOSS_T>) _param_com_hldl_loss_t,
		(ParamInt<px4::params::COM_HLDL_REG_T>) _param_com_hldl_reg_t,

		(ParamInt<px4::params::NAV_RCL_ACT>) _param_nav_rcl_act,
		(ParamFloat<px4::params::COM_RC_LOSS_T>) _param_com_rc_loss_t,

		(ParamFloat<px4::params::COM_HOME_H_T>) _param_com_home_h_t,
		(ParamFloat<px4::params::COM_HOME_V_T>) _param_com_home_v_t,

		(ParamFloat<px4::params::COM_POS_FS_EPH>) _param_com_pos_fs_eph,
		(ParamFloat<px4::params::COM_POS_FS_EPV>) _param_com_pos_fs_epv, 	/*Not realy used for now*/
		(ParamFloat<px4::params::COM_VEL_FS_EVH>) _param_com_vel_fs_evh,
		(ParamInt<px4::params::COM_POSCTL_NAVL>) _param_com_posctl_navl,	/* failsafe response to loss of navigation accuracy */

		(ParamInt<px4::params::COM_POS_FS_DELAY>) _param_com_pos_fs_delay,
		(ParamInt<px4::params::COM_POS_FS_PROB>) _param_com_pos_fs_prob,
		(ParamInt<px4::params::COM_POS_FS_GAIN>) _param_com_pos_fs_gain,

		(ParamInt<px4::params::COM_LOW_BAT_ACT>) _param_com_low_bat_act,
		(ParamFloat<px4::params::COM_DISARM_LAND>) _param_com_disarm_land,
		(ParamFloat<px4::params::COM_DISARM_PRFLT>) _param_com_disarm_preflight,

		(ParamInt<px4::params::COM_OBS_AVOID>) _param_com_obs_avoid,
		(ParamInt<px4::params::COM_OA_BOOT_T>) _param_com_oa_boot_t,

		(ParamFloat<px4::params::COM_TAS_FS_INNOV>) _tas_innov_threshold,
		(ParamFloat<px4::params::COM_TAS_FS_INTEG>) _tas_innov_integ_threshold,
		(ParamInt<px4::params::COM_TAS_FS_T1>) _tas_use_stop_delay,
		(ParamInt<px4::params::COM_TAS_FS_T2>) _tas_use_start_delay,
		(ParamInt<px4::params::COM_ASPD_FS_ACT>) _airspeed_fail_action,
		(ParamFloat<px4::params::COM_ASPD_STALL>) _airspeed_stall,
		(ParamInt<px4::params::COM_ASPD_FS_DLY>) _airspeed_rtl_delay,
		(ParamInt<px4::params::COM_FLT_PROFILE>) _param_com_flt_profile,

		(ParamFloat<px4::params::COM_OF_LOSS_T>) _param_com_of_loss_t,
		(ParamInt<px4::params::COM_OBL_ACT>) _param_com_obl_act,
		(ParamInt<px4::params::COM_OBL_RC_ACT>) _param_com_obl_rc_act,

		(ParamInt<px4::params::COM_PREARM_MODE>) _param_com_prearm_mode,
		(ParamInt<px4::params::COM_MOT_TEST_EN>) _param_com_mot_test_en,

		(ParamInt<px4::params::CBRK_SUPPLY_CHK>) _param_cbrk_supply_chk,
		(ParamInt<px4::params::CBRK_USB_CHK>) _param_cbrk_usb_chk,
		(ParamInt<px4::params::CBRK_AIRSPD_CHK>) _param_cbrk_airspd_chk,
		(ParamInt<px4::params::CBRK_ENGINEFAIL>) _param_cbrk_enginefail,
		(ParamInt<px4::params::CBRK_GPSFAIL>) _param_cbrk_gpsfail,
		(ParamInt<px4::params::CBRK_FLIGHTTERM>) _param_cbrk_flightterm,
		(ParamInt<px4::params::CBRK_VELPOSERR>) _param_cbrk_velposerr
	)

	enum class PrearmedMode {
		DISABLED = 0,
		SAFETY_BUTTON = 1,
		ALWAYS = 2
	};

	const int64_t POSVEL_PROBATION_MIN = 1_s;	/**< minimum probation duration (usec) */
	const int64_t POSVEL_PROBATION_MAX = 100_s;	/**< maximum probation duration (usec) */

	hrt_abstime	_last_gpos_fail_time_us{0};	/**< Last time that the global position validity recovery check failed (usec) */
	hrt_abstime	_last_lpos_fail_time_us{0};	/**< Last time that the local position validity recovery check failed (usec) */
	hrt_abstime	_last_lvel_fail_time_us{0};	/**< Last time that the local velocity validity recovery check failed (usec) */

	// Probation times for position and velocity validity checks to pass if failed
	hrt_abstime	_gpos_probation_time_us = POSVEL_PROBATION_MIN;
	hrt_abstime	_lpos_probation_time_us = POSVEL_PROBATION_MIN;
	hrt_abstime	_lvel_probation_time_us = POSVEL_PROBATION_MIN;

	/* class variables used to check for navigation failure after takeoff */
	hrt_abstime	_time_at_takeoff{0};		/**< last time we were on the ground */
	hrt_abstime	_time_last_innov_pass{0};	/**< last time velocity or position innovations passed */
	bool		_nav_test_passed{false};	/**< true if the post takeoff navigation test has passed */
	bool		_nav_test_failed{false};	/**< true if the post takeoff navigation test has failed */

	/* class variables used to check for airspeed sensor failure */
	bool		_tas_check_fail{false};	/**< true when airspeed innovations have failed consistency checks */
	hrt_abstime	_time_last_tas_pass{0};		/**< last time innovation checks passed */
	hrt_abstime	_time_last_tas_fail{0};		/**< last time innovation checks failed */
	static constexpr hrt_abstime TAS_INNOV_FAIL_DELAY{1_s};	/**< time required for innovation levels to pass or fail (usec) */
	bool		_tas_use_inhibit{false};	/**< true when the commander has instructed the control loops to not use airspeed data */
	hrt_abstime	_time_tas_good_declared{0};	/**< time TAS use was started (uSec) */
	hrt_abstime	_time_tas_bad_declared{0};	/**< time TAS use was stopped (uSec) */
	hrt_abstime	_time_last_airspeed{0};		/**< time last airspeed measurement was received (uSec) */
	hrt_abstime	_time_last_aspd_innov_check{0};	/**< time airspeed innovation was last checked (uSec) */
	char		*_airspeed_fault_type = new char[7];
	float		_load_factor_ratio{0.5f};	/**< ratio of maximum load factor predicted by stall speed to measured load factor */
	float		_apsd_innov_integ_state{0.0f};	/**< inegral of excess normalised airspeed innovation (sec) */

	bool _geofence_loiter_on{false};
	bool _geofence_rtl_on{false};
	bool _geofence_warning_action_on{false};
	bool _geofence_violated_prev{false};

	FailureDetector _failure_detector;
	bool _flight_termination_triggered{false};

	bool handle_command(vehicle_status_s *status, const vehicle_command_s &cmd, actuator_armed_s *armed,
			    uORB::PublicationQueued<vehicle_command_ack_s> &command_ack_pub, bool *changed);

	unsigned handle_command_motor_test(const vehicle_command_s &cmd);

	bool set_home_position();
	bool set_home_position_alt_only();

	// Set the main system state based on RC and override device inputs
	transition_result_t set_main_state(const vehicle_status_s &status, bool *changed);

	// Enable override (manual reversion mode) on the system
	transition_result_t set_main_state_override_on(const vehicle_status_s &status, bool *changed);

	// Set the system main state based on the current RC inputs
	transition_result_t set_main_state_rc(const vehicle_status_s &status, bool *changed);

	void update_control_mode();

	void check_valid(const hrt_abstime &timestamp, const hrt_abstime &timeout, const bool valid_in, bool *valid_out,
			 bool *changed);

	bool check_posvel_validity(const bool data_valid, const float data_accuracy, const float required_accuracy,
				   const hrt_abstime &data_timestamp_us, hrt_abstime *last_fail_time_us, hrt_abstime *probation_time_us, bool *valid_state,
				   bool *validity_changed);

	void reset_posvel_validity(bool *changed);

	void mission_init();

	void estimator_check(bool *status_changed);

	void offboard_control_update(bool &status_changed);

	void airspeed_use_check();

	void battery_status_check();

	void esc_status_check(const esc_status_s &esc_status);

	/**
	 * Checks the status of all available data links and handles switching between different system telemetry states.
	 */
	void		data_link_check(bool &status_changed);

	uORB::Subscription _telemetry_status_sub{ORB_ID(telemetry_status)};

	hrt_abstime	_datalink_last_heartbeat_gcs{0};

	hrt_abstime	_datalink_last_heartbeat_onboard_controller{0};
	bool 				_onboard_controller_lost{false};

	hrt_abstime	_datalink_last_heartbeat_avoidance_system{0};
	bool				_avoidance_system_lost{false};

	bool		_avoidance_system_status_change{false};
	uint8_t	_datalink_last_status_avoidance_system{telemetry_status_s::MAV_STATE_UNINIT};

	uORB::Subscription _iridiumsbd_status_sub{ORB_ID(iridiumsbd_status)};

	hrt_abstime	_high_latency_datalink_heartbeat{0};
	hrt_abstime	_high_latency_datalink_lost{0};

	int  _last_esc_online_flags{-1};

	uORB::Subscription _battery_sub{ORB_ID(battery_status)};
	uint8_t _battery_warning{battery_status_s::BATTERY_WARNING_NONE};
	float _battery_current{0.0f};

	systemlib::Hysteresis	_auto_disarm_landed{false};
	systemlib::Hysteresis	_auto_disarm_killed{false};

	bool _print_avoidance_msg_once{false};

	// Subscriptions
	uORB::Subscription					_parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription					_vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};

	uORB::SubscriptionData<airspeed_s>			_airspeed_sub{ORB_ID(airspeed)};
	uORB::SubscriptionData<estimator_status_s>		_estimator_status_sub{ORB_ID(estimator_status)};
	uORB::SubscriptionData<mission_result_s>		_mission_result_sub{ORB_ID(mission_result)};
	uORB::SubscriptionData<offboard_control_mode_s>		_offboard_control_mode_sub{ORB_ID(offboard_control_mode)};
	uORB::SubscriptionData<vehicle_global_position_s>	_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::SubscriptionData<vehicle_local_position_s>	_local_position_sub{ORB_ID(vehicle_local_position)};

	// Publications
	uORB::Publication<vehicle_control_mode_s>		_control_mode_pub{ORB_ID(vehicle_control_mode)};
	uORB::Publication<vehicle_status_s>			_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<actuator_armed_s>			_armed_pub{ORB_ID(actuator_armed)};
	uORB::Publication<commander_state_s>			_commander_state_pub{ORB_ID(commander_state)};
	uORB::Publication<vehicle_status_flags_s>		_vehicle_status_flags_pub{ORB_ID(vehicle_status_flags)};
	uORB::Publication<test_motor_s>				_test_motor_pub{ORB_ID(test_motor)};

	uORB::PublicationData<home_position_s>			_home_pub{ORB_ID(home_position)};

};

#endif /* COMMANDER_HPP_ */
