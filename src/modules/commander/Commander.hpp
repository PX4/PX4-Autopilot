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

#include <controllib/blocks.hpp>

// publications
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/Publication.hpp>

// subscriptions
#include <uORB/topics/geofence_result.h>
#include <uORB/Subscription.hpp>

using control::BlockParamBool;
using control::BlockParamFloat;
using control::BlockParamInt;
using uORB::Publication;
using uORB::Subscription;

class Commander : public control::SuperBlock
{
public:
	Commander() :
		SuperBlock(nullptr, "COM"),
		_param_arm_mission_required(this, "ARM_MIS_REQ"),
		_param_arm_switch_is_button(this, "ARM_SWISBTN"),
		_param_arm_without_gps(this, "ARM_WO_GPS"),
		_param_autostart_id(this, "SYS_AUTOSTART", false),
		_param_datalink_loss_act(this, "NAV_DLL_ACT", false),
		_param_datalink_loss_timeout(this, "DL_LOSS_T"),
		_param_datalink_regain_timeout(this, "DL_REG_T"),
		_param_disarm_land(this, "DISARM_LAND"),
		_param_ef_current2throttle_thres(this, "EF_C2T"),
		_param_ef_throttle_thres(this, "EF_THROT"),
		_param_ef_time_thres(this, "EF_TIME"),
		_param_eph(this, "HOME_H_T"),
		_param_epv(this, "HOME_V_T"),
		_param_fmode_1(this, "FLTMODE1"),
		_param_fmode_2(this, "FLTMODE2"),
		_param_fmode_3(this, "FLTMODE3"),
		_param_fmode_4(this, "FLTMODE4"),
		_param_fmode_5(this, "FLTMODE5"),
		_param_fmode_6(this, "FLTMODE6"),
		_param_low_bat_act(this, "LOW_BAT_ACT"),
		_param_mav_comp_id(this, "MAV_COMP_ID", false),
		_param_mav_sys_id(this, "MAV_SYS_ID", false),
		_param_mav_type(this, "MAV_TYPE", false),
		_param_min_stick_change(this, "RC_STICK_OV"),
		_param_offboard_loss_act(this, "OBL_ACT"),
		_param_offboard_loss_rc_act(this, "OBL_RC_ACT"),
		_param_offboard_loss_timeout(this, "OF_LOSS_T"),
		_param_posctl_nav_loss_act(this, "POSCTL_NAVL"),
		_param_rc_arm_hyst(this, "RC_ARM_HYST"),
		_param_rc_in_off(this, "RC_IN_MODE"),
		_param_rc_loss_act(this, "NAV_RCL_ACT", false),
		_param_rc_loss_timeout(this, "RC_LOSS_T"),
		_param_rc_override(this, "RC_OVERRIDE"),
		//_pub_commander_state(ORB_ID(commander_state), -1, &getPublications()),
		_pub_control_mode(ORB_ID(vehicle_control_mode), -1, &getPublications()),
		//_pub_status(ORB_ID(vehicle_status), -1, &getPublications()),
		_pub_status_flags(ORB_ID(vehicle_status_flags), -1, &getPublications()),
		_sub_geofence_result(ORB_ID(geofence_result), 0, 0, &getSubscriptions())
	{
		updateParams();
	}

	~Commander() = default;

	static void	task_main_trampoline(int argc, char *argv[]);
	int commander_thread_main(int argc, char *argv[]);

private:

	BlockParamBool _param_arm_mission_required;
	BlockParamInt _param_arm_switch_is_button;
	BlockParamBool _param_arm_without_gps;

	BlockParamInt _param_autostart_id;

	BlockParamInt _param_datalink_loss_act;
	BlockParamInt _param_datalink_loss_timeout;
	BlockParamInt _param_datalink_regain_timeout;

	BlockParamInt _param_disarm_land;

	BlockParamFloat _param_ef_current2throttle_thres;
	BlockParamFloat _param_ef_throttle_thres;
	BlockParamFloat _param_ef_time_thres;

	BlockParamFloat _param_eph;
	BlockParamFloat _param_epv;

	BlockParamInt _param_fmode_1;
	BlockParamInt _param_fmode_2;
	BlockParamInt _param_fmode_3;
	BlockParamInt _param_fmode_4;
	BlockParamInt _param_fmode_5;
	BlockParamInt _param_fmode_6;

	BlockParamInt _param_low_bat_act;

	BlockParamInt _param_mav_comp_id;
	BlockParamInt _param_mav_sys_id;
	BlockParamInt _param_mav_type;

	BlockParamFloat _param_min_stick_change;

	BlockParamInt _param_offboard_loss_act;
	BlockParamInt _param_offboard_loss_rc_act;
	BlockParamFloat _param_offboard_loss_timeout;

	// failsafe response to loss of navigation accuracy
	BlockParamInt _param_posctl_nav_loss_act;

	BlockParamInt _param_rc_arm_hyst;
	BlockParamInt _param_rc_in_off;
	BlockParamInt _param_rc_loss_act;
	BlockParamFloat _param_rc_loss_timeout;
	BlockParamBool _param_rc_override;


	void publish_control_mode();
	void publish_status_flags();

	// publications

	//Publication<actuator_armed> _pub_actuator_armed;
	//Publication<commander_state_s> _pub_commander_state;
	//Publication<home_position> _pub_home;
	//Publication<offboard_mission_s> _pub_offboard_mission;
	//Publication<vehicle_command_ack_s> _pub_command_ack;
	Publication<vehicle_control_mode_s> _pub_control_mode;
	//Publication<vehicle_roi_s> _pub_roi;
	//Publication<vehicle_status_s> _pub_status;
	Publication<vehicle_status_flags_s> _pub_status_flags;

	// subscriptions
	Subscription<geofence_result_s> _sub_geofence_result;

	// actuator_controls_0
	// battery_status
	// cpuload
	// differential_pressure
	// geofence_result
	// manual_control_setpoint
	// mission_result
	// offboard_control_mode
	// parameter_update
	// position_setpoint_triplet
	// safety
	// sensor_combined
	// subsystem_info
	// system_power
	// telemetry_status
	// vehicle_attitude
	// vehicle_command
	// vehicle_global_position
	// vehicle_gps_position
	// vehicle_land_detected
	// vehicle_local_position
	// vtol_vehicle_status

	static constexpr int32_t VEHICLE_TYPE_QUADROTOR = 2;
	static constexpr int32_t VEHICLE_TYPE_COAXIAL = 3;
	static constexpr int32_t VEHICLE_TYPE_HELICOPTER = 4;
	static constexpr int32_t VEHICLE_TYPE_HEXAROTOR = 13;
	static constexpr int32_t VEHICLE_TYPE_OCTOROTOR = 14;
	static constexpr int32_t VEHICLE_TYPE_TRICOPTER = 15;

	static constexpr int32_t VEHICLE_TYPE_VTOL_DUOROTOR = 19;
	static constexpr int32_t VEHICLE_TYPE_VTOL_QUADROTOR = 20;
	static constexpr int32_t VEHICLE_TYPE_VTOL_TILTROTOR = 21;
	static constexpr int32_t VEHICLE_TYPE_VTOL_RESERVED2 = 22;
	static constexpr int32_t VEHICLE_TYPE_VTOL_RESERVED3 = 23;
	static constexpr int32_t VEHICLE_TYPE_VTOL_RESERVED4 = 24;
	static constexpr int32_t VEHICLE_TYPE_VTOL_RESERVED5 = 25;

	bool is_multirotor() const;
	bool is_rotary_wing() const;
	bool is_vtol() const;


};

namespace commander
{
extern Commander *g_control;
} // namespace commander

#endif /* COMMANDER_HPP_ */
