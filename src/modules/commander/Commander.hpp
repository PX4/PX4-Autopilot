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
#include <px4_module.h>

// publications
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>

// subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

using control::BlockParamFloat;
using control::BlockParamInt;
using uORB::Publication;
using uORB::Subscription;

class Commander : public control::SuperBlock, public ModuleBase<Commander>
{
public:
	Commander() :
		SuperBlock(nullptr, "COM"),
		_param_datalink_loss_timeout(this, "DL_LOSS_T"),
		_param_datalink_regain_timeout(this, "DL_REG_T"),
		_param_takeoff_finished_action(this, "TAKEOFF_ACT"),
		_mission_result_sub(ORB_ID(mission_result), 0, 0, &getSubscriptions()),
		_offboard_control_mode_sub(ORB_ID(offboard_control_mode), 0, 0, &getSubscriptions())
	{
		updateParams();
	}

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

private:

	BlockParamInt	_param_datalink_loss_timeout;
	BlockParamInt	_param_datalink_regain_timeout;

	BlockParamInt	_param_takeoff_finished_action;

	// Subscriptions
	Subscription<mission_result_s> _mission_result_sub;
	Subscription<offboard_control_mode_s> _offboard_control_mode_sub;

	orb_advert_t _control_mode_pub{nullptr};

	bool handle_command(vehicle_status_s *status, const safety_s *safety, vehicle_command_s *cmd,
			    actuator_armed_s *armed, home_position_s *home, vehicle_global_position_s *global_pos,
			    vehicle_local_position_s *local_pos, orb_advert_t *home_pub,
			    orb_advert_t *command_ack_pub, bool *changed);

	bool set_home_position(orb_advert_t &homePub, home_position_s &home,
				const vehicle_local_position_s &localPosition, const vehicle_global_position_s &globalPosition,
				bool set_alt_only_to_lpos_ref);

	bool publish_control_mode(const vehicle_status_s &vstatus, const offboard_control_mode_s &offboard_control_mode);

	void mission_init();

	// system power
	int _system_power_sub{-1};
	void check_system_power(vehicle_status_flags_s& vehicle_status_flags, float& power_rail_voltage);

	void check_mission(vehicle_status_s& vehicle_status, bool& status_changed);

	// data link
	int _telemetry_subs[ORB_MULTI_MAX_INSTANCES];
	uint64_t _telemetry_last_heartbeat[ORB_MULTI_MAX_INSTANCES];
	uint64_t _telemetry_last_dl_loss[ORB_MULTI_MAX_INSTANCES];
	bool _telemetry_preflight_checks_reported[ORB_MULTI_MAX_INSTANCES];
	bool _telemetry_lost[ORB_MULTI_MAX_INSTANCES];
	void check_data_link(vehicle_status_s& vehicle_status, bool& status_changed, bool& hotplug_timeout, bool checkAirspeed);

	// engine failure detection
	// TODO: move out of commander
	int _actuator_controls_sub{-1};
	float _ef_throttle_thres{1.0f};
	float _ef_current2throttle_thres{0.0f};
	float _ef_time_thres{1000.0f};
	uint64_t _timestamp_engine_healthy{0}; /**< absolute time when engine was healty */
	void check_engine_failure(vehicle_status_s& vehicle_status, bool& status_changed);

};

#endif /* COMMANDER_HPP_ */
