/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file rocket_mode_manager.hpp
 *
 * Rocket Mode Manager for rocket-plane vehicles.
 *
 * @author Your Name <your.email@example.com>
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/manual_control_switches.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/action_request.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/wing_deploy_command.h>
#include <uORB/topics/telemetry_status.h>
#include <lib/mathlib/mathlib.h>
#include <lib/systemlib/mavlink_log.h>

using namespace time_literals;

class RocketModeManager : public ModuleBase<RocketModeManager>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	RocketModeManager();
	~RocketModeManager() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * Check for parameter changes and update them if needed.
	 */
	void parameters_update(bool force = false);

	enum class RocketState {
		WAITING_GCS_CONNECT, // Waiting for QGroundControl connection
		WAITING_LAUNCH,      // Ready disarmed, waiting for launch detection
		ROCKET_BOOST,        // Roll-only control during boost
		ROCKET_COAST,        // Coasting to apogee, monitoring for deployment
		WING_DEPLOYMENT,     // Wings deploying at apogee
		FIXED_WING,          // Fixed-wing manual mode
		LANDED               // Landed state
	};

	// State machine handlers
	void handle_waiting_gcs_connect_phase();
	void handle_waiting_launch_phase();
	void handle_rocket_boost_phase();
	void handle_rocket_coast_phase();
	void handle_wing_deployment_phase();
	void handle_fixed_wing_phase();

	// Actions
	void deploy_wings_function();
	void switch_to_rocket_passive_mode();
	void switch_to_rocket_roll_mode();
	void switch_to_fixed_wing_mode();
	void configure_rocket_ca_parameters();
	void configure_fixedwing_ca_parameters();
	void announce_transition(const char* message);

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _manual_control_switches_sub{ORB_ID(manual_control_switches)};
	uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _telemetry_status_sub{ORB_ID(telemetry_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// Publications
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<actuator_test_s> _actuator_test_pub{ORB_ID(actuator_test)};
	uORB::Publication<action_request_s> _action_request_pub{ORB_ID(action_request)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<wing_deploy_command_s> _wing_deploy_pub{ORB_ID(wing_deploy_command)};

	// Data
	vehicle_local_position_s _vehicle_local_position{};
	vehicle_status_s _vehicle_status{};
	manual_control_switches_s _manual_control_switches{};
	sensor_accel_s _sensor_accel{};
	vehicle_attitude_s _vehicle_attitude{};
	manual_control_setpoint_s _manual_control_setpoint{};
	telemetry_status_s _telemetry_status{};

	// State variables
	RocketState _rocket_state;
	RocketState _last_rocket_state; // Track state changes for transitions
	bool _wing_deployed;
	bool _apogee_detected;
	bool _launch_detected;
	bool _rocket_config_applied; // Track if rocket configuration has been applied
	float _max_altitude;
	hrt_abstime _wing_deploy_time;
	hrt_abstime _launch_detect_time;
	hrt_abstime _boost_end_time;
	hrt_abstime _coast_start_time;
	hrt_abstime _last_status_time;
	hrt_abstime _gcs_connect_time;

	// MAVLink logging
	orb_advert_t _mavlink_log_pub{nullptr};

	// Parameters
	DEFINE_PARAMETERS(
			   (ParamFloat<px4::params::RKT_ALT_THRESH>) _param_rocket_alt_thresh,
			   (ParamFloat<px4::params::RKT_LAUNCH_A>) _param_rocket_launch_a,
			   (ParamFloat<px4::params::RKT_LAUNCH_V>) _param_rocket_launch_v,
			   (ParamFloat<px4::params::RKT_BOOST_A>) _param_rocket_boost_a,
			   (ParamFloat<px4::params::RKT_BOOST_T>) _param_rocket_boost_t,
			   (ParamFloat<px4::params::RKT_COAST_T>) _param_rocket_coast_t,
			   (ParamInt<px4::params::RKT_NAV_MASK>) _param_rocket_nav_mask
	)
};
