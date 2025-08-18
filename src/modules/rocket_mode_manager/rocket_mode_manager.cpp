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
 * @file rocket_mode_manager.cpp
 *
 * Rocket Mode Manager for rocket-plane vehicles.
 * Handles:
 * - Roll-only control during rocket boost phase
 * - Apogee detection
 * - Wing deployment
 * - Transition to fixed-wing manual mode
 *
 * @author Your Name <your.email@example.com>
 */

#include "rocket_mode_manager.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <modules/commander/px4_custom_mode.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <cstring>
#include <cstdio>

extern orb_advert_t mavlink_log_pub;

using namespace time_literals;

RocketModeManager::RocketModeManager() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	// Initialize state
	_rocket_state = RocketState::WAITING_GCS_CONNECT;
	_last_rocket_state = RocketState::WAITING_GCS_CONNECT;
	_wing_deployed = false;
	_apogee_detected = false;
	_launch_detected = false;
	_rocket_config_applied = false;
	_max_altitude = -1000.0f;
	_wing_deploy_time = 0;
	_launch_detect_time = 0;
	_boost_end_time = 0;
	_coast_start_time = 0;
	_last_status_time = 0;
	_gcs_connect_time = 0;

	// No simulation logic needed - handled by PX4 simulation modules
}

RocketModeManager::~RocketModeManager()
{
}

bool RocketModeManager::init()
{
	// Subscribe to required topics
	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position callback registration failed!");
		return false;
	}

	// Initialize parameters
	parameters_update(true);

	// Start the work queue
	ScheduleOnInterval(10_ms); // Run at 100 Hz

	PX4_INFO("Rocket Mode Manager initialized - waiting for QGroundControl connection");

	return true;
}

void RocketModeManager::parameters_update(bool force)
{
	// Check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// Clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// Update parameters from storage
		updateParams();

	}
}

void RocketModeManager::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Update parameters
	parameters_update();

	// Update all subscriptions
	_vehicle_local_position_sub.update(&_vehicle_local_position);
	_vehicle_status_sub.update(&_vehicle_status);
	_manual_control_switches_sub.update(&_manual_control_switches);
	_sensor_accel_sub.update(&_sensor_accel);
	_vehicle_attitude_sub.update(&_vehicle_attitude);
	_manual_control_setpoint_sub.update(&_manual_control_setpoint);
	_telemetry_status_sub.update(&_telemetry_status);	switch (_rocket_state) {

	case RocketState::WAITING_GCS_CONNECT:
		handle_waiting_gcs_connect_phase();
		break;

	case RocketState::WAITING_LAUNCH:
		handle_waiting_launch_phase();
		break;

	case RocketState::ROCKET_BOOST:
		handle_rocket_boost_phase();
		break;

	case RocketState::ROCKET_COAST:
		handle_rocket_coast_phase();
		break;

	case RocketState::WING_DEPLOYMENT:
		handle_wing_deployment_phase();
		break;

	case RocketState::FIXED_WING:
		handle_fixed_wing_phase();
		break;

	case RocketState::LANDED:
		// No handling needed for landed state in rocket mode manager
		break;
	}

	// Publish rocket status
	publish_rocket_status();
}

void RocketModeManager::handle_waiting_gcs_connect_phase()
{
	// Wait for QGroundControl connection
	if (_telemetry_status.heartbeat_type_gcs) {
		// Record first time we detected GCS connection
		if (_gcs_connect_time == 0) {
			_gcs_connect_time = hrt_absolute_time();
			PX4_INFO("QGroundControl connected - waiting 2 seconds for initialization");
			return;
		}

		// Wait 2 seconds after GCS connection before applying rocket configuration
		const hrt_abstime elapsed = hrt_absolute_time() - _gcs_connect_time;
		const hrt_abstime delay_duration = 2_s; // 2 second delay

		if (elapsed > delay_duration) {
			// GCS connection stabilized - apply complete rocket configuration
			switch_to_rocket_roll_mode();

			// Transition to WAITING_LAUNCH (Ready disarmed state)
			_rocket_state = RocketState::WAITING_LAUNCH;

			PX4_INFO("GCS initialization complete - Applied rocket roll config, now Ready (disarmed)");
		}
	} else {
		// Reset connection timer if GCS disconnects
		_gcs_connect_time = 0;
	}
}

void RocketModeManager::handle_waiting_launch_phase()
{
	// Wait for launch detection when armed
	if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {

		if (_vehicle_local_position.timestamp > 0 && _sensor_accel.timestamp > 0) {
			const float current_altitude = -_vehicle_local_position.z; // NED frame
			const float vertical_velocity = _vehicle_local_position.vz;

			// Calculate total acceleration magnitude
			const float accel_magnitude = sqrtf(_sensor_accel.x * _sensor_accel.x +
			                                   _sensor_accel.y * _sensor_accel.y +
			                                   _sensor_accel.z * _sensor_accel.z);

			// Launch detection criteria: high acceleration OR significant upward velocity
			if (_rocket_state == RocketState::WAITING_LAUNCH) {
				const float launch_accel_threshold = _param_rocket_launch_a.get(); // e.g., 15 m/s^2
				const float launch_vel_threshold = _param_rocket_launch_v.get();     // e.g., -3 m/s (upward)

				   if (!_launch_detected &&
					   (accel_magnitude > launch_accel_threshold || vertical_velocity <= launch_vel_threshold)) {

						   PX4_INFO("Launch detected! Accel: %.1f m/s^2, Vel: %.1f m/s", (double)accel_magnitude, (double)vertical_velocity);
						   _launch_detected = true;
						   _launch_detect_time = hrt_absolute_time();
						   _rocket_state = RocketState::ROCKET_BOOST;
						   _max_altitude = current_altitude; // Initialize max altitude tracking

						   // Optionally, also request custom mode for QGC display
						   request_rocket_navigation_state();

						   // Send immediate status update
						   publish_rocket_status();
				   }
			}
		}
	}
}

void RocketModeManager::handle_rocket_boost_phase()
{
	// During boost phase, monitor for end of motor burn

	if (_vehicle_local_position.timestamp > 0 && _sensor_accel.timestamp > 0) {
		const float current_altitude = -_vehicle_local_position.z; // NED frame
		//const float vertical_velocity = _vehicle_local_position.vz;

		// Track maximum altitude
		if (current_altitude > _max_altitude) {
			_max_altitude = current_altitude;
		}

		// Calculate total acceleration magnitude
		const float accel_magnitude = sqrtf(_sensor_accel.x * _sensor_accel.x +
		                                   _sensor_accel.y * _sensor_accel.y +
		                                   _sensor_accel.z * _sensor_accel.z);

		// End of boost detection: acceleration drops below threshold
		if (_rocket_state == RocketState::ROCKET_BOOST) {
			const float boost_end_accel_threshold = _param_rocket_boost_a.get(); // e.g., 12 m/s^2

			if (accel_magnitude < boost_end_accel_threshold) {
				PX4_INFO("Boost phase ended, transitioning to coast. Accel: %.1f m/s^2", (double)accel_magnitude);
				_rocket_state = RocketState::ROCKET_COAST;
				_boost_end_time = hrt_absolute_time();
				_coast_start_time = hrt_absolute_time();

				// Update navigation state display in QGC
				request_rocket_navigation_state();

				// Send immediate status update
				publish_rocket_status();
			}

			// Failsafe: Maximum boost duration (in case acceleration never drops)
			const hrt_abstime boost_duration = hrt_absolute_time() - _launch_detect_time;
			const hrt_abstime max_boost_duration = (hrt_abstime)(_param_rocket_boost_t.get() * 1e6f); // Convert to microseconds

			if (boost_duration > max_boost_duration) {
				PX4_WARN("Boost phase timeout, forcing transition to coast");
				_rocket_state = RocketState::ROCKET_COAST;
				_boost_end_time = hrt_absolute_time();
				_coast_start_time = hrt_absolute_time();

				// Update navigation state display in QGC
				request_rocket_navigation_state();

				// Send immediate status update
				publish_rocket_status();
			}
		}
	}
}

void RocketModeManager::handle_rocket_coast_phase()
{
	// Monitor for apogee detection with timer failsafe
	if (_vehicle_local_position.timestamp > 0 && _vehicle_attitude.timestamp > 0) {
		const float current_altitude = -_vehicle_local_position.z;
		//const float vertical_velocity = _vehicle_local_position.vz;

		// Convert NED velocities to body frame using rotation matrix
		// Body frame: X=forward (roll axis), Y=right, Z=down
		matrix::Dcmf R(matrix::Quatf(_vehicle_attitude.q));
		matrix::Vector3f vel_ned(_vehicle_local_position.vx, _vehicle_local_position.vy, _vehicle_local_position.vz);
		matrix::Vector3f vel_body = R.transpose() * vel_ned;

		const float body_x_velocity = vel_body(0); // Velocity along roll axis (forward)

		// Update maximum altitude
		if (current_altitude > _max_altitude) {
			_max_altitude = current_altitude;
		}

		// Check coast phase timer failsafe first
		const hrt_abstime coast_duration = hrt_absolute_time() - _coast_start_time;
		const hrt_abstime max_coast_duration = (hrt_abstime)(_param_rocket_coast_t.get() * 1e6f);

		bool deploy_wings = false;
		const char* deploy_reason = "";

		// Primary deployment criteria: body X velocity threshold OR altitude descent OR failsafe
		const float velocity_threshold = _param_rocket_deploy_v.get(); // e.g., 5.0 m/s (body X-axis forward velocity)
		const float altitude_descent_threshold = _param_rocket_alt_thresh.get(); // Deploy if descended this many meters from max

		if (!_apogee_detected) {
			// Check deployment criteria
			if (fabsf(body_x_velocity) <= velocity_threshold) {
				deploy_wings = true;
				deploy_reason = "body X velocity threshold";
			} else if (current_altitude < (_max_altitude - altitude_descent_threshold)) {
				deploy_wings = true;
				deploy_reason = "altitude descent";
			} else if (coast_duration > max_coast_duration) {
				deploy_wings = true;
				deploy_reason = "coast timeout failsafe";
			}

			if (deploy_wings) {
				PX4_INFO("Apogee/deployment triggered by %s at altitude %.1f m (max: %.1f m), body X vel: %.1f m/s",
				         deploy_reason, (double)current_altitude, (double)_max_altitude, (double)body_x_velocity);
				_apogee_detected = true;
				_rocket_state = RocketState::WING_DEPLOYMENT;
				_wing_deploy_time = hrt_absolute_time();

				// Update navigation state display in QGC
				request_rocket_navigation_state();

				// Trigger wing deployment
				deploy_wings_function();

				// Send immediate status update
				publish_rocket_status();
			}
		}
	}
}

void RocketModeManager::handle_wing_deployment_phase()
{
	PX4_INFO("Wing deployment complete, switching to fixed-wing mode");
	_wing_deployed = true;
	_rocket_state = RocketState::FIXED_WING;

	// Update navigation state display in QGC
	request_rocket_navigation_state();

	// Request transition to fixed-wing mode with CA reconfiguration
	switch_to_fixed_wing_mode();

	// Send immediate status update
	publish_rocket_status();

	// Note: Rocket mode manager continues to monitor in fixed-wing phase
	PX4_INFO("Rocket logic complete, now monitoring fixed-wing phase");
}

void RocketModeManager::handle_fixed_wing_phase()
{
	// Monitor the vehicle in fixed-wing mode
	// Could add additional safety checks here

	// Allow pilot full manual control
	// No additional automated intervention needed
}

void RocketModeManager::deploy_wings_function()
{
	// Publish wing deployment command - the mixer module will handle the rest
	wing_deploy_command_s wing_cmd{};
	wing_cmd.timestamp = hrt_absolute_time();
	wing_cmd.deploy = true;
	_wing_deploy_pub.publish(wing_cmd);

	PX4_INFO("ROCKET: Publishing wing deployment command");
}

void RocketModeManager::configure_rocket_ca_parameters()
{
	PX4_INFO("Configuring control surfaces for rocket phase");

	// Configure each servo (0-5) with rocket phase settings
	for (int i = 0; i < 6; i++) {
		char param_name[32];

		// Get control surface type from RKT_CS_R0 through RKT_CS_R5
		snprintf(param_name, sizeof(param_name), "RKT_CS_R%d", i);
		param_t cs_param = param_find(param_name);
		int32_t cs_type = 0;
		if (cs_param != PARAM_INVALID) {
			param_get(cs_param, &cs_type);
		}

		// Set control surface type with validation
		snprintf(param_name, sizeof(param_name), "CA_SV_CS%d_TYPE", i);
		param_t type_param = param_find(param_name);
		if (type_param != PARAM_INVALID) {
			param_set(type_param, &cs_type);
			PX4_DEBUG("Set %s = %d", param_name, cs_type);
		} else {
			PX4_ERR("Parameter %s not found", param_name);
		}

		// Get torque matrix values from RKT_CA_R0 through RKT_CA_R17 (3 components per servo: X, Y, Z)
		const int matrix_idx = i * 3;
		float trq_values[3] = {0.0f, 0.0f, 0.0f};

		for (int j = 0; j < 3; j++) {
			snprintf(param_name, sizeof(param_name), "RKT_CA_R%d", matrix_idx + j);
			param_t trq_param = param_find(param_name);
			if (trq_param != PARAM_INVALID) {
				param_get(trq_param, &trq_values[j]);
				PX4_DEBUG("Got %s = %.3f", param_name, (double)trq_values[j]);
			} else {
				PX4_WARN("Parameter %s not found, using default 0.0", param_name);
			}
		}

		// Set torque values with validation
		snprintf(param_name, sizeof(param_name), "CA_SV_CS%d_TRQ_R", i);
		param_t trq_r_param = param_find(param_name);
		if (trq_r_param != PARAM_INVALID) {
			param_set(trq_r_param, &trq_values[0]); // X component -> Roll
			PX4_DEBUG("Set %s = %.3f", param_name, (double)trq_values[0]);
		}

		snprintf(param_name, sizeof(param_name), "CA_SV_CS%d_TRQ_P", i);
		param_t trq_p_param = param_find(param_name);
		if (trq_p_param != PARAM_INVALID) {
			param_set(trq_p_param, &trq_values[1]); // Y component -> Pitch
			PX4_DEBUG("Set %s = %.3f", param_name, (double)trq_values[1]);
		}

		snprintf(param_name, sizeof(param_name), "CA_SV_CS%d_TRQ_Y", i);
		param_t trq_y_param = param_find(param_name);
		if (trq_y_param != PARAM_INVALID) {
			param_set(trq_y_param, &trq_values[2]); // Z component -> Yaw
			PX4_DEBUG("Set %s = %.3f", param_name, (double)trq_values[2]);
		}
	}

	// Small delay to ensure all parameter sets are complete
	px4_usleep(10000); // 10ms delay

	// Notify parameter changes to trigger control allocator reconfiguration
	param_notify_changes();

	PX4_INFO("Rocket CA configuration applied: fins 1-4 for roll control, servos 5-6 inactive");
}

void RocketModeManager::configure_fixedwing_ca_parameters()
{
	PX4_INFO("Configuring control surfaces for fixed-wing phase");

	// Configure each servo (0-5) with fixed-wing phase settings
	for (int i = 0; i < 6; i++) {
		char param_name[32];

		// Get control surface type from RKT_CS_F0 through RKT_CS_F5
		snprintf(param_name, sizeof(param_name), "RKT_CS_F%d", i);
		param_t cs_param = param_find(param_name);
		int32_t cs_type = 0;
		if (cs_param != PARAM_INVALID) {
			param_get(cs_param, &cs_type);
		}

		// Set control surface type with validation
		snprintf(param_name, sizeof(param_name), "CA_SV_CS%d_TYPE", i);
		param_t type_param = param_find(param_name);
		if (type_param != PARAM_INVALID) {
			param_set(type_param, &cs_type);
			PX4_DEBUG("Set %s = %d", param_name, cs_type);
		} else {
			PX4_ERR("Parameter %s not found", param_name);
		}

		// Get torque matrix values from RKT_CA_F0 through RKT_CA_F17 (3 components per servo: X, Y, Z)
		const int matrix_idx = i * 3;
		float trq_values[3] = {0.0f, 0.0f, 0.0f};

		for (int j = 0; j < 3; j++) {
			snprintf(param_name, sizeof(param_name), "RKT_CA_F%d", matrix_idx + j);
			param_t trq_param = param_find(param_name);
			if (trq_param != PARAM_INVALID) {
				param_get(trq_param, &trq_values[j]);
				PX4_DEBUG("Got %s = %.3f", param_name, (double)trq_values[j]);
			} else {
				PX4_WARN("Parameter %s not found, using default 0.0", param_name);
			}
		}

		// Set torque values with validation
		snprintf(param_name, sizeof(param_name), "CA_SV_CS%d_TRQ_R", i);
		param_t trq_r_param = param_find(param_name);
		if (trq_r_param != PARAM_INVALID) {
			param_set(trq_r_param, &trq_values[0]); // X component -> Roll
			PX4_DEBUG("Set %s = %.3f", param_name, (double)trq_values[0]);
		}

		snprintf(param_name, sizeof(param_name), "CA_SV_CS%d_TRQ_P", i);
		param_t trq_p_param = param_find(param_name);
		if (trq_p_param != PARAM_INVALID) {
			param_set(trq_p_param, &trq_values[1]); // Y component -> Pitch
			PX4_DEBUG("Set %s = %.3f", param_name, (double)trq_values[1]);
		}

		snprintf(param_name, sizeof(param_name), "CA_SV_CS%d_TRQ_Y", i);
		param_t trq_y_param = param_find(param_name);
		if (trq_y_param != PARAM_INVALID) {
			param_set(trq_y_param, &trq_values[2]); // Z component -> Yaw
			PX4_DEBUG("Set %s = %.3f", param_name, (double)trq_values[2]);
		}
	}
	// Small delay to ensure all parameter sets are complete
	px4_usleep(10000); // 10ms delay

	// Notify parameter changes to trigger control allocator reconfiguration
	param_notify_changes();

	PX4_INFO("Fixed-wing CA configuration applied: fins 1,3=elevators, fins 2,4=rudders, servos 5,6=ailerons");
}void RocketModeManager::switch_to_fixed_wing_mode()
{
	// Configure fixed-wing control allocation
	configure_fixedwing_ca_parameters();

	// Update available navigation states to all fixed-wing modes
	// Allow standard fixed-wing modes: Manual, Altitude, Position, Auto Mission, Auto Loiter, Auto RTL, Acro
	// Bitmask: 0x0000007F = bits 0-6 for standard fixed-wing modes
	int32_t fw_mask = 0x0000007F;
	_param_rocket_nav_mask.set(fw_mask);
	_param_rocket_nav_mask.commit_no_notification();

	PX4_INFO("Navigation modes updated: All fixed-wing modes now available");

	// Request mode switch to manual mode for fixed-wing
	action_request_s action_request{};
	action_request.timestamp = hrt_absolute_time();
	action_request.action = action_request_s::ACTION_SWITCH_MODE;
	action_request.mode = vehicle_status_s::NAVIGATION_STATE_MANUAL; // Use manual mode for fixed-wing
	action_request.source = action_request_s::SOURCE_RC_SWITCH;

	_action_request_pub.publish(action_request);

	PX4_INFO("Switched to manual fixed-wing mode with dynamic CA configuration");
	PX4_INFO("Rocket phase complete - rocket_mode_manager will continue monitoring");
}

void RocketModeManager::switch_to_rocket_roll_mode()
{
	// Configure rocket control allocation
	configure_rocket_ca_parameters();

	// Force restore the rocket navigation mask to restrict available modes
	int32_t rocket_mask = -2147483648; // Default rocket roll only mask
	_param_rocket_nav_mask.set(rocket_mask);
	_param_rocket_nav_mask.commit_no_notification();
	PX4_INFO("Rocket navigation mask restored: %d", rocket_mask);

	// Switch to the custom NAVIGATION_STATE_ROCKET_ROLL that we implemented
	// The nav_state_names array should provide "Rocket Roll" display name to QGC via AVAILABLE_MODES
	action_request_s action_request{};
	action_request.timestamp = hrt_absolute_time();
	action_request.action = action_request_s::ACTION_SWITCH_MODE;
	action_request.mode = vehicle_status_s::NAVIGATION_STATE_ROCKET_ROLL; // Index 31, should show "Rocket Roll"
	action_request.source = action_request_s::SOURCE_RC_SWITCH;

	_action_request_pub.publish(action_request);

	// Also send custom status text for additional clarity
	request_rocket_navigation_state();

	_rocket_config_applied = true;

	PX4_INFO("Switched to NAVIGATION_STATE_ROCKET_ROLL mode (displays as 'Acro' in QGC)");
}

void RocketModeManager::request_rocket_navigation_state()
{
	// Send custom mode text to QGroundControl via MAVLink status text
	char mode_text[50];

	switch (_rocket_state) {
	case RocketState::WAITING_GCS_CONNECT:
		snprintf(mode_text, sizeof(mode_text), "WAITING QGC");
		break;
	case RocketState::WAITING_LAUNCH:
		snprintf(mode_text, sizeof(mode_text), "ROCKET READY");
		break;
	case RocketState::ROCKET_BOOST:
		snprintf(mode_text, sizeof(mode_text), "ROCKET BOOST");
		break;
	case RocketState::ROCKET_COAST:
		snprintf(mode_text, sizeof(mode_text), "ROCKET COAST");
		break;
	case RocketState::WING_DEPLOYMENT:
		snprintf(mode_text, sizeof(mode_text), "WING DEPLOY");
		break;
	case RocketState::FIXED_WING:
		snprintf(mode_text, sizeof(mode_text), "FIXED WING");
		break;
	case RocketState::LANDED:
		snprintf(mode_text, sizeof(mode_text), "LANDED");
		break;
	}

	// Send the mode as a status text (this will show in QGC)
	send_status_text(mode_text);
}

void RocketModeManager::send_status_text(const char* text)
{
	// Send status text to QGC via MAVLink
	mavlink_log_info(&mavlink_log_pub, "%s", text);
}

void RocketModeManager::publish_rocket_status()
{
	// Check if state changed or if it's time for a periodic update
	const hrt_abstime now = hrt_absolute_time();
	const bool state_changed = (_rocket_state != _last_rocket_state);
	const bool periodic_update = (now - _last_status_time > 5_s); // Update every 5 seconds

	if (state_changed || periodic_update) {
		char status_text[50];

		// Create status text based on current state
		switch (_rocket_state) {
		case RocketState::WAITING_GCS_CONNECT:
			snprintf(status_text, sizeof(status_text), "ROCKET: Waiting for QGroundControl");
			break;

		case RocketState::WAITING_LAUNCH:
			if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				snprintf(status_text, sizeof(status_text), "ROCKET: Armed - Waiting for Launch");
			} else {
				snprintf(status_text, sizeof(status_text), "ROCKET: Ready - Disarmed");
			}
			break;

		case RocketState::ROCKET_BOOST:
			snprintf(status_text, sizeof(status_text), "ROCKET: Boost Phase - Alt %.0fm", (double)(-_vehicle_local_position.z));
			break;

		case RocketState::ROCKET_COAST:
			snprintf(status_text, sizeof(status_text), "ROCKET: Coast to Apogee - Alt %.0fm", (double)(-_vehicle_local_position.z));
			break;

		case RocketState::WING_DEPLOYMENT:
			snprintf(status_text, sizeof(status_text), "ROCKET: Wing Deployment - Alt %.0fm", (double)(-_vehicle_local_position.z));
			break;

		case RocketState::FIXED_WING:
			snprintf(status_text, sizeof(status_text), "ROCKET: Fixed-Wing Mode - Alt %.0fm", (double)(-_vehicle_local_position.z));
			break;

		case RocketState::LANDED:
			snprintf(status_text, sizeof(status_text), "ROCKET: Landed");
			break;
		}


		// Send the status text to QGC
		send_status_text(status_text);

		// Update tracking variables
		_last_rocket_state = _rocket_state;
		_last_status_time = now;
	}
}

int RocketModeManager::task_spawn(int argc, char *argv[])
{
	RocketModeManager *instance = new RocketModeManager();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int RocketModeManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RocketModeManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rocket Mode Manager for rocket-plane vehicles.

Handles the flight phases:
1. Rocket boost phase - roll-only control
2. Coast phase - apogee detection
3. Wing deployment at apogee
4. Fixed-wing manual mode

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rocket_mode_manager", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rocket_mode_manager_main(int argc, char *argv[])
{
	return RocketModeManager::main(argc, argv);
}
