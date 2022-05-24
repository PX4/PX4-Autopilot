/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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

#include "logged_topics.h"
#include "messages.h"

#include <parameters/param.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/uORBTopics.hpp>

#include <string.h>

using namespace px4::logger;

void LoggedTopics::add_default_topics()
{
	add_topic("action_request");
	add_topic("actuator_armed");
	add_topic("actuator_controls_0", 50);
	add_topic("actuator_controls_1", 100);
	add_topic("actuator_controls_2", 100);
	add_topic("actuator_controls_3", 100);
	add_optional_topic("actuator_controls_status_0", 300);
	add_topic("airspeed", 1000);
	add_optional_topic("airspeed_validated", 200);
	add_optional_topic("autotune_attitude_control_status", 100);
	add_optional_topic("camera_capture");
	add_optional_topic("camera_trigger");
	add_topic("cellular_status", 200);
	add_topic("commander_state");
	add_topic("cpuload");
	add_optional_topic("esc_status", 250);
	add_topic("failure_detector_status", 100);
	add_optional_topic("follow_target", 500);
	add_optional_topic("generator_status");
	add_optional_topic("gps_dump");
	add_optional_topic("heater_status");
	add_topic("home_position");
	add_topic("hover_thrust_estimate", 100);
	add_topic("input_rc", 500);
	add_optional_topic("internal_combustion_engine_status", 10);
	add_optional_topic("irlock_report", 1000);
	add_optional_topic("landing_target_pose", 1000);
	add_optional_topic("magnetometer_bias_estimate", 200);
	add_topic("manual_control_setpoint", 200);
	add_topic("manual_control_switches");
	add_topic("mission_result");
	add_topic("navigator_mission_item");
	add_topic("npfg_status", 100);
	add_topic("offboard_control_mode", 100);
	add_topic("onboard_computer_status", 10);
	add_topic("parameter_update");
	add_topic("position_controller_status", 500);
	add_topic("position_setpoint_triplet", 200);
	add_optional_topic("px4io_status");
	add_topic("radio_status");
	add_topic("rtl_time_estimate", 1000);
	add_topic("sensor_combined");
	add_optional_topic("sensor_correction");
	add_optional_topic("sensor_gyro_fft", 50);
	add_topic("sensor_selection");
	add_topic("sensors_status_imu", 200);
	add_topic("system_power", 500);
	add_optional_topic("takeoff_status", 1000);
	add_optional_topic("tecs_status", 200);
	add_topic("trajectory_setpoint", 200);
	add_topic("transponder_report");
	add_topic("vehicle_acceleration", 50);
	add_topic("vehicle_air_data", 200);
	add_topic("vehicle_angular_velocity", 20);
	add_topic("vehicle_attitude", 50);
	add_topic("vehicle_attitude_setpoint", 50);
	add_topic("vehicle_command");
	add_topic("vehicle_constraints", 1000);
	add_topic("vehicle_control_mode");
	add_topic("vehicle_global_position", 200);
	add_topic("vehicle_gps_position", 500);
	add_topic("vehicle_land_detected");
	add_topic("vehicle_local_position", 100);
	add_topic("vehicle_local_position_setpoint", 100);
	add_topic("vehicle_magnetometer", 200);
	add_topic("vehicle_rates_setpoint", 20);
	add_topic("vehicle_roi", 1000);
	add_topic("vehicle_status");
	add_topic("vehicle_status_flags");
	add_optional_topic("vtol_vehicle_status", 200);
	add_topic("wind", 1000);

	// multi topics
	add_optional_topic_multi("actuator_outputs", 100, 3);
	add_optional_topic_multi("airspeed_wind", 1000, 4);
	add_optional_topic_multi("control_allocator_status", 200, 2);
	add_optional_topic_multi("rate_ctrl_status", 200, 2);
	add_optional_topic_multi("sensor_hygrometer", 500, 4);
	add_optional_topic_multi("rpm", 200);
	add_optional_topic_multi("telemetry_status", 1000, 4);

	// EKF multi topics (currently max 9 estimators)
#if CONSTRAINED_MEMORY
	static constexpr uint8_t MAX_ESTIMATOR_INSTANCES = 1;
#else
	static constexpr uint8_t MAX_ESTIMATOR_INSTANCES = 6; // artificially limited until PlotJuggler fixed
	add_optional_topic("estimator_selector_status");
	add_optional_topic_multi("estimator_attitude", 500, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_global_position", 1000, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_local_position", 500, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_wind", 1000, MAX_ESTIMATOR_INSTANCES);
#endif

	// always add the first instance
	add_topic("estimator_baro_bias", 500);
	add_topic("estimator_event_flags", 0);
	add_topic("estimator_gps_status", 1000);
	add_topic("estimator_innovation_test_ratios", 500);
	add_topic("estimator_innovation_variances", 500);
	add_topic("estimator_innovations", 500);
	add_topic("estimator_optical_flow_vel", 200);
	add_topic("estimator_sensor_bias", 0);
	add_topic("estimator_states", 1000);
	add_topic("estimator_status", 200);
	add_topic("estimator_status_flags", 0);
	add_topic("estimator_visual_odometry_aligned", 200);
	add_topic("yaw_estimator_status", 1000);

	add_optional_topic_multi("estimator_baro_bias", 500, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_event_flags", 0, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_gps_status", 1000, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_innovation_test_ratios", 500, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_innovation_variances", 500, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_innovations", 500, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_optical_flow_vel", 200, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_sensor_bias", 0, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_states", 1000, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_status", 200, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_status_flags", 0, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("estimator_visual_odometry_aligned", 200, MAX_ESTIMATOR_INSTANCES);
	add_optional_topic_multi("yaw_estimator_status", 1000, MAX_ESTIMATOR_INSTANCES);

	// log all raw sensors at minimal rate (at least 1 Hz)
	add_topic_multi("battery_status", 200, 2);
	add_topic_multi("differential_pressure", 1000, 2);
	add_topic_multi("distance_sensor", 1000, 2);
	add_topic_multi("optical_flow", 1000, 1);
	add_optional_topic_multi("sensor_accel", 1000, 4);
	add_optional_topic_multi("sensor_baro", 1000, 4);
	add_topic_multi("sensor_gps", 1000, 2);
	add_topic_multi("sensor_gnss_relative", 1000, 1);
	add_optional_topic("pps_capture", 1000);
	add_optional_topic_multi("sensor_gyro", 1000, 4);
	add_optional_topic_multi("sensor_mag", 1000, 4);
	add_topic_multi("vehicle_imu", 500, 4);
	add_topic_multi("vehicle_imu_status", 1000, 4);
	add_optional_topic_multi("vehicle_magnetometer", 500, 4);

	// SYS_CTRL_ALLOC: additional dynamic control allocation logging when enabled
	int32_t sys_ctrl_alloc = 0;
	param_get(param_find("SYS_CTRL_ALLOC"), &sys_ctrl_alloc);

	_dynamic_control_allocation = sys_ctrl_alloc >= 1;

	if (_dynamic_control_allocation) {
		add_topic("actuator_motors", 100);
		add_topic("actuator_servos", 100);
		add_topic("vehicle_angular_acceleration", 20);
		add_topic_multi("vehicle_thrust_setpoint", 20, 1);
		add_topic_multi("vehicle_torque_setpoint", 20, 2);
	}

	// SYS_HITL: default ground truth logging for simulation
	int32_t sys_hitl = 0;
	param_get(param_find("SYS_HITL"), &sys_hitl);

	if (sys_hitl >= 1) {
		add_topic("vehicle_angular_velocity_groundtruth", 10);
		add_topic("vehicle_attitude_groundtruth", 10);
		add_topic("vehicle_global_position_groundtruth", 100);
		add_topic("vehicle_local_position_groundtruth", 20);
	}

#ifdef CONFIG_ARCH_BOARD_PX4_SITL
	add_topic("actuator_controls_virtual_fw");
	add_topic("actuator_controls_virtual_mc");
	add_topic("fw_virtual_attitude_setpoint");
	add_topic("mc_virtual_attitude_setpoint");
	add_topic("time_offset");
	add_topic("vehicle_angular_acceleration", 10);
	add_topic("vehicle_angular_velocity", 10);
	add_topic("vehicle_attitude_groundtruth", 10);
	add_topic("vehicle_global_position_groundtruth", 100);
	add_topic("vehicle_local_position_groundtruth", 20);

	// EKF replay
	add_topic("estimator_baro_bias");
	add_topic("estimator_event_flags");
	add_topic("estimator_gps_status");
	add_topic("estimator_innovation_test_ratios");
	add_topic("estimator_innovation_variances");
	add_topic("estimator_innovations");
	add_topic("estimator_optical_flow_vel");
	add_topic("estimator_sensor_bias");
	add_topic("estimator_states");
	add_topic("estimator_status");
	add_topic("estimator_status_flags");
	add_topic("estimator_visual_odometry_aligned");
	add_topic("vehicle_attitude");
	add_topic("vehicle_global_position");
	add_topic("vehicle_local_position");
	add_topic("wind");
	add_topic("yaw_estimator_status");
#endif /* CONFIG_ARCH_BOARD_PX4_SITL */
}

void LoggedTopics::add_high_rate_topics()
{
	// maximum rate to analyze fast maneuvers (e.g. for racing)
	add_topic("manual_control_setpoint");
	add_topic("rate_ctrl_status", 20);
	add_topic("sensor_combined");
	add_topic("vehicle_angular_acceleration");
	add_topic("vehicle_angular_velocity");
	add_topic("vehicle_attitude");
	add_topic("vehicle_attitude_setpoint");
	add_topic("vehicle_rates_setpoint");

	if (_dynamic_control_allocation) {
		add_topic("actuator_motors");
		add_topic("vehicle_thrust_setpoint");
		add_topic("vehicle_torque_setpoint");

	} else {
		add_topic("actuator_controls_0");
		add_topic("actuator_outputs");
	}
}

void LoggedTopics::add_debug_topics()
{
	add_topic("debug_array");
	add_topic("debug_key_value");
	add_topic("debug_value");
	add_topic("debug_vect");
	add_topic_multi("satellite_info", 1000, 2);
	add_topic("mag_worker_data");
	add_topic("sensor_preflight_mag", 500);
	add_topic("test_motor", 500);
}

void LoggedTopics::add_estimator_replay_topics()
{
	// for estimator replay (need to be at full rate)
	add_topic("ekf2_timestamps");

	// current EKF2 subscriptions
	add_topic("airspeed");
	add_topic("optical_flow");
	add_topic("sensor_combined");
	add_topic("sensor_selection");
	add_topic("vehicle_air_data");
	add_topic("vehicle_gps_position");
	add_topic("vehicle_land_detected");
	add_topic("vehicle_magnetometer");
	add_topic("vehicle_status");
	add_topic("vehicle_visual_odometry");
	add_topic_multi("distance_sensor");
}

void LoggedTopics::add_thermal_calibration_topics()
{
	add_topic_multi("sensor_accel", 100, 3);
	add_topic_multi("sensor_baro", 100, 3);
	add_topic_multi("sensor_gyro", 100, 3);
}

void LoggedTopics::add_sensor_comparison_topics()
{
	add_topic_multi("sensor_accel", 100, 3);
	add_topic_multi("sensor_baro", 100, 3);
	add_topic_multi("sensor_gyro", 100, 3);
	add_topic_multi("sensor_mag", 100, 4);
}

void LoggedTopics::add_vision_and_avoidance_topics()
{
	add_topic("collision_constraints");
	add_topic("obstacle_distance_fused");
	add_topic("vehicle_mocap_odometry", 30);
	add_topic("vehicle_trajectory_waypoint", 200);
	add_topic("vehicle_trajectory_waypoint_desired", 200);
	add_topic("vehicle_visual_odometry", 30);
}

void LoggedTopics::add_raw_imu_gyro_fifo()
{
	add_topic("sensor_gyro_fifo");
}

void LoggedTopics::add_raw_imu_accel_fifo()
{
	add_topic("sensor_accel_fifo");
}

void LoggedTopics::add_system_identification_topics()
{
	// for system id need to log imu and controls at full rate
	add_topic("actuator_controls_0");
	add_topic("actuator_controls_1");
	add_topic("sensor_combined");
	add_topic("vehicle_angular_acceleration");
	add_topic("vehicle_torque_setpoint");
}

void LoggedTopics::add_mavlink_tunnel()
{
	add_topic("mavlink_tunnel");
}

int LoggedTopics::add_topics_from_file(const char *fname)
{
	int ntopics = 0;

	/* open the topic list file */
	FILE *fp = fopen(fname, "r");

	if (fp == nullptr) {
		return -1;
	}

	/* call add_topic for each topic line in the file */
	for (;;) {
		/* get a line, bail on error/EOF */
		char line[80];
		line[0] = '\0';

		if (fgets(line, sizeof(line), fp) == nullptr) {
			break;
		}

		/* skip comment lines */
		if ((strlen(line) < 2) || (line[0] == '#')) {
			continue;
		}

		// read line with format: <topic_name>[ <interval>[ <instance>]]
		char topic_name[80];
		uint32_t interval_ms = 0;
		uint32_t instance = 0;
		int nfields = sscanf(line, "%s %" PRIu32 " %" PRIu32, topic_name, &interval_ms, &instance);

		if (nfields > 0) {
			int name_len = strlen(topic_name);

			if (name_len > 0 && topic_name[name_len - 1] == ',') {
				topic_name[name_len - 1] = '\0';
			}

			/* add topic with specified interval_ms */
			if ((nfields > 2 && add_topic(topic_name, interval_ms, instance))
			    || add_topic_multi(topic_name, interval_ms)) {
				ntopics++;

			} else {
				PX4_ERR("Failed to add topic %s", topic_name);
			}
		}
	}

	fclose(fp);
	return ntopics;
}

void LoggedTopics::initialize_mission_topics(MissionLogType mission_log_type)
{
	if (mission_log_type == MissionLogType::Complete) {
		add_mission_topic("camera_capture");
		add_mission_topic("mission_result");
		add_mission_topic("vehicle_global_position", 1000);
		add_mission_topic("vehicle_status", 1000);

	} else if (mission_log_type == MissionLogType::Geotagging) {
		add_mission_topic("camera_capture");
	}
}

void LoggedTopics::add_mission_topic(const char *name, uint16_t interval_ms)
{
	if (add_topic(name, interval_ms)) {
		++_num_mission_subs;
	}
}

bool LoggedTopics::add_topic(const orb_metadata *topic, uint16_t interval_ms, uint8_t instance, bool optional)
{
	if (_subscriptions.count >= MAX_TOPICS_NUM) {
		PX4_WARN("Too many subscriptions, failed to add: %s %" PRIu8, topic->o_name, instance);
		return false;
	}

	if (optional && orb_exists(topic, instance) != 0) {
		PX4_DEBUG("Not adding non-existing optional topic %s %i", topic->o_name, instance);

		if (instance == 0 && _subscriptions.num_excluded_optional_topic_ids < MAX_EXCLUDED_OPTIONAL_TOPICS_NUM) {
			_subscriptions.excluded_optional_topic_ids[_subscriptions.num_excluded_optional_topic_ids++] = topic->o_id;
		}

		return false;
	}

	RequestedSubscription &sub = _subscriptions.sub[_subscriptions.count++];
	sub.interval_ms = interval_ms;
	sub.instance = instance;
	sub.id = static_cast<ORB_ID>(topic->o_id);
	return true;
}

bool LoggedTopics::add_topic(const char *name, uint16_t interval_ms, uint8_t instance, bool optional)
{
	interval_ms /= _rate_factor;

	const orb_metadata *const *topics = orb_get_topics();
	bool success = false;

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (strcmp(name, topics[i]->o_name) == 0) {
			bool already_added = false;

			// check if already added: if so, only update the interval
			for (int j = 0; j < _subscriptions.count; ++j) {
				if (_subscriptions.sub[j].id == static_cast<ORB_ID>(topics[i]->o_id) &&
				    _subscriptions.sub[j].instance == instance) {

					PX4_DEBUG("logging topic %s(%" PRIu8 "), interval: %" PRIu16 ", already added, only setting interval",
						  topics[i]->o_name, instance, interval_ms);

					_subscriptions.sub[j].interval_ms = interval_ms;
					success = true;
					already_added = true;
					break;
				}
			}

			if (!already_added) {
				success = add_topic(topics[i], interval_ms, instance, optional);

				if (success) {
					PX4_DEBUG("logging topic: %s(%" PRIu8 "), interval: %" PRIu16, topics[i]->o_name, instance, interval_ms);
				}

				break;
			}
		}
	}

	return success;
}

bool LoggedTopics::add_topic_multi(const char *name, uint16_t interval_ms, uint8_t max_num_instances, bool optional)
{
	// add all possible instances
	for (uint8_t instance = 0; instance < max_num_instances; instance++) {
		add_topic(name, interval_ms, instance, optional);
	}

	return true;
}

bool LoggedTopics::initialize_logged_topics(SDLogProfileMask profile)
{
	int ntopics = add_topics_from_file(PX4_STORAGEDIR "/etc/logging/logger_topics.txt");

	if (ntopics > 0) {
		PX4_INFO("logging %d topics from logger_topics.txt", ntopics);

	} else {
		initialize_configured_topics(profile);
	}

	return _subscriptions.count > 0;
}

void LoggedTopics::initialize_configured_topics(SDLogProfileMask profile)
{
	// load appropriate topics for profile
	// the order matters: if several profiles add the same topic, the logging rate of the last one will be used
	if (profile & SDLogProfileMask::DEFAULT) {
		add_default_topics();
	}

	if (profile & SDLogProfileMask::ESTIMATOR_REPLAY) {
		add_estimator_replay_topics();
	}

	if (profile & SDLogProfileMask::THERMAL_CALIBRATION) {
		add_thermal_calibration_topics();
	}

	if (profile & SDLogProfileMask::SYSTEM_IDENTIFICATION) {
		add_system_identification_topics();
	}

	if (profile & SDLogProfileMask::HIGH_RATE) {
		add_high_rate_topics();
	}

	if (profile & SDLogProfileMask::DEBUG_TOPICS) {
		add_debug_topics();
	}

	if (profile & SDLogProfileMask::SENSOR_COMPARISON) {
		add_sensor_comparison_topics();
	}

	if (profile & SDLogProfileMask::VISION_AND_AVOIDANCE) {
		add_vision_and_avoidance_topics();
	}

	if (profile & SDLogProfileMask::RAW_IMU_GYRO_FIFO) {
		add_raw_imu_gyro_fifo();
	}

	if (profile & SDLogProfileMask::RAW_IMU_ACCEL_FIFO) {
		add_raw_imu_accel_fifo();
	}

	if (profile & SDLogProfileMask::MAVLINK_TUNNEL) {
		add_mavlink_tunnel();
	}
}
