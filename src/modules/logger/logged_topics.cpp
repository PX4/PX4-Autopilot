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

namespace
{
constexpr uint8_t kOpt = px4::logger::LoggedTopics::TopicFlags::topic_optional;
constexpr uint8_t kMulti = px4::logger::LoggedTopics::TopicFlags::topic_all_instances;

using TopicSpec = px4::logger::LoggedTopics::TopicSpec;

constexpr TopicSpec default_topics_pre_estimator[] = {
	{"action_request", 0, 0, 0},
	{"actuator_armed", 0, 0, 0},
	{"actuator_controls_status_0", 300, 0, kOpt},
	{"airspeed", 1000, 0, 0},
	{"airspeed_validated", 200, 0, kOpt},
	{"autotune_attitude_control_status", 100, 0, kOpt},
	{"battery_info", 5000, 3, kMulti},
	{"camera_capture", 0, 0, kOpt},
	{"camera_trigger", 0, 0, kOpt},
	{"cellular_status", 200, 0, 0},
	{"commander_state", 0, 0, 0},
	{"config_overrides", 0, 0, 0},
	{"cpuload", 0, 0, 0},
	{"distance_sensor_mode_change_request", 0, 0, 0},
	{"device_information", 900, 0, 0},
	{"dronecan_node_status", 250, ORB_MULTI_MAX_INSTANCES, kMulti},
	{"external_ins_attitude", 0, 0, kOpt},
	{"external_ins_global_position", 0, 0, kOpt},
	{"external_ins_local_position", 0, 0, kOpt},
	{"esc_status", 100, 0, 0},
	{"failure_detector_status", 100, 0, 0},
	{"failsafe_flags", 0, 0, 0},
	{"follow_target", 500, 0, kOpt},
	{"follow_target_estimator", 200, 0, kOpt},
	{"follow_target_status", 400, 0, kOpt},
	{"flaps_setpoint", 1000, 0, kOpt},
	{"flight_phase_estimation", 1000, 0, kOpt},
	{"fuel_tank_status", 10, 0, kOpt},
	{"gain_compression", 100, 0, kOpt},
	{"gimbal_manager_set_attitude", 500, 0, 0},
	{"generator_status", 0, 0, kOpt},
	{"gps_dump", 0, 0, 0},
	{"gimbal_controls", 200, 0, kOpt},
	{"gripper", 0, 0, kOpt},
	{"heater_status", 0, ORB_MULTI_MAX_INSTANCES, kOpt | kMulti},
	{"home_position", 0, 0, 0},
	{"hover_thrust_estimate", 100, 0, 0},
	{"input_rc", 500, 0, 0},
	{"internal_combustion_engine_control", 10, 0, kOpt},
	{"internal_combustion_engine_status", 10, 0, kOpt},
	{"iridiumsbd_status", 1000, 0, kOpt},
	{"irlock_report", 1000, 0, kOpt},
	{"landing_gear", 200, 0, kOpt},
	{"landing_gear_wheel", 100, 0, kOpt},
	{"landing_target_pose", 1000, 0, kOpt},
	{"launch_detection_status", 200, 0, kOpt},
	{"logger_status", 200, 0, 0},
	{"magnetometer_bias_estimate", 200, 0, kOpt},
	{"manual_control_setpoint", 200, 0, 0},
	{"manual_control_switches", 0, 0, 0},
	{"mission_result", 0, 0, 0},
	{"navigator_mission_item", 0, 0, 0},
	{"navigator_status", 0, 0, 0},
	{"offboard_control_mode", 100, 0, 0},
	{"onboard_computer_status", 10, 0, 0},
	{"parameter_update", 0, 0, 0},
	{"position_controller_status", 500, 0, 0},
	{"position_controller_landing_status", 100, 0, 0},
	{"pure_pursuit_status", 100, 0, kOpt},
	{"goto_setpoint", 200, 0, 0},
	{"position_setpoint_triplet", 200, 0, 0},
	{"px4io_status", 0, 0, kOpt},
	{"radio_status", 0, 0, 0},
	{"rover_attitude_setpoint", 100, 0, kOpt},
	{"rover_attitude_status", 100, 0, kOpt},
	{"rover_position_setpoint", 100, 0, kOpt},
	{"rover_rate_setpoint", 100, 0, kOpt},
	{"rover_rate_status", 100, 0, kOpt},
	{"rover_speed_setpoint", 100, 0, kOpt},
	{"rover_speed_status", 100, 0, kOpt},
	{"rover_steering_setpoint", 100, 0, kOpt},
	{"rover_throttle_setpoint", 100, 0, kOpt},
	{"rtl_time_estimate", 1000, 0, 0},
	{"rtl_status", 2000, 0, 0},
	{"sensor_airflow", 100, 0, kOpt},
	{"sensor_combined", 0, 0, 0},
	{"sensor_correction", 0, 0, kOpt},
	{"sensor_gyro_fft", 50, 0, kOpt},
	{"sensor_selection", 0, 0, 0},
	{"sensors_status_imu", 200, 0, 0},
	{"spoilers_setpoint", 1000, 0, kOpt},
	{"system_power", 500, 0, 0},
	{"takeoff_status", 1000, 0, kOpt},
	{"tecs_status", 200, 0, kOpt},
	{"tiltrotor_extra_controls", 100, 0, kOpt},
	{"trajectory_setpoint", 200, 0, 0},
	{"transponder_report", 0, 0, 0},
	{"vehicle_acceleration", 50, 0, 0},
	{"vehicle_air_data", 200, 0, 0},
	{"vehicle_angular_velocity", 20, 0, 0},
	{"vehicle_attitude", 50, 0, 0},
	{"vehicle_attitude_setpoint", 50, 0, 0},
	{"vehicle_command", 0, 0, 0},
	{"vehicle_command_ack", 0, 0, 0},
	{"vehicle_constraints", 1000, 0, 0},
	{"vehicle_control_mode", 0, 0, 0},
	{"vehicle_global_position", 200, 0, 0},
	{"vehicle_gps_position", 100, 0, 0},
	{"vehicle_land_detected", 0, 0, 0},
	{"vehicle_local_position", 100, 0, 0},
	{"vehicle_local_position_setpoint", 100, 0, 0},
	{"vehicle_magnetometer", 200, 0, 0},
	{"vehicle_rates_setpoint", 20, 0, 0},
	{"vehicle_roi", 1000, 0, 0},
	{"vehicle_status", 0, 0, 0},
	{"vtx", 0, 0, 0},
	{"vtol_vehicle_status", 200, 0, kOpt},
	{"wind", 1000, 0, 0},
	{"fixed_wing_lateral_setpoint", 0, 0, 0},
	{"fixed_wing_longitudinal_setpoint", 0, 0, 0},
	{"longitudinal_control_configuration", 0, 0, 0},
	{"lateral_control_configuration", 0, 0, 0},
	{"fixed_wing_lateral_guidance_status", 100, 0, kOpt},
	{"fixed_wing_lateral_status", 100, 0, kOpt},
	{"fixed_wing_runway_control", 100, 0, kOpt},
	{"ranging_beacon", 100, 0, kOpt},

	// multi topics
	{"actuator_outputs", 100, 3, kOpt | kMulti},
	{"airspeed_wind", 1000, 4, kOpt | kMulti},
	{"control_allocator_status", 200, 2, kOpt | kMulti},
	{"rate_ctrl_status", 200, 2, kOpt | kMulti},
	{"sensor_hygrometer", 500, 4, kOpt | kMulti},
	{"sensor_temp", 100, 4, kOpt | kMulti},
	{"rpm", 200, ORB_MULTI_MAX_INSTANCES, kOpt | kMulti},
	{"timesync_status", 1000, 3, kMulti},
	{"telemetry_status", 1000, 4, kOpt | kMulti},

};

constexpr TopicSpec default_topics_post_estimator[] = {

	// important EKF topics (higher rate)
	{"estimator_selector_status", 10, 0, kOpt},
	{"estimator_event_flags", 10, ORB_MULTI_MAX_INSTANCES, kOpt | kMulti},
	{"estimator_optical_flow_vel", 200, ORB_MULTI_MAX_INSTANCES, kOpt | kMulti},
	{"estimator_fusion_control", 1000, ORB_MULTI_MAX_INSTANCES, kOpt | kMulti},
	{"estimator_sensor_bias", 1000, ORB_MULTI_MAX_INSTANCES, kOpt | kMulti},
	{"estimator_status", 200, ORB_MULTI_MAX_INSTANCES, kOpt | kMulti},
	{"estimator_status_flags", 10, ORB_MULTI_MAX_INSTANCES, kOpt | kMulti},
	{"yaw_estimator_status", 1000, ORB_MULTI_MAX_INSTANCES, kOpt | kMulti},

	// Vision target estimator topics
#if defined(CONFIG_MODULES_VISION_TARGET_ESTIMATOR) && CONFIG_MODULES_VISION_TARGET_ESTIMATOR
	{"vte_input", 50, 0, 0},
	{"vte_position", 100, 0, 0},
	{"vte_orientation", 100, 0, 0},
	{"vte_bias_init_status", 10, 0, 0},	// High rate because rarely published and only for a short period of time
	{"vte_aid_gps_pos_target", 100, 0, 0},
	{"vte_aid_gps_pos_mission", 100, 0, 0},
	{"vte_aid_gps_vel_uav", 100, 0, 0},
	{"vte_aid_gps_vel_target", 100, 0, 0},
	{"vte_aid_fiducial_marker", 100, 0, 0},
	{"vte_aid_ev_yaw", 100, 0, 0},
	{"fiducial_marker_pos_report", 100, 0, 0},
	{"fiducial_marker_yaw_report", 100, 0, 0},
	{"target_gnss", 100, 0, 0},
#endif // CONFIG_MODULES_VISION_TARGET_ESTIMATOR

	// log all raw sensors at minimal rate (at least 1 Hz)
	{"battery_status", 200, 3, kMulti},
	{"differential_pressure", 1000, 2, kMulti},
	{"distance_sensor", 1000, 2, kMulti},
	{"sensor_accel", 1000, 4, kOpt | kMulti},
	{"sensor_baro", 1000, 4, kMulti},
	{"sensor_gps", 1000, 2, kMulti},
	{"sensor_gnss_relative", 1000, 1, kMulti},
	{"sensor_gyro", 1000, 4, kOpt | kMulti},
	{"sensor_mag", 1000, 4, kMulti},
	{"sensor_optical_flow", 1000, 2, kMulti},

	{"vehicle_imu", 500, 4, kMulti},
	{"vehicle_imu_status", 1000, 4, kMulti},
	{"vehicle_magnetometer", 500, 4, kOpt | kMulti},
	{"vehicle_optical_flow", 500, 0, 0},
	{"aux_global_position", 500, ORB_MULTI_MAX_INSTANCES, kMulti},
	{"pps_capture", 0, 0, kOpt},

	// additional control allocation logging
	{"actuator_motors", 100, 0, 0},
	{"actuator_servos", 100, 0, 0},
	{"vehicle_thrust_setpoint", 20, 2, kMulti},
	{"vehicle_torque_setpoint", 20, 2, kMulti},

};

} // namespace

void LoggedTopics::add_topics(const TopicSpec *specs, unsigned count)
{
	for (unsigned i = 0; i < count; ++i) {
		const TopicSpec &spec = specs[i];

		if (spec.flags & TopicFlags::topic_all_instances) {
			add_topic_multi(spec.name, spec.interval_ms, spec.max_num_instances, spec.flags & TopicFlags::topic_optional);

		} else {
			add_topic(spec.name, spec.interval_ms, 0, spec.flags & TopicFlags::topic_optional);
		}
	}
}

void LoggedTopics::add_default_topics()
{
	add_topics(default_topics_pre_estimator);

	// EKF multi topics
	{
		// optionally log all estimator* topics at minimal rate
		const uint16_t kEKFVerboseIntervalMilliseconds = 500; // 2 Hz
		const struct orb_metadata *const *topic_list = orb_get_topics();

		for (size_t i = 0; i < orb_topics_count(); i++) {
			if (strncmp(topic_list[i]->o_name, "estimator", 9) == 0) {
				add_optional_topic_multi(topic_list[i]->o_name, kEKFVerboseIntervalMilliseconds);
			}
		}
	}

	add_topics(default_topics_post_estimator);

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
	add_topic("fw_virtual_attitude_setpoint");
	add_topic("mc_virtual_attitude_setpoint");
	add_optional_topic("vehicle_torque_setpoint_virtual_mc");
	add_optional_topic("vehicle_torque_setpoint_virtual_fw");
	add_optional_topic("vehicle_thrust_setpoint_virtual_mc");
	add_optional_topic("vehicle_thrust_setpoint_virtual_fw");
	add_topic("time_offset");
	add_topic("vehicle_angular_velocity", 10);
	add_topic("vehicle_angular_velocity_groundtruth", 10);
	add_topic("vehicle_attitude_groundtruth", 10);
	add_topic("vehicle_global_position_groundtruth", 100);
	add_topic("vehicle_local_position_groundtruth", 20);

	// EKF replay
	{
		// optionally log all estimator* topics at minimal rate
		const uint16_t kEKFVerboseIntervalMilliseconds = 10; // 100 Hz
		const struct orb_metadata *const *topic_list = orb_get_topics();

		for (size_t i = 0; i < orb_topics_count(); i++) {
			if (strncmp(topic_list[i]->o_name, "estimator", 9) == 0) {
				add_optional_topic_multi(topic_list[i]->o_name, kEKFVerboseIntervalMilliseconds);
			}
		}
	}

	add_topic("vehicle_attitude");
	add_topic("vehicle_global_position");
	add_topic("vehicle_local_position");
	add_topic("wind");
	add_optional_topic_multi("yaw_estimator_status");

#endif /* CONFIG_ARCH_BOARD_PX4_SITL */

#ifdef CONFIG_BOARD_UAVCAN_INTERFACES
	add_topic_multi("can_interface_status", 100, CONFIG_BOARD_UAVCAN_INTERFACES);
#endif
}

void LoggedTopics::add_high_rate_topics()
{
	// maximum rate to analyze fast maneuvers (e.g. for racing)
	add_topic("manual_control_setpoint");
	add_topic_multi("rate_ctrl_status", 20, 2);
	add_topic("sensor_combined");
	add_topic("vehicle_angular_velocity");
	add_topic("vehicle_attitude");
	add_topic("vehicle_attitude_setpoint");
	add_topic("vehicle_rates_setpoint");

	add_topic("esc_status", 5);
	add_topic("actuator_motors");
	add_topic("actuator_outputs_debug");
	add_topic("actuator_servos");
	add_topic_multi("vehicle_thrust_setpoint", 0, 2);
	add_topic_multi("vehicle_torque_setpoint", 0, 2);
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
	add_topic("actuator_test", 500);
	add_topic("neural_control", 50);
}

void LoggedTopics::add_estimator_replay_topics()
{
	// for estimator replay (need to be at full rate)
	add_topic("ekf2_timestamps");

	// current EKF2 subscriptions
	add_topic("airspeed");
	add_topic("airspeed_validated");
	add_topic("vehicle_optical_flow");
	add_topic("sensor_combined");
	add_topic("sensor_selection");
	add_topic("vehicle_air_data");
	add_topic("vehicle_gps_position");
	add_topic("vehicle_land_detected");
	add_topic("vehicle_magnetometer");
	add_topic("vehicle_status");
	add_topic("vehicle_visual_odometry");
	add_topic("ranging_beacon");
	add_topic_multi("aux_global_position");
	add_topic_multi("distance_sensor");
}

void LoggedTopics::add_thermal_calibration_topics()
{
	add_topic_multi("sensor_accel", 100, 4);
	add_topic_multi("sensor_baro", 100, 4);
	add_topic_multi("sensor_gyro", 100, 4);
	add_topic_multi("sensor_mag", 100, 4);
}

void LoggedTopics::add_sensor_comparison_topics()
{
	add_topic_multi("sensor_accel", 100, 4);
	add_topic_multi("sensor_baro", 100, 4);
	add_topic_multi("sensor_gyro", 100, 4);
	add_topic_multi("sensor_mag", 100, 4);
}

void LoggedTopics::add_vision_and_avoidance_topics()
{
	add_topic("collision_constraints");
	add_topic_multi("distance_sensor");
	add_topic("obstacle_distance_fused");
	add_topic("obstacle_distance");
	add_topic("vehicle_mocap_odometry", 30);
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
	add_topic("sensor_combined");
	add_topic("vehicle_angular_velocity");
	add_topic("vehicle_torque_setpoint");
	add_topic("vehicle_acceleration");
	add_topic("actuator_motors");
}

void LoggedTopics::add_high_rate_sensors_topics()
{
	add_topic_multi("distance_sensor", 10, 4);
	add_topic_multi("sensor_baro", 10, 4);
	add_topic_multi("sensor_optical_flow", 10, 2);
	add_topic_multi("sensor_gps", 10, 4);
	add_topic_multi("sensor_gnss_relative", 10, 1);
	add_topic_multi("sensor_mag", 10, 4);
	add_topic("estimator_aid_src_baro_hgt", 10);
	add_topic("vehicle_air_data", 10);
	add_topic("vehicle_magnetometer", 10);
	add_topic("vehicle_thrust_setpoint", 10);
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

	if (profile & SDLogProfileMask::HIGH_RATE_SENSORS) {
		add_high_rate_sensors_topics();
	}

	int ntopics = add_topics_from_file(PX4_STORAGEDIR "/etc/logging/logger_topics.txt");

	if (ntopics > 0) {
		PX4_INFO("logging %d topics from logger_topics.txt", ntopics);

	} else if ((int32_t)profile == 0) {
		PX4_WARN("No logging topics added. Using default set");
		add_default_topics();
	}

	return _subscriptions.count > 0;
}
