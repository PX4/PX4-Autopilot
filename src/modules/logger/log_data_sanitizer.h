#pragma once

#include <px4_log.h>

#include <lib/mathlib/mathlib.h>

#include <uORB/topics/uORBTopics.hpp>

#include <uORB/topics/aux_global_position.h>
#include <uORB/topics/camera_capture.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/estimator_gps_status.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/follow_target_estimator.h>
#include <uORB/topics/follow_target_status.h>
#include <uORB/topics/goto_setpoint.h>
#include <uORB/topics/gps_dump.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/mavlink_tunnel.h>
#include <uORB/topics/navigator_mission_item.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/ranging_beacon.h>
#include <uORB/topics/rover_position_setpoint.h>
#include <uORB/topics/rover_speed_setpoint.h>
#include <uORB/topics/rover_speed_status.h>
#include <uORB/topics/sensor_gnss_relative.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_roi.h>


typedef enum {
	UNRESTRICTED = 0,
	NO_GLOBAL    = 1,
	NO_LOCAL     = 2
} log_restriction;


template<typename T>
inline void redact_global(uint8_t *base, size_t offset, log_restriction restriction)
{
	if (restriction >= NO_GLOBAL) {
		memset(base + offset, 0, sizeof(T));
	}
}

template<typename T>
inline void redact_local(uint8_t *base, size_t offset, log_restriction restriction)
{
	if (restriction >= NO_LOCAL) {
		memset(base + offset, 0, sizeof(T));
	}
}

// Convenience macros to infer the right type.
// buffer and restriction are the same every time but baking them in feels wrong
#define REDACT_GLOBAL(buffer, struct_type, field, restriction) \
	redact_global<decltype(struct_type::field)>(buffer, offsetof(struct_type, field), restriction)

#define REDACT_LOCAL(buffer, struct_type, field, restriction) \
	redact_local<decltype(struct_type::field)>(buffer, offsetof(struct_type, field), restriction)


class LogDataSanitizer
{
public:

	void redact_position(const orb_id_t meta, uint8_t *buffer, int restriction_raw)
	{
		// Clamp to available values if invalid parameter is given
		const auto restriction = static_cast<log_restriction>(
						 math::constrain(restriction_raw, (int) UNRESTRICTED, (int) NO_LOCAL)
					 );

		const ORB_ID orb_id = static_cast<ORB_ID>(meta->o_id);

		switch (orb_id) {
		case ORB_ID::vehicle_global_position:
		case ORB_ID::vehicle_global_position_groundtruth:
		case ORB_ID::estimator_global_position:
		case ORB_ID::external_ins_global_position:
			REDACT_GLOBAL(buffer, vehicle_global_position_s, lat, restriction);
			REDACT_GLOBAL(buffer, vehicle_global_position_s, lon, restriction);
			break;

		case ORB_ID::vehicle_gps_position:
		case ORB_ID::sensor_gps:
			REDACT_GLOBAL(buffer, sensor_gps_s, latitude_deg, restriction);
			REDACT_GLOBAL(buffer, sensor_gps_s, longitude_deg, restriction);

			// Also redact norm of ground velocity
			// is pretty accurate local velocity together with heading / course (sp) / wind info
			REDACT_LOCAL(buffer, sensor_gps_s, vel_m_s, restriction);
			REDACT_LOCAL(buffer, sensor_gps_s, vel_n_m_s, restriction);
			REDACT_LOCAL(buffer, sensor_gps_s, vel_e_m_s, restriction);
			break;

		case ORB_ID::vehicle_local_position:
		case ORB_ID::vehicle_local_position_groundtruth:
		case ORB_ID::estimator_local_position:
		case ORB_ID::external_ins_local_position:
			REDACT_GLOBAL(buffer, vehicle_local_position_s, ref_lat, restriction);
			REDACT_GLOBAL(buffer, vehicle_local_position_s, ref_lon, restriction);
			REDACT_LOCAL(buffer, vehicle_local_position_s, x, restriction);
			REDACT_LOCAL(buffer, vehicle_local_position_s, y, restriction);
			REDACT_LOCAL(buffer, vehicle_local_position_s, vx, restriction);
			REDACT_LOCAL(buffer, vehicle_local_position_s, vy, restriction);
			break;

		case ORB_ID::aux_global_position:
			REDACT_GLOBAL(buffer, aux_global_position_s, lat, restriction);
			REDACT_GLOBAL(buffer, aux_global_position_s, lon, restriction);
			break;

		case ORB_ID::camera_capture:
			REDACT_GLOBAL(buffer, camera_capture_s, lat, restriction);
			REDACT_GLOBAL(buffer, camera_capture_s, lon, restriction);
			break;

		case ORB_ID::estimator_aid_src_aux_global_position:
		case ORB_ID::estimator_aid_src_fake_pos:
		case ORB_ID::estimator_aid_src_gnss_pos:
			REDACT_GLOBAL(buffer, estimator_aid_source2d_s, observation[0], restriction);
			REDACT_GLOBAL(buffer, estimator_aid_source2d_s, observation[1], restriction);
			break;

		case ORB_ID::estimator_aid_src_aux_vel:
		case ORB_ID::estimator_aid_src_gnss_vel:
		case ORB_ID::estimator_aid_src_ev_vel:
		case ORB_ID::estimator_aid_src_ev_pos:
			REDACT_LOCAL(buffer, estimator_aid_source2d_s, observation[0], restriction);
			REDACT_LOCAL(buffer, estimator_aid_source2d_s, observation[1], restriction);
			break;

		case ORB_ID::estimator_gps_status:
			// For fixed wing, is pretty accurate local velocity together with heading / course
			REDACT_LOCAL(buffer, estimator_gps_status_s, filtered_horizontal_speed_m_s, restriction);
			break;

		case ORB_ID::follow_target:
			// Target velocity OK - does not allow reconstructing any part of our trajectory
			REDACT_GLOBAL(buffer, follow_target_s, lat, restriction);
			REDACT_GLOBAL(buffer, follow_target_s, lon, restriction);
			break;

		case ORB_ID::follow_target_estimator:
			// Target velocity OK - does not allow reconstructing any part of our trajectory
			REDACT_GLOBAL(buffer, follow_target_estimator_s, lat_est, restriction);
			REDACT_GLOBAL(buffer, follow_target_estimator_s, lon_est, restriction);
			REDACT_LOCAL(buffer, follow_target_estimator_s, pos_est[0], restriction);
			REDACT_LOCAL(buffer, follow_target_estimator_s, pos_est[1], restriction);
			break;

		case ORB_ID::follow_target_status:
			// Conservatively redact this local position which can be related to the drone position in back-traceable ways
			REDACT_LOCAL(buffer, follow_target_status_s, desired_position_raw[0], restriction);
			REDACT_LOCAL(buffer, follow_target_status_s, desired_position_raw[1], restriction);
			break;

		case ORB_ID::goto_setpoint:
			REDACT_LOCAL(buffer, goto_setpoint_s, position[0], restriction);
			REDACT_LOCAL(buffer, goto_setpoint_s, position[1], restriction);
			break;

		case ORB_ID::gps_dump:
			// The uint8[79] data array contains the raw communicaiton with the GPS.
			// Redact it entirely. Local because it also contains velocities.
			REDACT_GLOBAL(buffer, gps_dump_s, data, restriction);
			break;

		case ORB_ID::home_position:
			REDACT_GLOBAL(buffer, home_position_s, lat, restriction);
			REDACT_GLOBAL(buffer, home_position_s, lon, restriction);
			REDACT_LOCAL(buffer, home_position_s, x, restriction);
			REDACT_LOCAL(buffer, home_position_s, y, restriction);
			break;

		case ORB_ID::landing_target_pose:
			// Other fields are positions relative to vehicle
			// This is relative to origin and too correlated with vehicle local position
			REDACT_LOCAL(buffer, landing_target_pose_s, x_abs, restriction);
			REDACT_LOCAL(buffer, landing_target_pose_s, y_abs, restriction);
			break;

		case ORB_ID::mavlink_tunnel:
			REDACT_GLOBAL(buffer, mavlink_tunnel_s, payload, restriction);
			break;

		case ORB_ID::navigator_mission_item:
			REDACT_GLOBAL(buffer, navigator_mission_item_s, latitude, restriction);
			REDACT_GLOBAL(buffer, navigator_mission_item_s, longitude, restriction);
			break;

		case ORB_ID::position_setpoint_triplet:
			// Nested structs end up with a flat layout so offsetof continues to work
			// previous
			REDACT_GLOBAL(buffer, position_setpoint_triplet_s, previous.lat, restriction);
			REDACT_GLOBAL(buffer, position_setpoint_triplet_s, previous.lon, restriction);
			REDACT_LOCAL(buffer, position_setpoint_triplet_s, previous.vx, restriction);
			REDACT_LOCAL(buffer, position_setpoint_triplet_s, previous.vy, restriction);
			// current
			REDACT_GLOBAL(buffer, position_setpoint_triplet_s, current.lat, restriction);
			REDACT_GLOBAL(buffer, position_setpoint_triplet_s, current.lon, restriction);
			REDACT_LOCAL(buffer, position_setpoint_triplet_s, current.vx, restriction);
			REDACT_LOCAL(buffer, position_setpoint_triplet_s, current.vy, restriction);
			// next
			REDACT_GLOBAL(buffer, position_setpoint_triplet_s, next.lat, restriction);
			REDACT_GLOBAL(buffer, position_setpoint_triplet_s, next.lon, restriction);
			REDACT_LOCAL(buffer, position_setpoint_triplet_s, next.vx, restriction);
			REDACT_LOCAL(buffer, position_setpoint_triplet_s, next.vy, restriction);
			break;

		case ORB_ID::rover_position_setpoint:
			REDACT_LOCAL(buffer, rover_position_setpoint_s, position_ned, restriction);
			REDACT_LOCAL(buffer, rover_position_setpoint_s, start_ned, restriction);
			break;

		case ORB_ID::rover_speed_setpoint:
			REDACT_LOCAL(buffer, rover_speed_setpoint_s, speed_body_x, restriction);
			REDACT_LOCAL(buffer, rover_speed_setpoint_s, speed_body_y, restriction);
			break;

		case ORB_ID::rover_speed_status:
			REDACT_LOCAL(buffer, rover_speed_status_s, measured_speed_body_x, restriction);
			REDACT_LOCAL(buffer, rover_speed_status_s, adjusted_speed_body_x_setpoint, restriction);
			REDACT_LOCAL(buffer, rover_speed_status_s, measured_speed_body_y, restriction);
			REDACT_LOCAL(buffer, rover_speed_status_s, adjusted_speed_body_y_setpoint, restriction);
			break;

		case ORB_ID::sensor_gnss_relative:
			REDACT_LOCAL(buffer, sensor_gnss_relative_s, position, restriction);
			// Heading & length together = horizontal relative position
			// Heading is available through many other topics, only redact length
			REDACT_LOCAL(buffer, sensor_gnss_relative_s, position_length, restriction);

			// If set of reference stations is known, this is a (coarse) global position
			REDACT_GLOBAL(buffer, sensor_gnss_relative_s, reference_station_id, restriction);
			break;

		case ORB_ID::trajectory_setpoint:
			REDACT_LOCAL(buffer, trajectory_setpoint_s, position[0], restriction);
			REDACT_LOCAL(buffer, trajectory_setpoint_s, position[1], restriction);
			REDACT_LOCAL(buffer, trajectory_setpoint_s, velocity[0], restriction);
			REDACT_LOCAL(buffer, trajectory_setpoint_s, velocity[1], restriction);
			break;

		case ORB_ID::transponder_report:
			REDACT_GLOBAL(buffer, transponder_report_s, lat, restriction);
			REDACT_GLOBAL(buffer, transponder_report_s, lon, restriction);
			// Redact horizontal velocity, keep heading
			REDACT_LOCAL(buffer, transponder_report_s, hor_velocity, restriction);
			break;

		case ORB_ID::vehicle_command:
			// Depending on the vehicle command, the params contain various position data
			// Checking the command and redacting minimally case by case would be overkill here...
			REDACT_GLOBAL(buffer, vehicle_command_s, param1, restriction);
			REDACT_GLOBAL(buffer, vehicle_command_s, param2, restriction);
			REDACT_GLOBAL(buffer, vehicle_command_s, param3, restriction);
			REDACT_GLOBAL(buffer, vehicle_command_s, param4, restriction);
			REDACT_GLOBAL(buffer, vehicle_command_s, param5, restriction);
			REDACT_GLOBAL(buffer, vehicle_command_s, param6, restriction);
			REDACT_GLOBAL(buffer, vehicle_command_s, param7, restriction);
			break;

		case ORB_ID::vehicle_local_position_setpoint:
			REDACT_LOCAL(buffer, vehicle_local_position_setpoint_s, x, restriction);
			REDACT_LOCAL(buffer, vehicle_local_position_setpoint_s, y, restriction);
			REDACT_LOCAL(buffer, vehicle_local_position_setpoint_s, vx, restriction);
			REDACT_LOCAL(buffer, vehicle_local_position_setpoint_s, vy, restriction);
			break;

		case ORB_ID::estimator_odometry:
		case ORB_ID::vehicle_odometry:
		case ORB_ID::vehicle_visual_odometry:
		case ORB_ID::vehicle_mocap_odometry: // Only used with legacy Q attitude & LPE position estimator...
			REDACT_GLOBAL(buffer, vehicle_odometry_s, position[0], restriction);
			REDACT_GLOBAL(buffer, vehicle_odometry_s, position[1], restriction);
			REDACT_LOCAL(buffer, vehicle_odometry_s, velocity[0], restriction);
			REDACT_LOCAL(buffer, vehicle_odometry_s, velocity[1], restriction);
			break;

		case ORB_ID::vehicle_roi:
			REDACT_GLOBAL(buffer, vehicle_roi_s, lat, restriction);
			REDACT_GLOBAL(buffer, vehicle_roi_s, lon, restriction);
			break;

		case ORB_ID::estimator_states:
			// Conservatively redact all the states. Redacting case by case we would have
			// to include the index definitions here, or fail silently when they change.
			REDACT_LOCAL(buffer, estimator_states_s, states, restriction);
			break;

		case ORB_ID::ranging_beacon:
			REDACT_GLOBAL(buffer, ranging_beacon_s, lat, restriction);
			REDACT_GLOBAL(buffer, ranging_beacon_s, lon, restriction);

			// Without knowing the beacon positions, the raw range
			// correspond at best to a 1D local position measurement
			REDACT_LOCAL(buffer, ranging_beacon_s, range, restriction);

		// These debug topics may contain any information, including position. However, by default:
		//  - they are not published and are in MAVLink as a catch-all debugging tool.
		//  - if they are published, they are only logged if enabled in SDLOG_PROFILE
		case ORB_ID::debug_array:     // https://mavlink.io/en/messages/common.html#DEBUG_FLOAT_ARRAY
		case ORB_ID::debug_key_value: // https://mavlink.io/en/messages/common.html#NAMED_VALUE_FLOAT
		case ORB_ID::debug_value:     // https://mavlink.io/en/messages/common.html#DEBUG
		case ORB_ID::debug_vect:      // https://mavlink.io/en/messages/common.html#DEBUG_VECT

		// These topics contain something closely related to
		// position/velocity but not close enough or accurate enough to
		// reconstruct the trajectory. Some of these are close calls,
		// revisit if necessary.
		case ORB_ID::airspeed: // Airspeed (even with heading) is inaccurate enough to log
		case ORB_ID::fixed_wing_lateral_guidance_status: // Contains course setpoint -- inaccurate enough to log even with airspeed and wind estimate
		case ORB_ID::sensor_mag: // Mag gives up global position but only up to a couple 100 km
		case ORB_ID::sensor_optical_flow: // OF is a velocity but too inaccurate / intermittent to reconstruct trajectory
		case ORB_ID::vehicle_optical_flow:
		case ORB_ID::estimator_optical_flow_vel:
		case ORB_ID::estimator_aid_src_optical_flow:
		case ORB_ID::wind: // Might redact wind to decrease precision of reconstruction through airspeed + heading

		// These topics contain no local/global positions, but require clarifying disclaimers
		case ORB_ID::collision_constraints: // The fields are described as velocity setpoints but are acceleration
		case ORB_ID::irlock_report: // Has some "pos" fields but only in camera coordinates
		case ORB_ID::neural_control: // Contains a local position error term - not informative, log

		// These topics contain no global or local positions - do not redact
		case ORB_ID::action_request:
		case ORB_ID::actuator_armed:
		case ORB_ID::actuator_controls_status_0:
		case ORB_ID::actuator_controls_status_1:
		case ORB_ID::actuator_motors:
		case ORB_ID::actuator_outputs:
		case ORB_ID::actuator_outputs_debug:
		case ORB_ID::actuator_servos:
		case ORB_ID::actuator_test:
		case ORB_ID::airspeed_validated:
		case ORB_ID::airspeed_wind:
		case ORB_ID::autotune_attitude_control_status:
		case ORB_ID::battery_info:
		case ORB_ID::battery_status:
		case ORB_ID::camera_trigger:
		case ORB_ID::can_interface_status:
		case ORB_ID::cellular_status:
		case ORB_ID::config_overrides:
		case ORB_ID::config_overrides_request:
		case ORB_ID::control_allocator_status:
		case ORB_ID::cpuload:
		case ORB_ID::device_information:
		case ORB_ID::differential_pressure:
		case ORB_ID::distance_sensor:
		case ORB_ID::distance_sensor_mode_change_request:
		case ORB_ID::dronecan_node_status:
		case ORB_ID::ekf2_timestamps:
		case ORB_ID::esc_status:
		case ORB_ID::estimator_aid_src_airspeed:
		case ORB_ID::estimator_aid_src_baro_hgt:
		case ORB_ID::estimator_aid_src_fake_hgt:
		case ORB_ID::estimator_aid_src_gnss_hgt:
		case ORB_ID::estimator_aid_src_gravity:
		case ORB_ID::estimator_aid_src_mag:
		case ORB_ID::estimator_aid_src_rng_hgt:
		case ORB_ID::estimator_baro_bias:
		case ORB_ID::estimator_event_flags:
		case ORB_ID::estimator_fusion_control:
		case ORB_ID::estimator_gnss_hgt_bias:
		case ORB_ID::estimator_innovation_test_ratios:
		case ORB_ID::estimator_innovation_variances:
		case ORB_ID::estimator_innovations:
		case ORB_ID::estimator_selector_status:
		case ORB_ID::estimator_sensor_bias:
		case ORB_ID::estimator_status:
		case ORB_ID::estimator_status_flags:
		case ORB_ID::external_ins_attitude:
		case ORB_ID::failsafe_flags:
		case ORB_ID::failure_detector_status:
		case ORB_ID::fixed_wing_lateral_setpoint:
		case ORB_ID::fixed_wing_lateral_status:
		case ORB_ID::fixed_wing_longitudinal_setpoint:
		case ORB_ID::fixed_wing_runway_control:
		case ORB_ID::flaps_setpoint:
		case ORB_ID::flight_phase_estimation:
		case ORB_ID::fuel_tank_status:
		case ORB_ID::fw_virtual_attitude_setpoint:
		case ORB_ID::gain_compression:
		case ORB_ID::generator_status:
		case ORB_ID::gimbal_controls:
		case ORB_ID::gimbal_manager_set_attitude:
		case ORB_ID::gripper:
		case ORB_ID::heater_status:
		case ORB_ID::hover_thrust_estimate:
		case ORB_ID::input_rc:
		case ORB_ID::internal_combustion_engine_control:
		case ORB_ID::internal_combustion_engine_status:
		case ORB_ID::iridiumsbd_status:
		case ORB_ID::landing_gear:
		case ORB_ID::landing_gear_wheel:
		case ORB_ID::lateral_control_configuration:
		case ORB_ID::launch_detection_status:
		case ORB_ID::logger_status:
		case ORB_ID::longitudinal_control_configuration:
		case ORB_ID::mag_worker_data:
		case ORB_ID::magnetometer_bias_estimate:
		case ORB_ID::manual_control_setpoint:
		case ORB_ID::manual_control_switches:
		case ORB_ID::mc_virtual_attitude_setpoint:
		case ORB_ID::mission_result:
		case ORB_ID::navigator_status:
		case ORB_ID::obstacle_distance:
		case ORB_ID::obstacle_distance_fused:
		case ORB_ID::offboard_control_mode:
		case ORB_ID::onboard_computer_status:
		case ORB_ID::parameter_update:
		case ORB_ID::position_controller_landing_status:
		case ORB_ID::position_controller_status:
		case ORB_ID::pps_capture:
		case ORB_ID::pure_pursuit_status:
		case ORB_ID::px4io_status:
		case ORB_ID::radio_status:
		case ORB_ID::rate_ctrl_status:
		case ORB_ID::rover_attitude_setpoint:
		case ORB_ID::rover_attitude_status:
		case ORB_ID::rover_rate_setpoint:
		case ORB_ID::rover_rate_status:
		case ORB_ID::rover_steering_setpoint:
		case ORB_ID::rover_throttle_setpoint:
		case ORB_ID::rpm:
		case ORB_ID::rtl_status:
		case ORB_ID::rtl_time_estimate:
		case ORB_ID::satellite_info:
		case ORB_ID::sensor_accel:
		case ORB_ID::sensor_accel_fifo:
		case ORB_ID::sensor_airflow:
		case ORB_ID::sensor_baro:
		case ORB_ID::sensor_combined:
		case ORB_ID::sensor_correction:
		case ORB_ID::sensor_gyro:
		case ORB_ID::sensor_gyro_fft:
		case ORB_ID::sensor_gyro_fifo:
		case ORB_ID::sensor_hygrometer:
		case ORB_ID::sensor_preflight_mag:
		case ORB_ID::sensor_selection:
		case ORB_ID::sensor_temp:
		case ORB_ID::sensors_status_imu:
		case ORB_ID::spoilers_setpoint:
		case ORB_ID::system_power:
		case ORB_ID::takeoff_status:
		case ORB_ID::tecs_status:
		case ORB_ID::telemetry_status:
		case ORB_ID::tiltrotor_extra_controls:
		case ORB_ID::timesync_status:
		case ORB_ID::vehicle_acceleration:
		case ORB_ID::vehicle_air_data:
		case ORB_ID::vehicle_angular_velocity:
		case ORB_ID::vehicle_angular_velocity_groundtruth:
		case ORB_ID::vehicle_attitude:
		case ORB_ID::vehicle_attitude_groundtruth:
		case ORB_ID::vehicle_attitude_setpoint:
		case ORB_ID::vehicle_command_ack:
		case ORB_ID::vehicle_constraints:
		case ORB_ID::vehicle_control_mode:
		case ORB_ID::vehicle_imu:
		case ORB_ID::vehicle_imu_status:
		case ORB_ID::vehicle_land_detected:
		case ORB_ID::vehicle_magnetometer:
		case ORB_ID::vehicle_rates_setpoint:
		case ORB_ID::vehicle_status:
		case ORB_ID::vehicle_thrust_setpoint:
		case ORB_ID::vehicle_thrust_setpoint_virtual_fw:
		case ORB_ID::vehicle_thrust_setpoint_virtual_mc:
		case ORB_ID::vehicle_torque_setpoint:
		case ORB_ID::vehicle_torque_setpoint_virtual_fw:
		case ORB_ID::vehicle_torque_setpoint_virtual_mc:
		case ORB_ID::vtol_vehicle_status:
		case ORB_ID::vtx:
		case ORB_ID::yaw_estimator_status:
			break;

		default:

			if (restriction != UNRESTRICTED) {
				// No redactiond defined: redact entire message to be safe
				memset(buffer, 0, meta->o_size);
			}

			if (_undefined_redaction_warnings_shown < MAX_WARNINGS) {

				// This warning means that you are logging a topic for which no position redaction is
				// defined. As a safety mechanism, the entire message is redacted before logging if
				// position-restricted logging is enabled with SDLOG_NO_POS_DAT.
				//
				// If the topic in question does not contain any global or local position data, add it
				// to the no-op block above to log it (with a clarifying comment if it is a close call).
				//
				// If it does contain position data, follow the pattern above and specify which fields
				// contain what position using REDACT_LOCAL and REDACT_GLOBAL.

				PX4_WARN(
					"Undefined position redaction for %s. %s",
					meta->o_name,
					restriction != UNRESTRICTED ? "Redacting entire message." : ""
				);

				_undefined_redaction_warnings_shown++;

			} else if (_undefined_redaction_warnings_shown == MAX_WARNINGS) {
				PX4_WARN("Suppressing further warnings about missing position redaction");
				_undefined_redaction_warnings_shown++;
			}
		}
	}

private:
	int _undefined_redaction_warnings_shown{0};
	static constexpr int MAX_WARNINGS{10};
};
