/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef SYS_STATUS_HPP
#define SYS_STATUS_HPP

#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/health_report.h>
#include <px4_platform_common/events.h>

class MavlinkStreamSysStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamSysStatus(mavlink); }

	static constexpr const char *get_name_static() { return "SYS_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_SYS_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_SYS_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamSysStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _cpuload_sub{ORB_ID(cpuload)};
	uORB::Subscription _health_report{ORB_ID(health_report)};
	uORB::SubscriptionMultiArray<battery_status_s, battery_status_s::MAX_INSTANCES> _battery_status_subs{ORB_ID::battery_status};

	using health_component_t = events::px4::enums::health_component_t;

	void fillOutComponent(const health_report_s &health_report, MAV_SYS_STATUS_SENSOR mav_sensor,
			      health_component_t health_component, mavlink_sys_status_t &msg)
	{
		if (health_report.health_is_present_flags & (uint64_t)health_component) {
			msg.onboard_control_sensors_present |= mav_sensor;
		}

		if (((health_report.arming_check_error_flags | health_report.arming_check_warning_flags |
		      health_report.health_error_flags | health_report.health_warning_flags) & (uint64_t)health_component) == 0) {
			msg.onboard_control_sensors_health |= mav_sensor;
		}

		// Set as enabled if present or unhealthy (required, but missing)
		if ((msg.onboard_control_sensors_present & mav_sensor) || (msg.onboard_control_sensors_health & mav_sensor) == 0) {
			msg.onboard_control_sensors_enabled |= mav_sensor;
		}
	}

	void fillOutComponent(const health_report_s &health_report, MAV_SYS_STATUS_SENSOR_EXTENDED mav_sensor,
			      health_component_t health_component, mavlink_sys_status_t &msg)
	{
		if (health_report.health_is_present_flags & (uint64_t)health_component) {
			msg.onboard_control_sensors_present_extended |= mav_sensor;
		}

		if (((health_report.arming_check_error_flags | health_report.arming_check_warning_flags |
		      health_report.health_error_flags | health_report.health_warning_flags) & (uint64_t)health_component) == 0) {
			msg.onboard_control_sensors_health_extended |= mav_sensor;
		}

		// Set as enabled if present or unhealthy (required, but missing)
		if ((msg.onboard_control_sensors_present_extended & mav_sensor)
		    || (msg.onboard_control_sensors_health_extended & mav_sensor) == 0) {
			msg.onboard_control_sensors_enabled_extended |= mav_sensor;
		}
	}

	bool send() override
	{
		if (_status_sub.updated() || _cpuload_sub.updated() || _battery_status_subs.updated()) {
			vehicle_status_s status{};
			_status_sub.copy(&status);

			cpuload_s cpuload{};
			_cpuload_sub.copy(&cpuload);

			health_report_s health_report{};
			_health_report.copy(&health_report);

			battery_status_s battery_status[battery_status_s::MAX_INSTANCES] {};

			for (int i = 0; i < _battery_status_subs.size(); i++) {
				_battery_status_subs[i].copy(&battery_status[i]);
			}

			int lowest_battery_index = 0;

			// No battery is connected, select the first group
			// Low battery judgment is performed only when the current battery is connected
			// When the last cached battery is not connected or the current battery level is lower than the cached battery level,
			// the current battery status is replaced with the cached value
			for (int i = 0; i < _battery_status_subs.size(); i++) {
				if (battery_status[i].connected && ((!battery_status[lowest_battery_index].connected)
								    || (battery_status[i].remaining <
									battery_status[lowest_battery_index].remaining))) {
					lowest_battery_index = i;
				}
			}

			mavlink_sys_status_t msg{};

			if (health_report.can_arm_mode_flags & (1u << status.nav_state)) {
				msg.onboard_control_sensors_health |= MAV_SYS_STATUS_PREARM_CHECK;
			}

			fillOutComponent(health_report, MAV_SYS_STATUS_SENSOR_BATTERY, health_component_t::battery, msg);
			fillOutComponent(health_report, MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS, health_component_t::motors_escs, msg);
			fillOutComponent(health_report, MAV_SYS_STATUS_RECOVERY_SYSTEM, health_component_t::parachute, msg);
			fillOutComponent(health_report, MAV_SYS_STATUS_OBSTACLE_AVOIDANCE, health_component_t::avoidance, msg);
			fillOutComponent(health_report, MAV_SYS_STATUS_SENSOR_3D_ACCEL, health_component_t::accel, msg);
			fillOutComponent(health_report, MAV_SYS_STATUS_SENSOR_3D_GYRO, health_component_t::gyro, msg);
			fillOutComponent(health_report, MAV_SYS_STATUS_SENSOR_3D_MAG, health_component_t::magnetometer, msg);
			fillOutComponent(health_report, MAV_SYS_STATUS_SENSOR_GPS, health_component_t::gps, msg);
			fillOutComponent(health_report, MAV_SYS_STATUS_SENSOR_RC_RECEIVER, health_component_t::remote_control, msg);
			fillOutComponent(health_report, MAV_SYS_STATUS_AHRS, health_component_t::local_position_estimate, msg);
			fillOutComponent(health_report, MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE, health_component_t::absolute_pressure, msg);
			fillOutComponent(health_report, MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE, health_component_t::differential_pressure,
					 msg);

			msg.load = cpuload.load * 1000.0f;

			// TODO: Determine what data should be put here when there are multiple batteries.
			//  Right now, it uses the lowest battery. This is a safety decision, because if a client is only checking
			//  one battery using this message, it should be the lowest.
			//  In the future, this should somehow determine the "main" battery, or use the "type" field of BATTERY_STATUS
			//  to determine which battery is more important at a given time.
			const battery_status_s &lowest_battery = battery_status[lowest_battery_index];

			if (lowest_battery.connected) {
				msg.voltage_battery = lowest_battery.voltage_filtered_v * 1000.0f;
				msg.current_battery = lowest_battery.current_filtered_a * 100.0f;
				msg.battery_remaining = ceilf(lowest_battery.remaining * 100.0f);

			} else {
				msg.voltage_battery = UINT16_MAX;
				msg.current_battery = -1;
				msg.battery_remaining = -1;
			}

			mavlink_msg_sys_status_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // SYS_STATUS_HPP
