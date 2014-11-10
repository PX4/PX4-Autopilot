/****************************************************************************
 *
 *   Copyright (C) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file objects_common.cpp
 *
 * Common object definitions without a better home.
 */

/**
 * @defgroup topics List of all uORB topics.
 */

#include <nuttx/config.h>

#include <drivers/drv_orb_dev.h>

#include <drivers/drv_mag.h>
ORB_DEFINE(sensor_mag0, struct mag_report);
ORB_DEFINE(sensor_mag1, struct mag_report);
ORB_DEFINE(sensor_mag2, struct mag_report);

#include <drivers/drv_accel.h>
ORB_DEFINE(sensor_accel0, struct accel_report);
ORB_DEFINE(sensor_accel1, struct accel_report);
ORB_DEFINE(sensor_accel2, struct accel_report);

#include <drivers/drv_gyro.h>
ORB_DEFINE(sensor_gyro0, struct gyro_report);
ORB_DEFINE(sensor_gyro1, struct gyro_report);
ORB_DEFINE(sensor_gyro2, struct gyro_report);

#include <drivers/drv_baro.h>
ORB_DEFINE(sensor_baro0, struct baro_report);
ORB_DEFINE(sensor_baro1, struct baro_report);

#include <drivers/drv_range_finder.h>
ORB_DEFINE(sensor_range_finder, struct range_finder_report);

#include <drivers/drv_pwm_output.h>
ORB_DEFINE(output_pwm, struct pwm_output_values);

#include <drivers/drv_rc_input.h>
ORB_DEFINE(input_rc, struct rc_input_values);

#include "topics/vehicle_attitude.h"
ORB_DEFINE(vehicle_attitude, struct vehicle_attitude_s);

#include "topics/sensor_combined.h"
ORB_DEFINE(sensor_combined, struct sensor_combined_s);

#include "topics/vehicle_gps_position.h"
ORB_DEFINE(vehicle_gps_position, struct vehicle_gps_position_s);

#include "topics/satellite_info.h"
ORB_DEFINE(satellite_info, struct satellite_info_s);

#include "topics/home_position.h"
ORB_DEFINE(home_position, struct home_position_s);

#include "topics/vehicle_status.h"
ORB_DEFINE(vehicle_status, struct vehicle_status_s);

#include "topics/safety.h"
ORB_DEFINE(safety, struct safety_s);

#include "topics/battery_status.h"
ORB_DEFINE(battery_status, struct battery_status_s);

#include "topics/servorail_status.h"
ORB_DEFINE(servorail_status, struct servorail_status_s);

#include "topics/system_power.h"
ORB_DEFINE(system_power, struct system_power_s);

#include "topics/vehicle_global_position.h"
ORB_DEFINE(vehicle_global_position, struct vehicle_global_position_s);

#include "topics/vehicle_local_position.h"
ORB_DEFINE(vehicle_local_position, struct vehicle_local_position_s);

#include "topics/vehicle_vicon_position.h"
ORB_DEFINE(vehicle_vicon_position, struct vehicle_vicon_position_s);

#include "topics/vehicle_rates_setpoint.h"
ORB_DEFINE(vehicle_rates_setpoint, struct vehicle_rates_setpoint_s);

#include "topics/rc_channels.h"
ORB_DEFINE(rc_channels, struct rc_channels_s);

#include "topics/vehicle_command.h"
ORB_DEFINE(vehicle_command, struct vehicle_command_s);

#include "topics/vehicle_control_mode.h"
ORB_DEFINE(vehicle_control_mode, struct vehicle_control_mode_s);

#include "topics/vehicle_local_position_setpoint.h"
ORB_DEFINE(vehicle_local_position_setpoint, struct vehicle_local_position_setpoint_s);

#include "topics/vehicle_bodyframe_speed_setpoint.h"
ORB_DEFINE(vehicle_bodyframe_speed_setpoint, struct vehicle_bodyframe_speed_setpoint_s);

#include "topics/position_setpoint_triplet.h"
ORB_DEFINE(position_setpoint_triplet, struct position_setpoint_triplet_s);

#include "topics/vehicle_global_velocity_setpoint.h"
ORB_DEFINE(vehicle_global_velocity_setpoint, struct vehicle_global_velocity_setpoint_s);

#include "topics/mission.h"
ORB_DEFINE(offboard_mission, struct mission_s);
ORB_DEFINE(onboard_mission, struct mission_s);

#include "topics/mission_result.h"
ORB_DEFINE(mission_result, struct mission_result_s);

#include "topics/fence.h"
ORB_DEFINE(fence, unsigned);

#include "topics/vehicle_attitude_setpoint.h"
ORB_DEFINE(vehicle_attitude_setpoint, struct vehicle_attitude_setpoint_s);

#include "topics/manual_control_setpoint.h"
ORB_DEFINE(manual_control_setpoint, struct manual_control_setpoint_s);

#include "topics/vehicle_control_debug.h"
ORB_DEFINE(vehicle_control_debug, struct vehicle_control_debug_s);

#include "topics/offboard_control_setpoint.h"
ORB_DEFINE(offboard_control_setpoint, struct offboard_control_setpoint_s);

#include "topics/optical_flow.h"
ORB_DEFINE(optical_flow, struct optical_flow_s);

#include "topics/filtered_bottom_flow.h"
ORB_DEFINE(filtered_bottom_flow, struct filtered_bottom_flow_s);

#include "topics/omnidirectional_flow.h"
ORB_DEFINE(omnidirectional_flow, struct omnidirectional_flow_s);

#include "topics/airspeed.h"
ORB_DEFINE(airspeed, struct airspeed_s);

#include "topics/differential_pressure.h"
ORB_DEFINE(differential_pressure, struct differential_pressure_s);

#include "topics/subsystem_info.h"
ORB_DEFINE(subsystem_info, struct subsystem_info_s);

/* actuator controls, as requested by controller */
#include "topics/actuator_controls.h"
ORB_DEFINE(actuator_controls_0, struct actuator_controls_s);
ORB_DEFINE(actuator_controls_1, struct actuator_controls_s);
ORB_DEFINE(actuator_controls_2, struct actuator_controls_s);
ORB_DEFINE(actuator_controls_3, struct actuator_controls_s);

#include "topics/actuator_armed.h"
ORB_DEFINE(actuator_armed, struct actuator_armed_s);

#include "topics/actuator_outputs.h"
ORB_DEFINE(actuator_outputs_0, struct actuator_outputs_s);
ORB_DEFINE(actuator_outputs_1, struct actuator_outputs_s);
ORB_DEFINE(actuator_outputs_2, struct actuator_outputs_s);
ORB_DEFINE(actuator_outputs_3, struct actuator_outputs_s);

#include "topics/multirotor_motor_limits.h"
ORB_DEFINE(multirotor_motor_limits, struct multirotor_motor_limits_s);

#include "topics/telemetry_status.h"
ORB_DEFINE(telemetry_status_0, struct telemetry_status_s);
ORB_DEFINE(telemetry_status_1, struct telemetry_status_s);
ORB_DEFINE(telemetry_status_2, struct telemetry_status_s);
ORB_DEFINE(telemetry_status_3, struct telemetry_status_s);

#include "topics/test_motor.h"
ORB_DEFINE(test_motor, struct test_motor_s);

#include "topics/debug_key_value.h"
ORB_DEFINE(debug_key_value, struct debug_key_value_s);

#include "topics/navigation_capabilities.h"
ORB_DEFINE(navigation_capabilities, struct navigation_capabilities_s);

#include "topics/esc_status.h"
ORB_DEFINE(esc_status, struct esc_status_s);

#include "topics/encoders.h"
ORB_DEFINE(encoders, struct encoders_s);

#include "topics/estimator_status.h"
ORB_DEFINE(estimator_status, struct estimator_status_report);

#include "topics/vision_position_estimate.h"
ORB_DEFINE(vision_position_estimate, struct vision_position_estimate);

#include "topics/vehicle_force_setpoint.h"
ORB_DEFINE(vehicle_force_setpoint, struct vehicle_force_setpoint_s);

#include "topics/tecs_status.h"
ORB_DEFINE(tecs_status, struct tecs_status_s);

#include "topics/wind_estimate.h"
ORB_DEFINE(wind_estimate, struct wind_estimate_s);
