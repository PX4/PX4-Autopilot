############################################################################
#
#   Copyright (c) 2016 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

set(msg_file_names
	actuator_armed.msg
	actuator_controls.msg
	actuator_direct.msg
	actuator_outputs.msg
	adc_report.msg
	airspeed.msg
	att_pos_mocap.msg
	battery_status.msg
	camera_trigger.msg
	camera_capture.msg
	collision_report.msg
	commander_state.msg
	cpuload.msg
	debug_key_value.msg
	differential_pressure.msg
	distance_sensor.msg
	ekf2_innovations.msg
	ekf2_replay.msg
	ekf2_timestamps.msg
	esc_report.msg
	esc_status.msg
	estimator_status.msg
	fence.msg
	fence_vertex.msg
	filtered_bottom_flow.msg
	follow_target.msg
	fw_pos_ctrl_status.msg
	geofence_result.msg
	gps_dump.msg
	gps_inject_data.msg
	hil_sensor.msg
	home_position.msg
	input_rc.msg
	led_control.msg
	log_message.msg
	manual_control_setpoint.msg
	mavlink_log.msg
	mc_att_ctrl_status.msg
	mission.msg
	mission_result.msg
	mount_orientation.msg
	multirotor_motor_limits.msg
	offboard_control_mode.msg
	optical_flow.msg
	output_pwm.msg
	parameter_update.msg
	position_setpoint.msg
	position_setpoint_triplet.msg
	pwm_input.msg
	qshell_req.msg
	rc_channels.msg
	rc_parameter_map.msg
	safety.msg
	satellite_info.msg
	sensor_accel.msg
	sensor_baro.msg
	sensor_combined.msg
	sensor_correction.msg
	sensor_selection.msg
	sensor_gyro.msg
	sensor_mag.msg
	sensor_preflight.msg
	servorail_status.msg
	subsystem_info.msg
	system_power.msg
	task_stack_info.msg
	tecs_status.msg
	telemetry_status.msg
	test_motor.msg
	time_offset.msg
	transponder_report.msg
	uavcan_parameter_request.msg
	uavcan_parameter_value.msg
	ulog_stream.msg
	ulog_stream_ack.msg
	vehicle_attitude.msg
	vehicle_attitude_setpoint.msg
	vehicle_command.msg
	vehicle_command_ack.msg
	vehicle_control_mode.msg
	vehicle_force_setpoint.msg
	vehicle_global_position.msg
	vehicle_gps_position.msg
	vehicle_land_detected.msg
	vehicle_local_position.msg
	vehicle_local_position_setpoint.msg
	vehicle_rates_setpoint.msg
	vehicle_roi.msg
	vehicle_status.msg
	vehicle_status_flags.msg
	vtol_vehicle_status.msg
	wind_estimate.msg
	)

# Get absolute paths
set(msg_files)
set(msg_include_paths)

foreach(msg_file ${msg_file_names})
	list(APPEND msg_files ${CMAKE_CURRENT_SOURCE_DIR}/${msg_file})
endforeach()

list(APPEND msg_include_paths ${CMAKE_CURRENT_SOURCE_DIR})
set(msg_include_paths ${msg_include_paths} PARENT_SCOPE)

set(msg_files ${msg_files} PARENT_SCOPE)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
