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

set(msg_files
	actuator_armed.msg
	actuator_controls.msg
	actuator_direct.msg
	actuator_outputs.msg
	adc_report.msg
	airspeed.msg
	att_pos_mocap.msg
	battery_status.msg
	camera_capture.msg
	camera_trigger.msg
	collision_report.msg
	commander_state.msg
	cpuload.msg
	debug_key_value.msg
	debug_value.msg
	debug_vect.msg
	differential_pressure.msg
	distance_sensor.msg
	ekf2_innovations.msg
	ekf2_timestamps.msg
	esc_report.msg
	esc_status.msg
	estimator_status.msg
	follow_target.msg
	fw_pos_ctrl_status.msg
	geofence_result.msg
	gps_dump.msg
	gps_inject_data.msg
	home_position.msg
	input_rc.msg
	irlock_report.msg
	landing_target_innovations.msg
	landing_target_pose.msg
	led_control.msg
	log_message.msg
	manual_control_setpoint.msg
	mavlink_log.msg
	mission.msg
	mission_result.msg
	mount_orientation.msg
	multirotor_motor_limits.msg
	offboard_control_mode.msg
	optical_flow.msg
	parameter_update.msg
	position_setpoint.msg
	position_setpoint_triplet.msg
	power_button_state.msg
	pwm_input.msg
	qshell_req.msg
	rate_ctrl_status.msg
	rc_channels.msg
	rc_parameter_map.msg
	safety.msg
	satellite_info.msg
	sensor_accel.msg
	sensor_baro.msg
	sensor_bias.msg
	sensor_combined.msg
	sensor_correction.msg
	sensor_gyro.msg
	sensor_mag.msg
	sensor_preflight.msg
	sensor_selection.msg
	servorail_status.msg
	subsystem_info.msg
	system_power.msg
	task_stack_info.msg
	tecs_status.msg
	telemetry_status.msg
	test_motor.msg
	time_offset.msg
	transponder_report.msg
	tune_control.msg
	uavcan_parameter_request.msg
	uavcan_parameter_value.msg
	ulog_stream.msg
	ulog_stream_ack.msg
	vehicle_attitude.msg
	vehicle_attitude_setpoint.msg
	vehicle_command.msg
	vehicle_command_ack.msg
	vehicle_control_mode.msg
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

px4_add_git_submodule(TARGET git_gencpp PATH tools/gencpp)
px4_add_git_submodule(TARGET git_genmsg PATH tools/genmsg)

# headers
set(msg_out_path ${PX4_BINARY_DIR}/uORB/topics)
set(msg_source_out_path	${CMAKE_CURRENT_BINARY_DIR}/topics_sources)

set(uorb_headers)
set(uorb_sources ${msg_source_out_path}/uORBTopics.cpp)
foreach(msg_file ${msg_files})
	get_filename_component(msg ${msg_file} NAME_WE)
	list(APPEND uorb_headers ${msg_out_path}/${msg}.h)
	list(APPEND uorb_sources ${msg_source_out_path}/${msg}.cpp)
endforeach()

# Generate uORB headers
add_custom_command(OUTPUT ${uorb_headers}
	COMMAND ${PYTHON_EXECUTABLE} tools/px_generate_uorb_topic_files.py
		--headers
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR}
		-o ${msg_out_path}
		-e templates/uorb
		-t ${CMAKE_CURRENT_BINARY_DIR}/tmp/headers
		-q
	DEPENDS ${msg_files}
	COMMENT "Generating uORB topic headers"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)
add_custom_target(uorb_headers DEPENDS ${uorb_headers})

# Generate uORB sources
add_custom_command(OUTPUT ${uorb_sources}
	COMMAND ${PYTHON_EXECUTABLE} tools/px_generate_uorb_topic_files.py
		--sources
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR}
		-o ${msg_source_out_path}
		-e templates/uorb
		-t ${CMAKE_CURRENT_BINARY_DIR}/tmp/sources
		-q
	DEPENDS ${msg_files}
	COMMENT "Generating uORB topic sources"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)

px4_add_library(uorb_msgs ${uorb_sources})
add_dependencies(uorb_msgs uorb_headers)
