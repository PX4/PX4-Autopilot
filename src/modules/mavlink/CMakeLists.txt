############################################################################
#
#   Copyright (c) 2015-2021 PX4 Development Team. All rights reserved.
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

set(MAVLINK_DIALECT "development") # standard, development, etc

# force mavlink dialect to standard if flash constrained
if(px4_constrained_flash_build)
	set(MAVLINK_DIALECT "standard")
endif()

set(MAVLINK_GIT_DIR "${CMAKE_CURRENT_LIST_DIR}/mavlink")
set(MAVLINK_LIBRARY_DIR "${CMAKE_BINARY_DIR}/mavlink")
file(RELATIVE_PATH MAVLINK_GIT_DIR_RELATIVE ${CMAKE_SOURCE_DIR} ${MAVLINK_GIT_DIR})

px4_add_git_submodule(TARGET git_mavlink_v2 PATH "${MAVLINK_GIT_DIR}")

add_custom_command(
	OUTPUT ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h
	COMMAND
		${PYTHON_EXECUTABLE} ${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
			--lang C --wire-protocol 2.0
			#--no-validate
			#--strict-units
			--output ${MAVLINK_LIBRARY_DIR}
			${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT}.xml
	DEPENDS
		git_mavlink_v2
		${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
		${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT}.xml
	COMMENT "Generating Mavlink ${MAVLINK_DIALECT}: ${MAVLINK_GIT_DIR_RELATIVE}/message_definitions/v1.0/${MAVLINK_DIALECT}.xml"
	USES_TERMINAL
)
add_custom_target(mavlink_c_generate DEPENDS ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h)

# mavlink header only library
add_library(mavlink_c INTERFACE)
target_sources(mavlink_c INTERFACE ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h)
set_source_files_properties(${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h PROPERTIES GENERATED true)
target_include_directories(mavlink_c INTERFACE ${MAVLINK_LIBRARY_DIR} ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT})
target_compile_options(mavlink_c INTERFACE -Wno-address-of-packed-member -Wno-cast-align)


px4_add_module(
	MODULE modules__mavlink
	MAIN mavlink
	COMPILE_FLAGS
		-Wno-enum-compare # ROTATION <-> MAV_SENSOR_ROTATION
		#-DDEBUG_BUILD
	SRCS
		mavlink.c
		mavlink_command_sender.cpp
		mavlink_events.cpp
		mavlink_ftp.cpp
		mavlink_log_handler.cpp
		mavlink_main.cpp
		mavlink_messages.cpp
		mavlink_mission.cpp
		mavlink_parameters.cpp
		mavlink_rate_limiter.cpp
		mavlink_receiver.cpp
		mavlink_shell.cpp
		mavlink_simple_analyzer.cpp
		mavlink_stream.cpp
		mavlink_timesync.cpp
		mavlink_ulog.cpp
		MavlinkStatustextHandler.cpp
		tune_publisher.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		airspeed
		component_general_json # for checksums.h
		drivers_accelerometer
		drivers_barometer
		drivers_gyroscope
		drivers_magnetometer
		conversion
		sensor_calibration
		geo
		mavlink_c
		tunes
		version
	UNITY_BUILD
	PUBLICATIONS
		"px4::msg::Mission /mission"
		"px4::msg::RcParameterMap /rc_parameter_map"
		"px4::msg::TelemetryStatus /telemetry_status"
		"px4::msg::TimesyncStatus /timesync_status"
		"px4::msg::UavcanParameterRequest /uavcan_parameter_request" # TODO: srv
		"px4::msg::UavcanParameterValue /uavcan_parameter_value" # TODO: srv
		"px4::msg::UlogStreamAck /ulog_stream_ack" # TODO: srv

		"px4::msg::InternalCombustionEngineStatus /internal_combustion_engine_status" # TODO: SITL

		# TODO: mavlink receiver
		"px4::msg::CollisionReport /collision_report"
		"px4::msg::VehicleOdometry /vehicle_visual_odometry"
		"px4::msg::VehicleOdometry /vehicle_mocap_odometry"
		"px4::msg::OffboardControlMode /offboard_control_mode"
		"px4::msg::CameraStatus /camera_status"
		"px4::msg::CellularStatus /cellular_status"
		"px4::msg::FollowTarget /follow_target"
		"px4::msg::GimbalManagerSetAttitude /gimbal_manager_set_attitude"
		"px4::msg::GimbalManagerSetManualControl /gimbal_manager_set_manual_control"
		"px4::msg::GimbalDeviceInformation /gimbal_device_information"
		"px4::msg::LandingTargetPose /landing_target_pose"
		"px4::msg::OnboardComputerStatus /onboard_computer_status"
		"px4::msg::Ping /ping"
		"px4::msg::GpsInjectData /gps_inject_data"
		"px4::msg::RadioStatus /radio_status"
		"px4::msg::TransponderReport /transponder_report"
		"px4::msg::GeneratorStatus /generator_status"
		"px4::msg::InputRc /input_rc"
		"px4::msg::OpticalFlow /optical_flow"
		"px4::msg::SensorGps /sensor_gps"
		"px4::msg::VehicleOdometry /vehicle_odometry"
		"px4::msg::ObstacleDistance /obstacle_distance"
		"px4::msg::VehicleRatesSetpoint /vehicle_rates_setpoint"

		"px4::msg::DebugArray /debug_array"
		"px4::msg::DebugKeyValue /debug_key_value"
		"px4::msg::DebugValue /debug_value"
		"px4::msg::DebugVect /debug_vect"
	SUBSCRIPTIONS
		"px4::msg::ActuatorOutputs /actuator_outputs"
		"px4::msg::AutotuneAttitudeControlStatus /autotune_attitude_control_status"
		"px4::msg::CameraCapture /camera_capture"
		"px4::msg::CameraTrigger /camera_trigger"
		"px4::msg::MountOrientation /mount_orientation"
		"px4::msg::SensorHygrometer /sensor_hygrometer"
		"px4::msg::SatelliteInfo /satellite_info"
		"px4::msg::TecsStatus /tecs_status"
		"px4::msg::VehicleAttitudeSetpoint /fw_virtual_attitude_setpoint" # VTOL
		"px4::msg::VehicleAttitudeSetpoint /mc_virtual_attitude_setpoint" # VTOL
		"px4::msg::GimbalDeviceAttitudeStatus /gimbal_device_attitude_status"
		"px4::msg::GimbalDeviceSetAttitude /gimbal_device_set_attitude"
		"px4::msg::GimbalManagerInformation /gimbal_manager_information"
		"px4::msg::GimbalManagerStatus /gimbal_manager_status"
		"px4::msg::VehicleAttitudeSetpoint /vehicle_attitude_setpoint"
		"px4::msg::VehicleTrajectoryBezier /vehicle_trajectory_bezier"
		"px4::msg::VehicleTrajectoryWaypoint /vehicle_trajectory_waypoint"
		"px4::msg::VehicleTrajectoryWaypoint /vehicle_trajectory_waypoint_desired"

		"px4::msg::ObstacleDistance /obstacle_distance_fused"
		"px4::msg::OrbitStatus /orbit_status"
		"px4::msg::VehicleLocalPositionSetpoint /vehicle_local_position_setpoint"

		"px4::msg::VehicleAngularVelocity /vehicle_angular_velocity_groundtruth"
		"px4::msg::VehicleAttitude /vehicle_attitude_groundtruth"
		"px4::msg::VehicleGlobalPosition /vehicle_global_position_groundtruth"
		"px4::msg::VehicleLocalPosition /vehicle_local_position_groundtruth"
	)

if(PX4_TESTING)
	add_subdirectory(mavlink_tests)
endif()

px4_add_unit_gtest(SRC MavlinkStatustextHandlerTest.cpp
	INCLUDES ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}
	COMPILE_FLAGS
		-Wno-address-of-packed-member # TODO: fix in c_library_v2
		LINKLIBS modules__mavlink
	)
