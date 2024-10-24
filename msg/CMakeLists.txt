############################################################################
#
#   Copyright (c) 2016-2022 PX4 Development Team. All rights reserved.
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

# Support IN_LIST if() operator
cmake_policy(SET CMP0057 NEW)

include(px4_list_make_absolute)

set(msg_files
	aux/ActionRequest.msg
	aux/ActuatorArmed.msg
	aux/ActuatorControlsStatus.msg
	aux/ActuatorOutputs.msg
	aux/ActuatorServosTrim.msg
	aux/ActuatorTest.msg
	aux/AdcReport.msg
	aux/Airspeed.msg
	aux/AirspeedValidated.msg
	aux/AirspeedWind.msg
	aux/AutotuneAttitudeControlStatus.msg
	aux/BatteryStatus.msg
	aux/Buffer128.msg
	aux/ButtonEvent.msg
	aux/CameraCapture.msg
	aux/CameraStatus.msg
	aux/CameraTrigger.msg
	aux/CanInterfaceStatus.msg
	aux/CellularStatus.msg
	aux/CollisionConstraints.msg
	aux/CollisionReport.msg
	aux/ControlAllocatorStatus.msg
	aux/Cpuload.msg
	aux/DatamanRequest.msg
	aux/DatamanResponse.msg
	aux/DebugArray.msg
	aux/DebugKeyValue.msg
	aux/DebugValue.msg
	aux/DebugVect.msg
	aux/DifferentialPressure.msg
	aux/DistanceSensorModeChangeRequest.msg
	aux/DistanceSensor.msg
	aux/Ekf2Timestamps.msg
	aux/EscReport.msg
	aux/EscStatus.msg
	aux/EstimatorAidSource1d.msg
	aux/EstimatorAidSource2d.msg
	aux/EstimatorAidSource3d.msg
	aux/EstimatorBias3d.msg
	aux/EstimatorBias.msg
	aux/EstimatorEventFlags.msg
	aux/EstimatorGpsStatus.msg
	aux/EstimatorInnovations.msg
	aux/EstimatorSelectorStatus.msg
	aux/EstimatorSensorBias.msg
	aux/EstimatorStates.msg
	aux/EstimatorStatusFlags.msg
	aux/EstimatorStatus.msg
	aux/Event.msg
	aux/FailsafeFlags.msg
	aux/FailureDetectorStatus.msg
	aux/FigureEightStatus.msg
	aux/FlightPhaseEstimation.msg
	aux/FollowTargetEstimator.msg
	aux/FollowTarget.msg
	aux/FollowTargetStatus.msg
	aux/FuelTankStatus.msg
	aux/GeneratorStatus.msg
	aux/GeofenceResult.msg
	aux/GeofenceStatus.msg
	aux/GimbalControls.msg
	aux/GimbalDeviceAttitudeStatus.msg
	aux/GimbalDeviceInformation.msg
	aux/GimbalDeviceSetAttitude.msg
	aux/GimbalManagerInformation.msg
	aux/GimbalManagerSetAttitude.msg
	aux/GimbalManagerSetManualControl.msg
	aux/GimbalManagerStatus.msg
	aux/GpioConfig.msg
	aux/GpioIn.msg
	aux/GpioOut.msg
	aux/GpioRequest.msg
	aux/GpsDump.msg
	aux/GpsInjectData.msg
	aux/Gripper.msg
	aux/HealthReport.msg
	aux/HeaterStatus.msg
	aux/HomePosition.msg
	aux/HoverThrustEstimate.msg
	aux/InputRc.msg
	aux/InternalCombustionEngineStatus.msg
	aux/IridiumsbdStatus.msg
	aux/IrlockReport.msg
	aux/LandingGear.msg
	aux/LandingGearWheel.msg
	aux/LandingTargetInnovations.msg
	aux/LandingTargetPose.msg
	aux/LaunchDetectionStatus.msg
	aux/LedControl.msg
	aux/LoggerStatus.msg
	aux/LogMessage.msg
	aux/MagnetometerBiasEstimate.msg
	aux/MagWorkerData.msg
	aux/ManualControlSwitches.msg
	aux/MavlinkLog.msg
	aux/MavlinkTunnel.msg
	aux/MessageFormatRequest.msg
	aux/MessageFormatResponse.msg
	aux/Mission.msg
	aux/MissionResult.msg
	aux/MountOrientation.msg
	aux/NavigatorMissionItem.msg
	aux/NavigatorStatus.msg
	aux/NormalizedUnsignedSetpoint.msg
	aux/NpfgStatus.msg
	aux/ObstacleDistance.msg
	aux/OffboardControlMode.msg
	aux/OnboardComputerStatus.msg
	aux/OpenDroneIdArmStatus.msg
	aux/OpenDroneIdOperatorId.msg
	aux/OpenDroneIdSelfId.msg
	aux/OpenDroneIdSystem.msg
	aux/OrbitStatus.msg
	aux/OrbTestLarge.msg
	aux/OrbTestMedium.msg
	aux/OrbTest.msg
	aux/ParameterResetRequest.msg
	aux/ParameterSetUsedRequest.msg
	aux/ParameterSetValueRequest.msg
	aux/ParameterSetValueResponse.msg
	aux/ParameterUpdate.msg
	aux/Ping.msg
	aux/PositionControllerLandingStatus.msg
	aux/PositionControllerStatus.msg
	aux/PositionSetpoint.msg
	aux/PositionSetpointTriplet.msg
	aux/PowerButtonState.msg
	aux/PowerMonitor.msg
	aux/PpsCapture.msg
	aux/PwmInput.msg
	aux/Px4ioStatus.msg
	aux/QshellReq.msg
	aux/QshellRetval.msg
	aux/RadioStatus.msg
	aux/RateCtrlStatus.msg
	aux/RcChannels.msg
	aux/RcParameterMap.msg
	aux/RoverAckermannGuidanceStatus.msg
	aux/RoverAckermannStatus.msg
	aux/RoverDifferentialGuidanceStatus.msg
	aux/RoverDifferentialSetpoint.msg
	aux/RoverDifferentialStatus.msg
	aux/RoverMecanumGuidanceStatus.msg
	aux/RoverMecanumSetpoint.msg
	aux/RoverMecanumStatus.msg
	aux/Rpm.msg
	aux/RtlStatus.msg
	aux/RtlTimeEstimate.msg
	aux/SatelliteInfo.msg
	aux/SensorAccelFifo.msg
	aux/SensorAccel.msg
	aux/SensorAirflow.msg
	aux/SensorBaro.msg
	aux/SensorCombined.msg
	aux/SensorCorrection.msg
	aux/SensorGnssRelative.msg
	aux/SensorGps.msg
	aux/SensorGyroFft.msg
	aux/SensorGyroFifo.msg
	aux/SensorGyro.msg
	aux/SensorHygrometer.msg
	aux/SensorMag.msg
	aux/SensorOpticalFlow.msg
	aux/SensorPreflightMag.msg
	aux/SensorSelection.msg
	aux/SensorsStatusImu.msg
	aux/SensorsStatus.msg
	aux/SensorUwb.msg
	aux/SystemPower.msg
	aux/TakeoffStatus.msg
	aux/TaskStackInfo.msg
	aux/TecsStatus.msg
	aux/TelemetryStatus.msg
	aux/TiltrotorExtraControls.msg
	aux/TimesyncStatus.msg
	aux/TrajectoryBezier.msg
	aux/TrajectoryWaypoint.msg
	aux/TransponderReport.msg
	aux/TuneControl.msg
	aux/UavcanParameterRequest.msg
	aux/UavcanParameterValue.msg
	aux/UlogStreamAck.msg
	aux/UlogStream.msg
	aux/VehicleAcceleration.msg
	aux/VehicleAirData.msg
	aux/VehicleAngularAccelerationSetpoint.msg
	aux/VehicleConstraints.msg
	aux/VehicleImu.msg
	aux/VehicleImuStatus.msg
	aux/VehicleLandDetected.msg
	aux/VehicleLocalPositionSetpoint.msg
	aux/VehicleMagnetometer.msg
	aux/VehicleOpticalFlow.msg
	aux/VehicleOpticalFlowVel.msg
	aux/VehicleRoi.msg
	aux/VehicleThrustSetpoint.msg
	aux/VehicleTorqueSetpoint.msg
	aux/VehicleTrajectoryBezier.msg
	aux/VehicleTrajectoryWaypoint.msg
	aux/VelocityLimits.msg
	aux/VtolVehicleStatus.msg
	aux/WheelEncoders.msg
	aux/Wind.msg
	aux/YawEstimatorStatus.msg
	core/ActuatorMotors.msg
	core/ActuatorServos.msg
	core/ArmingCheckReply.msg
	core/ArmingCheckRequest.msg
	core/ConfigOverrides.msg
	core/GotoSetpoint.msg
	core/ManualControlSetpoint.msg
	core/ModeCompleted.msg
	core/RegisterExtComponentReply.msg
	core/RegisterExtComponentRequest.msg
	core/TrajectorySetpoint.msg
	core/UnregisterExtComponent.msg
	core/VehicleAngularVelocity.msg
	core/VehicleAttitude.msg
	core/VehicleAttitudeSetpoint.msg
	core/VehicleCommandAck.msg
	core/VehicleCommand.msg
	core/VehicleControlMode.msg
	core/VehicleGlobalPosition.msg
	core/VehicleLocalPosition.msg
	core/VehicleOdometry.msg
	core/VehicleRatesSetpoint.msg
	core/VehicleStatus.msg
)
list(SORT msg_files)

px4_list_make_absolute(msg_files ${CMAKE_CURRENT_SOURCE_DIR} ${msg_files})

if(NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	# Check that the msg directory and the CMakeLists.txt file exists
	if(EXISTS ${EXTERNAL_MODULES_LOCATION}/msg/CMakeLists.txt)
		add_subdirectory(${EXTERNAL_MODULES_LOCATION}/msg external_msg)

		# Add each of the external message files to the global msg_files list
		foreach(external_msg_file ${config_msg_list_external})
			list(APPEND msg_files ${EXTERNAL_MODULES_LOCATION}/msg/${external_msg_file})
		endforeach()
	endif()
endif()

# headers
set(msg_out_path ${PX4_BINARY_DIR}/uORB/topics)
set(ucdr_out_path ${PX4_BINARY_DIR}/uORB/ucdr)
set(msg_source_out_path ${CMAKE_CURRENT_BINARY_DIR}/topics_sources)

set(uorb_headers)
set(uorb_sources)
set(uorb_ucdr_headers)
set(uorb_json_files)
foreach(msg_file ${msg_files})
	get_filename_component(msg ${msg_file} NAME_WE)

	# Pascal case to snake case (MsgFile -> msg_file)
	string(REGEX REPLACE "(.)([A-Z][a-z]+)" "\\1_\\2" msg "${msg}")
	string(REGEX REPLACE "([a-z0-9])([A-Z])" "\\1_\\2" msg "${msg}")
	string(TOLOWER "${msg}" msg)

	list(APPEND uorb_headers ${msg_out_path}/${msg}.h)
	list(APPEND uorb_sources ${msg_source_out_path}/${msg}.cpp)
	list(APPEND uorb_ucdr_headers ${ucdr_out_path}/${msg}.h)
	list(APPEND uorb_json_files ${msg_source_out_path}/${msg}.json)
endforeach()

# set parent scope msg_files for ROS
set(msg_files ${msg_files} PARENT_SCOPE)

# Generate uORB headers
add_custom_command(
	OUTPUT
		${uorb_headers}
		${msg_out_path}/uORBTopics.hpp
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--headers
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR}/aux ${CMAKE_CURRENT_SOURCE_DIR}/core
		-o ${msg_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/uorb
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/msg.h.em
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/uORBTopics.hpp.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic headers"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)
add_custom_target(uorb_headers DEPENDS ${uorb_headers})

add_custom_command(
	OUTPUT
		${uorb_json_files}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--json
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR}/aux ${CMAKE_CURRENT_SOURCE_DIR}/core
		-o ${msg_source_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/uorb
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/msg.json.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB json files"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
)
add_custom_target(uorb_json_files DEPENDS ${uorb_json_files})

set(uorb_message_fields_cpp_file ${msg_source_out_path}/uORBMessageFieldsGenerated.cpp)
set(uorb_message_fields_header_file ${msg_out_path}/uORBMessageFieldsGenerated.hpp)
add_custom_command(
	OUTPUT
		${uorb_message_fields_cpp_file}
		${uorb_message_fields_header_file}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_compressed_fields.py
		-f ${uorb_json_files}
		--source-output-file ${uorb_message_fields_cpp_file}
		--header-output-file ${uorb_message_fields_header_file}
	DEPENDS
		${uorb_json_files}
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_compressed_fields.py
	COMMENT "Generating uORB compressed fields"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
)

# Generate microcdr headers
add_custom_command(
	OUTPUT ${uorb_ucdr_headers}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--headers
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR}/aux ${CMAKE_CURRENT_SOURCE_DIR}/core
		-o ${ucdr_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/ucdr
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/ucdr/msg.h.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic ucdr headers"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)
add_custom_target(uorb_ucdr_headers DEPENDS ${uorb_ucdr_headers})

# Generate uORB sources
add_custom_command(
	OUTPUT
		${uorb_sources}
		${msg_source_out_path}/uORBTopics.cpp
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--sources
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR}/aux ${CMAKE_CURRENT_SOURCE_DIR}/core
		-o ${msg_source_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/uorb
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/msg.cpp.em
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/uORBTopics.cpp.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic sources"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)

add_library(uorb_msgs ${uorb_headers} ${msg_out_path}/uORBTopics.hpp ${uorb_sources} ${msg_source_out_path}/uORBTopics.cpp ${uorb_message_fields_cpp_file})
target_link_libraries(uorb_msgs PRIVATE m)
add_dependencies(uorb_msgs prebuild_targets uorb_headers)

if(CONFIG_LIB_CDRSTREAM)
	set(uorb_cdr_idl)
	set(uorb_cdr_msg)
	set(uorb_cdr_idl_uorb)
	set(idl_include_path ${PX4_BINARY_DIR}/uORB/idl)
	set(idl_out_path ${idl_include_path}/px4/msg)
	set(idl_uorb_path ${PX4_BINARY_DIR}/msg/px4/msg)

	# Make sure that CycloneDDS has been checkout out
	execute_process(COMMAND git submodule sync src/lib/cdrstream/cyclonedds
			WORKING_DIRECTORY ${PX4_SOURCE_DIR} )
	execute_process(COMMAND git submodule update --init --force src/lib/cdrstream/cyclonedds
			WORKING_DIRECTORY ${PX4_SOURCE_DIR} )

	# CycloneDDS-tools doesn't ship with the cdrstream-desc feature thus we've to compile idlc from source
	MESSAGE(STATUS "Configuring idlc :" ${CMAKE_CURRENT_BINARY_DIR}/idlc)
	file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc)
	execute_process(COMMAND ${CMAKE_COMMAND} ${PX4_SOURCE_DIR}/src/lib/cdrstream/cyclonedds
			-DCMAKE_C_COMPILER=/usr/bin/gcc
			-DBUILD_EXAMPLES=OFF
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc
			RESULT_VARIABLE CMD_ERROR
			OUTPUT_FILE CMD_OUTPUT )
	MESSAGE(STATUS "Building idlc :" ${CMAKE_CURRENT_BINARY_DIR}/idlc)
	execute_process(COMMAND ${CMAKE_COMMAND} --build . --target idlc
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc
			RESULT_VARIABLE CMD_ERROR
			OUTPUT_FILE CMD_OUTPUT )
	list(APPEND CMAKE_PROGRAM_PATH "${CMAKE_CURRENT_BINARY_DIR}/idlc/bin")

	# Copy .msg files
	foreach(msg_file ${msg_files})
		get_filename_component(msg ${msg_file} NAME_WE)
		get_filename_component(type ${msg_file} DIRECTORY)
		get_filename_component(type ${type} NAME)
		configure_file(${PX4_SOURCE_DIR}/msg/${type}/${msg}.msg ${idl_out_path}/${msg}.msg COPYONLY)
		list(APPEND uorb_cdr_idl ${idl_out_path}/${msg}.idl)
		list(APPEND uorb_cdr_msg ${idl_out_path}/${msg}.msg)
		list(APPEND uorb_cdr_idl_uorb ${idl_uorb_path}/${msg}.h)
	endforeach()

	# Generate IDL from .msg using rosidl_adapter
	# Note this a submodule inside PX4 hence no ROS2 installation required
	add_custom_command(
		OUTPUT ${uorb_cdr_idl}
		COMMAND ${CMAKE_COMMAND}
		        -E env "PYTHONPATH=${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_adapter:${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_cli"
			${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/src/lib/cdrstream/msg2idl.py
			${uorb_cdr_msg}
		DEPENDS
			${uorb_cdr_msg}
			git_cyclonedds
		COMMENT "Generating IDL from uORB topic headers"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
		)

	# Generate C definitions from IDL
	set(CYCLONEDDS_DIR ${PX4_SOURCE_DIR}/src/lib/cdrstream/cyclonedds)
	include("${CYCLONEDDS_DIR}/cmake/Modules/Generate.cmake")
	idlc_generate(TARGET uorb_cdrstream
                  FEATURES "cdrstream-desc"
                  FILES ${uorb_cdr_idl}
                  INCLUDES ${idl_include_path}
                  BASE_DIR ${idl_include_path}
                  WARNINGS no-implicit-extensibility)
	target_link_libraries(uorb_cdrstream INTERFACE cdr)

	# Generate and overwrite IDL header with custom headers for uORB operatability
	# We typedef the IDL struct the uORB struct so that the IDL offset calculate
	# the offset of internal uORB struct for serialization/deserialization

	# In the future we might want to turn this around let the IDL struct be the leading ABI
	# However we need to remove the padding for logging and remove the re-ordering of fields

	add_custom_target(
		uorb_idl_header
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
			--uorb-idl-header
			-f ${msg_files}
			-i ${CMAKE_CURRENT_SOURCE_DIR}/aux ${CMAKE_CURRENT_SOURCE_DIR}/core
			-o ${idl_uorb_path}
			-e ${PX4_SOURCE_DIR}/Tools/msg/templates/cdrstream
		DEPENDS
			uorb_cdrstream
			${msg_files}
			${PX4_SOURCE_DIR}/Tools/msg/templates/cdrstream/uorb_idl_header.h.em
			${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
			${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
		COMMENT "Generating uORB compatible IDL headers"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
	)
	add_dependencies(uorb_msgs uorb_idl_header)

	# Compile all CDR compatible message defnitions
	target_link_libraries(uorb_msgs PRIVATE uorb_cdrstream )
endif()

if(CONFIG_MODULES_ZENOH)
	# Update kconfig file for topics
	execute_process(COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/zenoh/px_generate_zenoh_topic_files.py
			--zenoh-config
			-f ${msg_files}
			-o ${PX4_SOURCE_DIR}/src/modules/zenoh/
			-e ${PX4_SOURCE_DIR}/Tools/zenoh/templates/zenoh
		)
	add_custom_command(
		OUTPUT
			${PX4_BINARY_DIR}/src/modules/zenoh/uorb_pubsub_factory.hpp
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/zenoh/px_generate_zenoh_topic_files.py
			--zenoh-pub-sub
			-f ${msg_files}
			-o ${PX4_BINARY_DIR}/src/modules/zenoh/
			-e ${PX4_SOURCE_DIR}/Tools/zenoh/templates/zenoh
		DEPENDS
			${msg_files}
			${PX4_SOURCE_DIR}/Tools/zenoh/templates/zenoh/uorb_pubsub_factory.hpp.em
			${PX4_SOURCE_DIR}/Tools/zenoh/px_generate_zenoh_topic_files.py
		COMMENT "Generating Zenoh Topic Code"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
		)
		add_library(zenoh_topics ${PX4_BINARY_DIR}/src/modules/zenoh/uorb_pubsub_factory.hpp)
		set_target_properties(zenoh_topics PROPERTIES LINKER_LANGUAGE CXX)
endif()
