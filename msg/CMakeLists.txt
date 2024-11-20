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
	ActionRequest.msg
	ActuatorArmed.msg
	ActuatorControlsStatus.msg
	ActuatorOutputs.msg
	ActuatorServosTrim.msg
	ActuatorTest.msg
	AdcReport.msg
	Airspeed.msg
	AirspeedValidated.msg
	AirspeedWind.msg
	AutotuneAttitudeControlStatus.msg
	Buffer128.msg
	ButtonEvent.msg
	CameraCapture.msg
	CameraStatus.msg
	CameraTrigger.msg
	CanInterfaceStatus.msg
	CellularStatus.msg
	CollisionConstraints.msg
	ControlAllocatorStatus.msg
	Cpuload.msg
	DatamanRequest.msg
	DatamanResponse.msg
	DebugArray.msg
	DebugKeyValue.msg
	DebugValue.msg
	DebugVect.msg
	DifferentialPressure.msg
	DistanceSensor.msg
	DistanceSensorModeChangeRequest.msg
	Ekf2Timestamps.msg
	EscReport.msg
	EscStatus.msg
	EstimatorAidSource1d.msg
	EstimatorAidSource2d.msg
	EstimatorAidSource3d.msg
	EstimatorBias.msg
	EstimatorBias3d.msg
	EstimatorEventFlags.msg
	EstimatorGpsStatus.msg
	EstimatorInnovations.msg
	EstimatorSelectorStatus.msg
	EstimatorSensorBias.msg
	EstimatorStates.msg
	EstimatorStatus.msg
	EstimatorStatusFlags.msg
	Event.msg
	FigureEightStatus.msg
	FailsafeFlags.msg
	FailureDetectorStatus.msg
	FlightPhaseEstimation.msg
	FollowTarget.msg
	FollowTargetEstimator.msg
	FollowTargetStatus.msg
	FuelTankStatus.msg
	GeneratorStatus.msg
	GeofenceResult.msg
	GeofenceStatus.msg
	GimbalControls.msg
	GimbalDeviceAttitudeStatus.msg
	GimbalDeviceInformation.msg
	GimbalDeviceSetAttitude.msg
	GimbalManagerInformation.msg
	GimbalManagerSetAttitude.msg
	GimbalManagerSetManualControl.msg
	GimbalManagerStatus.msg
	GpioConfig.msg
	GpioIn.msg
	GpioOut.msg
	GpioRequest.msg
	GpsDump.msg
	GpsInjectData.msg
	Gripper.msg
	HealthReport.msg
	HeaterStatus.msg
	HoverThrustEstimate.msg
	InputRc.msg
	InternalCombustionEngineStatus.msg
	IridiumsbdStatus.msg
	IrlockReport.msg
	LandingGear.msg
	LandingGearWheel.msg
	LandingTargetInnovations.msg
	LandingTargetPose.msg
	LaunchDetectionStatus.msg
	LedControl.msg
	LoggerStatus.msg
	LogMessage.msg
	MagnetometerBiasEstimate.msg
	MagWorkerData.msg
	ManualControlSwitches.msg
	MavlinkLog.msg
	MavlinkTunnel.msg
	MessageFormatRequest.msg
	MessageFormatResponse.msg
	Mission.msg
	MissionResult.msg
	MountOrientation.msg
	NavigatorMissionItem.msg
	NavigatorStatus.msg
	NormalizedUnsignedSetpoint.msg
	NpfgStatus.msg
	ObstacleDistance.msg
	OffboardControlMode.msg
	OnboardComputerStatus.msg
	OpenDroneIdArmStatus.msg
	OpenDroneIdOperatorId.msg
	OpenDroneIdSelfId.msg
	OpenDroneIdSystem.msg
	OrbitStatus.msg
	OrbTest.msg
	OrbTestLarge.msg
	OrbTestMedium.msg
	ParameterResetRequest.msg
	ParameterSetUsedRequest.msg
	ParameterSetValueRequest.msg
	ParameterSetValueResponse.msg
	ParameterUpdate.msg
	Ping.msg
	PositionControllerLandingStatus.msg
	PositionControllerStatus.msg
	PositionSetpoint.msg
	PositionSetpointTriplet.msg
	PowerButtonState.msg
	PowerMonitor.msg
	PpsCapture.msg
	PwmInput.msg
	Px4ioStatus.msg
	QshellReq.msg
	QshellRetval.msg
	RadioStatus.msg
	RateCtrlStatus.msg
	RcChannels.msg
	RcParameterMap.msg
	RoverAckermannGuidanceStatus.msg
	RoverAckermannSetpoint.msg
	RoverAckermannStatus.msg
	RoverDifferentialGuidanceStatus.msg
	RoverDifferentialSetpoint.msg
	RoverDifferentialStatus.msg
	RoverMecanumGuidanceStatus.msg
	RoverMecanumSetpoint.msg
	RoverMecanumStatus.msg
	Rpm.msg
	RtlStatus.msg
	RtlTimeEstimate.msg
	SatelliteInfo.msg
	SensorAccel.msg
	SensorAccelFifo.msg
	SensorBaro.msg
	SensorCombined.msg
	SensorCorrection.msg
	SensorGnssRelative.msg
	SensorGps.msg
	SensorGyro.msg
	SensorGyroFft.msg
	SensorGyroFifo.msg
	SensorHygrometer.msg
	SensorMag.msg
	SensorOpticalFlow.msg
	SensorPreflightMag.msg
	SensorSelection.msg
	SensorsStatus.msg
	SensorsStatusImu.msg
	SensorUwb.msg
	SensorAirflow.msg
	SystemPower.msg
	TakeoffStatus.msg
	TaskStackInfo.msg
	TecsStatus.msg
	TelemetryStatus.msg
	TiltrotorExtraControls.msg
	TimesyncStatus.msg
	TrajectoryBezier.msg
	TrajectoryWaypoint.msg
	TransponderReport.msg
	TuneControl.msg
	UavcanParameterRequest.msg
	UavcanParameterValue.msg
	UlogStream.msg
	UlogStreamAck.msg
	VehicleAcceleration.msg
	VehicleAirData.msg
	VehicleAngularAccelerationSetpoint.msg
	VehicleConstraints.msg
	VehicleImu.msg
	VehicleImuStatus.msg
	VehicleLocalPositionSetpoint.msg
	VehicleMagnetometer.msg
	VehicleOpticalFlow.msg
	VehicleOpticalFlowVel.msg
	VehicleRoi.msg
	VehicleThrustSetpoint.msg
	VehicleTorqueSetpoint.msg
	VehicleTrajectoryBezier.msg
	VehicleTrajectoryWaypoint.msg
	VelocityLimits.msg
	WheelEncoders.msg
	Wind.msg
	YawEstimatorStatus.msg
	versioned/ActuatorMotors.msg
	versioned/ActuatorServos.msg
	versioned/ArmingCheckReply.msg
	versioned/ArmingCheckRequest.msg
	versioned/BatteryStatus.msg
	versioned/ConfigOverrides.msg
	versioned/GotoSetpoint.msg
	versioned/HomePosition.msg
	versioned/ManualControlSetpoint.msg
	versioned/ModeCompleted.msg
	versioned/RegisterExtComponentReply.msg
	versioned/RegisterExtComponentRequest.msg
	versioned/TrajectorySetpoint.msg
	versioned/UnregisterExtComponent.msg
	versioned/VehicleAngularVelocity.msg
	versioned/VehicleAttitude.msg
	versioned/VehicleAttitudeSetpoint.msg
	versioned/VehicleCommandAck.msg
	versioned/VehicleCommand.msg
	versioned/VehicleControlMode.msg
	versioned/VehicleGlobalPosition.msg
	versioned/VehicleLandDetected.msg
	versioned/VehicleLocalPosition.msg
	versioned/VehicleOdometry.msg
	versioned/VehicleRatesSetpoint.msg
	versioned/VehicleStatus.msg
	versioned/VtolVehicleStatus.msg
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
		-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
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
		-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
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
		-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
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
		-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
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
		configure_file(${msg_file} ${idl_out_path}/${msg}.msg COPYONLY)
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
			-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
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
