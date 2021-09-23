############################################################################
#
#   Copyright (c) 2016-2021 PX4 Development Team. All rights reserved.
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
	ActuatorArmed.msg
	ActuatorControls.msg
	ActuatorOutputs.msg
	AdcReport.msg
	Airspeed.msg
	AirspeedValidated.msg
	AirspeedWind.msg
	BatteryStatus.msg
	CameraCapture.msg
	CameraStatus.msg
	CameraTrigger.msg
	CellularStatus.msg
	CollisionConstraints.msg
	CollisionReport.msg
	CommanderState.msg
	ControlAllocatorStatus.msg
	Cpuload.msg
	DifferentialPressure.msg
	DistanceSensor.msg
	Ekf2Timestamps.msg
	EkfGpsDrift.msg
	EscReport.msg
	EscStatus.msg
	EstimatorBaroBias.msg
	EstimatorEventFlags.msg
	EstimatorInnovations.msg
	EstimatorOpticalFlowVel.msg
	EstimatorSelectorStatus.msg
	EstimatorSensorBias.msg
	EstimatorStates.msg
	EstimatorStatusFlags.msg
	EstimatorStatus.msg
	Event.msg
	FollowTarget.msg
	GeneratorStatus.msg
	GeofenceResult.msg
	GimbalDeviceAttitudeStatus.msg
	GimbalDeviceInformation.msg
	GimbalDeviceSetAttitude.msg
	GimbalManagerInformation.msg
	GimbalManagerSetAttitude.msg
	GimbalManagerSetManualControl.msg
	GimbalManagerStatus.msg
	GpsDump.msg
	GpsInjectData.msg
	HeaterStatus.msg
	HomePosition.msg
	HoverThrustEstimate.msg
	InputRc.msg
	InternalCombustionEngineStatus.msg
	IridiumsbdStatus.msg
	IrlockReport.msg
	LandingGear.msg
	LandingTargetInnovations.msg
	LandingTargetPose.msg
	LedControl.msg
	LoggerStatus.msg
	LogMessage.msg
	MagWorkerData.msg
	ManualControlSetpoint.msg
	ManualControlSwitches.msg
	MavlinkLog.msg
	Mission.msg
	MissionResult.msg
	MountOrientation.msg
	MultirotorMotorLimits.msg
	NavigatorMissionItem.msg
	ObstacleDistance.msg
	OffboardControlMode.msg
	OnboardComputerStatus.msg
	OpticalFlow.msg
	OrbitStatus.msg
	ParameterUpdate.msg
	Ping.msg
	PositionControllerLandingStatus.msg
	PositionControllerStatus.msg
	PositionSetpoint.msg
	PositionSetpointTriplet.msg
	PowerButtonState.msg
	PowerMonitor.msg
	PwmInput.msg
	Px4ioStatus.msg
	RadioStatus.msg
	RateCtrlStatus.msg
	RcChannels.msg
	RcParameterMap.msg
	Rpm.msg
	RtlFlightTime.msg
	Safety.msg
	SatelliteInfo.msg
	SensorAccelFifo.msg
	SensorAccel.msg
	SensorBaro.msg
	SensorCombined.msg
	SensorCorrection.msg
	SensorGps.msg
	SensorGyroFft.msg
	SensorGyroFifo.msg
	SensorGyro.msg
	SensorMag.msg
	SensorPreflightMag.msg
	SensorSelection.msg
	SensorsStatusImu.msg
	SystemPower.msg
	TakeoffStatus.msg
	TaskStackInfo.msg
	TecsStatus.msg
	TelemetryStatus.msg
	TestMotor.msg
	Timesync.msg
	TimesyncStatus.msg
	TrajectoryBezier.msg
	TrajectoryWaypoint.msg
	TransponderReport.msg
	TuneControl.msg
	UavcanParameterRequest.msg
	UavcanParameterValue.msg
	UlogStreamAck.msg
	UlogStream.msg
	VehicleAcceleration.msg
	VehicleActuatorSetpoint.msg
	VehicleAirData.msg
	VehicleAngularAcceleration.msg
	VehicleAngularAccelerationSetpoint.msg
	VehicleAngularVelocity.msg
	VehicleAttitude.msg
	VehicleAttitudeSetpoint.msg
	VehicleCommandAck.msg
	VehicleCommand.msg
	VehicleConstraints.msg
	VehicleControlMode.msg
	VehicleGlobalPosition.msg
	VehicleGpsPosition.msg
	VehicleImu.msg
	VehicleImuStatus.msg
	VehicleLandDetected.msg
	VehicleLocalPosition.msg
	VehicleLocalPositionSetpoint.msg
	VehicleMagnetometer.msg
	VehicleOdometry.msg
	VehicleRatesSetpoint.msg
	VehicleRoi.msg
	VehicleStatusFlags.msg
	VehicleStatus.msg
	VehicleThrustSetpoint.msg
	VehicleTorqueSetpoint.msg
	VehicleTrajectoryBezier.msg
	VehicleTrajectoryWaypoint.msg
	VtolVehicleStatus.msg
	WheelEncoders.msg
	Wind.msg
	YawEstimatorStatus.msg
)

if(NOT px4_constrained_flash_build)
	list(APPEND msg_files
		DebugArray.msg
		DebugKeyValue.msg
		DebugValue.msg
		DebugVect.msg
	)
endif()

if(PX4_TESTING)
	list(APPEND msg_files
		OrbTestLarge.msg
		OrbTestMedium.msg
		OrbTest.msg
	)
endif()

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

# set parent scope msg_files for other modules to consume (eg topic_listener)
set(msg_files ${msg_files} PARENT_SCOPE)

# headers
set(msg_out_path ${PX4_BINARY_DIR}/uORB/topics)
set(msg_source_out_path	${CMAKE_CURRENT_BINARY_DIR}/topics_sources)

set(uorb_headers)
set(uorb_sources)
foreach(msg_file ${msg_files})
	get_filename_component(msg ${msg_file} NAME_WE)

	# Pascal case to snake case (MsgFile -> msg_file)
	string(REGEX REPLACE "(.)([A-Z][a-z]+)" "\\1_\\2" msg "${msg}")
	string(REGEX REPLACE "([a-z0-9])([A-Z])" "\\1_\\2" msg "${msg}")
	string(TOLOWER "${msg}" msg)

	list(APPEND uorb_headers ${msg_out_path}/${msg}.h)
	list(APPEND uorb_sources ${msg_source_out_path}/${msg}.cpp)
endforeach()

if(px4_constrained_flash_build)
	set(added_arguments --constrained-flash)
endif()

# Generate uORB headers
add_custom_command(
	OUTPUT
		${uorb_headers}
		${msg_out_path}/uORBTopics.hpp
	COMMAND ${PYTHON_EXECUTABLE} tools/px_generate_uorb_topic_files.py
		--headers
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR}
		-o ${msg_out_path}
		-e templates/uorb
		${added_arguments}
	DEPENDS
		${msg_files}
		templates/uorb/msg.h.em
		templates/uorb/uORBTopics.hpp.em
		tools/px_generate_uorb_topic_files.py
		tools/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic headers"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)
add_custom_target(uorb_headers DEPENDS ${uorb_headers})

# Generate uORB sources
add_custom_command(
	OUTPUT
		${uorb_sources}
		${msg_source_out_path}/uORBTopics.cpp
	COMMAND ${PYTHON_EXECUTABLE} tools/px_generate_uorb_topic_files.py
		--sources
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR}
		-o ${msg_source_out_path}
		-e templates/uorb
		${added_arguments}
	DEPENDS
		${msg_files}
		templates/uorb/msg.cpp.em
		templates/uorb/uORBTopics.cpp.em
		tools/px_generate_uorb_topic_files.py
		tools/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic sources"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)

add_library(uorb_msgs ${uorb_headers} ${msg_out_path}/uORBTopics.hpp ${uorb_sources} ${msg_source_out_path}/uORBTopics.cpp)
target_link_libraries(uorb_msgs PRIVATE m)
add_dependencies(uorb_msgs prebuild_targets uorb_headers)
