# uORB Message Reference

:::info
This list is [auto-generated](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/msg/generate_msg_docs.py) from the source code.
:::

This topic lists the UORB messages available in PX4 (some of which may be may be shared by the [PX4-ROS 2 Bridge](../ros/ros2_comm.md)).

[Versioned messages](../middleware/uorb.md#message-versioning) track changes to their definitions, with each modification resulting in a version increment.
These messages are most likely shared through the PX4-ROS 2 Bridge.

Graphs showing how these are used [can be found here](../middleware/uorb_graph.md).

## Versioned Messages

- [ActuatorMotors](ActuatorMotors.md) — Motor control message
- [ActuatorServos](ActuatorServos.md) — Servo control message
- [ArmingCheckReply](ArmingCheckReply.md)
- [ArmingCheckRequest](ArmingCheckRequest.md)
- [BatteryStatus](BatteryStatus.md)
- [ConfigOverrides](ConfigOverrides.md) — Configurable overrides by (external) modes or mode executors
- [GotoSetpoint](GotoSetpoint.md) — Position and (optional) heading setpoints with corresponding speed constraints
  Setpoints are intended as inputs to position and heading smoothers, respectively
  Setpoints do not need to be kinematically consistent
  Optional heading setpoints may be specified as controlled by the respective flag
  Unset optional setpoints are not controlled
  Unset optional constraints default to vehicle specifications
- [HomePosition](HomePosition.md) — GPS home position in WGS84 coordinates.
- [ManualControlSetpoint](ManualControlSetpoint.md)
- [ModeCompleted](ModeCompleted.md) — Mode completion result, published by an active mode.
  The possible values of nav_state are defined in the VehicleStatus msg.
  Note that this is not always published (e.g. when a user switches modes or on
  failsafe activation)
- [RegisterExtComponentReply](RegisterExtComponentReply.md)
- [RegisterExtComponentRequest](RegisterExtComponentRequest.md) — Request to register an external component
- [TrajectorySetpoint](TrajectorySetpoint.md) — Trajectory setpoint in NED frame
  Input to PID position controller.
  Needs to be kinematically consistent and feasible for smooth flight.
  setting a value to NaN means the state should not be controlled
- [UnregisterExtComponent](UnregisterExtComponent.md)
- [VehicleAngularVelocity](VehicleAngularVelocity.md)
- [VehicleAttitude](VehicleAttitude.md) — This is similar to the mavlink message ATTITUDE_QUATERNION, but for onboard use
  The quaternion uses the Hamilton convention, and the order is q(w, x, y, z)
- [VehicleAttitudeSetpoint](VehicleAttitudeSetpoint.md)
- [VehicleCommand](VehicleCommand.md) — Vehicle Command uORB message. Used for commanding a mission / action / etc.
  Follows the MAVLink COMMAND_INT / COMMAND_LONG definition
- [VehicleCommandAck](VehicleCommandAck.md) — Vehicle Command Ackonwledgement uORB message.
  Used for acknowledging the vehicle command being received.
  Follows the MAVLink COMMAND_ACK message definition
- [VehicleControlMode](VehicleControlMode.md)
- [VehicleGlobalPosition](VehicleGlobalPosition.md) — Fused global position in WGS84.
  This struct contains global position estimation. It is not the raw GPS
  measurement (@see vehicle_gps_position). This topic is usually published by the position
  estimator, which will take more sources of information into account than just GPS,
  e.g. control inputs of the vehicle in a Kalman-filter implementation.
- [VehicleLandDetected](VehicleLandDetected.md)
- [VehicleLocalPosition](VehicleLocalPosition.md) — Fused local position in NED.
  The coordinate system origin is the vehicle position at the time when the EKF2-module was started.
- [VehicleOdometry](VehicleOdometry.md) — Vehicle odometry data. Fits ROS REP 147 for aerial vehicles
- [VehicleRatesSetpoint](VehicleRatesSetpoint.md)
- [VehicleStatus](VehicleStatus.md) — Encodes the system state of the vehicle published by commander
- [VtolVehicleStatus](VtolVehicleStatus.md) — VEHICLE_VTOL_STATE, should match 1:1 MAVLinks's MAV_VTOL_STATE

## Unversioned Messages

- [ActionRequest](ActionRequest.md)

- [ActuatorArmed](ActuatorArmed.md)

- [ActuatorControlsStatus](ActuatorControlsStatus.md)

- [ActuatorOutputs](ActuatorOutputs.md)

- [ActuatorServosTrim](ActuatorServosTrim.md) — Servo trims, added as offset to servo outputs

- [ActuatorTest](ActuatorTest.md)

- [AdcReport](AdcReport.md)

- [Airspeed](Airspeed.md)

- [AirspeedValidated](AirspeedValidated.md)

- [AirspeedWind](AirspeedWind.md)

- [AutotuneAttitudeControlStatus](AutotuneAttitudeControlStatus.md)

- [ButtonEvent](ButtonEvent.md)

- [CameraCapture](CameraCapture.md)

- [CameraStatus](CameraStatus.md)

- [CameraTrigger](CameraTrigger.md)

- [CanInterfaceStatus](CanInterfaceStatus.md)

- [CellularStatus](CellularStatus.md)

- [CollisionConstraints](CollisionConstraints.md) — Local setpoint constraints in NED frame
  setting something to NaN means that no limit is provided

- [ControlAllocatorStatus](ControlAllocatorStatus.md)

- [Cpuload](Cpuload.md)

- [DatamanRequest](DatamanRequest.md)

- [DatamanResponse](DatamanResponse.md)

- [DebugArray](DebugArray.md)

- [DebugKeyValue](DebugKeyValue.md)

- [DebugValue](DebugValue.md)

- [DebugVect](DebugVect.md)

- [DifferentialPressure](DifferentialPressure.md)

- [DistanceSensor](DistanceSensor.md) — DISTANCE_SENSOR message data

- [DistanceSensorModeChangeRequest](DistanceSensorModeChangeRequest.md)

- [Ekf2Timestamps](Ekf2Timestamps.md) — this message contains the (relative) timestamps of the sensor inputs used by EKF2.
  It can be used for reproducible replay.

- [EscReport](EscReport.md)

- [EscStatus](EscStatus.md)

- [EstimatorAidSource1d](EstimatorAidSource1d.md)

- [EstimatorAidSource2d](EstimatorAidSource2d.md)

- [EstimatorAidSource3d](EstimatorAidSource3d.md)

- [EstimatorBias](EstimatorBias.md)

- [EstimatorBias3d](EstimatorBias3d.md)

- [EstimatorEventFlags](EstimatorEventFlags.md)

- [EstimatorGpsStatus](EstimatorGpsStatus.md)

- [EstimatorInnovations](EstimatorInnovations.md)

- [EstimatorSelectorStatus](EstimatorSelectorStatus.md)

- [EstimatorSensorBias](EstimatorSensorBias.md) — Sensor readings and in-run biases in SI-unit form. Sensor readings are compensated for static offsets,
  scale errors, in-run bias and thermal drift (if thermal compensation is enabled and available).

- [EstimatorStates](EstimatorStates.md)

- [EstimatorStatus](EstimatorStatus.md)

- [EstimatorStatusFlags](EstimatorStatusFlags.md)

- [Event](Event.md) — Events interface

- [FailsafeFlags](FailsafeFlags.md) — Input flags for the failsafe state machine set by the arming & health checks.

- [FailureDetectorStatus](FailureDetectorStatus.md)

- [FigureEightStatus](FigureEightStatus.md)

- [FlightPhaseEstimation](FlightPhaseEstimation.md)

- [FollowTarget](FollowTarget.md)

- [FollowTargetEstimator](FollowTargetEstimator.md)

- [FollowTargetStatus](FollowTargetStatus.md)

- [FuelTankStatus](FuelTankStatus.md)

- [GeneratorStatus](GeneratorStatus.md)

- [GeofenceResult](GeofenceResult.md)

- [GeofenceStatus](GeofenceStatus.md)

- [GimbalControls](GimbalControls.md)

- [GimbalDeviceAttitudeStatus](GimbalDeviceAttitudeStatus.md)

- [GimbalDeviceInformation](GimbalDeviceInformation.md)

- [GimbalDeviceSetAttitude](GimbalDeviceSetAttitude.md)

- [GimbalManagerInformation](GimbalManagerInformation.md)

- [GimbalManagerSetAttitude](GimbalManagerSetAttitude.md)

- [GimbalManagerSetManualControl](GimbalManagerSetManualControl.md)

- [GimbalManagerStatus](GimbalManagerStatus.md)

- [GpioConfig](GpioConfig.md) — GPIO configuration

- [GpioIn](GpioIn.md) — GPIO mask and state

- [GpioOut](GpioOut.md) — GPIO mask and state

- [GpioRequest](GpioRequest.md) — Request GPIO mask to be read

- [GpsDump](GpsDump.md) — This message is used to dump the raw gps communication to the log.

- [GpsInjectData](GpsInjectData.md)

- [Gripper](Gripper.md) — # Used to command an actuation in the gripper, which is mapped to a specific output in the control allocation module

- [HealthReport](HealthReport.md)

- [HeaterStatus](HeaterStatus.md)

- [HoverThrustEstimate](HoverThrustEstimate.md)

- [InputRc](InputRc.md)

- [InternalCombustionEngineControl](InternalCombustionEngineControl.md)

- [InternalCombustionEngineStatus](InternalCombustionEngineStatus.md)

- [IridiumsbdStatus](IridiumsbdStatus.md)

- [IrlockReport](IrlockReport.md) — IRLOCK_REPORT message data

- [LandingGear](LandingGear.md)

- [LandingGearWheel](LandingGearWheel.md)

- [LandingTargetInnovations](LandingTargetInnovations.md)

- [LandingTargetPose](LandingTargetPose.md) — Relative position of precision land target in navigation (body fixed, north aligned, NED) and inertial (world fixed, north aligned, NED) frames

- [LaunchDetectionStatus](LaunchDetectionStatus.md) — Status of the launch detection state machine (fixed-wing only)

- [LedControl](LedControl.md) — LED control: control a single or multiple LED's.
  These are the externally visible LED's, not the board LED's

- [LogMessage](LogMessage.md) — A logging message, output with PX4_WARN, PX4_ERR, PX4_INFO

- [LoggerStatus](LoggerStatus.md)

- [MagWorkerData](MagWorkerData.md)

- [MagnetometerBiasEstimate](MagnetometerBiasEstimate.md)

- [ManualControlSwitches](ManualControlSwitches.md)

- [MavlinkLog](MavlinkLog.md)

- [MavlinkTunnel](MavlinkTunnel.md) — MAV_TUNNEL_PAYLOAD_TYPE enum

- [MessageFormatRequest](MessageFormatRequest.md)

- [MessageFormatResponse](MessageFormatResponse.md)

- [Mission](Mission.md)

- [MissionResult](MissionResult.md)

- [MountOrientation](MountOrientation.md)

- [NavigatorMissionItem](NavigatorMissionItem.md)

- [NavigatorStatus](NavigatorStatus.md) — Current status of a Navigator mode
  The possible values of nav_state are defined in the VehicleStatus msg.

- [NormalizedUnsignedSetpoint](NormalizedUnsignedSetpoint.md)

- [NpfgStatus](NpfgStatus.md)

- [ObstacleDistance](ObstacleDistance.md) — Obstacle distances in front of the sensor.

- [OffboardControlMode](OffboardControlMode.md) — Off-board control mode

- [OnboardComputerStatus](OnboardComputerStatus.md) — ONBOARD_COMPUTER_STATUS message data

- [OpenDroneIdArmStatus](OpenDroneIdArmStatus.md)

- [OpenDroneIdOperatorId](OpenDroneIdOperatorId.md)

- [OpenDroneIdSelfId](OpenDroneIdSelfId.md)

- [OpenDroneIdSystem](OpenDroneIdSystem.md)

- [OrbTest](OrbTest.md)

- [OrbTestLarge](OrbTestLarge.md)

- [OrbTestMedium](OrbTestMedium.md)

- [OrbitStatus](OrbitStatus.md) — ORBIT_YAW_BEHAVIOUR

- [ParameterResetRequest](ParameterResetRequest.md) — ParameterResetRequest : Used by the primary to reset one or all parameter value(s) on the remote

- [ParameterSetUsedRequest](ParameterSetUsedRequest.md) — ParameterSetUsedRequest : Used by a remote to update the used flag for a parameter on the primary

- [ParameterSetValueRequest](ParameterSetValueRequest.md) — ParameterSetValueRequest : Used by a remote or primary to update the value for a parameter at the other end

- [ParameterSetValueResponse](ParameterSetValueResponse.md) — ParameterSetValueResponse : Response to a set value request by either primary or secondary

- [ParameterUpdate](ParameterUpdate.md) — This message is used to notify the system about one or more parameter changes

- [Ping](Ping.md)

- [PositionControllerLandingStatus](PositionControllerLandingStatus.md)

- [PositionControllerStatus](PositionControllerStatus.md)

- [PositionSetpoint](PositionSetpoint.md) — this file is only used in the position_setpoint triple as a dependency

- [PositionSetpointTriplet](PositionSetpointTriplet.md) — Global position setpoint triplet in WGS84 coordinates.
  This are the three next waypoints (or just the next two or one).

- [PowerButtonState](PowerButtonState.md) — power button state notification message

- [PowerMonitor](PowerMonitor.md) — power monitor message

- [PpsCapture](PpsCapture.md)

- [PurePursuitStatus](PurePursuitStatus.md)

- [PwmInput](PwmInput.md)

- [Px4ioStatus](Px4ioStatus.md)

- [QshellReq](QshellReq.md)

- [QshellRetval](QshellRetval.md)

- [RadioStatus](RadioStatus.md)

- [RateCtrlStatus](RateCtrlStatus.md)

- [RcChannels](RcChannels.md)

- [RcParameterMap](RcParameterMap.md)

- [RoverAttitudeSetpoint](RoverAttitudeSetpoint.md)

- [RoverAttitudeStatus](RoverAttitudeStatus.md)

- [RoverRateSetpoint](RoverRateSetpoint.md)

- [RoverRateStatus](RoverRateStatus.md)

- [RoverSteeringSetpoint](RoverSteeringSetpoint.md)

- [RoverThrottleSetpoint](RoverThrottleSetpoint.md)

- [RoverVelocityStatus](RoverVelocityStatus.md)

- [Rpm](Rpm.md)

- [RtlStatus](RtlStatus.md)

- [RtlTimeEstimate](RtlTimeEstimate.md)

- [SatelliteInfo](SatelliteInfo.md)

- [SensorAccel](SensorAccel.md)

- [SensorAccelFifo](SensorAccelFifo.md)

- [SensorAirflow](SensorAirflow.md)

- [SensorBaro](SensorBaro.md)

- [SensorCombined](SensorCombined.md) — Sensor readings in SI-unit form.
  These fields are scaled and offset-compensated where possible and do not
  change with board revisions and sensor updates.

- [SensorCorrection](SensorCorrection.md) — Sensor corrections in SI-unit form for the voted sensor

- [SensorGnssRelative](SensorGnssRelative.md) — GNSS relative positioning information in NED frame. The NED frame is defined as the local topological system at the reference station.

- [SensorGps](SensorGps.md) — GPS position in WGS84 coordinates.
  the field 'timestamp' is for the position & velocity (microseconds)

- [SensorGyro](SensorGyro.md)

- [SensorGyroFft](SensorGyroFft.md)

- [SensorGyroFifo](SensorGyroFifo.md)

- [SensorHygrometer](SensorHygrometer.md)

- [SensorMag](SensorMag.md)

- [SensorOpticalFlow](SensorOpticalFlow.md)

- [SensorPreflightMag](SensorPreflightMag.md) — Pre-flight sensor check metrics.
  The topic will not be updated when the vehicle is armed

- [SensorSelection](SensorSelection.md) — Sensor ID's for the voted sensors output on the sensor_combined topic.
  Will be updated on startup of the sensor module and when sensor selection changes

- [SensorUwb](SensorUwb.md) — UWB distance contains the distance information measured by an ultra-wideband positioning system,
  such as Pozyx or NXP Rddrone.

- [SensorsStatus](SensorsStatus.md) — Sensor check metrics. This will be zero for a sensor that's primary or unpopulated.

- [SensorsStatusImu](SensorsStatusImu.md) — Sensor check metrics. This will be zero for a sensor that's primary or unpopulated.

- [SystemPower](SystemPower.md)

- [TakeoffStatus](TakeoffStatus.md) — Status of the takeoff state machine currently just available for multicopters

- [TaskStackInfo](TaskStackInfo.md) — stack information for a single running process

- [TecsStatus](TecsStatus.md)

- [TelemetryStatus](TelemetryStatus.md)

- [TiltrotorExtraControls](TiltrotorExtraControls.md)

- [TimesyncStatus](TimesyncStatus.md)

- [TransponderReport](TransponderReport.md)

- [TuneControl](TuneControl.md) — This message is used to control the tunes, when the tune_id is set to CUSTOM
  then the frequency, duration are used otherwise those values are ignored.

- [UavcanParameterRequest](UavcanParameterRequest.md) — UAVCAN-MAVLink parameter bridge request type

- [UavcanParameterValue](UavcanParameterValue.md) — UAVCAN-MAVLink parameter bridge response type

- [UlogStream](UlogStream.md) — Message to stream ULog data from the logger. Corresponds to the LOGGING_DATA
  mavlink message

- [UlogStreamAck](UlogStreamAck.md) — Ack a previously sent ulog_stream message that had
  the NEED_ACK flag set

- [VehicleAcceleration](VehicleAcceleration.md)

- [VehicleAirData](VehicleAirData.md)

- [VehicleAngularAccelerationSetpoint](VehicleAngularAccelerationSetpoint.md)

- [VehicleConstraints](VehicleConstraints.md) — Local setpoint constraints in NED frame
  setting something to NaN means that no limit is provided

- [VehicleImu](VehicleImu.md) — IMU readings in SI-unit form.

- [VehicleImuStatus](VehicleImuStatus.md)

- [VehicleLocalPositionSetpoint](VehicleLocalPositionSetpoint.md) — Local position setpoint in NED frame
  Telemetry of PID position controller to monitor tracking.
  NaN means the state was not controlled

- [VehicleMagnetometer](VehicleMagnetometer.md)

- [VehicleOpticalFlow](VehicleOpticalFlow.md) — Optical flow in XYZ body frame in SI units.

- [VehicleOpticalFlowVel](VehicleOpticalFlowVel.md)

- [VehicleRoi](VehicleRoi.md) — Vehicle Region Of Interest (ROI)

- [VehicleThrustSetpoint](VehicleThrustSetpoint.md)

- [VehicleTorqueSetpoint](VehicleTorqueSetpoint.md)

- [VelocityLimits](VelocityLimits.md) — Velocity and yaw rate limits for a multicopter position slow mode only

- [WheelEncoders](WheelEncoders.md)

- [Wind](Wind.md)

- [YawEstimatorStatus](YawEstimatorStatus.md)

- [VehicleStatusV0](VehicleStatusV0.md) — Encodes the system state of the vehicle published by commander