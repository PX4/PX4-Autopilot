# Опис повідомлень uORB

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
  Можливі значення nav_state визначені в повідомленні VehicleStatus.
  Зверніть увагу, що це не завжди публікується (наприклад, коли користувач переходить у режими або при активації failsafe)
- [RegisterExtComponentReply](RegisterExtComponentReply.md)
- [RegisterExtComponentRequest](RegisterExtComponentRequest.md) — Request to register an external component
- [TrajectorySetpoint](TrajectorySetpoint.md) — Trajectory setpoint in NED frame
  Input to PID position controller.
  Потрібно мати кінематичну консистентність і бути можливим для плавного польоту.
  встановлення значення NaN означає, що стан не повинен контролюватися
- [UnregisterExtComponent](UnregisterExtComponent.md)
- [VehicleAngularVelocity](VehicleAngularVelocity.md)
- [VehicleAttitude](VehicleAttitude.md) — This is similar to the mavlink message ATTITUDE_QUATERNION, but for onboard use
  The quaternion uses the Hamilton convention, and the order is q(w, x, y, z)
- [VehicleAttitudeSetpoint](VehicleAttitudeSetpoint.md)
- [VehicleCommand](VehicleCommand.md) — Vehicle Command uORB message. Використовується для управління місією / дією / тощо.
  Дотримується визначення MAVLink COMMAND_INT / COMMAND_LONG
- [VehicleCommandAck](VehicleCommandAck.md) — Vehicle Command Ackonwledgement uORB message.
  Використовується для підтвердження отримання команди для транспортного засобу.
  Дотримується визначення MAVLink COMMAND_ACK повідомлення
- [VehicleControlMode](VehicleControlMode.md)
- [VehicleGlobalPosition](VehicleGlobalPosition.md) — Fused global position in WGS84.
  Ця структура містить глобальну оцінку позиції. Це не сирі GPS
  вимірювання (@see vehicle_gps_position). Ця тема зазвичай публікується позиціонером, який враховує більше джерел інформації, ніж просто GPS, наприклад, керування введеннями транспортного засобу в реалізації фільтра Калмана.
- [VehicleLandDetected](VehicleLandDetected.md)
- [VehicleLocalPosition](VehicleLocalPosition.md) — Fused local position in NED.
  Початкова точка координатної системи - це позиція транспортного засобу в момент запуску модуля EKF2.
- [VehicleOdometry](VehicleOdometry.md) — Vehicle odometry data. Відповідає ROS REP 147 для повітряних суден
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
  Це може бути використано для відтворення.

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

- [EstimatorSensorBias](EstimatorSensorBias.md) — Sensor readings and in-run biases in SI-unit form. Показання датчиків компенсуються для статичних зсувів,
  похибки шкали, зсув під час роботи та тепловий зсув (якщо термокомпенсація увімкнена та доступна).

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
  Це зовнішні світлодіоди, а не світлодіоди плати

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
  Ось наступні три способи вказівань (або просто наступні два або один).

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
  Ці поля масштабуються та компенсуються зміщенням, де це можливо, і не змінюються з ревізіями плати та оновленнями сенсора.

- [SensorCorrection](SensorCorrection.md) — Sensor corrections in SI-unit form for the voted sensor

- [SensorGnssRelative](SensorGnssRelative.md) — GNSS relative positioning information in NED frame. NED кадр визначається як локальна топологічна система на задній станції.

- [SensorGps](SensorGps.md) — GPS position in WGS84 coordinates.
  the field 'timestamp' is for the position & velocity (microseconds)

- [SensorGyro](SensorGyro.md)

- [SensorGyroFft](SensorGyroFft.md)

- [SensorGyroFifo](SensorGyroFifo.md)

- [SensorHygrometer](SensorHygrometer.md)

- [SensorMag](SensorMag.md)

- [SensorOpticalFlow](SensorOpticalFlow.md)

- [SensorPreflightMag](SensorPreflightMag.md) — Pre-flight sensor check metrics.
  Тема не буде оновлена, коли транспортний засіб зброєний

- [SensorSelection](SensorSelection.md) — Sensor ID's for the voted sensors output on the sensor_combined topic.
  Буде оновлено при запуску модуля датчика та при зміні вибору датчика

- [SensorUwb](SensorUwb.md) — UWB distance contains the distance information measured by an ultra-wideband positioning system,
  such as Pozyx or NXP Rddrone.

- [SensorsStatus](SensorsStatus.md) — Sensor check metrics. Це значення буде нульовим для датчика, який є первинним або незаповненим.

- [SensorsStatusImu](SensorsStatusImu.md) — Sensor check metrics. Це значення буде нульовим для датчика, який є первинним або незаповненим.

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

- [UlogStream](UlogStream.md) — Message to stream ULog data from the logger. Відповідає повідомленню mavlink LOGGING_DATA
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
  NaN означає, що стан не був контрольований

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