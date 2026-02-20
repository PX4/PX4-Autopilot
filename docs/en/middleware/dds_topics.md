# dds_topics.yaml — PX4 Topics Exposed to ROS 2

::: info
This document is [auto-generated](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/msg/generate_msg_docs.py) from the source code.
:::

The [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml) file specifies which uORB message definitions are compiled into the [uxrce_dds_client](../modules/modules_system.md#uxrce-dds-client) module when [PX4 is built](../middleware/uxrce_dds.md#code-generation), and hence which topics are available for ROS 2 applications to subscribe or publish (by default).

This document shows a markdown-rendered version of [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml), listing the publications, subscriptions, and so on.

## Publications

| Topic                                    | Type                                                                                   | Rate Limit |
| ---------------------------------------- | -------------------------------------------------------------------------------------- | ---------- |
| `/fmu/out/register_ext_component_reply`  | [px4_msgs::msg::RegisterExtComponentReply](../msg_docs/RegisterExtComponentReply.md)   |
| `/fmu/out/arming_check_request`          | [px4_msgs::msg::ArmingCheckRequest](../msg_docs/ArmingCheckRequest.md)                 | 5.0        |
| `/fmu/out/mode_completed`                | [px4_msgs::msg::ModeCompleted](../msg_docs/ModeCompleted.md)                           | 50.0       |
| `/fmu/out/battery_status`                | [px4_msgs::msg::BatteryStatus](../msg_docs/BatteryStatus.md)                           | 1.0        |
| `/fmu/out/collision_constraints`         | [px4_msgs::msg::CollisionConstraints](../msg_docs/CollisionConstraints.md)             | 50.0       |
| `/fmu/out/estimator_status_flags`        | [px4_msgs::msg::EstimatorStatusFlags](../msg_docs/EstimatorStatusFlags.md)             | 5.0        |
| `/fmu/out/failsafe_flags`                | [px4_msgs::msg::FailsafeFlags](../msg_docs/FailsafeFlags.md)                           | 5.0        |
| `/fmu/out/manual_control_setpoint`       | [px4_msgs::msg::ManualControlSetpoint](../msg_docs/ManualControlSetpoint.md)           | 25.0       |
| `/fmu/out/message_format_response`       | [px4_msgs::msg::MessageFormatResponse](../msg_docs/MessageFormatResponse.md)           |
| `/fmu/out/position_setpoint_triplet`     | [px4_msgs::msg::PositionSetpointTriplet](../msg_docs/PositionSetpointTriplet.md)       | 5.0        |
| `/fmu/out/sensor_combined`               | [px4_msgs::msg::SensorCombined](../msg_docs/SensorCombined.md)                         |
| `/fmu/out/timesync_status`               | [px4_msgs::msg::TimesyncStatus](../msg_docs/TimesyncStatus.md)                         | 10.0       |
| `/fmu/out/transponder_report`            | [px4_msgs::msg::TransponderReport](../msg_docs/TransponderReport.md)                   |
| `/fmu/out/vehicle_land_detected`         | [px4_msgs::msg::VehicleLandDetected](../msg_docs/VehicleLandDetected.md)               | 5.0        |
| `/fmu/out/vehicle_attitude`              | [px4_msgs::msg::VehicleAttitude](../msg_docs/VehicleAttitude.md)                       | 50.0       |
| `/fmu/out/vehicle_control_mode`          | [px4_msgs::msg::VehicleControlMode](../msg_docs/VehicleControlMode.md)                 | 50.0       |
| `/fmu/out/vehicle_command_ack`           | [px4_msgs::msg::VehicleCommandAck](../msg_docs/VehicleCommandAck.md)                   |
| `/fmu/out/vehicle_global_position`       | [px4_msgs::msg::VehicleGlobalPosition](../msg_docs/VehicleGlobalPosition.md)           | 50.0       |
| `/fmu/out/vehicle_gps_position`          | [px4_msgs::msg::SensorGps](../msg_docs/SensorGps.md)                                   | 50.0       |
| `/fmu/out/vehicle_local_position`        | [px4_msgs::msg::VehicleLocalPosition](../msg_docs/VehicleLocalPosition.md)             | 50.0       |
| `/fmu/out/vehicle_odometry`              | [px4_msgs::msg::VehicleOdometry](../msg_docs/VehicleOdometry.md)                       | 100.0      |
| `/fmu/out/vehicle_status`                | [px4_msgs::msg::VehicleStatus](../msg_docs/VehicleStatus.md)                           | 5.0        |
| `/fmu/out/airspeed_validated`            | [px4_msgs::msg::AirspeedValidated](../msg_docs/AirspeedValidated.md)                   | 50.0       |
| `/fmu/out/vtol_vehicle_status`           | [px4_msgs::msg::VtolVehicleStatus](../msg_docs/VtolVehicleStatus.md)                   |
| `/fmu/out/home_position`                 | [px4_msgs::msg::HomePosition](../msg_docs/HomePosition.md)                             | 5.0        |
| `/fmu/out/wind`                          | [px4_msgs::msg::Wind](../msg_docs/Wind.md)                                             | 1.0        |
| `/fmu/out/gimbal_device_attitude_status` | [px4_msgs::msg::GimbalDeviceAttitudeStatus](../msg_docs/GimbalDeviceAttitudeStatus.md) | 20.0       |

## Subscriptions

| Topic                                      | Type                                                                                               |
| ------------------------------------------ | -------------------------------------------------------------------------------------------------- |
| /fmu/in/register_ext_component_request     | [px4_msgs::msg::RegisterExtComponentRequest](../msg_docs/RegisterExtComponentRequest.md)           |
| /fmu/in/unregister_ext_component           | [px4_msgs::msg::UnregisterExtComponent](../msg_docs/UnregisterExtComponent.md)                     |
| /fmu/in/config_overrides_request           | [px4_msgs::msg::ConfigOverrides](../msg_docs/ConfigOverrides.md)                                   |
| /fmu/in/arming_check_reply                 | [px4_msgs::msg::ArmingCheckReply](../msg_docs/ArmingCheckReply.md)                                 |
| /fmu/in/message_format_request             | [px4_msgs::msg::MessageFormatRequest](../msg_docs/MessageFormatRequest.md)                         |
| /fmu/in/mode_completed                     | [px4_msgs::msg::ModeCompleted](../msg_docs/ModeCompleted.md)                                       |
| /fmu/in/config_control_setpoints           | [px4_msgs::msg::VehicleControlMode](../msg_docs/VehicleControlMode.md)                             |
| /fmu/in/distance_sensor                    | [px4_msgs::msg::DistanceSensor](../msg_docs/DistanceSensor.md)                                     |
| /fmu/in/manual_control_input               | [px4_msgs::msg::ManualControlSetpoint](../msg_docs/ManualControlSetpoint.md)                       |
| /fmu/in/offboard_control_mode              | [px4_msgs::msg::OffboardControlMode](../msg_docs/OffboardControlMode.md)                           |
| /fmu/in/onboard_computer_status            | [px4_msgs::msg::OnboardComputerStatus](../msg_docs/OnboardComputerStatus.md)                       |
| /fmu/in/obstacle_distance                  | [px4_msgs::msg::ObstacleDistance](../msg_docs/ObstacleDistance.md)                                 |
| /fmu/in/sensor_optical_flow                | [px4_msgs::msg::SensorOpticalFlow](../msg_docs/SensorOpticalFlow.md)                               |
| /fmu/in/goto_setpoint                      | [px4_msgs::msg::GotoSetpoint](../msg_docs/GotoSetpoint.md)                                         |
| /fmu/in/telemetry_status                   | [px4_msgs::msg::TelemetryStatus](../msg_docs/TelemetryStatus.md)                                   |
| /fmu/in/trajectory_setpoint                | [px4_msgs::msg::TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md)                             |
| /fmu/in/vehicle_attitude_setpoint          | [px4_msgs::msg::VehicleAttitudeSetpoint](../msg_docs/VehicleAttitudeSetpoint.md)                   |
| /fmu/in/vehicle_mocap_odometry             | [px4_msgs::msg::VehicleOdometry](../msg_docs/VehicleOdometry.md)                                   |
| /fmu/in/vehicle_rates_setpoint             | [px4_msgs::msg::VehicleRatesSetpoint](../msg_docs/VehicleRatesSetpoint.md)                         |
| /fmu/in/vehicle_visual_odometry            | [px4_msgs::msg::VehicleOdometry](../msg_docs/VehicleOdometry.md)                                   |
| /fmu/in/vehicle_command                    | [px4_msgs::msg::VehicleCommand](../msg_docs/VehicleCommand.md)                                     |
| /fmu/in/vehicle_command_mode_executor      | [px4_msgs::msg::VehicleCommand](../msg_docs/VehicleCommand.md)                                     |
| /fmu/in/vehicle_thrust_setpoint            | [px4_msgs::msg::VehicleThrustSetpoint](../msg_docs/VehicleThrustSetpoint.md)                       |
| /fmu/in/vehicle_torque_setpoint            | [px4_msgs::msg::VehicleTorqueSetpoint](../msg_docs/VehicleTorqueSetpoint.md)                       |
| /fmu/in/actuator_motors                    | [px4_msgs::msg::ActuatorMotors](../msg_docs/ActuatorMotors.md)                                     |
| /fmu/in/actuator_servos                    | [px4_msgs::msg::ActuatorServos](../msg_docs/ActuatorServos.md)                                     |
| /fmu/in/aux_global_position                | [px4_msgs::msg::VehicleGlobalPosition](../msg_docs/VehicleGlobalPosition.md)                       |
| /fmu/in/fixed_wing_longitudinal_setpoint   | [px4_msgs::msg::FixedWingLongitudinalSetpoint](../msg_docs/FixedWingLongitudinalSetpoint.md)       |
| /fmu/in/fixed_wing_lateral_setpoint        | [px4_msgs::msg::FixedWingLateralSetpoint](../msg_docs/FixedWingLateralSetpoint.md)                 |
| /fmu/in/longitudinal_control_configuration | [px4_msgs::msg::LongitudinalControlConfiguration](../msg_docs/LongitudinalControlConfiguration.md) |
| /fmu/in/lateral_control_configuration      | [px4_msgs::msg::LateralControlConfiguration](../msg_docs/LateralControlConfiguration.md)           |
| /fmu/in/rover_position_setpoint            | [px4_msgs::msg::RoverPositionSetpoint](../msg_docs/RoverPositionSetpoint.md)                       |
| /fmu/in/rover_speed_setpoint               | [px4_msgs::msg::RoverSpeedSetpoint](../msg_docs/RoverSpeedSetpoint.md)                             |
| /fmu/in/rover_attitude_setpoint            | [px4_msgs::msg::RoverAttitudeSetpoint](../msg_docs/RoverAttitudeSetpoint.md)                       |
| /fmu/in/rover_rate_setpoint                | [px4_msgs::msg::RoverRateSetpoint](../msg_docs/RoverRateSetpoint.md)                               |
| /fmu/in/rover_throttle_setpoint            | [px4_msgs::msg::RoverThrottleSetpoint](../msg_docs/RoverThrottleSetpoint.md)                       |
| /fmu/in/rover_steering_setpoint            | [px4_msgs::msg::RoverSteeringSetpoint](../msg_docs/RoverSteeringSetpoint.md)                       |
| /fmu/in/landing_gear                       | [px4_msgs::msg::LandingGear](../msg_docs/LandingGear.md)                                           |

## Subscriptions Multi

None

## Not Exported

These messages are not listed in the yaml file.
They are not build into the module, and hence are neither published or subscribed.

::: details See messages

- [EstimatorAidSource1d](../msg_docs/EstimatorAidSource1d.md)
- [PpsCapture](../msg_docs/PpsCapture.md)
- [VehicleStatusV0](../msg_docs/VehicleStatusV0.md)
- [UlogStreamAck](../msg_docs/UlogStreamAck.md)
- [GpioOut](../msg_docs/GpioOut.md)
- [ControlAllocatorStatus](../msg_docs/ControlAllocatorStatus.md)
- [SensorTemp](../msg_docs/SensorTemp.md)
- [GpsInjectData](../msg_docs/GpsInjectData.md)
- [Airspeed](../msg_docs/Airspeed.md)
- [SensorBaro](../msg_docs/SensorBaro.md)
- [FollowTarget](../msg_docs/FollowTarget.md)
- [TuneControl](../msg_docs/TuneControl.md)
- [Event](../msg_docs/Event.md)
- [VehicleAcceleration](../msg_docs/VehicleAcceleration.md)
- [SensorGyroFifo](../msg_docs/SensorGyroFifo.md)
- [MagWorkerData](../msg_docs/MagWorkerData.md)
- [PositionControllerLandingStatus](../msg_docs/PositionControllerLandingStatus.md)
- [DebugArray](../msg_docs/DebugArray.md)
- [InternalCombustionEngineControl](../msg_docs/InternalCombustionEngineControl.md)
- [ParameterResetRequest](../msg_docs/ParameterResetRequest.md)
- [DatamanRequest](../msg_docs/DatamanRequest.md)
- [GpioRequest](../msg_docs/GpioRequest.md)
- [InternalCombustionEngineStatus](../msg_docs/InternalCombustionEngineStatus.md)
- [RegisterExtComponentRequestV0](../msg_docs/RegisterExtComponentRequestV0.md)
- [NormalizedUnsignedSetpoint](../msg_docs/NormalizedUnsignedSetpoint.md)
- [NavigatorStatus](../msg_docs/NavigatorStatus.md)
- [SensorAccelFifo](../msg_docs/SensorAccelFifo.md)
- [ActuatorControlsStatus](../msg_docs/ActuatorControlsStatus.md)
- [MissionResult](../msg_docs/MissionResult.md)
- [RtlStatus](../msg_docs/RtlStatus.md)
- [CameraTrigger](../msg_docs/CameraTrigger.md)
- [FollowTargetEstimator](../msg_docs/FollowTargetEstimator.md)
- [ActuatorTest](../msg_docs/ActuatorTest.md)
- [EstimatorAidSource3d](../msg_docs/EstimatorAidSource3d.md)
- [VehicleRoi](../msg_docs/VehicleRoi.md)
- [MagnetometerBiasEstimate](../msg_docs/MagnetometerBiasEstimate.md)
- [UlogStream](../msg_docs/UlogStream.md)
- [VehicleLocalPositionSetpoint](../msg_docs/VehicleLocalPositionSetpoint.md)
- [RoverSpeedStatus](../msg_docs/RoverSpeedStatus.md)
- [WheelEncoders](../msg_docs/WheelEncoders.md)
- [PwmInput](../msg_docs/PwmInput.md)
- [VehicleOpticalFlow](../msg_docs/VehicleOpticalFlow.md)
- [Gripper](../msg_docs/Gripper.md)
- [EstimatorEventFlags](../msg_docs/EstimatorEventFlags.md)
- [FlightPhaseEstimation](../msg_docs/FlightPhaseEstimation.md)
- [OrbTest](../msg_docs/OrbTest.md)
- [PowerMonitor](../msg_docs/PowerMonitor.md)
- [ArmingCheckReplyV0](../msg_docs/ArmingCheckReplyV0.md)
- [EstimatorBias](../msg_docs/EstimatorBias.md)
- [ArmingCheckRequestV0](../msg_docs/ArmingCheckRequestV0.md)
- [EstimatorStatus](../msg_docs/EstimatorStatus.md)
- [OpenDroneIdSystem](../msg_docs/OpenDroneIdSystem.md)
- [VehicleAirData](../msg_docs/VehicleAirData.md)
- [DifferentialPressure](../msg_docs/DifferentialPressure.md)
- [Px4ioStatus](../msg_docs/Px4ioStatus.md)
- [Rpm](../msg_docs/Rpm.md)
- [EstimatorGpsStatus](../msg_docs/EstimatorGpsStatus.md)
- [DistanceSensorModeChangeRequest](../msg_docs/DistanceSensorModeChangeRequest.md)
- [GimbalDeviceInformation](../msg_docs/GimbalDeviceInformation.md)
- [NavigatorMissionItem](../msg_docs/NavigatorMissionItem.md)
- [ActuatorArmed](../msg_docs/ActuatorArmed.md)
- [GimbalManagerSetAttitude](../msg_docs/GimbalManagerSetAttitude.md)
- [ActionRequest](../msg_docs/ActionRequest.md)
- [ActuatorOutputs](../msg_docs/ActuatorOutputs.md)
- [InputRc](../msg_docs/InputRc.md)
- [DatamanResponse](../msg_docs/DatamanResponse.md)
- [LoggerStatus](../msg_docs/LoggerStatus.md)
- [SensorHygrometer](../msg_docs/SensorHygrometer.md)
- [VehicleAngularAccelerationSetpoint](../msg_docs/VehicleAngularAccelerationSetpoint.md)
- [FailureDetectorStatus](../msg_docs/FailureDetectorStatus.md)
- [DeviceInformation](../msg_docs/DeviceInformation.md)
- [AirspeedWind](../msg_docs/AirspeedWind.md)
- [SensorsStatusImu](../msg_docs/SensorsStatusImu.md)
- [ParameterSetValueRequest](../msg_docs/ParameterSetValueRequest.md)
- [Vtx](../msg_docs/Vtx.md)
- [HeaterStatus](../msg_docs/HeaterStatus.md)
- [OrbitStatus](../msg_docs/OrbitStatus.md)
- [RoverAttitudeStatus](../msg_docs/RoverAttitudeStatus.md)
- [RateCtrlStatus](../msg_docs/RateCtrlStatus.md)
- [LandingTargetInnovations](../msg_docs/LandingTargetInnovations.md)
- [OrbTestMedium](../msg_docs/OrbTestMedium.md)
- [FuelTankStatus](../msg_docs/FuelTankStatus.md)
- [GpsDump](../msg_docs/GpsDump.md)
- [FixedWingLateralStatus](../msg_docs/FixedWingLateralStatus.md)
- [EstimatorAidSource2d](../msg_docs/EstimatorAidSource2d.md)
- [TiltrotorExtraControls](../msg_docs/TiltrotorExtraControls.md)
- [LandingTargetPose](../msg_docs/LandingTargetPose.md)
- [PositionSetpoint](../msg_docs/PositionSetpoint.md)
- [VehicleImuStatus](../msg_docs/VehicleImuStatus.md)
- [SensorGnssRelative](../msg_docs/SensorGnssRelative.md)
- [GimbalManagerInformation](../msg_docs/GimbalManagerInformation.md)
- [MavlinkLog](../msg_docs/MavlinkLog.md)
- [PowerButtonState](../msg_docs/PowerButtonState.md)
- [VehicleImu](../msg_docs/VehicleImu.md)
- [Ping](../msg_docs/Ping.md)
- [SensorsStatus](../msg_docs/SensorsStatus.md)
- [TaskStackInfo](../msg_docs/TaskStackInfo.md)
- [GimbalManagerStatus](../msg_docs/GimbalManagerStatus.md)
- [OrbTestLarge](../msg_docs/OrbTestLarge.md)
- [GimbalManagerSetManualControl](../msg_docs/GimbalManagerSetManualControl.md)
- [VehicleMagnetometer](../msg_docs/VehicleMagnetometer.md)
- [VehicleAttitudeSetpointV0](../msg_docs/VehicleAttitudeSetpointV0.md)
- [EstimatorInnovations](../msg_docs/EstimatorInnovations.md)
- [CellularStatus](../msg_docs/CellularStatus.md)
- [EstimatorSelectorStatus](../msg_docs/EstimatorSelectorStatus.md)
- [LaunchDetectionStatus](../msg_docs/LaunchDetectionStatus.md)
- [UavcanParameterValue](../msg_docs/UavcanParameterValue.md)
- [LandingGearWheel](../msg_docs/LandingGearWheel.md)
- [VelocityLimits](../msg_docs/VelocityLimits.md)
- [ManualControlSwitches](../msg_docs/ManualControlSwitches.md)
- [DebugValue](../msg_docs/DebugValue.md)
- [SensorMag](../msg_docs/SensorMag.md)
- [VehicleConstraints](../msg_docs/VehicleConstraints.md)
- [EscStatus](../msg_docs/EscStatus.md)
- [AirspeedValidatedV0](../msg_docs/AirspeedValidatedV0.md)
- [ParameterSetValueResponse](../msg_docs/ParameterSetValueResponse.md)
- [IridiumsbdStatus](../msg_docs/IridiumsbdStatus.md)
- [DronecanNodeStatus](../msg_docs/DronecanNodeStatus.md)
- [QshellReq](../msg_docs/QshellReq.md)
- [EscReport](../msg_docs/EscReport.md)
- [HomePositionV0](../msg_docs/HomePositionV0.md)
- [SensorAccel](../msg_docs/SensorAccel.md)
- [RaptorStatus](../msg_docs/RaptorStatus.md)
- [GeneratorStatus](../msg_docs/GeneratorStatus.md)
- [GimbalDeviceSetAttitude](../msg_docs/GimbalDeviceSetAttitude.md)
- [TakeoffStatus](../msg_docs/TakeoffStatus.md)
- [UavcanParameterRequest](../msg_docs/UavcanParameterRequest.md)
- [GeofenceStatus](../msg_docs/GeofenceStatus.md)
- [PurePursuitStatus](../msg_docs/PurePursuitStatus.md)
- [SensorSelection](../msg_docs/SensorSelection.md)
- [MountOrientation](../msg_docs/MountOrientation.md)
- [EventV0](../msg_docs/EventV0.md)
- [QshellRetval](../msg_docs/QshellRetval.md)
- [RcParameterMap](../msg_docs/RcParameterMap.md)
- [LogMessage](../msg_docs/LogMessage.md)
- [SensorAirflow](../msg_docs/SensorAirflow.md)
- [GpioConfig](../msg_docs/GpioConfig.md)
- [AutotuneAttitudeControlStatus](../msg_docs/AutotuneAttitudeControlStatus.md)
- [SensorGyroFft](../msg_docs/SensorGyroFft.md)
- [Ekf2Timestamps](../msg_docs/Ekf2Timestamps.md)
- [OpenDroneIdSelfId](../msg_docs/OpenDroneIdSelfId.md)
- [PositionControllerStatus](../msg_docs/PositionControllerStatus.md)
- [FigureEightStatus](../msg_docs/FigureEightStatus.md)
- [Cpuload](../msg_docs/Cpuload.md)
- [SensorPreflightMag](../msg_docs/SensorPreflightMag.md)
- [TrajectorySetpoint6dof](../msg_docs/TrajectorySetpoint6dof.md)
- [LedControl](../msg_docs/LedControl.md)
- [VehicleLocalPositionV0](../msg_docs/VehicleLocalPositionV0.md)
- [SensorUwb](../msg_docs/SensorUwb.md)
- [FixedWingRunwayControl](../msg_docs/FixedWingRunwayControl.md)
- [CameraCapture](../msg_docs/CameraCapture.md)
- [BatteryStatusV0](../msg_docs/BatteryStatusV0.md)
- [RcChannels](../msg_docs/RcChannels.md)
- [SensorGyro](../msg_docs/SensorGyro.md)
- [OpenDroneIdOperatorId](../msg_docs/OpenDroneIdOperatorId.md)
- [MavlinkTunnel](../msg_docs/MavlinkTunnel.md)
- [CameraStatus](../msg_docs/CameraStatus.md)
- [FollowTargetStatus](../msg_docs/FollowTargetStatus.md)
- [ActuatorServosTrim](../msg_docs/ActuatorServosTrim.md)
- [ParameterSetUsedRequest](../msg_docs/ParameterSetUsedRequest.md)
- [EstimatorSensorBias](../msg_docs/EstimatorSensorBias.md)
- [AdcReport](../msg_docs/AdcReport.md)
- [RadioStatus](../msg_docs/RadioStatus.md)
- [SatelliteInfo](../msg_docs/SatelliteInfo.md)
- [GainCompression](../msg_docs/GainCompression.md)
- [SensorCorrection](../msg_docs/SensorCorrection.md)
- [SensorGnssStatus](../msg_docs/SensorGnssStatus.md)
- [RtlTimeEstimate](../msg_docs/RtlTimeEstimate.md)
- [VehicleAngularVelocity](../msg_docs/VehicleAngularVelocity.md)
- [VehicleOpticalFlowVel](../msg_docs/VehicleOpticalFlowVel.md)
- [TecsStatus](../msg_docs/TecsStatus.md)
- [RaptorInput](../msg_docs/RaptorInput.md)
- [ConfigOverridesV0](../msg_docs/ConfigOverridesV0.md)
- [ButtonEvent](../msg_docs/ButtonEvent.md)
- [SystemPower](../msg_docs/SystemPower.md)
- [EstimatorBias3d](../msg_docs/EstimatorBias3d.md)
- [RegisterExtComponentReplyV0](../msg_docs/RegisterExtComponentReplyV0.md)
- [Mission](../msg_docs/Mission.md)
- [RoverRateStatus](../msg_docs/RoverRateStatus.md)
- [ParameterUpdate](../msg_docs/ParameterUpdate.md)
- [GpioIn](../msg_docs/GpioIn.md)
- [OpenDroneIdArmStatus](../msg_docs/OpenDroneIdArmStatus.md)
- [EstimatorStates](../msg_docs/EstimatorStates.md)
- [CanInterfaceStatus](../msg_docs/CanInterfaceStatus.md)
- [DebugVect](../msg_docs/DebugVect.md)
- [GimbalControls](../msg_docs/GimbalControls.md)
- [BatteryInfo](../msg_docs/BatteryInfo.md)
- [HealthReport](../msg_docs/HealthReport.md)
- [HoverThrustEstimate](../msg_docs/HoverThrustEstimate.md)
- [VehicleStatusV1](../msg_docs/VehicleStatusV1.md)
- [GeofenceResult](../msg_docs/GeofenceResult.md)
- [DebugKeyValue](../msg_docs/DebugKeyValue.md)
- [NeuralControl](../msg_docs/NeuralControl.md)
- [IrlockReport](../msg_docs/IrlockReport.md)
- [YawEstimatorStatus](../msg_docs/YawEstimatorStatus.md)
- [FixedWingLateralGuidanceStatus](../msg_docs/FixedWingLateralGuidanceStatus.md)
  :::
