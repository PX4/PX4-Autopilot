# dds_topics.yaml — PX4 Topics Exposed to ROS 2

:::info
This document is [auto-generated](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/msg/generate_msg_docs.py) from the source code.
:::

The [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml) file specifies which uORB message definitions are compiled into the [uxrce_dds_client](../modules/modules_system.md#uxrce-dds-client) and/or [zenoh](../modules/modules_driver.md#zenoh) module when [PX4 is built](../middleware/uxrce_dds.md#code-generation), and hence which topics are available for ROS 2 applications to subscribe or publish (by default).

This document shows a markdown-rendered version of [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml), listing the publications, subscriptions, and so on.

## Publications

| Topic                                   | 类型                                                                                                                                                                        | Rate Limit           |
| --------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------- |
| `/fmu/out/register_ext_component_reply` | [px4_msgs::msg::RegisterExtComponentReply](../msg_docs/RegisterExtComponentReply.md) |                      |
| `/fmu/out/arming_check_request`         | [px4_msgs::msg::ArmingCheckRequest](../msg_docs/ArmingCheckRequest.md)               | 5.0  |
| `/fmu/out/mode_completed`               | [px4_msgs::msg::ModeCompleted](../msg_docs/ModeCompleted.md)                         | 50.0 |
| `/fmu/out/battery_status`               | [px4_msgs::msg::BatteryStatus](../msg_docs/BatteryStatus.md)                         | 1.0  |
| `/fmu/out/collision_constraints`        | [px4_msgs::msg::CollisionConstraints](../msg_docs/CollisionConstraints.md)           | 50.0 |
| `/fmu/out/estimator_status_flags`       | [px4_msgs::msg::EstimatorStatusFlags](../msg_docs/EstimatorStatusFlags.md)           | 5.0  |
| `/fmu/out/failsafe_flags`               | [px4_msgs::msg::FailsafeFlags](../msg_docs/FailsafeFlags.md)                         | 5.0  |
| `/fmu/out/manual_control_setpoint`      | [px4_msgs::msg::ManualControlSetpoint](../msg_docs/ManualControlSetpoint.md)         | 25.0 |
| `/fmu/out/message_format_response`      | [px4_msgs::msg::MessageFormatResponse](../msg_docs/MessageFormatResponse.md)         |                      |
| `/fmu/out/position_setpoint_triplet`    | [px4_msgs::msg::PositionSetpointTriplet](../msg_docs/PositionSetpointTriplet.md)     | 5.0  |
| `/fmu/out/sensor_combined`              | [px4_msgs::msg::SensorCombined](../msg_docs/SensorCombined.md)                       |                      |
| `/fmu/out/timesync_status`              | [px4_msgs::msg::TimesyncStatus](../msg_docs/TimesyncStatus.md)                       | 10.0 |
| `/fmu/out/vehicle_land_detected`        | [px4_msgs::msg::VehicleLandDetected](../msg_docs/VehicleLandDetected.md)             | 5.0  |
| `/fmu/out/vehicle_attitude`             | [px4_msgs::msg::VehicleAttitude](../msg_docs/VehicleAttitude.md)                     |                      |
| `/fmu/out/vehicle_control_mode`         | [px4_msgs::msg::VehicleControlMode](../msg_docs/VehicleControlMode.md)               | 50.0 |
| `/fmu/out/vehicle_command_ack`          | [px4_msgs::msg::VehicleCommandAck](../msg_docs/VehicleCommandAck.md)                 |                      |
| `/fmu/out/vehicle_global_position`      | [px4_msgs::msg::VehicleGlobalPosition](../msg_docs/VehicleGlobalPosition.md)         | 50.0 |
| `/fmu/out/vehicle_gps_position`         | [px4_msgs::msg::SensorGps](../msg_docs/SensorGps.md)                                 | 50.0 |
| `/fmu/out/vehicle_local_position`       | [px4_msgs::msg::VehicleLocalPosition](../msg_docs/VehicleLocalPosition.md)           | 50.0 |
| `/fmu/out/vehicle_odometry`             | [px4_msgs::msg::VehicleOdometry](../msg_docs/VehicleOdometry.md)                     |                      |
| `/fmu/out/vehicle_status`               | [px4_msgs::msg::VehicleStatus](../msg_docs/VehicleStatus.md)                         | 5.0  |
| `/fmu/out/airspeed_validated`           | [px4_msgs::msg::AirspeedValidated](../msg_docs/AirspeedValidated.md)                 | 50.0 |
| `/fmu/out/vtol_vehicle_status`          | [px4_msgs::msg::VtolVehicleStatus](../msg_docs/VtolVehicleStatus.md)                 |                      |
| `/fmu/out/home_position`                | [px4_msgs::msg::HomePosition](../msg_docs/HomePosition.md)                           | 5.0  |

## Subscriptions

| Topic                                                                                                   | 类型                                                                                                                                                                                      |
| ------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| /fmu/in/register_ext_component_request   | [px4_msgs::msg::RegisterExtComponentRequest](../msg_docs/RegisterExtComponentRequest.md)           |
| /fmu/in/unregister_ext_component                              | [px4_msgs::msg::UnregisterExtComponent](../msg_docs/UnregisterExtComponent.md)                     |
| /fmu/in/config_overrides_request                              | [px4_msgs::msg::ConfigOverrides](../msg_docs/ConfigOverrides.md)                                   |
| /fmu/in/arming_check_reply                                    | [px4_msgs::msg::ArmingCheckReply](../msg_docs/ArmingCheckReply.md)                                 |
| /fmu/in/message_format_request                                | [px4_msgs::msg::MessageFormatRequest](../msg_docs/MessageFormatRequest.md)                         |
| /fmu/in/mode_completed                                                             | [px4_msgs::msg::ModeCompleted](../msg_docs/ModeCompleted.md)                                       |
| /fmu/in/config_control_setpoints                              | [px4_msgs::msg::VehicleControlMode](../msg_docs/VehicleControlMode.md)                             |
| /fmu/in/distance_sensor                                                            | [px4_msgs::msg::DistanceSensor](../msg_docs/DistanceSensor.md)                                     |
| /fmu/in/manual_control_input                                  | [px4_msgs::msg::ManualControlSetpoint](../msg_docs/ManualControlSetpoint.md)                       |
| /fmu/in/offboard_control_mode                                 | [px4_msgs::msg::OffboardControlMode](../msg_docs/OffboardControlMode.md)                           |
| /fmu/in/onboard_computer_status                               | [px4_msgs::msg::OnboardComputerStatus](../msg_docs/OnboardComputerStatus.md)                       |
| /fmu/in/obstacle_distance                                                          | [px4_msgs::msg::ObstacleDistance](../msg_docs/ObstacleDistance.md)                                 |
| /fmu/in/sensor_optical_flow                                   | [px4_msgs::msg::SensorOpticalFlow](../msg_docs/SensorOpticalFlow.md)                               |
| /fmu/in/goto_setpoint                                                              | [px4_msgs::msg::GotoSetpoint](../msg_docs/GotoSetpoint.md)                                         |
| /fmu/in/telemetry_status                                                           | [px4_msgs::msg::TelemetryStatus](../msg_docs/TelemetryStatus.md)                                   |
| /fmu/in/trajectory_setpoint                                                        | [px4_msgs::msg::TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md)                             |
| /fmu/in/vehicle_attitude_setpoint                             | [px4_msgs::msg::VehicleAttitudeSetpoint](../msg_docs/VehicleAttitudeSetpoint.md)                   |
| /fmu/in/vehicle_mocap_odometry                                | [px4_msgs::msg::VehicleOdometry](../msg_docs/VehicleOdometry.md)                                   |
| /fmu/in/vehicle_rates_setpoint                                | [px4_msgs::msg::VehicleRatesSetpoint](../msg_docs/VehicleRatesSetpoint.md)                         |
| /fmu/in/vehicle_visual_odometry                               | [px4_msgs::msg::VehicleOdometry](../msg_docs/VehicleOdometry.md)                                   |
| /fmu/in/vehicle_command                                                            | [px4_msgs::msg::VehicleCommand](../msg_docs/VehicleCommand.md)                                     |
| /fmu/in/vehicle_command_mode_executor    | [px4_msgs::msg::VehicleCommand](../msg_docs/VehicleCommand.md)                                     |
| /fmu/in/vehicle_thrust_setpoint                               | [px4_msgs::msg::VehicleThrustSetpoint](../msg_docs/VehicleThrustSetpoint.md)                       |
| /fmu/in/vehicle_torque_setpoint                               | [px4_msgs::msg::VehicleTorqueSetpoint](../msg_docs/VehicleTorqueSetpoint.md)                       |
| /fmu/in/actuator_motors                                                            | [px4_msgs::msg::ActuatorMotors](../msg_docs/ActuatorMotors.md)                                     |
| /fmu/in/actuator_servos                                                            | [px4_msgs::msg::ActuatorServos](../msg_docs/ActuatorServos.md)                                     |
| /fmu/in/aux_global_position                                   | [px4_msgs::msg::VehicleGlobalPosition](../msg_docs/VehicleGlobalPosition.md)                       |
| /fmu/in/fixed_wing_longitudinal_setpoint | [px4_msgs::msg::FixedWingLongitudinalSetpoint](../msg_docs/FixedWingLongitudinalSetpoint.md)       |
| /fmu/in/fixed_wing_lateral_setpoint      | [px4_msgs::msg::FixedWingLateralSetpoint](../msg_docs/FixedWingLateralSetpoint.md)                 |
| /fmu/in/longitudinal_control_configuration                    | [px4_msgs::msg::LongitudinalControlConfiguration](../msg_docs/LongitudinalControlConfiguration.md) |
| /fmu/in/lateral_control_configuration                         | [px4_msgs::msg::LateralControlConfiguration](../msg_docs/LateralControlConfiguration.md)           |

## Subscriptions Multi

None

## Not Exported

These messages are not listed in the yaml file.
They are not build into the module, and hence are neither published or subscribed.

:::details
See messages

- [SensorCorrection](../msg_docs/SensorCorrection.md)
- [ActuatorOutputs](../msg_docs/ActuatorOutputs.md)
- [FixedWingRunwayControl](../msg_docs/FixedWingRunwayControl.md)
- [EstimatorInnovations](../msg_docs/EstimatorInnovations.md)
- [FlightPhaseEstimation](../msg_docs/FlightPhaseEstimation.md)
- [PurePursuitStatus](../msg_docs/PurePursuitStatus.md)
- [Px4ioStatus](../msg_docs/Px4ioStatus.md)
- [SatelliteInfo](../msg_docs/SatelliteInfo.md)
- [GeofenceResult](../msg_docs/GeofenceResult.md)
- [GimbalManagerStatus](../msg_docs/GimbalManagerStatus.md)
- [ManualControlSwitches](../msg_docs/ManualControlSwitches.md)
- [OpenDroneIdSelfId](../msg_docs/OpenDroneIdSelfId.md)
- [OpenDroneIdSystem](../msg_docs/OpenDroneIdSystem.md)
- [EventV0](../msg_docs/EventV0.md)
- [QshellRetval](../msg_docs/QshellRetval.md)
- [RoverThrottleSetpoint](../msg_docs/RoverThrottleSetpoint.md)
- [AirspeedValidatedV0](../msg_docs/AirspeedValidatedV0.md)
- [RcChannels](../msg_docs/RcChannels.md)
- [SensorAccel](../msg_docs/SensorAccel.md)
- [GimbalDeviceAttitudeStatus](../msg_docs/GimbalDeviceAttitudeStatus.md)
- [EscStatus](../msg_docs/EscStatus.md)
- [RoverAttitudeSetpoint](../msg_docs/RoverAttitudeSetpoint.md)
- [RateCtrlStatus](../msg_docs/RateCtrlStatus.md)
- [AirspeedWind](../msg_docs/AirspeedWind.md)
- [InputRc](../msg_docs/InputRc.md)
- [GpioIn](../msg_docs/GpioIn.md)
- [LaunchDetectionStatus](../msg_docs/LaunchDetectionStatus.md)
- [VehicleImu](../msg_docs/VehicleImu.md)
- [Event](../msg_docs/Event.md)
- [SensorUwb](../msg_docs/SensorUwb.md)
- [ActuatorServosTrim](../msg_docs/ActuatorServosTrim.md)
- [DatamanResponse](../msg_docs/DatamanResponse.md)
- [OrbTest](../msg_docs/OrbTest.md)
- [VehicleLocalPositionSetpoint](../msg_docs/VehicleLocalPositionSetpoint.md)
- [VehicleAngularVelocity](../msg_docs/VehicleAngularVelocity.md)
- [FollowTargetStatus](../msg_docs/FollowTargetStatus.md)
- [NormalizedUnsignedSetpoint](../msg_docs/NormalizedUnsignedSetpoint.md)
- [YawEstimatorStatus](../msg_docs/YawEstimatorStatus.md)
- [TakeoffStatus](../msg_docs/TakeoffStatus.md)
- [UlogStreamAck](../msg_docs/UlogStreamAck.md)
- [OrbTestLarge](../msg_docs/OrbTestLarge.md)
- [RoverSteeringSetpoint](../msg_docs/RoverSteeringSetpoint.md)
- [CameraCapture](../msg_docs/CameraCapture.md)
- [VehicleRoi](../msg_docs/VehicleRoi.md)
- [ActuatorArmed](../msg_docs/ActuatorArmed.md)
- [FixedWingLateralGuidanceStatus](../msg_docs/FixedWingLateralGuidanceStatus.md)
- [ParameterSetValueResponse](../msg_docs/ParameterSetValueResponse.md)
- [GeofenceStatus](../msg_docs/GeofenceStatus.md)
- [VehicleAngularAccelerationSetpoint](../msg_docs/VehicleAngularAccelerationSetpoint.md)
- [SensorGnssRelative](../msg_docs/SensorGnssRelative.md)
- [PowerMonitor](../msg_docs/PowerMonitor.md)
- [RoverVelocityStatus](../msg_docs/RoverVelocityStatus.md)
- [ParameterResetRequest](../msg_docs/ParameterResetRequest.md)
- [RoverAttitudeStatus](../msg_docs/RoverAttitudeStatus.md)
- [TecsStatus](../msg_docs/TecsStatus.md)
- [EstimatorSelectorStatus](../msg_docs/EstimatorSelectorStatus.md)
- [CanInterfaceStatus](../msg_docs/CanInterfaceStatus.md)
- [Ping](../msg_docs/Ping.md)
- [LedControl](../msg_docs/LedControl.md)
- [Wind](../msg_docs/Wind.md)
- [VehicleStatusV0](../msg_docs/VehicleStatusV0.md)
- [ActuatorTest](../msg_docs/ActuatorTest.md)
- [IridiumsbdStatus](../msg_docs/IridiumsbdStatus.md)
- [FailureDetectorStatus](../msg_docs/FailureDetectorStatus.md)
- [GimbalManagerSetAttitude](../msg_docs/GimbalManagerSetAttitude.md)
- [Gripper](../msg_docs/Gripper.md)
- [SensorMag](../msg_docs/SensorMag.md)
- [DebugValue](../msg_docs/DebugValue.md)
- [SensorPreflightMag](../msg_docs/SensorPreflightMag.md)
- [RcParameterMap](../msg_docs/RcParameterMap.md)
- [LandingGear](../msg_docs/LandingGear.md)
- [GimbalDeviceInformation](../msg_docs/GimbalDeviceInformation.md)
- [VehicleOpticalFlow](../msg_docs/VehicleOpticalFlow.md)
- [UlogStream](../msg_docs/UlogStream.md)
- [GimbalControls](../msg_docs/GimbalControls.md)
- [RoverRateSetpoint](../msg_docs/RoverRateSetpoint.md)
- [LogMessage](../msg_docs/LogMessage.md)
- [RoverVelocitySetpoint](../msg_docs/RoverVelocitySetpoint.md)
- [GpioOut](../msg_docs/GpioOut.md)
- [TaskStackInfo](../msg_docs/TaskStackInfo.md)
- [VelocityLimits](../msg_docs/VelocityLimits.md)
- [MagWorkerData](../msg_docs/MagWorkerData.md)
- [ParameterUpdate](../msg_docs/ParameterUpdate.md)
- [TrajectorySetpoint6dof](../msg_docs/TrajectorySetpoint6dof.md)
- [SensorBaro](../msg_docs/SensorBaro.md)
- [VehicleImuStatus](../msg_docs/VehicleImuStatus.md)
- [InternalCombustionEngineStatus](../msg_docs/InternalCombustionEngineStatus.md)
- [VehicleOpticalFlowVel](../msg_docs/VehicleOpticalFlowVel.md)
- [GimbalManagerSetManualControl](../msg_docs/GimbalManagerSetManualControl.md)
- [Rpm](../msg_docs/Rpm.md)
- [MagnetometerBiasEstimate](../msg_docs/MagnetometerBiasEstimate.md)
- [MountOrientation](../msg_docs/MountOrientation.md)
- [ActionRequest](../msg_docs/ActionRequest.md)
- [OpenDroneIdArmStatus](../msg_docs/OpenDroneIdArmStatus.md)
- [SensorAccelFifo](../msg_docs/SensorAccelFifo.md)
- [LoggerStatus](../msg_docs/LoggerStatus.md)
- [GeneratorStatus](../msg_docs/GeneratorStatus.md)
- [InternalCombustionEngineControl](../msg_docs/InternalCombustionEngineControl.md)
- [Ekf2Timestamps](../msg_docs/Ekf2Timestamps.md)
- [LandingTargetPose](../msg_docs/LandingTargetPose.md)
- [PositionControllerLandingStatus](../msg_docs/PositionControllerLandingStatus.md)
- [UavcanParameterValue](../msg_docs/UavcanParameterValue.md)
- [OrbitStatus](../msg_docs/OrbitStatus.md)
- [PositionControllerStatus](../msg_docs/PositionControllerStatus.md)
- [EstimatorStatus](../msg_docs/EstimatorStatus.md)
- [DatamanRequest](../msg_docs/DatamanRequest.md)
- [HoverThrustEstimate](../msg_docs/HoverThrustEstimate.md)
- [FixedWingLateralStatus](../msg_docs/FixedWingLateralStatus.md)
- [NavigatorMissionItem](../msg_docs/NavigatorMissionItem.md)
- [Cpuload](../msg_docs/Cpuload.md)
- [EstimatorAidSource3d](../msg_docs/EstimatorAidSource3d.md)
- [RoverRateStatus](../msg_docs/RoverRateStatus.md)
- [EscReport](../msg_docs/EscReport.md)
- [DebugArray](../msg_docs/DebugArray.md)
- [ControlAllocatorStatus](../msg_docs/ControlAllocatorStatus.md)
- [SensorHygrometer](../msg_docs/SensorHygrometer.md)
- [EstimatorSensorBias](../msg_docs/EstimatorSensorBias.md)
- [EstimatorBias3d](../msg_docs/EstimatorBias3d.md)
- [GimbalManagerInformation](../msg_docs/GimbalManagerInformation.md)
- [QshellReq](../msg_docs/QshellReq.md)
- [CameraStatus](../msg_docs/CameraStatus.md)
- [GpsInjectData](../msg_docs/GpsInjectData.md)
- [FigureEightStatus](../msg_docs/FigureEightStatus.md)
- [TransponderReport](../msg_docs/TransponderReport.md)
- [UavcanParameterRequest](../msg_docs/UavcanParameterRequest.md)
- [MavlinkLog](../msg_docs/MavlinkLog.md)
- [EstimatorGpsStatus](../msg_docs/EstimatorGpsStatus.md)
- [FuelTankStatus](../msg_docs/FuelTankStatus.md)
- [Mission](../msg_docs/Mission.md)
- [PositionSetpoint](../msg_docs/PositionSetpoint.md)
- [MissionResult](../msg_docs/MissionResult.md)
- [EstimatorEventFlags](../msg_docs/EstimatorEventFlags.md)
- [VehicleMagnetometer](../msg_docs/VehicleMagnetometer.md)
- [MavlinkTunnel](../msg_docs/MavlinkTunnel.md)
- [DifferentialPressure](../msg_docs/DifferentialPressure.md)
- [CellularStatus](../msg_docs/CellularStatus.md)
- [GpsDump](../msg_docs/GpsDump.md)
- [GimbalDeviceSetAttitude](../msg_docs/GimbalDeviceSetAttitude.md)
- [ArmingCheckReplyV0](../msg_docs/ArmingCheckReplyV0.md)
- [NavigatorStatus](../msg_docs/NavigatorStatus.md)
- [RoverPositionSetpoint](../msg_docs/RoverPositionSetpoint.md)
- [FollowTarget](../msg_docs/FollowTarget.md)
- [SensorsStatusImu](../msg_docs/SensorsStatusImu.md)
- [EstimatorStates](../msg_docs/EstimatorStates.md)
- [SensorGyro](../msg_docs/SensorGyro.md)
- [SensorAirflow](../msg_docs/SensorAirflow.md)
- [ButtonEvent](../msg_docs/ButtonEvent.md)
- [DebugKeyValue](../msg_docs/DebugKeyValue.md)
- [GpioConfig](../msg_docs/GpioConfig.md)
- [CameraTrigger](../msg_docs/CameraTrigger.md)
- [LandingGearWheel](../msg_docs/LandingGearWheel.md)
- [VehicleConstraints](../msg_docs/VehicleConstraints.md)
- [HealthReport](../msg_docs/HealthReport.md)
- [PowerButtonState](../msg_docs/PowerButtonState.md)
- [RadioStatus](../msg_docs/RadioStatus.md)
- [SensorGyroFifo](../msg_docs/SensorGyroFifo.md)
- [EstimatorBias](../msg_docs/EstimatorBias.md)
- [DebugVect](../msg_docs/DebugVect.md)
- [DistanceSensorModeChangeRequest](../msg_docs/DistanceSensorModeChangeRequest.md)
- [RtlTimeEstimate](../msg_docs/RtlTimeEstimate.md)
- [PpsCapture](../msg_docs/PpsCapture.md)
- [SensorSelection](../msg_docs/SensorSelection.md)
- [SystemPower](../msg_docs/SystemPower.md)
- [ActuatorControlsStatus](../msg_docs/ActuatorControlsStatus.md)
- [SensorGyroFft](../msg_docs/SensorGyroFft.md)
- [VehicleAirData](../msg_docs/VehicleAirData.md)
- [FollowTargetEstimator](../msg_docs/FollowTargetEstimator.md)
- [ParameterSetUsedRequest](../msg_docs/ParameterSetUsedRequest.md)
- [GpioRequest](../msg_docs/GpioRequest.md)
- [OpenDroneIdOperatorId](../msg_docs/OpenDroneIdOperatorId.md)
- [RtlStatus](../msg_docs/RtlStatus.md)
- [Airspeed](../msg_docs/Airspeed.md)
- [VehicleAcceleration](../msg_docs/VehicleAcceleration.md)
- [ParameterSetValueRequest](../msg_docs/ParameterSetValueRequest.md)
- [IrlockReport](../msg_docs/IrlockReport.md)
- [HeaterStatus](../msg_docs/HeaterStatus.md)
- [AdcReport](../msg_docs/AdcReport.md)
- [PwmInput](../msg_docs/PwmInput.md)
- [TiltrotorExtraControls](../msg_docs/TiltrotorExtraControls.md)
- [EstimatorAidSource1d](../msg_docs/EstimatorAidSource1d.md)
- [OrbTestMedium](../msg_docs/OrbTestMedium.md)
- [VehicleAttitudeSetpointV0](../msg_docs/VehicleAttitudeSetpointV0.md)
- [EstimatorAidSource2d](../msg_docs/EstimatorAidSource2d.md)
- [TuneControl](../msg_docs/TuneControl.md)
- [WheelEncoders](../msg_docs/WheelEncoders.md)
- [AutotuneAttitudeControlStatus](../msg_docs/AutotuneAttitudeControlStatus.md)
- [LandingTargetInnovations](../msg_docs/LandingTargetInnovations.md)
- [SensorsStatus](../msg_docs/SensorsStatus.md)

:::
