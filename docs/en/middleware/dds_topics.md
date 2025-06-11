# dds_topics.yaml — PX4 Topics Exposed to ROS 2

::: info
This document is [auto-generated](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/msg/generate_msg_docs.py) from the source code.
:::


The [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml) file specifies which uORB message definitions are compiled into the [uxrce_dds_client](../modules/modules_system.md#uxrce-dds-client) module when [PX4 is built](../middleware/uxrce_dds.md#code-generation), and hence which topics are available for ROS 2 applications to subscribe or publish (by default).

This document shows a markdown-rendered version of [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml), listing the publications, subscriptions, and so on.

## Publications

Topic | Type| Rate Limit
--- | --- | ---
`/fmu/out/airspeed_validated` | [px4_msgs::msg::AirspeedValidated](../msg_docs/AirspeedValidated.md) | 50.0
`/fmu/out/arming_check_request` | [px4_msgs::msg::ArmingCheckRequest](../msg_docs/ArmingCheckRequest.md) | 5.0
`/fmu/out/battery_status` | [px4_msgs::msg::BatteryStatus](../msg_docs/BatteryStatus.md) | 1.0
`/fmu/out/collision_constraints` | [px4_msgs::msg::CollisionConstraints](../msg_docs/CollisionConstraints.md) | 50.0
`/fmu/out/estimator_status_flags` | [px4_msgs::msg::EstimatorStatusFlags](../msg_docs/EstimatorStatusFlags.md) | 5.0
`/fmu/out/failsafe_flags` | [px4_msgs::msg::FailsafeFlags](../msg_docs/FailsafeFlags.md) | 5.0
`/fmu/out/home_position` | [px4_msgs::msg::HomePosition](../msg_docs/HomePosition.md) | 5.0
`/fmu/out/manual_control_setpoint` | [px4_msgs::msg::ManualControlSetpoint](../msg_docs/ManualControlSetpoint.md) | 25.0
`/fmu/out/message_format_response` | [px4_msgs::msg::MessageFormatResponse](../msg_docs/MessageFormatResponse.md) | 
`/fmu/out/mode_completed` | [px4_msgs::msg::ModeCompleted](../msg_docs/ModeCompleted.md) | 50.0
`/fmu/out/position_setpoint_triplet` | [px4_msgs::msg::PositionSetpointTriplet](../msg_docs/PositionSetpointTriplet.md) | 5.0
`/fmu/out/register_ext_component_reply` | [px4_msgs::msg::RegisterExtComponentReply](../msg_docs/RegisterExtComponentReply.md) | 
`/fmu/out/sensor_combined` | [px4_msgs::msg::SensorCombined](../msg_docs/SensorCombined.md) | 
`/fmu/out/vehicle_gps_position` | [px4_msgs::msg::SensorGps](../msg_docs/SensorGps.md) | 50.0
`/fmu/out/timesync_status` | [px4_msgs::msg::TimesyncStatus](../msg_docs/TimesyncStatus.md) | 10.0
`/fmu/out/vehicle_attitude` | [px4_msgs::msg::VehicleAttitude](../msg_docs/VehicleAttitude.md) | 
`/fmu/out/vehicle_command_ack` | [px4_msgs::msg::VehicleCommandAck](../msg_docs/VehicleCommandAck.md) | 
`/fmu/out/vehicle_control_mode` | [px4_msgs::msg::VehicleControlMode](../msg_docs/VehicleControlMode.md) | 50.0
`/fmu/out/vehicle_global_position` | [px4_msgs::msg::VehicleGlobalPosition](../msg_docs/VehicleGlobalPosition.md) | 50.0
`/fmu/out/vehicle_land_detected` | [px4_msgs::msg::VehicleLandDetected](../msg_docs/VehicleLandDetected.md) | 5.0
`/fmu/out/vehicle_local_position` | [px4_msgs::msg::VehicleLocalPosition](../msg_docs/VehicleLocalPosition.md) | 50.0
`/fmu/out/vehicle_odometry` | [px4_msgs::msg::VehicleOdometry](../msg_docs/VehicleOdometry.md) | 
`/fmu/out/vehicle_status` | [px4_msgs::msg::VehicleStatus](../msg_docs/VehicleStatus.md) | 5.0
`/fmu/out/vtol_vehicle_status` | [px4_msgs::msg::VtolVehicleStatus](../msg_docs/VtolVehicleStatus.md) | 

## Subscriptions

Topic | Type
--- | ---
/fmu/in/actuator_motors | [px4_msgs::msg::ActuatorMotors](../msg_docs/ActuatorMotors.md)
/fmu/in/actuator_servos | [px4_msgs::msg::ActuatorServos](../msg_docs/ActuatorServos.md)
/fmu/in/arming_check_reply | [px4_msgs::msg::ArmingCheckReply](../msg_docs/ArmingCheckReply.md)
/fmu/in/config_overrides_request | [px4_msgs::msg::ConfigOverrides](../msg_docs/ConfigOverrides.md)
/fmu/in/distance_sensor | [px4_msgs::msg::DistanceSensor](../msg_docs/DistanceSensor.md)
/fmu/in/fixed_wing_lateral_setpoint | [px4_msgs::msg::FixedWingLateralSetpoint](../msg_docs/FixedWingLateralSetpoint.md)
/fmu/in/fixed_wing_longitudinal_setpoint | [px4_msgs::msg::FixedWingLongitudinalSetpoint](../msg_docs/FixedWingLongitudinalSetpoint.md)
/fmu/in/goto_setpoint | [px4_msgs::msg::GotoSetpoint](../msg_docs/GotoSetpoint.md)
/fmu/in/lateral_control_configuration | [px4_msgs::msg::LateralControlConfiguration](../msg_docs/LateralControlConfiguration.md)
/fmu/in/longitudinal_control_configuration | [px4_msgs::msg::LongitudinalControlConfiguration](../msg_docs/LongitudinalControlConfiguration.md)
/fmu/in/manual_control_input | [px4_msgs::msg::ManualControlSetpoint](../msg_docs/ManualControlSetpoint.md)
/fmu/in/message_format_request | [px4_msgs::msg::MessageFormatRequest](../msg_docs/MessageFormatRequest.md)
/fmu/in/mode_completed | [px4_msgs::msg::ModeCompleted](../msg_docs/ModeCompleted.md)
/fmu/in/obstacle_distance | [px4_msgs::msg::ObstacleDistance](../msg_docs/ObstacleDistance.md)
/fmu/in/offboard_control_mode | [px4_msgs::msg::OffboardControlMode](../msg_docs/OffboardControlMode.md)
/fmu/in/onboard_computer_status | [px4_msgs::msg::OnboardComputerStatus](../msg_docs/OnboardComputerStatus.md)
/fmu/in/register_ext_component_request | [px4_msgs::msg::RegisterExtComponentRequest](../msg_docs/RegisterExtComponentRequest.md)
/fmu/in/sensor_optical_flow | [px4_msgs::msg::SensorOpticalFlow](../msg_docs/SensorOpticalFlow.md)
/fmu/in/telemetry_status | [px4_msgs::msg::TelemetryStatus](../msg_docs/TelemetryStatus.md)
/fmu/in/trajectory_setpoint | [px4_msgs::msg::TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md)
/fmu/in/unregister_ext_component | [px4_msgs::msg::UnregisterExtComponent](../msg_docs/UnregisterExtComponent.md)
/fmu/in/vehicle_attitude_setpoint | [px4_msgs::msg::VehicleAttitudeSetpoint](../msg_docs/VehicleAttitudeSetpoint.md)
/fmu/in/vehicle_command | [px4_msgs::msg::VehicleCommand](../msg_docs/VehicleCommand.md)
/fmu/in/vehicle_command_mode_executor | [px4_msgs::msg::VehicleCommand](../msg_docs/VehicleCommand.md)
/fmu/in/config_control_setpoints | [px4_msgs::msg::VehicleControlMode](../msg_docs/VehicleControlMode.md)
/fmu/in/aux_global_position | [px4_msgs::msg::VehicleGlobalPosition](../msg_docs/VehicleGlobalPosition.md)
/fmu/in/vehicle_mocap_odometry | [px4_msgs::msg::VehicleOdometry](../msg_docs/VehicleOdometry.md)
/fmu/in/vehicle_visual_odometry | [px4_msgs::msg::VehicleOdometry](../msg_docs/VehicleOdometry.md)
/fmu/in/vehicle_rates_setpoint | [px4_msgs::msg::VehicleRatesSetpoint](../msg_docs/VehicleRatesSetpoint.md)
/fmu/in/vehicle_thrust_setpoint | [px4_msgs::msg::VehicleThrustSetpoint](../msg_docs/VehicleThrustSetpoint.md)
/fmu/in/vehicle_torque_setpoint | [px4_msgs::msg::VehicleTorqueSetpoint](../msg_docs/VehicleTorqueSetpoint.md)

## Subscriptions Multi

None

## Not Exported

These messages are not listed in the yaml file.
They are not build into the module, and hence are neither published or subscribed.

::: details See messages

- [ActionRequest](../msg_docs/ActionRequest.md)
- [ActuatorArmed](../msg_docs/ActuatorArmed.md)
- [ActuatorControlsStatus](../msg_docs/ActuatorControlsStatus.md)
- [ActuatorOutputs](../msg_docs/ActuatorOutputs.md)
- [ActuatorServosTrim](../msg_docs/ActuatorServosTrim.md)
- [ActuatorTest](../msg_docs/ActuatorTest.md)
- [AdcReport](../msg_docs/AdcReport.md)
- [Airspeed](../msg_docs/Airspeed.md)
- [AirspeedValidatedV0](../msg_docs/AirspeedValidatedV0.md)
- [AirspeedWind](../msg_docs/AirspeedWind.md)
- [ArmingCheckReplyV0](../msg_docs/ArmingCheckReplyV0.md)
- [AutotuneAttitudeControlStatus](../msg_docs/AutotuneAttitudeControlStatus.md)
- [ButtonEvent](../msg_docs/ButtonEvent.md)
- [CameraCapture](../msg_docs/CameraCapture.md)
- [CameraStatus](../msg_docs/CameraStatus.md)
- [CameraTrigger](../msg_docs/CameraTrigger.md)
- [CanInterfaceStatus](../msg_docs/CanInterfaceStatus.md)
- [CellularStatus](../msg_docs/CellularStatus.md)
- [ControlAllocatorStatus](../msg_docs/ControlAllocatorStatus.md)
- [Cpuload](../msg_docs/Cpuload.md)
- [DatamanRequest](../msg_docs/DatamanRequest.md)
- [DatamanResponse](../msg_docs/DatamanResponse.md)
- [DebugArray](../msg_docs/DebugArray.md)
- [DebugKeyValue](../msg_docs/DebugKeyValue.md)
- [DebugValue](../msg_docs/DebugValue.md)
- [DebugVect](../msg_docs/DebugVect.md)
- [DifferentialPressure](../msg_docs/DifferentialPressure.md)
- [DistanceSensorModeChangeRequest](../msg_docs/DistanceSensorModeChangeRequest.md)
- [Ekf2Timestamps](../msg_docs/Ekf2Timestamps.md)
- [EscReport](../msg_docs/EscReport.md)
- [EscStatus](../msg_docs/EscStatus.md)
- [EstimatorAidSource1d](../msg_docs/EstimatorAidSource1d.md)
- [EstimatorAidSource2d](../msg_docs/EstimatorAidSource2d.md)
- [EstimatorAidSource3d](../msg_docs/EstimatorAidSource3d.md)
- [EstimatorBias](../msg_docs/EstimatorBias.md)
- [EstimatorBias3d](../msg_docs/EstimatorBias3d.md)
- [EstimatorEventFlags](../msg_docs/EstimatorEventFlags.md)
- [EstimatorGpsStatus](../msg_docs/EstimatorGpsStatus.md)
- [EstimatorInnovations](../msg_docs/EstimatorInnovations.md)
- [EstimatorSelectorStatus](../msg_docs/EstimatorSelectorStatus.md)
- [EstimatorSensorBias](../msg_docs/EstimatorSensorBias.md)
- [EstimatorStates](../msg_docs/EstimatorStates.md)
- [EstimatorStatus](../msg_docs/EstimatorStatus.md)
- [Event](../msg_docs/Event.md)
- [EventV0](../msg_docs/EventV0.md)
- [FailureDetectorStatus](../msg_docs/FailureDetectorStatus.md)
- [FigureEightStatus](../msg_docs/FigureEightStatus.md)
- [FixedWingLateralGuidanceStatus](../msg_docs/FixedWingLateralGuidanceStatus.md)
- [FixedWingLateralStatus](../msg_docs/FixedWingLateralStatus.md)
- [FixedWingRunwayControl](../msg_docs/FixedWingRunwayControl.md)
- [FlightPhaseEstimation](../msg_docs/FlightPhaseEstimation.md)
- [FollowTarget](../msg_docs/FollowTarget.md)
- [FollowTargetEstimator](../msg_docs/FollowTargetEstimator.md)
- [FollowTargetStatus](../msg_docs/FollowTargetStatus.md)
- [FuelTankStatus](../msg_docs/FuelTankStatus.md)
- [GeneratorStatus](../msg_docs/GeneratorStatus.md)
- [GeofenceResult](../msg_docs/GeofenceResult.md)
- [GeofenceStatus](../msg_docs/GeofenceStatus.md)
- [GimbalControls](../msg_docs/GimbalControls.md)
- [GimbalDeviceAttitudeStatus](../msg_docs/GimbalDeviceAttitudeStatus.md)
- [GimbalDeviceInformation](../msg_docs/GimbalDeviceInformation.md)
- [GimbalDeviceSetAttitude](../msg_docs/GimbalDeviceSetAttitude.md)
- [GimbalManagerInformation](../msg_docs/GimbalManagerInformation.md)
- [GimbalManagerSetAttitude](../msg_docs/GimbalManagerSetAttitude.md)
- [GimbalManagerSetManualControl](../msg_docs/GimbalManagerSetManualControl.md)
- [GimbalManagerStatus](../msg_docs/GimbalManagerStatus.md)
- [GpioConfig](../msg_docs/GpioConfig.md)
- [GpioIn](../msg_docs/GpioIn.md)
- [GpioOut](../msg_docs/GpioOut.md)
- [GpioRequest](../msg_docs/GpioRequest.md)
- [GpsDump](../msg_docs/GpsDump.md)
- [GpsInjectData](../msg_docs/GpsInjectData.md)
- [Gripper](../msg_docs/Gripper.md)
- [HealthReport](../msg_docs/HealthReport.md)
- [HeaterStatus](../msg_docs/HeaterStatus.md)
- [HoverThrustEstimate](../msg_docs/HoverThrustEstimate.md)
- [InputRc](../msg_docs/InputRc.md)
- [InternalCombustionEngineControl](../msg_docs/InternalCombustionEngineControl.md)
- [InternalCombustionEngineStatus](../msg_docs/InternalCombustionEngineStatus.md)
- [IridiumsbdStatus](../msg_docs/IridiumsbdStatus.md)
- [IrlockReport](../msg_docs/IrlockReport.md)
- [LandingGear](../msg_docs/LandingGear.md)
- [LandingGearWheel](../msg_docs/LandingGearWheel.md)
- [LandingTargetInnovations](../msg_docs/LandingTargetInnovations.md)
- [LandingTargetPose](../msg_docs/LandingTargetPose.md)
- [LaunchDetectionStatus](../msg_docs/LaunchDetectionStatus.md)
- [LedControl](../msg_docs/LedControl.md)
- [LogMessage](../msg_docs/LogMessage.md)
- [LoggerStatus](../msg_docs/LoggerStatus.md)
- [MagWorkerData](../msg_docs/MagWorkerData.md)
- [MagnetometerBiasEstimate](../msg_docs/MagnetometerBiasEstimate.md)
- [ManualControlSwitches](../msg_docs/ManualControlSwitches.md)
- [MavlinkLog](../msg_docs/MavlinkLog.md)
- [MavlinkTunnel](../msg_docs/MavlinkTunnel.md)
- [Mission](../msg_docs/Mission.md)
- [MissionResult](../msg_docs/MissionResult.md)
- [MountOrientation](../msg_docs/MountOrientation.md)
- [NavigatorMissionItem](../msg_docs/NavigatorMissionItem.md)
- [NavigatorStatus](../msg_docs/NavigatorStatus.md)
- [NormalizedUnsignedSetpoint](../msg_docs/NormalizedUnsignedSetpoint.md)
- [OpenDroneIdArmStatus](../msg_docs/OpenDroneIdArmStatus.md)
- [OpenDroneIdOperatorId](../msg_docs/OpenDroneIdOperatorId.md)
- [OpenDroneIdSelfId](../msg_docs/OpenDroneIdSelfId.md)
- [OpenDroneIdSystem](../msg_docs/OpenDroneIdSystem.md)
- [OrbTest](../msg_docs/OrbTest.md)
- [OrbTestLarge](../msg_docs/OrbTestLarge.md)
- [OrbTestMedium](../msg_docs/OrbTestMedium.md)
- [OrbitStatus](../msg_docs/OrbitStatus.md)
- [ParameterResetRequest](../msg_docs/ParameterResetRequest.md)
- [ParameterSetUsedRequest](../msg_docs/ParameterSetUsedRequest.md)
- [ParameterSetValueRequest](../msg_docs/ParameterSetValueRequest.md)
- [ParameterSetValueResponse](../msg_docs/ParameterSetValueResponse.md)
- [ParameterUpdate](../msg_docs/ParameterUpdate.md)
- [Ping](../msg_docs/Ping.md)
- [PositionControllerLandingStatus](../msg_docs/PositionControllerLandingStatus.md)
- [PositionControllerStatus](../msg_docs/PositionControllerStatus.md)
- [PositionSetpoint](../msg_docs/PositionSetpoint.md)
- [PowerButtonState](../msg_docs/PowerButtonState.md)
- [PowerMonitor](../msg_docs/PowerMonitor.md)
- [PpsCapture](../msg_docs/PpsCapture.md)
- [PurePursuitStatus](../msg_docs/PurePursuitStatus.md)
- [PwmInput](../msg_docs/PwmInput.md)
- [Px4ioStatus](../msg_docs/Px4ioStatus.md)
- [QshellReq](../msg_docs/QshellReq.md)
- [QshellRetval](../msg_docs/QshellRetval.md)
- [RadioStatus](../msg_docs/RadioStatus.md)
- [RateCtrlStatus](../msg_docs/RateCtrlStatus.md)
- [RcChannels](../msg_docs/RcChannels.md)
- [RcParameterMap](../msg_docs/RcParameterMap.md)
- [RoverAttitudeSetpoint](../msg_docs/RoverAttitudeSetpoint.md)
- [RoverAttitudeStatus](../msg_docs/RoverAttitudeStatus.md)
- [RoverPositionSetpoint](../msg_docs/RoverPositionSetpoint.md)
- [RoverRateSetpoint](../msg_docs/RoverRateSetpoint.md)
- [RoverRateStatus](../msg_docs/RoverRateStatus.md)
- [RoverSteeringSetpoint](../msg_docs/RoverSteeringSetpoint.md)
- [RoverThrottleSetpoint](../msg_docs/RoverThrottleSetpoint.md)
- [RoverVelocitySetpoint](../msg_docs/RoverVelocitySetpoint.md)
- [RoverVelocityStatus](../msg_docs/RoverVelocityStatus.md)
- [Rpm](../msg_docs/Rpm.md)
- [RtlStatus](../msg_docs/RtlStatus.md)
- [RtlTimeEstimate](../msg_docs/RtlTimeEstimate.md)
- [SatelliteInfo](../msg_docs/SatelliteInfo.md)
- [SensorAccel](../msg_docs/SensorAccel.md)
- [SensorAccelFifo](../msg_docs/SensorAccelFifo.md)
- [SensorAirflow](../msg_docs/SensorAirflow.md)
- [SensorBaro](../msg_docs/SensorBaro.md)
- [SensorCorrection](../msg_docs/SensorCorrection.md)
- [SensorGnssRelative](../msg_docs/SensorGnssRelative.md)
- [SensorGyro](../msg_docs/SensorGyro.md)
- [SensorGyroFft](../msg_docs/SensorGyroFft.md)
- [SensorGyroFifo](../msg_docs/SensorGyroFifo.md)
- [SensorHygrometer](../msg_docs/SensorHygrometer.md)
- [SensorMag](../msg_docs/SensorMag.md)
- [SensorPreflightMag](../msg_docs/SensorPreflightMag.md)
- [SensorSelection](../msg_docs/SensorSelection.md)
- [SensorUwb](../msg_docs/SensorUwb.md)
- [SensorsStatus](../msg_docs/SensorsStatus.md)
- [SensorsStatusImu](../msg_docs/SensorsStatusImu.md)
- [SystemPower](../msg_docs/SystemPower.md)
- [TakeoffStatus](../msg_docs/TakeoffStatus.md)
- [TaskStackInfo](../msg_docs/TaskStackInfo.md)
- [TecsStatus](../msg_docs/TecsStatus.md)
- [TiltrotorExtraControls](../msg_docs/TiltrotorExtraControls.md)
- [TrajectorySetpoint6dof](../msg_docs/TrajectorySetpoint6dof.md)
- [TransponderReport](../msg_docs/TransponderReport.md)
- [TuneControl](../msg_docs/TuneControl.md)
- [UavcanParameterRequest](../msg_docs/UavcanParameterRequest.md)
- [UavcanParameterValue](../msg_docs/UavcanParameterValue.md)
- [UlogStream](../msg_docs/UlogStream.md)
- [UlogStreamAck](../msg_docs/UlogStreamAck.md)
- [VehicleAcceleration](../msg_docs/VehicleAcceleration.md)
- [VehicleAirData](../msg_docs/VehicleAirData.md)
- [VehicleAngularAccelerationSetpoint](../msg_docs/VehicleAngularAccelerationSetpoint.md)
- [VehicleAngularVelocity](../msg_docs/VehicleAngularVelocity.md)
- [VehicleAttitudeSetpointV0](../msg_docs/VehicleAttitudeSetpointV0.md)
- [VehicleConstraints](../msg_docs/VehicleConstraints.md)
- [VehicleImu](../msg_docs/VehicleImu.md)
- [VehicleImuStatus](../msg_docs/VehicleImuStatus.md)
- [VehicleLocalPositionSetpoint](../msg_docs/VehicleLocalPositionSetpoint.md)
- [VehicleMagnetometer](../msg_docs/VehicleMagnetometer.md)
- [VehicleOpticalFlow](../msg_docs/VehicleOpticalFlow.md)
- [VehicleOpticalFlowVel](../msg_docs/VehicleOpticalFlowVel.md)
- [VehicleRoi](../msg_docs/VehicleRoi.md)
- [VehicleStatusV0](../msg_docs/VehicleStatusV0.md)
- [VelocityLimits](../msg_docs/VelocityLimits.md)
- [WheelEncoders](../msg_docs/WheelEncoders.md)
- [Wind](../msg_docs/Wind.md)
- [YawEstimatorStatus](../msg_docs/YawEstimatorStatus.md)
:::
