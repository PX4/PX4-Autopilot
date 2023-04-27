/*
 * agilex_message.h
 *
 * Created on: Dec 10, 2020 11:47
 * Description:
 *  all values are using SI units (e.g. meter/second/radian)
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef AGILEX_MESSAGE_H
#define AGILEX_MESSAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "agilex_types.h"

/***************** Control messages *****************/

typedef struct {
	float linear_velocity;
	float angular_velocity;
	float lateral_velocity;
	float steering_angle;
} MotionCommandMessage;

typedef struct {
	bool enable_cmd_ctrl;
	LightOperation front_light;
	LightOperation rear_light;
} LightCommandMessage;

typedef struct {
	bool enable_braking;
} BrakingCommandMessage;

typedef struct {
	uint8_t motion_mode;
} MotionModeCommandMessage;

/**************** Feedback messages *****************/

#define SYSTEM_ERROR_MOTOR_DRIVER_MASK ((uint16_t)0x0100)
#define SYSTEM_ERROR_HL_COMM_MASK ((uint16_t)0x0200)
#define SYSTEM_ERROR_BATTERY_FAULT_MASK ((uint16_t)0x0001)
#define SYSTEM_ERROR_BATTERY_WARNING_MASK ((uint16_t)0x0002)
#define SYSTEM_ERROR_RC_SIGNAL_LOSS_MASK ((uint16_t)0x0004)
#define SYSTEM_ERROR_MOTOR1_COMM_MASK ((uint16_t)0x0008)
#define SYSTEM_ERROR_MOTOR2_COMM_MASK ((uint16_t)0x0010)
#define SYSTEM_ERROR_MOTOR3_COMM_MASK ((uint16_t)0x0020)
#define SYSTEM_ERROR_MOTOR4_COMM_MASK ((uint16_t)0x0040)
#define SYSTEM_ERROR_STEER_ENCODER_MASK ((uint16_t)0x0080)

typedef struct {
	VehicleState vehicle_state;
	ControlMode control_mode;
	float battery_voltage;
	uint16_t error_code;
} SystemStateMessage;

typedef struct {
	float linear_velocity;
	float angular_velocity;  // only valid for differential drivering
	float lateral_velocity;
	float steering_angle;  // only valid for ackermann steering
} MotionStateMessage;

typedef LightCommandMessage LightStateMessage;

typedef struct {
	RcSwitchState swa;
	RcSwitchState swb;
	RcSwitchState swc;
	RcSwitchState swd;
	int8_t stick_right_v;
	int8_t stick_right_h;
	int8_t stick_left_v;
	int8_t stick_left_h;
	int8_t var_a;
} RcStateMessage;

typedef struct {
	uint8_t motor_id;
	int16_t rpm;
	float current;
	int32_t pulse_count;
} ActuatorHSStateMessage;

#define DRIVER_STATE_INPUT_VOLTAGE_LOW_MASK ((uint8_t)0x01)
#define DRIVER_STATE_MOTOR_OVERHEAT_MASK ((uint8_t)0x02)
#define DRIVER_STATE_DRIVER_OVERLOAD_MASK ((uint8_t)0x04)
#define DRIVER_STATE_DRIVER_OVERHEAT_MASK ((uint8_t)0x08)
#define DRIVER_STATE_SENSOR_FAULT_MASK ((uint8_t)0x10)
#define DRIVER_STATE_DRIVER_FAULT_MASK ((uint8_t)0x20)
#define DRIVER_STATE_DRIVER_ENABLED_MASK ((uint8_t)0x40)
#define DRIVER_STATE_DRIVER_RESET_MASK ((uint8_t)0x80)

typedef struct {
	uint8_t motor_id;
	float driver_voltage;
	float driver_temp;
	float motor_temp;
	uint8_t driver_state;
} ActuatorLSStateMessage;

typedef struct {
	uint8_t motion_mode;
	uint8_t mode_changing;
} MotionModeStateMessage;

/***************** Sensor messages ******************/

typedef struct {
	float left_wheel;
	float right_wheel;
} OdometryMessage;

typedef struct {
	float accel_x;
	float accel_y;
	float accel_z;
} ImuAccelMessage;

typedef struct {
	float gyro_x;
	float gyro_y;
	float gyro_z;
} ImuGyroMessage;

typedef struct {
	float yaw;
	float pitch;
	float roll;
} ImuEulerMessage;

typedef struct {
	uint8_t trigger_state;
} SafetyBumperMessage;

typedef struct {
	uint8_t sensor_id;
	uint8_t distance[8];
} UltrasonicMessage;

typedef struct {
	uint8_t sensor_id;
	float relative_distance;
	float relative_angle;
	bool is_normal;
	int8_t channels[3];
} UwbMessage;

typedef struct {
	uint8_t battery_soc;
	uint8_t battery_soh;
	float voltage;
	float current;
	float temperature;
} BmsBasicMessage;

typedef struct {
	float angle_5;
	float angle_6;
	float angle_7;
	float angle_8;
} MotorAngleMessage;

typedef struct {
	float speed_1;
	float speed_2;
	float speed_3;
	float speed_4;
} MotorSpeedMessage;

#define BMS_PROT1_CHARGING_CURRENT_NONZERO_MASK ((uint8_t)0x01)
#define BMS_PROT1_CHARGING_OVERCURRENT_SET_MASK ((uint8_t)0x02)
#define BMS_PROT1_DISCHARGING_CURRENT_NONZERO_MASK ((uint8_t)0x10)
#define BMS_PROT1_DISCHARGING_OVERCURRENT_SET_MASK ((uint8_t)0x20)
#define BMS_PROT1_DISCHARGING_SHORTCIRCUIT_SET_MASK ((uint8_t)0x40)

#define BMS_PROT2_CORE_OPENCIRCUIT_SET_MASK ((uint8_t)0x01)
#define BMS_PROT2_TEMP_SENSOR_OPENCIRCUIT_SET_MASK ((uint8_t)0x02)
#define BMS_PROT2_CORE_OVERVOLTAGE_SET_MASK ((uint8_t)0x10)
#define BMS_PROT2_CORE_UNDERVOLTAGE_SET_MASK ((uint8_t)0x20)
#define BMS_PROT2_TOTAL_OVERVOLTAGE_SET_MASK ((uint8_t)0x40)
#define BMS_PROT2_TOTAL_UNDERVOLTAGE_SET_MASK ((uint8_t)0x80)

#define BMS_PROT3_CHARGING_OVERTEMP_SET_MASK ((uint8_t)0x04)
#define BMS_PROT3_DISCHARGING_OVERTEMP_SET_MASK ((uint8_t)0x08)
#define BMS_PROT3_CHARGING_UNDERTEMP_SET_MASK ((uint8_t)0x10)
#define BMS_PROT3_DISCHARGING_UNDERTEMP_SET_MASK ((uint8_t)0x20)
#define BMS_PROT3_CHARGING_TEMPDIFF_SET_MASK ((uint8_t)0x40)
#define BMS_PROT3_DISCHARGING_TEMPDIFF_SET_MASK ((uint8_t)0x80)

#define BMS_PROT4_CHARGING_MOS_STATE_SET_MASK ((uint8_t)0x01)
#define BMS_PROT4_DISCHARGING_MOS_STATE_SET_MASK ((uint8_t)0x02)
#define BMS_PROT4_CHARGING_MOS_FAILURE_SET_MASK ((uint8_t)0x04)
#define BMS_PROT4_DISCHARGING_MOS_FAILURE_SET_MASK ((uint8_t)0x08)
#define BMS_PROT4_WEAK_SIGNAL_SWITCH_OPEN_SET_MASK ((uint8_t)0x10)

typedef struct {
	uint8_t protection_code1;
	uint8_t protection_code2;
	uint8_t protection_code3;
	uint8_t protection_code4;
	uint8_t battery_max_teperature;
	uint8_t battery_min_teperature;
} BmsExtendedMessage;

/************  Query/config messages ****************/

typedef struct {
	bool request;
} VersionRequestMessage;

typedef struct {
	uint16_t controller_hw_version;
	uint16_t motor_driver_hw_version;
	uint16_t controller_sw_version;
	uint16_t motor_driver_sw_version;
} VersionResponseMessage;

typedef struct {
	ControlMode mode;
} ControlModeConfigMessage;

typedef struct {
	BrakeMode mode;
} BrakeModeConfigMessage;

typedef struct {
	bool set_as_neutral;
} SteerNeutralRequestMessage;

typedef struct {
	bool neutral_set_successful;
} SteerNeutralResponseMessage;

typedef enum {
	CLEAR_ALL_FAULT = 0x00,
	CLEAR_MOTOR1_FAULT = 0x01,
	CLEAR_MOTOR2_FAULT = 0x02,
	CLEAR_MOTOR3_FAULT = 0x03,
	CLEAR__MOTOR4_FAULT = 0x04
} FaultClearCode;

typedef struct {
	uint8_t error_clear_byte;
} StateResetConfigMessage;

//////////////////////////////////////////////////////

typedef enum {
	AgxMsgUnknown = 0x00,
	// command
	AgxMsgMotionCommand,
	AgxMsgLightCommand,
	AgxMsgBrakingCommand,
	AgxMsgSetMotionModeCommand,
	// state feedback
	AgxMsgSystemState,
	AgxMsgMotionState,
	AgxMsgLightState,
	AgxMsgMotionModeState,
	AgxMsgRcState,
	// actuator feedback
	AgxMsgActuatorHSState,
	AgxMsgActuatorLSState,
	AgxMsgMotorAngle,
	AgxMsgMotorSpeed,
	// sensor
	AgxMsgOdometry,
	AgxMsgImuAccel,
	AgxMsgImuGyro,
	AgxMsgImuEuler,
	AgxMsgSafetyBumper,
	AgxMsgUltrasonic,
	AgxMsgUwb,
	AgxMsgBmsBasic,
	AgxMsgBmsExtended,
	// query/config
	AgxMsgVersionRequest,
	AgxMsgVersionResponse,
	AgxMsgControlModeConfig,
	AgxMsgSteerNeutralRequest,
	AgxMsgSteerNeutralResponse,
	AgxMsgStateResetConfig,
	AgxMsgBrakeModeConfig
} MsgType;

typedef struct {
	MsgType type;
	union {
		// command
		MotionCommandMessage motion_command_msg;
		LightCommandMessage light_command_msg;
		BrakingCommandMessage braking_command_msg;
		MotionModeCommandMessage motion_mode_msg;
		// core state feedback
		SystemStateMessage system_state_msg;
		MotionStateMessage motion_state_msg;
		LightStateMessage light_state_msg;
		MotionModeStateMessage motion_mode_state_msg;
		RcStateMessage rc_state_msg;
		// actuator feedback
		ActuatorHSStateMessage actuator_hs_state_msg;
		ActuatorLSStateMessage actuator_ls_state_msg;
		// sensor
		OdometryMessage odometry_msg;
		ImuAccelMessage imu_accel_msg;
		ImuGyroMessage imu_gyro_msg;
		ImuEulerMessage imu_euler_msg;
		SafetyBumperMessage safety_bumper_msg;
		UltrasonicMessage ultrasonic_msg;
		UwbMessage uwb_msg;
		BmsBasicMessage bms_basic_msg;
		BmsExtendedMessage bms_extended_msg;
		// query/config
		VersionRequestMessage version_request_msg;
		VersionResponseMessage version_response_msg;
//    uint8_t version_str[10][8];
		uint8_t version_str[8];
		ControlModeConfigMessage control_mode_config_msg;
		BrakeModeConfigMessage brake_mode_config_msg;
		SteerNeutralRequestMessage steer_neutral_request_msg;
		SteerNeutralResponseMessage steer_neutral_response_msg;
		StateResetConfigMessage state_reset_config_msg;

		MotorAngleMessage motor_angle_msg;
		MotorSpeedMessage motor_speed_msg;

	} body;
} AgxMessage;

#ifdef __cplusplus
}
#endif

#endif /* AGILEX_MESSAGE_H */
