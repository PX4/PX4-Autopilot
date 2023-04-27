/*
 * agilex_protocol_v2.h
 *
 * Created on: Nov 04, 2020 13:54
 * Description:
 *
 * Copyright (c) 2020 Weston Robot Pte. Ltd.
 */

#ifndef AGILEX_PROTOCOL_V2_H
#define AGILEX_PROTOCOL_V2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// define endianess of the platform
#if (!defined(USE_LITTLE_ENDIAN) && !defined(USE_BIG_ENDIAN))
#define USE_LITTLE_ENDIAN
#endif

#ifdef USE_BIG_ENDIAN
#error "BIG ENDIAN IS CURRENTLY NOT SUPPORTED"
#endif

/*---------------------------- Motor IDs -------------------------------*/

#define ACTUATOR1_ID ((uint8_t)0x00)
#define ACTUATOR2_ID ((uint8_t)0x01)
#define ACTUATOR3_ID ((uint8_t)0x02)
#define ACTUATOR4_ID ((uint8_t)0x03)
#define ACTUATOR5_ID ((uint8_t)0x04)
#define ACTUATOR6_ID ((uint8_t)0x05)
#define ACTUATOR7_ID ((uint8_t)0x06)
#define ACTUATOR8_ID ((uint8_t)0x07)

/*--------------------------- Message IDs ------------------------------*/

// control group: 0x1
#define CAN_MSG_MOTION_COMMAND_ID ((uint32_t)0x111)
#define CAN_MSG_LIGHT_COMMAND_ID ((uint32_t)0x121)
#define CAN_MSG_BRAKING_COMMAND_ID ((uint32_t)0x131)
#define CAN_MSG_SET_MOTION_MODE_ID ((uint32_t)0x141)

// state feedback group: 0x2
#define CAN_MSG_SYSTEM_STATE_ID ((uint32_t)0x211)
#define CAN_MSG_MOTION_STATE_ID ((uint32_t)0x221)
#define CAN_MSG_LIGHT_STATE_ID ((uint32_t)0x231)
#define CAN_MSG_RC_STATE_ID ((uint32_t)0x241)

#define CAN_MSG_ACTUATOR1_HS_STATE_ID ((uint32_t)0x251)
#define CAN_MSG_ACTUATOR2_HS_STATE_ID ((uint32_t)0x252)
#define CAN_MSG_ACTUATOR3_HS_STATE_ID ((uint32_t)0x253)
#define CAN_MSG_ACTUATOR4_HS_STATE_ID ((uint32_t)0x254)
#define CAN_MSG_ACTUATOR5_HS_STATE_ID ((uint32_t)0x255)
#define CAN_MSG_ACTUATOR6_HS_STATE_ID ((uint32_t)0x256)
#define CAN_MSG_ACTUATOR7_HS_STATE_ID ((uint32_t)0x257)
#define CAN_MSG_ACTUATOR8_HS_STATE_ID ((uint32_t)0x258)

#define CAN_MSG_ACTUATOR1_LS_STATE_ID ((uint32_t)0x261)
#define CAN_MSG_ACTUATOR2_LS_STATE_ID ((uint32_t)0x262)
#define CAN_MSG_ACTUATOR3_LS_STATE_ID ((uint32_t)0x263)
#define CAN_MSG_ACTUATOR4_LS_STATE_ID ((uint32_t)0x264)
#define CAN_MSG_ACTUATOR5_LS_STATE_ID ((uint32_t)0x265)
#define CAN_MSG_ACTUATOR6_LS_STATE_ID ((uint32_t)0x266)
#define CAN_MSG_ACTUATOR7_LS_STATE_ID ((uint32_t)0x267)
#define CAN_MSG_ACTUATOR8_LS_STATE_ID ((uint32_t)0x268)

#define CAN_MSG_MOTOR_ANGLE_INFO ((uint32_t)0x271)
#define CAN_MSG_MOTOR_SPEED_INFO ((uint32_t)0x281)
#define CAN_MSG_CURRENT_CTRL_MODE ((uint32_t)0x291)

// sensor data group: 0x3
#define CAN_MSG_ODOMETRY_ID ((uint32_t)0x311)

#define CAN_MSG_IMU_ACCEL_ID ((uint32_t)0x321)
#define CAN_MSG_IMU_GYRO_ID ((uint32_t)0x322)
#define CAN_MSG_IMU_EULER_ID ((uint32_t)0x323)

#define CAN_MSG_SAFETY_BUMPER_ID ((uint32_t)0x331)

#define CAN_MSG_ULTRASONIC_1_ID ((uint32_t)0x341)
#define CAN_MSG_ULTRASONIC_2_ID ((uint32_t)0x342)
#define CAN_MSG_ULTRASONIC_3_ID ((uint32_t)0x343)
#define CAN_MSG_ULTRASONIC_4_ID ((uint32_t)0x344)
#define CAN_MSG_ULTRASONIC_5_ID ((uint32_t)0x345)
#define CAN_MSG_ULTRASONIC_6_ID ((uint32_t)0x346)
#define CAN_MSG_ULTRASONIC_7_ID ((uint32_t)0x347)
#define CAN_MSG_ULTRASONIC_8_ID ((uint32_t)0x348)

#define CAN_MSG_UWB_1_ID ((uint32_t)0x351)
#define CAN_MSG_UWB_2_ID ((uint32_t)0x352)
#define CAN_MSG_UWB_3_ID ((uint32_t)0x353)
#define CAN_MSG_UWB_4_ID ((uint32_t)0x354)

#define CAN_MSG_BMS_BASIC_ID ((uint32_t)0x361)
#define CAN_MSG_BMS_EXTENDED_ID ((uint32_t)0x362)

// query/config group: 0x4
#define CAN_MSG_VERSION_REQUEST_ID ((uint32_t)0x411)
#define CAN_MSG_VERSION_RESPONSE_ID ((uint32_t)0x4a1)

#define CAN_MSG_CTRL_MODE_CONFIG_ID ((uint32_t)0x421)

#define CAN_MSG_STEER_NEUTRAL_REQUEST_ID ((uint32_t)0x431)
#define CAN_MSG_STEER_NEUTRAL_RESPONSE_ID ((uint32_t)0x43a)

#define CAN_MSG_STATE_RESET_CONFIG_ID ((uint32_t)0x441)

/*------------------------ Frame Memory Layout -------------------------*/

/* No padding in the struct */
// reference: https://stackoverflow.com/questions/3318410/pragma-pack-effect
#pragma pack(push, 1)

#ifdef USE_LITTLE_ENDIAN
typedef struct {
  uint8_t high_byte;
  uint8_t low_byte;
} struct16_t;
typedef struct {
  uint8_t msb;
  uint8_t high_byte;
  uint8_t low_byte;
  uint8_t lsb;
} struct32_t;
#elif defined(USE_BIG_ENDIAN)
typedef struct {
  uint8_t low_byte;
  uint8_t high_byte;
} struct16_t;
typedef struct {
  uint8_t lsb;
  uint8_t low_byte;
  uint8_t high_byte;
  uint8_t msb;
} struct32_t;
#endif

// Control messages
typedef struct {
  struct16_t linear_velocity;
  struct16_t angular_velocity;
  struct16_t lateral_velocity;
  struct16_t steering_angle;
} MotionCommandFrame;

#define LIGHT_ENABLE_CMD_CTRL ((uint8_t)0x01)
#define LIGHT_DISABLE_CMD_CTRL ((uint8_t)0x00)

typedef struct {
  uint8_t enable_cmd_ctrl;
  uint8_t front_mode;
  uint8_t front_custom;
  uint8_t rear_mode;
  uint8_t rear_custom;
  uint8_t reserved0;
  uint8_t reserved1;
  uint8_t count;
} LightCommandFrame;

typedef struct {
  uint8_t enable_brake;
  uint8_t count;
} BrakingCommandFrame;

typedef struct {
  uint8_t motion_mode;
  uint8_t reserved0;
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t reserved4;
  uint8_t reserved5;
  uint8_t reserved6;
} SetMotionModeFrame;

// State feedback messages
typedef struct {
  uint8_t vehicle_state;
  uint8_t control_mode;
  struct16_t battery_voltage;
  struct16_t error_code;
  uint8_t reserved0;
  uint8_t count;
} SystemStateFrame;

typedef struct {
  struct16_t linear_velocity;
  struct16_t angular_velocity;
  struct16_t lateral_velocity;
  struct16_t steering_angle;
} MotionStateFrame;

typedef struct {
  uint8_t enable_cmd_ctrl;
  uint8_t front_mode;
  uint8_t front_custom;
  uint8_t rear_mode;
  uint8_t rear_custom;
  uint8_t reserved0;
  uint8_t reserved1;
  uint8_t count;
} LightStateFrame;

#define RC_SWA_MASK ((uint8_t)0b00000011)
#define RC_SWA_UP_MASK ((uint8_t)0b00000010)
#define RC_SWA_DOWN_MASK ((uint8_t)0b00000011)

#define RC_SWB_MASK ((uint8_t)0b00001100)
#define RC_SWB_UP_MASK ((uint8_t)0b00001000)
#define RC_SWB_MIDDLE_MASK ((uint8_t)0b00000100)
#define RC_SWB_DOWN_MASK ((uint8_t)0b00001100)

#define RC_SWC_MASK ((uint8_t)0b00110000)
#define RC_SWC_UP_MASK ((uint8_t)0b00100000)
#define RC_SWC_MIDDLE_MASK ((uint8_t)0b00010000)
#define RC_SWC_DOWN_MASK ((uint8_t)0b00110000)

#define RC_SWD_MASK ((uint8_t)0b11000000)
#define RC_SWD_UP_MASK ((uint8_t)0b10000000)
#define RC_SWD_DOWN_MASK ((uint8_t)0b11000000)

typedef struct {
  uint8_t sws;
  int8_t stick_right_h;
  int8_t stick_right_v;
  int8_t stick_left_v;
  int8_t stick_left_h;
  int8_t var_a;
  uint8_t reserved0;
  uint8_t count;
} RcStateFrame;

typedef struct {
  struct16_t rpm;
  struct16_t current;
  struct32_t pulse_count;
} ActuatorHSStateFrame;

typedef struct {
  struct16_t driver_voltage;
  struct16_t driver_temp;
  int8_t motor_temp;
  uint8_t driver_state;
  uint8_t reserved0;
  uint8_t reserved1;
} ActuatorLSStateFrame;

// 0x291
typedef struct {
  uint8_t motion_mode;
  uint8_t mode_changing;
} MotionModeStateFrame;

// sensors
typedef struct {
  struct32_t left_wheel;
  struct32_t right_wheel;
} OdometryFrame;

typedef struct {
  struct16_t accel_x;
  struct16_t accel_y;
  struct16_t accel_z;
  uint8_t reserverd0;
  uint8_t count;
} ImuAccelFrame;

typedef struct {
  struct16_t gyro_x;
  struct16_t gyro_y;
  struct16_t gyro_z;
  uint8_t reserverd0;
  uint8_t count;
} ImuGyroFrame;

typedef struct {
  struct16_t yaw;
  struct16_t pitch;
  struct16_t roll;
  uint8_t reserverd0;
  uint8_t count;
} ImuEulerFrame;

typedef struct {
  uint8_t trigger_state;
  uint8_t reserved0;
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t reserved4;
  uint8_t reserved5;
  uint8_t reserved6;
} SafetyBumperFrame;

typedef struct {
  uint8_t distance[8];
} UltrasonicFrame;

typedef struct {
  struct16_t relative_distance;
  struct16_t relative_angle;
  uint8_t is_normal;
  int8_t channels[3];
} UwbFrame;

typedef struct {
  uint8_t battery_soc;
  uint8_t battery_soh;
  struct16_t voltage;
  struct16_t current;
  struct16_t temperature;
} BmsBasicFrame;

typedef struct {
  uint8_t protection_code1;
  uint8_t protection_code2;
  uint8_t protection_code3;
  uint8_t protection_code4;
  uint8_t battery_max_teperature;
  uint8_t battery_min_teperature;
  struct16_t count;
} BmsExtendedFrame;

// query/config
#define VERSION_REQUEST_VALUE ((uint8_t)0x01)
#define STEER_NEUTRAL_REQUEST_VALUE ((uint8_t)0xee)
#define STEER_NEUTRAL_RESPONSE_SUCCESS_VALUE ((uint8_t)0xee)
#define STEER_NEUTRAL_RESPONSE_FAILURE_VALUE ((uint8_t)0xff)

typedef struct {
  uint8_t request;
  uint8_t reserved0;
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t reserved4;
  uint8_t reserved5;
  uint8_t reserved6;
} VersionRequestFrame;

//typedef struct {
//  struct16_t controller_hw_version;
//  struct16_t motor_driver_hw_version;
//  struct16_t controller_sw_version;
//  struct16_t motor_driver_sw_version;
//} VersionResponseFrame;

typedef struct {
  uint8_t res0;
  uint8_t res1;
  uint8_t res2;
  uint8_t res3;
  uint8_t res4;
  uint8_t res5;
  uint8_t res6;
  uint8_t res7;
} VersionResponseFrame;


typedef struct {
  uint8_t mode;
  uint8_t reserved0;
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t reserved4;
  uint8_t reserved5;
  uint8_t reserved6;
} ControlModeConfigFrame;

typedef struct {
  uint8_t mode;
  uint8_t reserved0;
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t reserved4;
  uint8_t reserved5;
  uint8_t reserved6;
} BrakeModeConfigFrame;

typedef struct {
  uint8_t set_as_neutral;
  uint8_t reserved0;
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t reserved4;
  uint8_t reserved5;
  uint8_t reserved6;
} SteerNeutralRequestFrame;

typedef struct {
  uint8_t neutral_set_successful;
  uint8_t reserved0;
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t reserved4;
  uint8_t reserved5;
  uint8_t reserved6;
} SteerNeutralResponseFrame;

typedef struct {
  uint8_t error_clear_byte;
  uint8_t reserved0;
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t reserved4;
  uint8_t reserved5;
  uint8_t reserved6;
} StateResetConfigFrame;

typedef struct {
  struct16_t angle_5;
  struct16_t angle_6;
  struct16_t angle_7;
  struct16_t angle_8;
} MoterAngleFrame;

typedef struct {
  struct16_t speed_1;
  struct16_t speed_2;
  struct16_t speed_3;
  struct16_t speed_4;
} MoterSpeedFrame;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* AGILEX_PROTOCOL_V2_H */
