#include "scout_sdk/agilex_protocol/agilex_protocol_v2.h"
#include "scout_sdk/agilex_protocol/agilex_msg_parser_v2.h"

#include "stdio.h"
#include "string.h"

bool DecodeCanFrameV2(const CANFrame *can_frame, AgxMessage *msg)
{
	msg->type = AgxMsgUnknown;

	switch (can_frame->can_id) {
	/***************** command frame *****************/
	case CAN_MSG_MOTION_COMMAND_ID: {
			msg->type = AgxMsgMotionCommand;
			// parse frame buffer to message
			MotionCommandFrame *frame = (MotionCommandFrame *)(can_frame->payload);
			msg->body.motion_command_msg.linear_velocity =
				(int16_t)((uint16_t)(frame->linear_velocity.low_byte) |
					  (uint16_t)(frame->linear_velocity.high_byte) << 8) /
				1000.0;
			msg->body.motion_command_msg.angular_velocity =
				(int16_t)((uint16_t)(frame->angular_velocity.low_byte) |
					  (uint16_t)(frame->angular_velocity.high_byte) << 8) /
				1000.0;
			msg->body.motion_command_msg.lateral_velocity =
				(int16_t)((uint16_t)(frame->lateral_velocity.low_byte) |
					  (uint16_t)(frame->lateral_velocity.high_byte) << 8) /
				1000.0;
			msg->body.motion_command_msg.steering_angle =
				(int16_t)((uint16_t)(frame->steering_angle.low_byte) |
					  (uint16_t)(frame->steering_angle.high_byte) << 8) /
				1000.0;
			break;
		}

	case CAN_MSG_LIGHT_COMMAND_ID: {
			msg->type = AgxMsgLightCommand;
			// parse frame buffer to message
			LightCommandFrame *frame = (LightCommandFrame *)(can_frame->payload);
			msg->body.light_command_msg.enable_cmd_ctrl =
				(frame->enable_cmd_ctrl != 0) ? true : false;
			msg->body.light_command_msg.front_light.mode = frame->front_mode;
			msg->body.light_command_msg.front_light.custom_value =
				frame->front_custom;
			msg->body.light_command_msg.rear_light.mode = frame->rear_mode;
			msg->body.light_command_msg.rear_light.custom_value = frame->rear_custom;
			break;
		}

	case CAN_MSG_BRAKING_COMMAND_ID: {
			msg->type = AgxMsgBrakingCommand;
			break;
		}

	/***************** feedback frame ****************/
	case CAN_MSG_SYSTEM_STATE_ID: {
			msg->type = AgxMsgSystemState;
			SystemStateFrame *frame = (SystemStateFrame *)(can_frame->payload);
			msg->body.system_state_msg.vehicle_state = frame->vehicle_state;
			msg->body.system_state_msg.control_mode = frame->control_mode;
			msg->body.system_state_msg.battery_voltage =
				(int16_t)((uint16_t)(frame->battery_voltage.low_byte) |
					  (uint16_t)(frame->battery_voltage.high_byte) << 8) *
				0.1;
			msg->body.system_state_msg.error_code =
				(uint16_t)(frame->error_code.low_byte) |
				(uint16_t)(frame->error_code.high_byte) << 8;
			break;
		}

	case CAN_MSG_MOTION_STATE_ID: {
			msg->type = AgxMsgMotionState;
			MotionStateFrame *frame = (MotionStateFrame *)(can_frame->payload);
			msg->body.motion_state_msg.linear_velocity =
				(int16_t)((uint16_t)(frame->linear_velocity.low_byte) |
					  (uint16_t)(frame->linear_velocity.high_byte) << 8) /
				1000.0;
			msg->body.motion_state_msg.angular_velocity =
				(int16_t)((uint16_t)(frame->angular_velocity.low_byte) |
					  (uint16_t)(frame->angular_velocity.high_byte) << 8) /
				1000.0;
			msg->body.motion_state_msg.lateral_velocity =
				(int16_t)((uint16_t)(frame->lateral_velocity.low_byte) |
					  (uint16_t)(frame->lateral_velocity.high_byte) << 8) /
				1000.0;
			msg->body.motion_state_msg.steering_angle =
				(int16_t)((uint16_t)(frame->steering_angle.low_byte) |
					  (uint16_t)(frame->steering_angle.high_byte) << 8) /
				100.0;
			break;
		}

	case CAN_MSG_LIGHT_STATE_ID: {
			msg->type = AgxMsgLightState;
			LightStateFrame *frame = (LightStateFrame *)(can_frame->payload);
			msg->body.light_command_msg.enable_cmd_ctrl =
				(frame->enable_cmd_ctrl != 0) ? true : false;
			msg->body.light_command_msg.front_light.mode = frame->front_mode;
			msg->body.light_command_msg.front_light.custom_value =
				frame->front_custom;
			msg->body.light_command_msg.rear_light.mode = frame->rear_mode;
			msg->body.light_command_msg.rear_light.custom_value = frame->rear_custom;
			break;
		}

	case CAN_MSG_RC_STATE_ID: {
			msg->type = AgxMsgRcState;
			RcStateFrame *frame = (RcStateFrame *)(can_frame->payload);

			// switch a
			if ((frame->sws & RC_SWA_MASK) == RC_SWA_UP_MASK) {
				msg->body.rc_state_msg.swa = RC_SWITCH_UP;

			} else if ((frame->sws & RC_SWA_MASK) == RC_SWA_DOWN_MASK) {
				msg->body.rc_state_msg.swa = RC_SWITCH_DOWN;
			}

			// switch b
			if ((frame->sws & RC_SWB_MASK) == RC_SWB_UP_MASK) {
				msg->body.rc_state_msg.swb = RC_SWITCH_UP;

			} else if ((frame->sws & RC_SWB_MASK) == RC_SWB_MIDDLE_MASK) {
				msg->body.rc_state_msg.swb = RC_SWITCH_MIDDLE;

			} else if ((frame->sws & RC_SWB_MASK) == RC_SWB_DOWN_MASK) {
				msg->body.rc_state_msg.swb = RC_SWITCH_DOWN;
			}

			// switch c
			if ((frame->sws & RC_SWC_MASK) == RC_SWC_UP_MASK) {
				msg->body.rc_state_msg.swc = RC_SWITCH_UP;

			} else if ((frame->sws & RC_SWC_MASK) == RC_SWC_MIDDLE_MASK) {
				msg->body.rc_state_msg.swc = RC_SWITCH_MIDDLE;

			} else if ((frame->sws & RC_SWC_MASK) == RC_SWC_DOWN_MASK) {
				msg->body.rc_state_msg.swc = RC_SWITCH_DOWN;
			}

			// switch d
			if ((frame->sws & RC_SWD_MASK) == RC_SWD_UP_MASK) {
				msg->body.rc_state_msg.swd = RC_SWITCH_UP;

			} else if ((frame->sws & RC_SWD_MASK) == RC_SWD_DOWN_MASK) {
				msg->body.rc_state_msg.swd = RC_SWITCH_DOWN;
			}

			msg->body.rc_state_msg.stick_right_v = frame->stick_right_v;
			msg->body.rc_state_msg.stick_right_h = frame->stick_right_h;
			msg->body.rc_state_msg.stick_left_v = frame->stick_left_v;
			msg->body.rc_state_msg.stick_left_h = frame->stick_left_h;
			msg->body.rc_state_msg.var_a = frame->var_a;
			break;
		}

	case CAN_MSG_ACTUATOR1_HS_STATE_ID:
	case CAN_MSG_ACTUATOR2_HS_STATE_ID:
	case CAN_MSG_ACTUATOR3_HS_STATE_ID:
	case CAN_MSG_ACTUATOR4_HS_STATE_ID:
	case CAN_MSG_ACTUATOR5_HS_STATE_ID:
	case CAN_MSG_ACTUATOR6_HS_STATE_ID:
	case CAN_MSG_ACTUATOR7_HS_STATE_ID:
	case CAN_MSG_ACTUATOR8_HS_STATE_ID: {
			msg->type = AgxMsgActuatorHSState;
			ActuatorHSStateFrame *frame = (ActuatorHSStateFrame *)(can_frame->payload);
			msg->body.actuator_hs_state_msg.motor_id =
				can_frame->can_id - CAN_MSG_ACTUATOR1_HS_STATE_ID;
			msg->body.actuator_hs_state_msg.rpm =
				(int16_t)((uint16_t)(frame->rpm.low_byte) |
					  (uint16_t)(frame->rpm.high_byte) << 8);
			msg->body.actuator_hs_state_msg.current =
				((uint16_t)(frame->current.low_byte) |
				 (uint16_t)(frame->current.high_byte) << 8) * 0.1;
			msg->body.actuator_hs_state_msg.pulse_count =
				(int32_t)((uint32_t)(frame->pulse_count.lsb) |
					  (uint32_t)(frame->pulse_count.low_byte) << 8 |
					  (uint32_t)(frame->pulse_count.high_byte) << 16 |
					  (uint32_t)(frame->pulse_count.msb) << 24);
			break;
		}

	case CAN_MSG_ACTUATOR1_LS_STATE_ID:
	case CAN_MSG_ACTUATOR2_LS_STATE_ID:
	case CAN_MSG_ACTUATOR3_LS_STATE_ID:
	case CAN_MSG_ACTUATOR4_LS_STATE_ID:
	case CAN_MSG_ACTUATOR5_LS_STATE_ID:
	case CAN_MSG_ACTUATOR6_LS_STATE_ID:
	case CAN_MSG_ACTUATOR7_LS_STATE_ID:
	case CAN_MSG_ACTUATOR8_LS_STATE_ID: {
			msg->type = AgxMsgActuatorLSState;
			ActuatorLSStateFrame *frame = (ActuatorLSStateFrame *)(can_frame->payload);
			msg->body.actuator_hs_state_msg.motor_id =
				can_frame->can_id - CAN_MSG_ACTUATOR1_LS_STATE_ID;
			msg->body.actuator_ls_state_msg.driver_voltage =
				((uint16_t)(frame->driver_voltage.low_byte) |
				 (uint16_t)(frame->driver_voltage.high_byte) << 8) *
				0.1;
			msg->body.actuator_ls_state_msg.driver_temp =
				(int16_t)((uint16_t)(frame->driver_temp.low_byte) |
					  (uint16_t)(frame->driver_temp.high_byte) << 8);
			msg->body.actuator_ls_state_msg.motor_temp =
				frame->motor_temp;
			msg->body.actuator_ls_state_msg.driver_state = frame->driver_state;
			break;
		}

	case CAN_MSG_CURRENT_CTRL_MODE: {
			msg->type = AgxMsgMotionModeState;
			MotionModeStateFrame *frame = (MotionModeStateFrame *)(can_frame->payload);
			msg->body.motion_mode_state_msg.motion_mode = frame->motion_mode;
			msg->body.motion_mode_state_msg.mode_changing = frame->mode_changing;
			break;
		}

	/****************** sensor frame *****************/
	case CAN_MSG_ODOMETRY_ID: {
			msg->type = AgxMsgOdometry;
			OdometryFrame *frame = (OdometryFrame *)(can_frame->payload);
			msg->body.odometry_msg.left_wheel =
				(int32_t)((uint32_t)(frame->left_wheel.lsb) |
					  (uint32_t)(frame->left_wheel.low_byte) << 8 |
					  (uint32_t)(frame->left_wheel.high_byte) << 16 |
					  (uint32_t)(frame->left_wheel.msb) << 24);
			msg->body.odometry_msg.right_wheel =
				(int32_t)((uint32_t)(frame->right_wheel.lsb) |
					  (uint32_t)(frame->right_wheel.low_byte) << 8 |
					  (uint32_t)(frame->right_wheel.high_byte) << 16 |
					  (uint32_t)(frame->right_wheel.msb) << 24);
			break;
		}

	case CAN_MSG_IMU_ACCEL_ID: {
			msg->type = AgxMsgImuAccel;
			break;
		}

	case CAN_MSG_IMU_GYRO_ID: {
			msg->type = AgxMsgImuGyro;
			break;
		}

	case CAN_MSG_IMU_EULER_ID: {
			msg->type = AgxMsgImuEuler;
			break;
		}

	case CAN_MSG_SAFETY_BUMPER_ID: {
			msg->type = AgxMsgSafetyBumper;
			break;
		}

	case CAN_MSG_ULTRASONIC_1_ID:
	case CAN_MSG_ULTRASONIC_2_ID:
	case CAN_MSG_ULTRASONIC_3_ID:
	case CAN_MSG_ULTRASONIC_4_ID:
	case CAN_MSG_ULTRASONIC_5_ID:
	case CAN_MSG_ULTRASONIC_6_ID:
	case CAN_MSG_ULTRASONIC_7_ID:
	case CAN_MSG_ULTRASONIC_8_ID: {
			msg->type = AgxMsgUltrasonic;
			break;
		}

	case CAN_MSG_UWB_1_ID:
	case CAN_MSG_UWB_2_ID:
	case CAN_MSG_UWB_3_ID:
	case CAN_MSG_UWB_4_ID: {
			msg->type = AgxMsgUwb;
			break;
		}

	case CAN_MSG_BMS_BASIC_ID: {
			msg->type = AgxMsgBmsBasic;
			BmsBasicFrame *frame = (BmsBasicFrame *)(can_frame->payload);
			msg->body.bms_basic_msg.battery_soc = frame->battery_soc;
			msg->body.bms_basic_msg.battery_soh = frame->battery_soh;
			msg->body.bms_basic_msg.voltage =
				(int16_t)((uint16_t)(frame->voltage.low_byte) |
					  (uint16_t)(frame->voltage.high_byte) << 8) * 0.1;
			msg->body.bms_basic_msg.current =
				(int16_t)((uint16_t)(frame->current.low_byte) |
					  (uint16_t)(frame->current.high_byte) << 8) * 0.1;
			msg->body.bms_basic_msg.temperature =
				(int16_t)((uint16_t)(frame->temperature.low_byte) |
					  (uint16_t)(frame->temperature.high_byte) << 8) * 0.1;
			break;
		}

	case CAN_MSG_BMS_EXTENDED_ID: {
			msg->type = AgxMsgBmsExtended;
			break;
		}

	/*************** query/config frame **************/
	case CAN_MSG_VERSION_REQUEST_ID: {
			msg->type = AgxMsgVersionRequest;
			break;
		}

	case CAN_MSG_VERSION_RESPONSE_ID: {
			msg->type = AgxMsgVersionResponse;
			VersionResponseFrame *frame = (VersionResponseFrame *)(can_frame->payload);
			memcpy(msg->body.version_str, (uint8_t *)frame, 8);
			break;
		}

	case CAN_MSG_CTRL_MODE_CONFIG_ID: {
			msg->type = AgxMsgControlModeConfig;
			ControlModeConfigFrame *frame = (ControlModeConfigFrame *)(can_frame->payload);
			msg->body.control_mode_config_msg.mode = frame->mode;
			break;
		}

	case CAN_MSG_STEER_NEUTRAL_REQUEST_ID: {
			msg->type = AgxMsgSteerNeutralRequest;
			break;
		}

	case CAN_MSG_STEER_NEUTRAL_RESPONSE_ID: {
			msg->type = AgxMsgSteerNeutralResponse;
			break;
		}

	case CAN_MSG_STATE_RESET_CONFIG_ID: {
			msg->type = AgxMsgStateResetConfig;
			StateResetConfigFrame *frame = (StateResetConfigFrame *)(can_frame->payload);
			msg->body.state_reset_config_msg.error_clear_byte =
				frame->error_clear_byte;
			break;
		}

	case CAN_MSG_MOTOR_ANGLE_INFO: {
			msg->type = AgxMsgMotorAngle;
			MoterAngleFrame *frame = (MoterAngleFrame *)(can_frame->payload);
			msg->body.motor_angle_msg.angle_5 =
				(int16_t)((uint16_t)(frame->angle_5.low_byte) |
					  (uint16_t)(frame->angle_5.high_byte) << 8) * 0.01;
			msg->body.motor_angle_msg.angle_6 =
				(int16_t)((uint16_t)(frame->angle_6.low_byte) |
					  (uint16_t)(frame->angle_6.high_byte) << 8) * 0.01;
			msg->body.motor_angle_msg.angle_7 =
				(int16_t)((uint16_t)(frame->angle_7.low_byte) |
					  (uint16_t)(frame->angle_7.high_byte) << 8) * 0.01;
			msg->body.motor_angle_msg.angle_8 =
				(int16_t)((uint16_t)(frame->angle_8.low_byte) |
					  (uint16_t)(frame->angle_8.high_byte) << 8) * 0.01;
			break;
		}

	case CAN_MSG_MOTOR_SPEED_INFO: {
			msg->type = AgxMsgMotorSpeed;
			MoterSpeedFrame *frame = (MoterSpeedFrame *)(can_frame->payload);
			msg->body.motor_speed_msg.speed_1 =
				(int16_t)((uint16_t)(frame->speed_1.low_byte) |
					  (uint16_t)(frame->speed_1.high_byte) << 8) * 0.001;
			msg->body.motor_speed_msg.speed_2 =
				(int16_t)((uint16_t)(frame->speed_2.low_byte) |
					  (uint16_t)(frame->speed_2.high_byte) << 8) * 0.001;
			msg->body.motor_speed_msg.speed_3 =
				(int16_t)((uint16_t)(frame->speed_3.low_byte) |
					  (uint16_t)(frame->speed_3.high_byte) << 8) * 0.001;
			msg->body.motor_speed_msg.speed_4 =
				(int16_t)((uint16_t)(frame->speed_4.low_byte) |
					  (uint16_t)(frame->speed_4.high_byte) << 8) * 0.001;
			break;
		}

	default:
		break;
	}

	return true;
}

bool EncodeCanFrameV2(const AgxMessage *msg, CANFrame *can_frame)
{
	bool ret = true;

	switch (msg->type) {
	/***************** command frame *****************/
	case AgxMsgMotionCommand: {
			can_frame->can_id = CAN_MSG_MOTION_COMMAND_ID;
			can_frame->payload_size = 8;
			MotionCommandFrame frame;
			int16_t linear_cmd =
				(int16_t)(msg->body.motion_command_msg.linear_velocity * 1000);
			int16_t angular_cmd =
				(int16_t)(msg->body.motion_command_msg.angular_velocity * 1000);
			int16_t lateral_cmd =
				(int16_t)(msg->body.motion_command_msg.lateral_velocity * 1000);
			int16_t steering_cmd =
				(int16_t)(msg->body.motion_command_msg.steering_angle * 1000);
			frame.linear_velocity.high_byte = (uint8_t)(linear_cmd >> 8);
			frame.linear_velocity.low_byte = (uint8_t)(linear_cmd & 0x00ff);
			frame.angular_velocity.high_byte = (uint8_t)(angular_cmd >> 8);
			frame.angular_velocity.low_byte = (uint8_t)(angular_cmd & 0x00ff);
			frame.lateral_velocity.high_byte = (uint8_t)(lateral_cmd >> 8);
			frame.lateral_velocity.low_byte = (uint8_t)(lateral_cmd & 0x00ff);
			frame.steering_angle.high_byte = (uint8_t)(steering_cmd >> 8);
			frame.steering_angle.low_byte = (uint8_t)(steering_cmd & 0x00ff);
			memcpy(can_frame->payload, (uint8_t *)(&frame), can_frame->payload_size);
			break;
		}

	case AgxMsgLightCommand: {
			static uint8_t count = 0;
			can_frame->can_id = CAN_MSG_LIGHT_COMMAND_ID;
			can_frame->payload_size = 8;
			LightCommandFrame frame;

			if (msg->body.light_command_msg.enable_cmd_ctrl) {
				frame.enable_cmd_ctrl = LIGHT_ENABLE_CMD_CTRL;
				frame.front_mode = msg->body.light_command_msg.front_light.mode;
				frame.front_custom =
					msg->body.light_command_msg.front_light.custom_value;
				frame.rear_mode = msg->body.light_command_msg.rear_light.mode;
				frame.rear_custom = msg->body.light_command_msg.rear_light.custom_value;

			} else {
				frame.enable_cmd_ctrl = LIGHT_DISABLE_CMD_CTRL;
				frame.front_mode = 0;
				frame.front_custom = 0;
				frame.rear_mode = 0;
				frame.rear_custom = 0;
			}

			frame.reserved0 = 0;
			frame.reserved1 = 0;
			frame.count = count++;
			memcpy(can_frame->payload, (uint8_t *)(&frame), can_frame->payload_size);
			break;
		}

	case AgxMsgBrakingCommand: {
			static uint8_t count = 0;
			can_frame->can_id = CAN_MSG_BRAKING_COMMAND_ID;
			can_frame->payload_size = 2;
			BrakingCommandFrame frame;
			frame.enable_brake = msg->body.braking_command_msg.enable_braking;
			frame.count = count++;
			memcpy(can_frame->payload, (uint8_t *)(&frame), can_frame->payload_size);
			break;
		}

	case AgxMsgSetMotionModeCommand: {
			can_frame->can_id = CAN_MSG_SET_MOTION_MODE_ID;
			can_frame->payload_size = 8;
			SetMotionModeFrame frame;
			frame.motion_mode = msg->body.motion_mode_msg.motion_mode;
			frame.reserved0 = 0;
			frame.reserved1 = 0;
			frame.reserved2 = 0;
			frame.reserved3 = 0;
			frame.reserved4 = 0;
			frame.reserved5 = 0;
			frame.reserved6 = 0;
			memcpy(can_frame->payload, (uint8_t *)(&frame), can_frame->payload_size);
			break;
		}

	/*************** query/config frame **************/
	case AgxMsgVersionRequest: {
			can_frame->can_id = CAN_MSG_VERSION_REQUEST_ID;
			can_frame->payload_size = 8;
			VersionRequestFrame frame;
			frame.request = msg->body.version_request_msg.request;
			frame.reserved0 = 0;
			frame.reserved1 = 0;
			frame.reserved2 = 0;
			frame.reserved3 = 0;
			frame.reserved4 = 0;
			frame.reserved5 = 0;
			frame.reserved6 = 0;
			memcpy(can_frame->payload, (uint8_t *)(&frame), can_frame->payload_size);
			break;
		}

	case AgxMsgControlModeConfig: {
			can_frame->can_id = CAN_MSG_CTRL_MODE_CONFIG_ID;
			can_frame->payload_size = 8;
			ControlModeConfigFrame frame;
			frame.mode = msg->body.control_mode_config_msg.mode;
			frame.reserved0 = 0;
			frame.reserved1 = 0;
			frame.reserved2 = 0;
			frame.reserved3 = 0;
			frame.reserved4 = 0;
			frame.reserved5 = 0;
			frame.reserved6 = 0;
			memcpy(can_frame->payload, (uint8_t *)(&frame), can_frame->payload_size);
			break;
		}

	case AgxMsgBrakeModeConfig: {
			can_frame->can_id = CAN_MSG_BRAKING_COMMAND_ID;
			can_frame->payload_size = 8;
			BrakeModeConfigFrame frame;
			frame.mode = msg->body.control_mode_config_msg.mode;
			frame.reserved0 = 0;
			frame.reserved1 = 0;
			frame.reserved2 = 0;
			frame.reserved3 = 0;
			frame.reserved4 = 0;
			frame.reserved5 = 0;
			frame.reserved6 = 0;
			memcpy(can_frame->payload, (uint8_t *)(&frame), can_frame->payload_size);
			break;
		}

	default: {
			ret = false;
			break;
		}
	}

	return ret;
}

uint8_t CalcCanFrameChecksumV2(uint16_t id, uint8_t *data, uint8_t dlc)
{
	uint8_t checksum = 0x00;
	checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;

	for (int i = 0; i < (dlc - 1); ++i) { checksum += data[i]; }

	return checksum;
}
