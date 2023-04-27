#include "scout_sdk/ScoutRobot.hpp"

#define MODULE_NAME "SCOUT_SDK"

#include <px4_platform_common/log.h>

namespace scoutsdk
{
ScoutRobot::ScoutRobot(ProtocolVersion protocol, bool is_mini_model)
{
	if (!is_mini_model) {
		PX4_ERR("Only Scout Mini model is supported. Aborted");

	} else {
		if (protocol == ProtocolVersion::AGX_V1) {
			PX4_ERR("Protocol AGX V1 is not supported. Aborted");

		} else if (protocol == ProtocolVersion::AGX_V2) {
			PX4_INFO("Protocol AGX V2 is supported");
		}
	}

	// Setup buffers
	_tx_frame.frame.payload = &_tx_data;
	_rx_frame.frame.payload = &_rx_data;

	_version_response_msgs.str_version_response = (const char *)_version_response_string;
}


ScoutRobot::~ScoutRobot()
{
	Disconnect();
}


void ScoutRobot::Connect(const char *const can_dev, const uint32_t can_bitrate)
{
	_can = new SocketCAN();

	if (_can->Init(can_dev, can_bitrate) == PX4_OK) { _can_connected = true; }
}


void ScoutRobot::Disconnect()
{
	if (_can_connected) { _can->Close(); }

	if (_can != nullptr) {
		delete _can;
		_can = nullptr;
	}
}


void ScoutRobot::UpdateRobotCoreState(const AgxMessage &status_msg)
{
	switch (status_msg.type) {
	case AgxMsgSystemState: {
			//   std::cout << "system status feedback received" << std::endl;
			_core_state_msgs.system_state = status_msg.body.system_state_msg;
			break;
		}

	case AgxMsgMotionState: {
			// std::cout << "motion control feedback received" << std::endl;
			_core_state_msgs.motion_state = status_msg.body.motion_state_msg;
			break;
		}

	case AgxMsgLightState: {
			// std::cout << "light control feedback received" << std::endl;
			_core_state_msgs.light_state = status_msg.body.light_state_msg;
			break;
		}

	case AgxMsgRcState: {
			// std::cout << "rc feedback received" << std::endl;
			_core_state_msgs.rc_state = status_msg.body.rc_state_msg;
			break;
		}

	default:
		break;
	}
}

void ScoutRobot::UpdateActuatorState(const AgxMessage &status_msg)
{
	switch (status_msg.type) {
	case AgxMsgActuatorHSState: {
			// std::cout << "actuator hs feedback received" << std::endl;
			_actuator_state_msgs
			.actuator_hs_state[status_msg.body.actuator_hs_state_msg.motor_id] =
				status_msg.body.actuator_hs_state_msg;
			break;
		}

	case AgxMsgActuatorLSState: {
			// std::cout << "actuator ls feedback received" << std::endl;
			_actuator_state_msgs
			.actuator_ls_state[status_msg.body.actuator_ls_state_msg.motor_id] =
				status_msg.body.actuator_ls_state_msg;
			break;
		}

	default:
		break;
	}
}


void ScoutRobot::UpdateMotorState(const AgxMessage &status_msg)
{
	switch (status_msg.type) {
	case AgxMsgMotorAngle: {
			_motor_state_msgs.MotorAngle.angle_5 = status_msg.body.motor_angle_msg.angle_5;
			_motor_state_msgs.MotorAngle.angle_6 = status_msg.body.motor_angle_msg.angle_6;
			_motor_state_msgs.MotorAngle.angle_7 = status_msg.body.motor_angle_msg.angle_7;
			_motor_state_msgs.MotorAngle.angle_8 = status_msg.body.motor_angle_msg.angle_8;
			break;
		}

	case AgxMsgMotorSpeed: {
			_motor_state_msgs.MotorSpeed.speed_1 = status_msg.body.motor_speed_msg.speed_1;
			_motor_state_msgs.MotorSpeed.speed_2 = status_msg.body.motor_speed_msg.speed_2;
			_motor_state_msgs.MotorSpeed.speed_3 = status_msg.body.motor_speed_msg.speed_3;
			_motor_state_msgs.MotorSpeed.speed_4 = status_msg.body.motor_speed_msg.speed_4;
			break;
		}

	default:
		break;
	}
}


int ScoutRobot::UpdateVersionResponse(const AgxMessage &status_msg)
{
	switch (status_msg.type) {
	case AgxMsgVersionResponse: {
			char temp_version_response[9] = {0};

			for (int i = 0; i < 8; i++) {
				uint8_t data = status_msg.body.version_str[i];

				if (data < 32 || data > 126) { data = 32; }

				snprintf(temp_version_response + i, 2, "%c", data);
			}

			strcpy(_version_response_string, temp_version_response);
			return PX4_OK;
		}

	default:
		break;
	}

	return PX4_ERROR;
}


/******************
 * PUBLIC METHODS *
******************/
void ScoutRobot::CheckUpdateFromRover()
{
	_can->ReceiveFrame(&_rx_frame);

	AgxMessage status_msg;

	if (_parser.DecodeMessage(&_rx_frame, &status_msg)) {
		UpdateRobotCoreState(status_msg);	// 0x211
		UpdateActuatorState(status_msg);	// 0x251-0x254
		UpdateMotorState(status_msg);
	}
}

void ScoutRobot::EnableCommandMode()
{
	PX4_INFO("EnableCommandMode");
	// Construct message
	AgxMessage msg;
	msg.type = AgxMsgControlModeConfig;
	msg.body.control_mode_config_msg.mode = CONTROL_MODE_CAN;

	// Encode msg to can frame and send to bus
	if (_parser.EncodeMessage(&msg, &_tx_frame)) {
		uint64_t count = 0;

		while (count < 100) {
			count++;
			_can->SendFrame(_tx_frame);
		}
	}
}

void ScoutRobot::QuerySystemVersion(const uint64_t timeout_msec)
{
	// Construct message
	AgxMessage msg;
	msg.type = AgxMsgVersionRequest;
	msg.body.version_request_msg.request = 1;

	const hrt_abstime begin = hrt_absolute_time();

	while (hrt_elapsed_time(&begin) < timeout_msec) {
		// Send request
		if (_parser.EncodeMessage(&msg, &_tx_frame)) { _can->SendFrame(_tx_frame); }

		// Receive response
		_can->ReceiveFrame(&_rx_frame);

		AgxMessage status_msg;

		if (_parser.DecodeMessage(&_rx_frame, &status_msg)) {
			if (UpdateVersionResponse(status_msg) == PX4_OK) { break; }
		}
	}
}


void ScoutRobot::SetMotionCommand(float linear_vel, float angular_vel)
{
	if (_can_connected) {
		// motion control message
		AgxMessage msg;
		msg.type = AgxMsgMotionCommand;
		msg.body.motion_command_msg.linear_velocity = linear_vel;
		msg.body.motion_command_msg.angular_velocity = -angular_vel;
		msg.body.motion_command_msg.lateral_velocity = 0.0;
		msg.body.motion_command_msg.steering_angle = 0.0;

		PX4_DEBUG("SetMotionCommand: linear_vel: %f, angular_vel: %f", static_cast<double>(linear_vel),
			  static_cast<double>(angular_vel));

		// send to can bus
		if (_parser.EncodeMessage(&msg, &_tx_frame)) { _can->SendFrame(_tx_frame); }
	}
}


void ScoutRobot::SetLightCommand(LightMode f_mode, uint8_t f_value,
				 LightMode r_mode, uint8_t r_value)
{
	if (_can_connected) {
		AgxMessage msg;
		msg.type = AgxMsgLightCommand;
		msg.body.light_command_msg.enable_cmd_ctrl = true;
		msg.body.light_command_msg.front_light.mode = f_mode;
		msg.body.light_command_msg.front_light.custom_value = f_value;
		msg.body.light_command_msg.rear_light.mode = r_mode;
		msg.body.light_command_msg.rear_light.custom_value = r_value;

		// send to can bus
		if (_parser.EncodeMessage(&msg, &_tx_frame)) { _can->SendFrame(_tx_frame); }
	}
}


void ScoutRobot::DisableLightControl()
{
	if (_can_connected) {
		AgxMessage msg;
		msg.type = AgxMsgLightCommand;

		msg.body.light_command_msg.enable_cmd_ctrl = false;

		// send to can bus
		if (_parser.EncodeMessage(&msg, &_tx_frame)) { _can->SendFrame(_tx_frame); }
	}
}
} // namespace scoutsdk
