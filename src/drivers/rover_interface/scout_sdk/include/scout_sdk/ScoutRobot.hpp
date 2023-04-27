#pragma once

#include <cstdint>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>

#include "agilex_protocol/agilex_message.h"
#include "agilex_protocol/agilex_protocol_v2_parser.hpp"

namespace scoutsdk
{
struct ScoutCoreState {
	SystemStateMessage system_state;
	MotionStateMessage motion_state;
	LightStateMessage light_state;
	RcStateMessage rc_state;
};

struct ScoutActuatorState {
	ActuatorHSStateMessage actuator_hs_state[4];
	ActuatorLSStateMessage actuator_ls_state[4];
};

struct ScoutMotorState {
	MotorAngleMessage MotorAngle;
	MotorSpeedMessage MotorSpeed;
};

struct VersionResponse {
	const char *str_version_response;
	VersionResponseMessage version_response;
};


class ScoutRobot
{
public:
	ScoutRobot(ProtocolVersion protocol = ProtocolVersion::AGX_V2,
		   bool is_mini_model = false);
	~ScoutRobot();

	// do not allow copy or assignment
	ScoutRobot(const ScoutRobot &robot) = delete;
	ScoutRobot &operator=(const ScoutRobot &robot) = delete;

	// CAN connection
	void Connect(const char *const can_dev, const uint32_t can_bitrate);
	void Disconnect();

	// Send commands
	void EnableCommandMode();
	void QuerySystemVersion(const uint64_t timeout_msec);
	void SetMotionCommand(float linear_vel, float angular_vel);
	void SetLightCommand(LightMode f_mode, uint8_t f_value,
			     LightMode r_mode = LightMode::CONST_ON, uint8_t r_value = 0);
	void DisableLightControl();

	// Receive status from rover
	void CheckUpdateFromRover();

	// Get connection and rover general info
	bool GetCANConnected() const { return _can_connected; }
	const char *GetSystemVersion() const { return _version_response_msgs.str_version_response; }

	// Return rover states
	const ScoutCoreState &GetRobotState() const { return _core_state_msgs; }
	const ScoutActuatorState &GetActuatorState() const { return _actuator_state_msgs; }
	const ScoutMotorState &GetMotorState() const { return _motor_state_msgs; }

private:
	void UpdateRobotCoreState(const AgxMessage &status_msg);
	void UpdateActuatorState(const AgxMessage &status_msg);
	void UpdateMotorState(const AgxMessage &status_msg);
	int UpdateVersionResponse(const AgxMessage &status_msg);

	AgileXProtocolV2Parser _parser;

	// Feedback group 1: core state
	ScoutCoreState _core_state_msgs;

	// Feedback group 2: actuator state
	ScoutActuatorState _actuator_state_msgs;

	ScoutMotorState _motor_state_msgs;

	VersionResponse _version_response_msgs;

	char _version_response_string[9] {};

	// TX buffer
	uint8_t _tx_data[8] {};
	TxFrame _tx_frame{};

	// RX buffer
	uint8_t _rx_data[8] {};
	RxFrame _rx_frame{};

	// Communication interface
	bool _can_connected = false;
	SocketCAN *_can{nullptr};
};
} // namespace scoutsdk
