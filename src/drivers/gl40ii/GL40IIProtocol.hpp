#pragma once

#include <cstddef>
#include <cstdint>

namespace gl40ii
{

static constexpr uint8_t NUM_MOTORS = 6;
static constexpr uint32_t CAN_BITRATE = 1'000'000;

enum class Mode : uint8_t {
	MIT = 0,
	PositionVelocity = 1,
	Velocity = 2,
};

enum class SpecialCommand : uint8_t {
	ClearFault = 0xFB,
	Enable = 0xFC,
	Disable = 0xFD,
	SetZero = 0xFE,
};

struct Limits {
	float position{12.5f};
	float velocity{200.f};
	float torque{10.f};

	bool valid() const;
};

struct Frame {
	uint16_t id{0};
	uint8_t dlc{0};
	uint8_t data[8] {};
};

struct Feedback {
	uint8_t slave_id{0};
	uint8_t state{0};
	float position{0.f};
	float velocity{0.f};
	float torque{0.f};
	int8_t driver_temperature{0};
	int8_t motor_temperature{0};
};

uint16_t commandId(Mode mode, uint8_t slave_id);
uint16_t masterId(uint8_t rotor_index);
uint8_t busIndex(uint8_t rotor_index);

bool packSpecial(Mode mode, uint8_t slave_id, SpecialCommand command, Frame &frame);
bool packMit(uint8_t slave_id, const Limits &limits, float position, float velocity,
	     float kp, float kd, float torque_ff, Frame &frame);
bool packPositionVelocity(uint8_t slave_id, float position, float velocity, Frame &frame);
bool packVelocity(uint8_t slave_id, float velocity, Frame &frame);
bool parseFeedback(const Frame &frame, uint16_t expected_master_id, uint8_t expected_slave_id,
		   const Limits &limits, Feedback &feedback);

} // namespace gl40ii
