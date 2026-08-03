#include "GL40IIProtocol.hpp"

#include <cmath>
#include <cstring>

namespace gl40ii
{

namespace
{

constexpr float KP_MIN = 0.f;
constexpr float KP_MAX = 500.f;
constexpr float KD_MIN = 0.f;
constexpr float KD_MAX = 5.f;

bool finiteInRange(float value, float minimum, float maximum)
{
	return std::isfinite(value) && value >= minimum && value <= maximum;
}

uint32_t floatToUint(float value, float minimum, float maximum, unsigned bits)
{
	const float constrained = value < minimum ? minimum : (value > maximum ? maximum : value);
	const uint32_t max_code = (1u << bits) - 1u;
	const float normalized = (constrained - minimum) / (maximum - minimum);
	return static_cast<uint32_t>(normalized * static_cast<float>(max_code));
}

float uintToFloat(uint32_t value, float minimum, float maximum, unsigned bits)
{
	const uint32_t max_code = (1u << bits) - 1u;
	return static_cast<float>(value) * (maximum - minimum) / static_cast<float>(max_code) + minimum;
}

void appendFloatLittleEndian(float value, uint8_t *destination)
{
	static_assert(sizeof(float) == 4, "GL40II protocol requires IEEE-754 float32");
	uint32_t raw = 0;
	std::memcpy(&raw, &value, sizeof(raw));
	destination[0] = static_cast<uint8_t>(raw);
	destination[1] = static_cast<uint8_t>(raw >> 8);
	destination[2] = static_cast<uint8_t>(raw >> 16);
	destination[3] = static_cast<uint8_t>(raw >> 24);
}

} // namespace

bool Limits::valid() const
{
	return std::isfinite(position) && position > 0.f
	       && std::isfinite(velocity) && velocity > 0.f
	       && std::isfinite(torque) && torque > 0.f;
}

uint16_t commandId(Mode mode, uint8_t slave_id)
{
	return static_cast<uint16_t>((static_cast<uint16_t>(mode) << 8) | slave_id);
}

uint16_t masterId(uint8_t rotor_index)
{
	return static_cast<uint16_t>(0x011u + rotor_index);
}

uint8_t busIndex(uint8_t rotor_index)
{
	return rotor_index & 1u;
}

bool packSpecial(Mode mode, uint8_t slave_id, SpecialCommand command, Frame &frame)
{
	if (slave_id == 0) {
		return false;
	}

	frame = {};
	frame.id = commandId(mode, slave_id);
	frame.dlc = 8;
	std::memset(frame.data, 0xFF, 7);
	frame.data[7] = static_cast<uint8_t>(command);
	return true;
}

bool packMit(uint8_t slave_id, const Limits &limits, float position, float velocity,
	     float kp, float kd, float torque_ff, Frame &frame)
{
	if (slave_id == 0 || !limits.valid()
	    || !finiteInRange(position, -limits.position, limits.position)
	    || !finiteInRange(velocity, -limits.velocity, limits.velocity)
	    || !finiteInRange(kp, KP_MIN, KP_MAX)
	    || !finiteInRange(kd, KD_MIN, KD_MAX)
	    || !finiteInRange(torque_ff, -limits.torque, limits.torque)) {
		return false;
	}

	const uint32_t position_raw = floatToUint(position, -limits.position, limits.position, 16);
	const uint32_t velocity_raw = floatToUint(velocity, -limits.velocity, limits.velocity, 12);
	const uint32_t kp_raw = floatToUint(kp, KP_MIN, KP_MAX, 12);
	const uint32_t kd_raw = floatToUint(kd, KD_MIN, KD_MAX, 12);
	const uint32_t torque_raw = floatToUint(torque_ff, -limits.torque, limits.torque, 12);

	frame = {};
	frame.id = commandId(Mode::MIT, slave_id);
	frame.dlc = 8;
	frame.data[0] = static_cast<uint8_t>(position_raw >> 8);
	frame.data[1] = static_cast<uint8_t>(position_raw);
	frame.data[2] = static_cast<uint8_t>(velocity_raw >> 4);
	frame.data[3] = static_cast<uint8_t>(((velocity_raw & 0xFu) << 4) | (kp_raw >> 8));
	frame.data[4] = static_cast<uint8_t>(kp_raw);
	frame.data[5] = static_cast<uint8_t>(kd_raw >> 4);
	frame.data[6] = static_cast<uint8_t>(((kd_raw & 0xFu) << 4) | (torque_raw >> 8));
	frame.data[7] = static_cast<uint8_t>(torque_raw);
	return true;
}

bool packPositionVelocity(uint8_t slave_id, float position, float velocity, Frame &frame)
{
	if (slave_id == 0 || !std::isfinite(position) || !std::isfinite(velocity)) {
		return false;
	}

	frame = {};
	frame.id = commandId(Mode::PositionVelocity, slave_id);
	frame.dlc = 8;
	appendFloatLittleEndian(position, &frame.data[0]);
	appendFloatLittleEndian(velocity, &frame.data[4]);
	return true;
}

bool packVelocity(uint8_t slave_id, float velocity, Frame &frame)
{
	if (slave_id == 0 || !std::isfinite(velocity)) {
		return false;
	}

	frame = {};
	frame.id = commandId(Mode::Velocity, slave_id);
	frame.dlc = 4;
	appendFloatLittleEndian(velocity, frame.data);
	return true;
}

bool parseFeedback(const Frame &frame, uint16_t expected_master_id, uint8_t expected_slave_id,
		   const Limits &limits, Feedback &feedback)
{
	if (!limits.valid() || frame.id != expected_master_id || frame.dlc != 8
	    || (frame.data[0] & 0x0Fu) != (expected_slave_id & 0x0Fu)) {
		return false;
	}

	const uint32_t position_raw = (static_cast<uint32_t>(frame.data[1]) << 8) | frame.data[2];
	const uint32_t velocity_raw = (static_cast<uint32_t>(frame.data[3]) << 4) | (frame.data[4] >> 4);
	const uint32_t torque_raw = (static_cast<uint32_t>(frame.data[4] & 0x0Fu) << 8) | frame.data[5];

	feedback.slave_id = frame.data[0] & 0x0Fu;
	feedback.state = frame.data[0] >> 4;
	feedback.position = uintToFloat(position_raw, -limits.position, limits.position, 16);
	feedback.velocity = uintToFloat(velocity_raw, -limits.velocity, limits.velocity, 12);
	feedback.torque = uintToFloat(torque_raw, -limits.torque, limits.torque, 12);
	feedback.driver_temperature = static_cast<int8_t>(frame.data[6]);
	feedback.motor_temperature = static_cast<int8_t>(frame.data[7]);
	return true;
}

} // namespace gl40ii
