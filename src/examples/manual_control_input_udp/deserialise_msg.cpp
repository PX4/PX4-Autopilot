#include "deserialise_msg.hpp"
#include <px4_platform_common/log.h>
#include <crc32.h>


template<typename T>
constexpr const T &clamp(const T &v, const T &lo, const T &hi)
{
	const T &t = (v < lo) ? lo : v;
	return (t > hi) ? hi : t;
}

uint32_t deserialise_u32(const std::array<uint8_t, MSG_MAX_SIZE> &msg, size_t i)
{
	const size_t offset = sizeof(uint8_t) + i * sizeof(float) + ((i < 4) ? 0 : sizeof(uint8_t));
	return (msg[offset + 0] << 24) | (msg[offset + 1] << 16) | (msg[offset + 2] << 8) | (msg[offset + 3]);
}

float deserialise_float(const std::array<uint8_t, MSG_MAX_SIZE> &msg, size_t i)
{
	const uint32_t bb = deserialise_u32(msg, i);
	return *(float *)&bb;
}

bool deserialise_msg(
	const std::array<uint8_t, MSG_MAX_SIZE> &msg,
	const size_t &msg_size,
	manual_control_setpoint_s &manual_control_setpoint
)
{
	manual_control_setpoint = {};

	if (msg_size > MSG_MAX_SIZE) {
		PX4_WARN("Message too large!");
		return false;
	}

	if (msg_size < MSG_MIN_SIZE) {
		PX4_WARN("Message too small!");
		return false;
	}

	if ((msg[0]) != 0x0F) {
		PX4_WARN("Invalid package!");
		return false;
	}

	// number of AUX channels sent
	const uint8_t naux = msg[sizeof(uint8_t) + 4 * sizeof(float)];

	if (naux > MAX_AUX_CHANNELS) {
		PX4_WARN("Too many AUX channels (%d > %d)!", naux, MAX_AUX_CHANNELS);
		return false;
	}

	const size_t msg_size_pred = MSG_SIZE(naux);

	if (msg_size != msg_size_pred) {
		PX4_WARN("Message size does not match (%lu != %lu)!", msg_size, msg_size_pred);
		return false;
	}

	const uint32_t crc_recv = deserialise_u32(msg, 4 + naux);
	const uint32_t crc = crc32part(msg.data(), msg_size - sizeof(uint32_t), 0xffffffff) ^ 0xffffffff;

	if (crc_recv != crc) {
		PX4_WARN("CRC does not match (%u != %u)!", crc_recv, crc);
		return false;
	}

	hrt_abstime ts = hrt_absolute_time();

	manual_control_setpoint.timestamp = ts;
	manual_control_setpoint.timestamp_sample = ts;

	manual_control_setpoint.roll = clamp<float>(deserialise_float(msg, 0), -1, +1);
	manual_control_setpoint.pitch = clamp<float>(deserialise_float(msg, 1), -1, +1);
	manual_control_setpoint.yaw = clamp<float>(deserialise_float(msg, 2), -1, +1);
	manual_control_setpoint.throttle = clamp<float>(deserialise_float(msg, 3), -1, +1);

	switch (naux) {
	case 6:
		manual_control_setpoint.aux6 = clamp<float>(deserialise_float(msg, 9), 0, 1);
		[[fallthrough]];

	case 5:
		manual_control_setpoint.aux5 = clamp<float>(deserialise_float(msg, 8), 0, 1);
		[[fallthrough]];

	case 4:
		manual_control_setpoint.aux4 = clamp<float>(deserialise_float(msg, 7), 0, 1);
		[[fallthrough]];

	case 3:
		manual_control_setpoint.aux3 = clamp<float>(deserialise_float(msg, 6), 0, 1);
		[[fallthrough]];

	case 2:
		manual_control_setpoint.aux2 = clamp<float>(deserialise_float(msg, 5), 0, 1);
		[[fallthrough]];

	case 1:
		manual_control_setpoint.aux1 = clamp<float>(deserialise_float(msg, 4), 0, 1);
		[[fallthrough]];

	default:
		break;
	}

	manual_control_setpoint.valid = true;
	manual_control_setpoint.data_source = 2;
	manual_control_setpoint.sticks_moving = true;

	return true;
}
