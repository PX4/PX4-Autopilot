#pragma once

#include <uORB/topics/manual_control_setpoint.h>
#include <array>

#define MSG_SIZE(N) (sizeof(uint8_t) + 4 * sizeof(float) + sizeof(uint8_t) + N * sizeof(float) + sizeof(uint32_t))

constexpr uint8_t MAX_AUX_CHANNELS = 6;

constexpr size_t MSG_MIN_SIZE = MSG_SIZE(0);
constexpr size_t MSG_MAX_SIZE = MSG_SIZE(MAX_AUX_CHANNELS);

bool deserialise_msg(const std::array<uint8_t, MSG_MAX_SIZE> &msg, const size_t &msg_size,
		     manual_control_setpoint_s &manual_control_setpoint);
