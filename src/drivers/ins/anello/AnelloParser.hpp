// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#pragma once

#include <cmath>
#include <cstdint>
#include <matrix/matrix/math.hpp>
#include <mavlink.h>

namespace anello
{
/** MAVLink decoder with private parser state, no GCS channel.
 * Timestamps are local receipt times: the INS clock is not synchronised.
 */
class Parser
{
public:
	enum class Update { None, Attitude, Position, Heartbeat, CommandAck };
	static constexpr uint64_t AttitudeTimeout = 250000;
	static constexpr uint64_t PositionTimeout = 1000000;

	Update parse(uint8_t byte, uint64_t now)
	{
		mavlink_message_t message{};
		mavlink_status_t status{};

		if (mavlink_frame_char_buffer(&_rx_message, &_rx_status, byte, &message, &status) != MAVLINK_FRAMING_OK) {
			return Update::None;
		}

		if (message.sysid == 0 || message.compid == 0) {
			return Update::None;
		}

		if (_source_locked && (message.sysid != _sysid || message.compid != _compid)) {
			return Update::None;
		}

		const Update update = handle(message, now);
		_mavlink1 = message.magic == MAVLINK_STX_MAVLINK1;

		if (update != Update::None && !_source_locked) {
			_sysid = message.sysid;
			_compid = message.compid;
			_source_locked = true;
		}

		return update;
	}

	bool attitudeFresh(uint64_t now) const { return _have_attitude && now - attitude_time <= AttitudeTimeout; }
	bool positionFresh(uint64_t now) const { return _have_position && now - position_time <= PositionTimeout; }
	uint8_t sourceSystem() const { return _sysid; }
	uint8_t sourceComponent() const { return _compid; }
	bool mavlink1() const { return _mavlink1; }
	uint32_t boot_time_ms{0};
	mavlink_command_ack_t ack{};
	matrix::Quatf attitude{};
	double latitude{0};
	double longitude{0};
	float altitude{0}; // AMSL, metres
	matrix::Vector3f velocity{}; // NED, metres/second
	uint64_t attitude_time{0};
	uint64_t position_time{0};

private:
	Update handle(const mavlink_message_t &message, uint64_t now)
	{
		matrix::Quatf q;

		switch (message.msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT: {
				mavlink_heartbeat_t heartbeat{};
				mavlink_msg_heartbeat_decode(&message, &heartbeat);
				// A GCS forwarded onto this dedicated INS link is not the sensor.
				return heartbeat.type != MAV_TYPE_GCS && message.sysid != 0 && message.compid != 0
				       ? Update::Heartbeat : Update::None;
			}

		case MAVLINK_MSG_ID_COMMAND_ACK:
			if (!_source_locked) {
				return Update::None;
			}

			mavlink_msg_command_ack_decode(&message, &ack);
			return Update::CommandAck;

		case MAVLINK_MSG_ID_ATTITUDE: {
				mavlink_attitude_t p{};
				mavlink_msg_attitude_decode(&message, &p);

				if (!std::isfinite(p.roll) || !std::isfinite(p.pitch) || !std::isfinite(p.yaw)
				    || !ratesFinite(p.rollspeed, p.pitchspeed, p.yawspeed)
				    || (_have_quaternion && now - _quaternion_time <= AttitudeTimeout)) {
					return Update::None;
				}

				q = matrix::Quatf(matrix::Eulerf(p.roll, p.pitch, p.yaw));
				boot_time_ms = p.time_boot_ms;
				break;
			}

		case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {
				mavlink_attitude_quaternion_t p{};
				mavlink_msg_attitude_quaternion_decode(&message, &p);
				q = matrix::Quatf(p.q1, p.q2, p.q3, p.q4);
				const float norm = q.norm();

				// Reject malformed rotations; tolerate sender rounding.
				if (!std::isfinite(norm) || norm < 0.5f || norm > 1.5f
				    || !ratesFinite(p.rollspeed, p.pitchspeed, p.yawspeed)) {
					return Update::None;
				}

				q.normalize();
				boot_time_ms = p.time_boot_ms;
				_quaternion_time = now;
				_have_quaternion = true;
				break;
			}

		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
				mavlink_global_position_int_t p{};
				mavlink_msg_global_position_int_decode(&message, &p);

				if (p.lat < -900000000 || p.lat > 900000000 || p.lon < -1800000000 || p.lon > 1800000000) {
					return Update::None;
				}

				latitude = p.lat * 1e-7;
				longitude = p.lon * 1e-7;
				altitude = p.alt * 1e-3f;
				velocity = matrix::Vector3f(p.vx, p.vy, p.vz) * 1e-2f;
				boot_time_ms = p.time_boot_ms;
				position_time = now;
				_have_position = true;
				return Update::Position;
			}

		default:
			return Update::None;
		}

		attitude = q;
		attitude_time = now;
		_have_attitude = true;
		return Update::Attitude;
	}

	static bool ratesFinite(float x, float y, float z)
	{
		return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
	}

	mavlink_message_t _rx_message{};
	mavlink_status_t _rx_status{};
	bool _source_locked{false};
	bool _mavlink1{false};
	uint8_t _sysid{0};
	uint8_t _compid{0};
	bool _have_attitude{false};
	bool _have_position{false};
	bool _have_quaternion{false};
	uint64_t _quaternion_time{0};
};
} // namespace anello
