/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "gnss_failover.h"

#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>

using namespace mavsdk;

namespace
{

// A receiver whose MAVLink message is older than this has no fix for the injection preconditions
constexpr int64_t RECEIVER_TIMEOUT_US = 2'000'000;

constexpr float ODOMETRY_RATE_HZ = 30.f;
constexpr float ESTIMATOR_STATUS_RATE_HZ = 10.f;
constexpr float GPS_RATE_HZ = 5.f;

// ESTIMATOR_STATUS flags of a position good enough for position control
constexpr uint16_t POSITION_OK_FLAGS = ESTIMATOR_POS_HORIZ_REL | ESTIMATOR_POS_HORIZ_ABS | ESTIMATOR_POS_VERT_ABS;

float wrap_pi(float angle)
{
	while (angle > static_cast<float>(M_PI)) { angle -= 2.f * static_cast<float>(M_PI); }

	while (angle < -static_cast<float>(M_PI)) { angle += 2.f * static_cast<float>(M_PI); }

	return angle;
}

const char *type_name(Failure::FailureType type)
{
	switch (type) {
	case Failure::FailureType::Ok: return "ok";

	case Failure::FailureType::Off: return "off";

	case Failure::FailureType::Stuck: return "stuck";

	case Failure::FailureType::Garbage: return "garbage";

	case Failure::FailureType::Wrong: return "wrong";

	case Failure::FailureType::Slow: return "slow";

	case Failure::FailureType::Delayed: return "delayed";

	case Failure::FailureType::Intermittent: return "intermittent";
	}

	return "unknown";
}

bool position_controlled(Telemetry::FlightMode mode)
{
	return mode == Telemetry::FlightMode::Posctl || mode == Telemetry::FlightMode::Hold
	       || mode == Telemetry::FlightMode::Mission || mode == Telemetry::FlightMode::ReturnToLaunch;
}

} // namespace

std::ostream &operator<<(std::ostream &str, const GnssFailover::Injection &injection)
{
	str << "gps " << type_name(injection.type) << " instance " << injection.instance;

	if (injection.type == Failure::FailureType::Wrong) {
		const GnssFailover::WrongPayload &wrong = injection.wrong;
		str << " (fix " << wrong.fix_type << ", eph " << wrong.eph << ", epv " << wrong.epv << ", sacc " << wrong.speed_accuracy
		    << ", sats " << wrong.satellites << ", jamming " << wrong.jamming_state << ", spoofing " << wrong.spoofing_state
		    << ")";

	} else if (injection.type == Failure::FailureType::Slow) {
		str << " (one sample in " << injection.slow_divider << ")";
	}

	return str;
}

GnssFailover::GnssFailover(std::shared_ptr<System> system, bool subscribe_events) :
	_system(system),
	_failure(new Failure(system)),
	_param(new Param(system)),
	_telemetry(new Telemetry(system)),
	_passthrough(new MavlinkPassthrough(system))
{
	_odometry_handle = _passthrough->subscribe_message(MAVLINK_MSG_ID_ODOMETRY, [this](const mavlink_message_t &message) {
		handle_odometry(message);
	});
	_gps_raw_int_handle = _passthrough->subscribe_message(MAVLINK_MSG_ID_GPS_RAW_INT,
	[this](const mavlink_message_t &message) {
		handle_gps_raw_int(message);
	});
	_gps2_raw_handle = _passthrough->subscribe_message(MAVLINK_MSG_ID_GPS2_RAW, [this](const mavlink_message_t &message) {
		handle_gps2_raw(message);
	});

	_estimator_status_handle = _passthrough->subscribe_message(MAVLINK_MSG_ID_ESTIMATOR_STATUS,
	[this](const mavlink_message_t &message) {
		handle_estimator_status(message);
	});

	_flight_mode_handle = _telemetry->subscribe_flight_mode([this](Telemetry::FlightMode mode) {
		bool changed = false;
		{
			std::lock_guard<std::mutex> lock(_mutex);

			if (_mode_changes.empty() || (_mode_changes.back().mode != mode)) {
				_mode_changes.push_back({_odometry[0].time_us, mode});
				changed = true;
			}
		}

		if (changed) {
			std::ostringstream text;
			text << "mode " << mode;
			record(text.str());
		}
	});

	if (subscribe_events) {
		_events.reset(new Events(system));
		_events_handle = _events->subscribe_events([this](const Events::Event & event) { on_event(event); });
	}
}

GnssFailover::~GnssFailover()
{
	if (_events) {
		_events->unsubscribe_events(_events_handle);
	}

	_telemetry->unsubscribe_flight_mode(_flight_mode_handle);
	_passthrough->unsubscribe_message(MAVLINK_MSG_ID_ESTIMATOR_STATUS, _estimator_status_handle);
	_passthrough->unsubscribe_message(MAVLINK_MSG_ID_GPS2_RAW, _gps2_raw_handle);
	_passthrough->unsubscribe_message(MAVLINK_MSG_ID_GPS_RAW_INT, _gps_raw_int_handle);
	_passthrough->unsubscribe_message(MAVLINK_MSG_ID_ODOMETRY, _odometry_handle);
}

void GnssFailover::set_message_interval(uint16_t message_id, float rate_hz)
{
	MavlinkPassthrough::CommandLong command{};
	command.target_sysid = _passthrough->get_target_sysid();
	command.target_compid = _passthrough->get_target_compid();
	command.command = MAV_CMD_SET_MESSAGE_INTERVAL;
	command.param1 = static_cast<float>(message_id);
	command.param2 = 1e6f / rate_hz;
	command.param3 = NAN;
	command.param4 = NAN;
	command.param5 = NAN;
	command.param6 = NAN;
	command.param7 = 0.f;
	_passthrough->send_command_long(command);
}

bool GnssFailover::request_streams()
{
	set_message_interval(MAVLINK_MSG_ID_ODOMETRY, ODOMETRY_RATE_HZ);
	set_message_interval(MAVLINK_MSG_ID_ESTIMATOR_STATUS, ESTIMATOR_STATUS_RATE_HZ);
	set_message_interval(MAVLINK_MSG_ID_GPS_RAW_INT, GPS_RATE_HZ);
	set_message_interval(MAVLINK_MSG_ID_GPS2_RAW, GPS_RATE_HZ);

	return wait_until([this]() { return vehicle_time_us() > 0; }, 10.);
}

void GnssFailover::handle_odometry(const mavlink_message_t &message)
{
	mavlink_odometry_t odometry;
	mavlink_msg_odometry_decode(&message, &odometry);

	OdometrySample sample{};
	sample.time_us = static_cast<int64_t>(odometry.time_usec);
	sample.position = {odometry.x, odometry.y, odometry.z};
	const float *q = odometry.q;
	sample.yaw = atan2f(2.f * (q[0] * q[3] + q[1] * q[2]), 1.f - 2.f * (q[2] * q[2] + q[3] * q[3]));
	sample.reset_counter = odometry.reset_counter;

	std::string reset_text;
	{
		std::lock_guard<std::mutex> lock(_mutex);

		if (_odometry_samples > 0 && sample.time_us < _odometry[0].time_us) {
			// The vehicle rebooted: start over
			_odometry_samples = 0;
		}

		if ((_odometry_samples > 0) && (sample.reset_counter != _odometry[0].reset_counter)) {
			const OdometrySample &previous = _odometry[0];

			Reset reset{};
			reset.vehicle_time_us = sample.time_us;
			reset.counter_increase = static_cast<uint8_t>(sample.reset_counter - previous.reset_counter);

			// The motion between the two samples before the reset stands in for the motion across it
			for (int i = 0; i < 3; i++) {
				const float motion = (_odometry_samples > 1) ? (previous.position[i] - _odometry[1].position[i]) : 0.f;
				reset.position_jump[i] = sample.position[i] - previous.position[i] - motion;
			}

			const float turn = (_odometry_samples > 1) ? wrap_pi(previous.yaw - _odometry[1].yaw) : 0.f;
			reset.yaw_jump = wrap_pi(sample.yaw - previous.yaw - turn);

			_reset_count += reset.counter_increase;
			_resets.push_back(reset);

			std::ostringstream text;
			text << "estimator reset +" << reset.counter_increase << " position jump N " << reset.position_jump[0]
			     << " E " << reset.position_jump[1] << " D " << reset.position_jump[2] << " yaw jump " << reset.yaw_jump;
			reset_text = text.str();
		}

		_odometry[2] = _odometry[1];
		_odometry[1] = _odometry[0];
		_odometry[0] = sample;
		_odometry_samples++;
	}

	if (!reset_text.empty()) {
		record(reset_text);
	}
}

void GnssFailover::handle_estimator_status(const mavlink_message_t &message)
{
	mavlink_estimator_status_t status;
	mavlink_msg_estimator_status_decode(&message, &status);

	const bool position_ok = (status.flags & POSITION_OK_FLAGS) == POSITION_OK_FLAGS;
	bool changed = false;
	{
		std::lock_guard<std::mutex> lock(_mutex);

		if (_position_changes.empty() || (_position_changes.back().position_ok != position_ok)) {
			_position_changes.push_back({_odometry[0].time_us, position_ok});
			changed = true;
		}
	}

	if (changed) {
		std::ostringstream text;
		text << (position_ok ? "position valid" : "position not valid") << ", estimator flags 0x" << std::hex << status.flags;
		record(text.str());
	}
}

void GnssFailover::handle_gps_raw_int(const mavlink_message_t &message)
{
	mavlink_gps_raw_int_t gps;
	mavlink_msg_gps_raw_int_decode(&message, &gps);

	std::lock_guard<std::mutex> lock(_mutex);
	Receiver &receiver = _receivers[0];
	receiver.vehicle_time_us = _odometry[0].time_us;
	receiver.fix_type = gps.fix_type;
	receiver.satellites = gps.satellites_visible;
	receiver.eph = (gps.h_acc > 0) ? 1e-3f * gps.h_acc : 0.f;
}

void GnssFailover::handle_gps2_raw(const mavlink_message_t &message)
{
	mavlink_gps2_raw_t gps;
	mavlink_msg_gps2_raw_decode(&message, &gps);

	std::lock_guard<std::mutex> lock(_mutex);
	Receiver &receiver = _receivers[1];
	receiver.vehicle_time_us = _odometry[0].time_us;
	receiver.fix_type = gps.fix_type;
	receiver.satellites = gps.satellites_visible;
	receiver.eph = (gps.h_acc > 0) ? 1e-3f * gps.h_acc : 0.f;
}

std::string GnssFailover::injection_refusal(bool require_flying)
{
	const std::pair<Param::Result, int32_t> enabled = _param->get_param_int("SYS_FAILURE_EN");

	if (enabled.first != Param::Result::Success || enabled.second != 1) {
		return "SYS_FAILURE_EN is not set";
	}

	const int64_t now = vehicle_time_us();

	for (int i = 0; i < 2; i++) {
		const Receiver gps = receiver(i);

		if ((gps.vehicle_time_us == 0) || (now > gps.vehicle_time_us + RECEIVER_TIMEOUT_US)) {
			return std::string(i == 0 ? "GPS_RAW_INT" : "GPS2_RAW") + " receiver is silent";
		}

		if (gps.fix_type < GPS_FIX_TYPE_3D_FIX) {
			return std::string(i == 0 ? "GPS_RAW_INT" : "GPS2_RAW") + " receiver has no 3D fix";
		}
	}

	if (require_flying) {
		if (!_telemetry->armed()) {
			return "not armed";
		}

		if (!_telemetry->in_air()) {
			return "not in the air";
		}

		const Telemetry::FlightMode mode = _telemetry->flight_mode();

		if (!position_controlled(mode)) {
			std::ostringstream text;
			text << "mode " << mode << " is not position controlled";
			return text.str();
		}
	}

	return {};
}

bool GnssFailover::set_param_int(const std::string &name, int32_t value)
{
	{
		std::lock_guard<std::mutex> lock(_mutex);

		if (_saved_int_params.count(name) == 0) {
			const std::pair<Param::Result, int32_t> current = _param->get_param_int(name);

			if (current.first != Param::Result::Success) {
				return false;
			}

			_saved_int_params[name] = current.second;
		}
	}

	return _param->set_param_int(name, value) == Param::Result::Success;
}

bool GnssFailover::set_param_float(const std::string &name, float value)
{
	{
		std::lock_guard<std::mutex> lock(_mutex);

		if (_saved_float_params.count(name) == 0) {
			const std::pair<Param::Result, float> current = _param->get_param_float(name);

			if (current.first != Param::Result::Success) {
				return false;
			}

			_saved_float_params[name] = current.second;
		}
	}

	return _param->set_param_float(name, value) == Param::Result::Success;
}

bool GnssFailover::inject(const Injection &injection)
{
	bool parameters_set = true;

	if (injection.type == Failure::FailureType::Wrong) {
		const WrongPayload &wrong = injection.wrong;
		parameters_set = set_param_int("SYS_FAIL_GPS_WRG", wrong.fix_type)
				 && set_param_float("SYS_FAIL_GPS_EPH", wrong.eph)
				 && set_param_float("SYS_FAIL_GPS_EPV", wrong.epv)
				 && set_param_float("SYS_FAIL_GPS_SAC", wrong.speed_accuracy)
				 && set_param_int("SYS_FAIL_GPS_SAT", wrong.satellites)
				 && set_param_int("SYS_FAIL_GPS_JAM", wrong.jamming_state)
				 && set_param_int("SYS_FAIL_GPS_SPF", wrong.spoofing_state);

	} else if (injection.type == Failure::FailureType::Slow) {
		parameters_set = set_param_int("SYS_FAIL_GPS_DIV", injection.slow_divider);
	}

	std::ostringstream text;
	text << injection;

	if (!parameters_set) {
		record(text.str() + ": setting SYS_FAIL_GPS_* failed");
		return false;
	}

	const Failure::Result result = _failure->inject(Failure::FailureUnit::SensorGps, injection.type, injection.instance);
	text << ": " << result;
	record(text.str());
	return result == Failure::Result::Success;
}

void GnssFailover::inject_without_ack(Failure::FailureType type, int instance)
{
	const uint8_t target_sysid = _passthrough->get_target_sysid();
	const uint8_t target_compid = _passthrough->get_target_compid();

	_passthrough->queue_message([&](MavlinkAddress address, uint8_t channel) {
		mavlink_message_t message;
		mavlink_msg_command_long_pack_chan(address.system_id, address.component_id, channel, &message,
						   target_sysid, target_compid, MAV_CMD_INJECT_FAILURE, 0,
						   FAILURE_UNIT_SENSOR_GPS, static_cast<float>(type), static_cast<float>(instance), NAN, NAN, NAN, NAN);
		return message;
	});

	Injection injection{};
	injection.type = type;
	injection.instance = instance;
	std::ostringstream text;
	text << injection << ": sent";
	record(text.str());
}

bool GnssFailover::clear(int instance)
{
	Injection injection{};
	injection.type = Failure::FailureType::Ok;
	injection.instance = instance;
	return inject(injection);
}

bool GnssFailover::restore_parameters()
{
	std::map<std::string, int32_t> int_params;
	std::map<std::string, float> float_params;
	{
		std::lock_guard<std::mutex> lock(_mutex);
		int_params.swap(_saved_int_params);
		float_params.swap(_saved_float_params);
	}

	bool restored = true;

	for (const auto &param : int_params) {
		restored = (_param->set_param_int(param.first, param.second) == Param::Result::Success) && restored;
	}

	for (const auto &param : float_params) {
		restored = (_param->set_param_float(param.first, param.second) == Param::Result::Success) && restored;
	}

	return restored;
}

void GnssFailover::on_event(const Events::Event &event)
{
	EventRecord event_record{};
	event_record.name = event.event_namespace + "/" + event.event_name;
	event_record.message = event.message;
	{
		std::lock_guard<std::mutex> lock(_mutex);
		event_record.vehicle_time_us = _odometry[0].time_us;
		_event_records.push_back(event_record);
	}

	record("event " + event_record.name + ": " + event_record.message);
}

int64_t GnssFailover::vehicle_time_us() const
{
	std::lock_guard<std::mutex> lock(_mutex);
	return _odometry[0].time_us;
}

bool GnssFailover::wait_until(const std::function<bool()> &condition, double timeout_s) const
{
	const int64_t timeout_us = static_cast<int64_t>(timeout_s * 1e6);
	const int64_t vehicle_start_us = vehicle_time_us();
	const auto host_start = std::chrono::steady_clock::now();

	while (!condition()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(5));

		const int64_t vehicle_now_us = vehicle_time_us();

		if ((vehicle_start_us > 0) && (vehicle_now_us > 0)) {
			if (vehicle_now_us - vehicle_start_us > timeout_us) {
				return false;
			}

		} else if (std::chrono::steady_clock::now() - host_start > std::chrono::microseconds(timeout_us)) {
			return false;
		}
	}

	return true;
}

void GnssFailover::sleep(double seconds) const
{
	wait_until([]() { return false; }, seconds);
}

unsigned GnssFailover::reset_count() const
{
	std::lock_guard<std::mutex> lock(_mutex);
	return _reset_count;
}

std::vector<GnssFailover::Reset> GnssFailover::resets() const
{
	std::lock_guard<std::mutex> lock(_mutex);
	return _resets;
}

GnssFailover::Receiver GnssFailover::receiver(int index) const
{
	std::lock_guard<std::mutex> lock(_mutex);
	return (index >= 0 && index < 2) ? _receivers[index] : Receiver{};
}

std::vector<GnssFailover::PositionChange> GnssFailover::position_changes() const
{
	std::lock_guard<std::mutex> lock(_mutex);
	return _position_changes;
}

bool GnssFailover::position_ok() const
{
	std::lock_guard<std::mutex> lock(_mutex);
	return !_position_changes.empty() && _position_changes.back().position_ok;
}

std::vector<GnssFailover::ModeChange> GnssFailover::mode_changes() const
{
	std::lock_guard<std::mutex> lock(_mutex);
	return _mode_changes;
}

std::vector<GnssFailover::EventRecord> GnssFailover::events() const
{
	std::lock_guard<std::mutex> lock(_mutex);
	return _event_records;
}

bool GnssFailover::position_ok_since(int64_t since_us) const
{
	std::lock_guard<std::mutex> lock(_mutex);

	bool known = false;
	bool ok = true;

	for (const PositionChange &change : _position_changes) {
		if (change.vehicle_time_us <= since_us) {
			// the state at since_us
			known = true;
			ok = change.position_ok;

		} else {
			known = true;
			ok = ok && change.position_ok;
		}
	}

	return known && ok;
}

std::vector<Telemetry::FlightMode> GnssFailover::modes_since(int64_t since_us) const
{
	std::lock_guard<std::mutex> lock(_mutex);

	std::vector<Telemetry::FlightMode> modes;

	for (const ModeChange &change : _mode_changes) {
		if (change.vehicle_time_us <= since_us) {
			modes.assign(1, change.mode);

		} else {
			modes.push_back(change.mode);
		}
	}

	return modes;
}

void GnssFailover::record(const std::string &what)
{
	const int64_t wall_us = std::chrono::duration_cast<std::chrono::microseconds>
				(std::chrono::system_clock::now().time_since_epoch()).count();
	const int64_t vehicle_us = vehicle_time_us();

	std::lock_guard<std::mutex> lock(_record_mutex);

	if (_record) {
		std::string quoted = what;

		for (char &c : quoted) {
			if (c == '"') { c = '\''; }
		}

		*_record << wall_us << "," << vehicle_us << ",\"" << quoted << "\"" << std::endl;
	}
}
