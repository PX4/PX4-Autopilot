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

/**
 * @file EscLogReader.hpp
 *
 * Shared ULog input of the MotorFailureDetector host tools. Reads esc_status, actuator_motors and
 * vehicle_status by field name from the log's own format, so one binary handles logs of any PX4
 * message version, and hands out ESC samples shaped the way escCheck.cpp feeds the detector.
 */

#pragma once

#include "MotorFailureDetector.hpp"

#include <ulog_cpp/data_container.hpp>
#include <ulog_cpp/exception.hpp>
#include <ulog_cpp/reader.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cfloat>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>

/** One ESC slot's sample in an esc_status record, as escCheck would feed it to the detector. */
struct EscSample {
	int slot;               ///< ESC slot, the detector index
	int motor;              ///< 0-based motor index (actuator_function - MOTOR1), selects the command
	uint64_t timestamp_us;  ///< the ESC's own sample time
	bool fresh;             ///< first record carrying this sample (esc_status repeats ESCs that did not report)
	float command; ///< latest actuator_motors control of that motor, NAN if it is reversible (like escCheck)
	float current;          ///< [A]
};

class EscLogReader : public ulog_cpp::DataContainer
{
public:
	static constexpr int kMaxSlots = MotorFailureDetector::kMaxMotors;

	EscLogReader() : ulog_cpp::DataContainer(ulog_cpp::DataContainer::StorageConfig::Header) {}

	void addLoggedMessage(const ulog_cpp::AddLoggedMessage &msg) override
	{
		ulog_cpp::DataContainer::addLoggedMessage(msg);

		if (msg.multiId() != 0) { return; }

		auto it = messageFormats().find(msg.messageName());

		if (it == messageFormats().end()) { return; }

		if (msg.messageName() == "esc_status") {
			_esc_id = msg.msgId();
			_esc_fmt = it->second;
			_esc_array_f = _esc_fmt->field("esc");
			_esc_slots = std::min(_esc_array_f->arrayLength(), kMaxSlots);
			_esc_timestamp_f = _esc_fmt->field("timestamp");
			_esc_count_f = _esc_fmt->field("esc_count");
			_esc_function_f = _esc_array_f->nestedField("actuator_function");
			_esc_current_f = _esc_array_f->nestedField("esc_current");
			_esc_slot_timestamp_f = _esc_array_f->nestedField("timestamp");

		} else if (msg.messageName() == "actuator_motors") {
			_actuator_id = msg.msgId();
			_actuator_fmt = it->second;
			_actuator_control_f = _actuator_fmt->field("control");
			_control_channels = std::min(_actuator_control_f->arrayLength(), kMaxSlots);
			_actuator_reversible_f = _actuator_fmt->field("reversible_flags");

		} else if (msg.messageName() == "vehicle_status") {
			_status_id = msg.msgId();
			_status_fmt = it->second;
			_status_arming_f = _status_fmt->field("arming_state");
			_have_status = true;
		}
	}

	void data(const ulog_cpp::Data &record) override
	{
		const uint16_t id = record.msgId();

		if (id == _status_id) {
			const bool armed = ulog_cpp::TypedDataView(record, *_status_fmt)[_status_arming_f].as<int>() == kArmed;

			if (_armed && !armed) { disarmed(); }

			_armed = armed;
			return;
		}

		if (id == _actuator_id) {
			ulog_cpp::TypedDataView view(record, *_actuator_fmt);
			const uint32_t reversible_mask = view[_actuator_reversible_f].as<uint32_t>();
			const auto control = view[_actuator_control_f];

			for (int m = 0; m < _control_channels; ++m) {
				_command_latest[m] = control[m].as<float>();
				_reversible_latest[m] = (reversible_mask >> m) & 1;
			}

			return;
		}

		if (id != _esc_id) { return; }

		ulog_cpp::TypedDataView view(record, *_esc_fmt);

		if (view[_esc_count_f].as<int>() <= 0) { return; } // escCheck skips esc_status with esc_count <= 0

		if (_have_status && !_armed) { return; } // escCheck only runs the detector while armed

		std::array<EscSample, kMaxSlots> samples;
		int count = 0;
		const auto esc_arr = view[_esc_array_f];

		for (int slot = 0; slot < _esc_slots; ++slot) {
			const auto esc = esc_arr[slot];
			const int function = esc[_esc_function_f].as<int>();

			if (function < kMotor1 || function - kMotor1 >= kMaxSlots) { continue; }

			const float current = esc[_esc_current_f].as<float>();

			// Like escCheck: an ESC is not judged until it has reported current once.
			if (current > FLT_EPSILON) { _has_reported_current[slot] = true; }

			if (!_has_reported_current[slot]) { continue; }

			const int motor = function - kMotor1;
			const uint64_t timestamp_us = esc[_esc_slot_timestamp_f].as<uint64_t>();
			const float command = _reversible_latest[motor] ? NAN : _command_latest[motor];
			samples[count++] = {slot, motor, timestamp_us, timestamp_us != _last_sample_us[slot], command, current};
			_last_sample_us[slot] = timestamp_us;
		}

		escRecord(view[_esc_timestamp_f].as<uint64_t>(), samples.data(), count);
	}

	bool haveStatus() const { return _have_status; }

protected:
	/** An armed esc_status record: its publication time and the samples of every ESC escCheck would feed. */
	virtual void escRecord(uint64_t record_us, const EscSample *samples, int count) = 0;

	/** The vehicle disarmed, where escCheck resets the detector. */
	virtual void disarmed() {}

private:
	static constexpr int kMotor1 = 101; // esc_report::ACTUATOR_FUNCTION_MOTOR1
	static constexpr int kArmed = 2;    // vehicle_status::ARMING_STATE_ARMED

	bool _armed = false;
	bool _have_status = false;
	std::array<float, kMaxSlots> _command_latest {};       // 0 until an actuator_motors arrives, like the runtime
	std::array<bool, kMaxSlots> _reversible_latest {};
	std::array<bool, kMaxSlots> _has_reported_current {};  // per ESC slot, never reset (like escCheck)
	std::array<uint64_t, kMaxSlots> _last_sample_us {};     // per ESC slot

	// Each message's id and layout, and handles to the fields read, resolved once from the log's legend.
	uint16_t _esc_id = 0xffff; // 0xffff = message type not seen yet
	uint16_t _actuator_id = 0xffff;
	uint16_t _status_id = 0xffff;
	std::shared_ptr<ulog_cpp::MessageFormat> _esc_fmt;
	std::shared_ptr<ulog_cpp::MessageFormat> _actuator_fmt;
	std::shared_ptr<ulog_cpp::MessageFormat> _status_fmt;
	int _esc_slots = 0;
	int _control_channels = 0;

	std::shared_ptr<ulog_cpp::Field> _esc_timestamp_f;
	std::shared_ptr<ulog_cpp::Field> _esc_array_f;
	std::shared_ptr<ulog_cpp::Field> _esc_count_f;
	std::shared_ptr<ulog_cpp::Field> _esc_function_f;
	std::shared_ptr<ulog_cpp::Field> _esc_current_f;
	std::shared_ptr<ulog_cpp::Field> _esc_slot_timestamp_f;
	std::shared_ptr<ulog_cpp::Field> _actuator_control_f;
	std::shared_ptr<ulog_cpp::Field> _actuator_reversible_f;
	std::shared_ptr<ulog_cpp::Field> _status_arming_f;
};

/** Stream a whole log through reader. Returns false on a fatal error (already reported to stderr). */
inline bool readLog(const char *ulog_path, const std::shared_ptr<EscLogReader> &reader)
{
	FILE *file = fopen(ulog_path, "rb");

	if (!file) { std::fprintf(stderr, "cannot open %s\n", ulog_path); return false; }

	std::array<uint8_t, 4096> buf;
	int bytes_read;
	ulog_cpp::Reader ulog_reader{reader};

	while ((bytes_read = fread(buf.data(), 1, buf.size(), file)) > 0) {
		try {
			ulog_reader.readChunk(buf.data(), bytes_read);

		} catch (const ulog_cpp::ExceptionBase &e) {
			std::fprintf(stderr, "%s: %s\n  (a needed field is missing -- the log's message layout differs from what this tool expects)\n",
				     ulog_path, e.what());
			fclose(file);
			return false;
		}

		if (reader->hadFatalError()) { break; }
	}

	fclose(file);

	if (reader->hadFatalError()) { std::fprintf(stderr, "fatal parse error in %s\n", ulog_path); return false; }

	return true;
}
