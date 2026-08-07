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
 * @file mfd_fit.cpp
 *
 * Fits the MotorFailureDetector current model, I_expected = MOTFAIL_C2T * u + MOTFAIL_IDLE, from
 * recorded .ulg files, and prints the parameter values to set.
 *
 * Built on ulog_cpp like mfd_replay, reading ESC current / motor command / arming state by field name
 * from each log's own format. Every log is a single streaming pass that only accumulates the six sums a
 * least-squares line needs, so memory is constant and unrelated to log size or corpus size: pass the
 * whole corpus at once and the fit is pooled over all of it, which is what the detector wants (one
 * model shared by every motor).
 */

#include "MotorFailureDetector.hpp"

#include <ulog_cpp/data_container.hpp>
#include <ulog_cpp/exception.hpp>
#include <ulog_cpp/reader.hpp>

#include <array>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace
{
constexpr int kMM = MotorFailureDetector::kMaxMotors;
constexpr int kMotor1 = 101;   // esc_report::ACTUATOR_FUNCTION_MOTOR1
constexpr int kArmed = 2;      // vehicle_status::ARMING_STATE_ARMED

/**
 * Sums for a least-squares fit of I = slope * u + idle.
 *
 * Only these six numbers are needed, so they can be accumulated per motor and pooled by addition;
 * no sample is ever stored.
 */
struct Sums {
	double n{0};
	double u{0};
	double uu{0};
	double i{0};
	double ui{0};
	double ii{0};

	void add(double command, double current)
	{
		n += 1.0;
		u += command;
		uu += command * command;
		i += current;
		ui += command * current;
		ii += current * current;
	}

	void operator+=(const Sums &o)
	{
		n += o.n; u += o.u; uu += o.uu; i += o.i; ui += o.ui; ii += o.ii;
	}

	/** Solve for the line. Returns false if the commands span too little range to define a slope. */
	bool solve(double &slope, double &idle) const
	{
		const double det = n * uu - u * u;

		if (n < 2.0 || std::fabs(det) < 1e-9) { return false; }

		slope = (n * ui - u * i) / det;
		idle = (i - slope * u) / n;
		return true;
	}

	/** Standard deviation of the residual about the given line (the spread the trip bands must clear). */
	double residualStdDev(double slope, double idle) const
	{
		const double sq = ii - 2.0 * slope * ui - 2.0 * idle * i
				  + slope * slope * uu + 2.0 * slope * idle * u + idle * idle * n;
		return (n > 1.0 && sq > 0.0) ? std::sqrt(sq / n) : 0.0;
	}
};

class Fit : public ulog_cpp::DataContainer
{
public:
	Fit() : ulog_cpp::DataContainer(ulog_cpp::DataContainer::StorageConfig::Header) {}

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
			_esc_slots = _esc_array_f->arrayLength();
			_esc_count_f = _esc_fmt->field("esc_count");
			_esc_function_f = _esc_array_f->nestedField("actuator_function");
			_esc_current_f = _esc_array_f->nestedField("esc_current");

		} else if (msg.messageName() == "actuator_motors") {
			_actuator_id = msg.msgId();
			_actuator_fmt = it->second;
			_actuator_control_f = _actuator_fmt->field("control");
			_control_channels = _actuator_control_f->arrayLength();
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
			_armed = ulog_cpp::TypedDataView(record, *_status_fmt)[_status_arming_f].as<int>() == kArmed;
			return;
		}

		if (id == _actuator_id) {
			ulog_cpp::TypedDataView view(record, *_actuator_fmt);
			const uint32_t reversible_mask = view[_actuator_reversible_f].as<uint32_t>();
			const auto control = view[_actuator_control_f];

			for (int m = 0; m < kMM && m < _control_channels; ++m) {
				_command_latest[m] = control[m].as<float>();
				_reversible_latest[m] = (reversible_mask >> m) & 1;
			}

			return;
		}

		if (id != _esc_id) { return; }

		ulog_cpp::TypedDataView view(record, *_esc_fmt);

		if (view[_esc_count_f].as<int>() <= 0) { return; }

		if (_have_status && !_armed) { return; }   // only armed flight describes the model

		const auto esc_arr = view[_esc_array_f];

		for (int slot = 0; slot < _esc_slots; ++slot) {
			const auto esc_i = esc_arr[slot];
			const int function = esc_i[_esc_function_f].as<int>();

			if (function < kMotor1 || function - kMotor1 >= kMM) { continue; }

			const int m = function - kMotor1;
			const float current = esc_i[_esc_current_f].as<float>();
			const float command = _command_latest[m];

			// A motor the allocation switched off (NAN), one running in reverse, or an ESC not yet
			// reporting current describes nothing about the healthy forward model.
			if (!std::isfinite(current) || current <= FLT_EPSILON
			    || !std::isfinite(command) || _reversible_latest[m]) {
				continue;
			}

			_seen[m] = true;
			_per_motor[m].add(command, current);
		}
	}

	bool seen(int m) const { return _seen[m]; }
	const Sums &motor(int m) const { return _per_motor[m]; }

private:
	std::array<Sums, kMM> _per_motor {};
	std::array<bool, kMM> _seen {};

	bool _armed{false};
	bool _have_status{false};
	std::array<float, kMM> _command_latest {};
	std::array<bool, kMM> _reversible_latest {};

	uint16_t _esc_id = 0xffff;
	uint16_t _actuator_id = 0xffff;
	uint16_t _status_id = 0xffff;
	std::shared_ptr<ulog_cpp::MessageFormat> _esc_fmt;
	std::shared_ptr<ulog_cpp::MessageFormat> _actuator_fmt;
	std::shared_ptr<ulog_cpp::MessageFormat> _status_fmt;
	int _esc_slots = 0;
	int _control_channels = 0;

	std::shared_ptr<ulog_cpp::Field> _esc_array_f;
	std::shared_ptr<ulog_cpp::Field> _esc_count_f;
	std::shared_ptr<ulog_cpp::Field> _esc_function_f;
	std::shared_ptr<ulog_cpp::Field> _esc_current_f;
	std::shared_ptr<ulog_cpp::Field> _actuator_control_f;
	std::shared_ptr<ulog_cpp::Field> _actuator_reversible_f;
	std::shared_ptr<ulog_cpp::Field> _status_arming_f;
};

/** Stream one log through a fresh Fit. Returns nullptr on a fatal error (already reported). */
std::shared_ptr<Fit> parseLog(const char *ulog_path)
{
	auto fit = std::make_shared<Fit>();
	FILE *file = fopen(ulog_path, "rb");

	if (!file) { std::fprintf(stderr, "cannot open %s\n", ulog_path); return nullptr; }

	std::array<uint8_t, 4096> buf;
	int bytes_read;
	ulog_cpp::Reader reader{fit};

	while ((bytes_read = fread(buf.data(), 1, buf.size(), file)) > 0) {
		try {
			reader.readChunk(buf.data(), bytes_read);

		} catch (const ulog_cpp::ExceptionBase &e) {
			std::fprintf(stderr, "%s: %s\n  (a needed field is missing -- the log's message layout differs from what this tool expects)\n",
				     ulog_path, e.what());
			fclose(file);
			return nullptr;
		}

		if (fit->hadFatalError()) { break; }
	}

	fclose(file);

	if (fit->hadFatalError()) { std::fprintf(stderr, "fatal parse error in %s\n", ulog_path); return nullptr; }

	return fit;
}
}

int main(int argc, char **argv)
{
	const char *usage = "usage: %s <log.ulg> [<log.ulg> ...]\n"
			    "  Fits I_expected = MOTFAIL_C2T * u + MOTFAIL_IDLE, pooled over every log given.\n";

	if (argc < 2) { std::fprintf(stderr, usage, argv[0]); return 2; }

	std::array<Sums, kMM> per_motor {};
	std::array<bool, kMM> seen {};
	int logs_used = 0;

	for (int arg = 1; arg < argc; ++arg) {
		if (argv[arg][0] == '-') { std::fprintf(stderr, usage, argv[0]); return 2; }

		auto fit = parseLog(argv[arg]);

		if (!fit) { continue; }   // reported; keep going so one bad log does not lose the corpus

		bool any = false;

		for (int m = 0; m < kMM; ++m) {
			if (!fit->seen(m)) { continue; }

			seen[m] = true;
			any = true;
			per_motor[m] += fit->motor(m);
		}

		if (any) { ++logs_used; }

		else { std::fprintf(stderr, "%s: no armed ESC current samples\n", argv[arg]); }
	}

	if (logs_used == 0) { std::fprintf(stderr, "no usable logs\n"); return 2; }

	Sums pooled;

	for (int m = 0; m < kMM; ++m) {
		if (seen[m]) { pooled += per_motor[m]; }
	}

	double slope = 0.0;
	double idle = 0.0;

	if (!pooled.solve(slope, idle)) {
		std::fprintf(stderr, "commands span too little range to fit a slope (was the vehicle flying?)\n");
		return 2;
	}

	const double residual_sd = pooled.residualStdDev(slope, idle);
	const double mean_command = pooled.u / pooled.n;
	const double mean_current = pooled.i / pooled.n;

	std::printf("# %d log%s, %.0f armed samples\n", logs_used, logs_used == 1 ? "" : "s", pooled.n);
	std::printf("# I_expected = %.2f*u + %.2f A   (residual sigma %.2f A)\n", slope, idle, residual_sd);
	std::printf("# mean command %.3f, mean current %.2f A\n", mean_command, mean_current);
	std::printf("\nMOTFAIL_C2T %.2f\nMOTFAIL_IDLE %.2f\n", slope, idle);

	if (idle < 0.0) {
		std::printf("\n# NOTE: the fitted offset is negative, and MOTFAIL_IDLE cannot be set below 0.\n"
			    "#       Use MOTFAIL_IDLE 0; an offset this small is within the residual anyway.\n");
	}

	// Per-motor bias against the shared model: how much a single model costs each motor. Large spread
	// here is the only argument for not sharing one model, and it comes straight out of the same sums.
	std::printf("\n# per-motor mean residual against this model [A]\n");
	double worst = 0.0;

	for (int m = 0; m < kMM; ++m) {
		if (!seen[m]) { continue; }

		const Sums &s = per_motor[m];
		const double bias = s.i / s.n - (slope * (s.u / s.n) + idle);
		// 1-based, matching the "Motor N ..." the vehicle reports; the arrays stay 0-based.
		std::printf("#   motor %d: %+.2f  (sigma %.2f, n %.0f)\n", m + 1, bias, s.residualStdDev(slope, idle), s.n);
		worst = std::fmax(worst, std::fabs(bias));
	}

	std::printf("# worst per-motor bias %.2f A -- the trip bands have to clear this before any fault.\n", worst);

	return 0;
}
