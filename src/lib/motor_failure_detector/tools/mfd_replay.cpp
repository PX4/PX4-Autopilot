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
 * @file mfd_replay.cpp
 *
 * Replays the real MotorFailureDetector over a recorded .ulg and prints a per-motor trip verdict.
 * Built on ulog_cpp, so it reads ESC current / motor command / arming state by field name from the
 * log's own format -- a single binary replays logs of any PX4 message version with no rebuild.
 * Mirrors escCheck.cpp: detector runs only when armed; config from the log's MOTFAIL_* params
 * (per-field fallback to a built-in calibration), with --set NAME=value overrides for what-if retuning.
 *
 * The runtime's exact 10 Hz sample phase can't be reconstructed offline, so in a single pass over the
 * log it runs kNumPhases detector instances on different grid offsets and reports a per-motor peak
 * BAND + trip fraction -- a single-phase verdict is only a lower bound.
 */

#include "MotorFailureDetector.hpp"

#include <ulog_cpp/data_container.hpp>
#include <ulog_cpp/exception.hpp>
#include <ulog_cpp/reader.hpp>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace
{
constexpr int kMM = MotorFailureDetector::kMaxMotors;
constexpr int kMotor1 = 101;   // esc_report::ACTUATOR_FUNCTION_MOTOR1
constexpr int kArmed = 2;      // vehicle_status::ARMING_STATE_ARMED
constexpr uint64_t kRuntimeTickUs = 100000;   // runtime runs the check at 10 Hz (Commander.cpp) -- decimate to match
constexpr int kNumPhases = 5;   // phase-sweep: run N detectors on staggered grid offsets to expose sampling-phase sensitivity

// Default model fit; the FALLBACK used when the log carries no MOTFAIL_* params.
// Threshold band calibrated at the runtime's 10 Hz cadence (kMaxGap = 0.3 s, phase-swept):
// clean across the reference multirotor corpus. Illustrative for this airframe, not a shipped default.
MotorFailureDetector::Config makeConfig()
{
	MotorFailureDetector::Config cfg{};
	cfg.model_b = 34.8f;
	cfg.model_c = 2.88f;
	cfg.residual_lpf_tau_s = 0.2f;
	cfg.threshold_a = 5.0f;
	cfg.persistence_s = 0.4f;
	return cfg;
}

// MOTFAIL_* params -> Config fields, mirroring escCheck.cpp. tau is the compile-time constant there too.
struct ParamBind {
	const char *name;
	float MotorFailureDetector::Config::*field;
};

constexpr std::array kParamBinds {
	ParamBind{"MOTFAIL_C2T",  &MotorFailureDetector::Config::model_b},
	ParamBind{"MOTFAIL_IDLE", &MotorFailureDetector::Config::model_c},
	ParamBind{"MOTFAIL_OFF",  &MotorFailureDetector::Config::threshold_a},
	ParamBind{"MOTFAIL_TIME", &MotorFailureDetector::Config::persistence_s},
};
constexpr int kNumParams = (int)kParamBinds.size();

// Set the Config field bound to param `name` (length `name_len`) to `value`. Returns its index, or -1.
int setParam(MotorFailureDetector::Config &cfg, const char *name, int name_len, float value)
{
	for (int i = 0; i < kNumParams; ++i) {
		if ((int)std::strlen(kParamBinds[i].name) == name_len && std::strncmp(kParamBinds[i].name, name, name_len) == 0) {
			cfg.*(kParamBinds[i].field) = value;
			return i;
		}
	}

	return -1;
}
}

class Replay : public ulog_cpp::DataContainer
{
public:
	// CLI overrides (from --set) are captured here and applied on top of the log's params in
	// headerComplete(), once the log's own MOTFAIL_* values are known.
	Replay(const MotorFailureDetector::Config &cli_cfg, const std::array<bool, kNumParams> &cli_set)
		: ulog_cpp::DataContainer(ulog_cpp::DataContainer::StorageConfig::Header),
		  _cli_cfg(cli_cfg), _cli_set(cli_set)
	{
		for (auto &phase : _trip_time) { phase.fill(-1.0); }
	}

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
			_esc_timestamp_f = _esc_fmt->field("timestamp");
			_esc_count_f = _esc_fmt->field("esc_count");
			_esc_function_f = _esc_array_f->nestedField("actuator_function");
			_esc_current_f = _esc_array_f->nestedField("esc_current");
			_esc_current_seen.assign(_esc_slots, 0);

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

	void parameter(const ulog_cpp::Parameter &p) override
	{
		ulog_cpp::DataContainer::parameter(p);

		if (_configured || p.field().type().type != ulog_cpp::Field::BasicType::FLOAT) { return; }

		const std::string &name = p.field().name();
		const int idx = setParam(_cfg, name.c_str(), (int)name.size(), p.value().as<float>());

		if (idx >= 0 && !_log_set[idx]) {
			_log_set[idx] = true;
			++_params_from_log;
		}
	}

	void headerComplete() override
	{
		ulog_cpp::DataContainer::headerComplete();

		// The header is fully parsed: the log's MOTFAIL_* params and message layouts are known.
		// Apply the CLI overrides on top and configure every phase's detector once, before any data.
		for (int i = 0; i < kNumParams; ++i) {
			if (_cli_set[i]) { _cfg.*(kParamBinds[i].field) = _cli_cfg.*(kParamBinds[i].field); }
		}

		for (auto &d : _det) { d.configure(_cfg); }

		_configured = true;
	}

	void data(const ulog_cpp::Data &record) override
	{
		const uint16_t id = record.msgId();

		if (id == _status_id) {
			const bool now_armed = (ulog_cpp::TypedDataView(record, *_status_fmt)[_status_arming_f].as<int>() == kArmed);

			if (_armed && !now_armed) {   // disarm resets the detector (mirrors configure-on-disarm)
				for (int p = 0; p < kNumPhases; ++p) {
					_det[p].configure(_cfg);
					_next_eval_us[p] = 0;
				}
			}

			_armed = now_armed;
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

		if (view[_esc_count_f].as<int>() <= 0) { return; }       // runtime skips esc_status with esc_count <= 0

		if (_have_status && !_armed) { return; }                 // armed gate, like escCheck.cpp

		const uint64_t sample_us = view[_esc_timestamp_f].as<uint64_t>();

		// Shape the per-motor inputs once -- they don't depend on the sampling phase, only on the
		// record. Which phase grids consume this sample is decided in the phase loop below.
		std::array<float, kMM> command;
		std::array<float, kMM> current {};
		std::array<bool, kMM> reversible {};
		command.fill(NAN);

		const auto esc_arr = view[_esc_array_f];

		for (int i = 0; i < _esc_slots; ++i) {
			const auto esc_i = esc_arr[i];
			const int function = esc_i[_esc_function_f].as<int>();

			if (function < kMotor1 || function - kMotor1 >= kMM) { continue; }

			const float slot_current = esc_i[_esc_current_f].as<float>();

			if (slot_current > FLT_EPSILON) { _esc_current_seen[i] = 1; }   // latch first current report (per ESC slot)

			if (!_esc_current_seen[i]) { continue; }                        // don't judge until the ESC reports current

			const int m = function - kMotor1;
			_seen[m] = true;
			current[m] = slot_current;
			command[m] = _command_latest[m];                                // 0 until an actuator arrives (runtime zero-init)
			reversible[m] = _reversible_latest[m];
		}

		// One parse, kNumPhases detectors: feed each phase whose 10 Hz grid this sample crosses. Each
		// phase's grid is offset by p/kNumPhases of a tick, so a single pass over the log covers the
		// sampling-phase spread the runtime's unknown tick alignment could land on (no file re-read).
		for (int p = 0; p < kNumPhases; ++p) {
			if (_next_eval_us[p] == 0) {   // first tick: anchor this phase's grid at its offset
				_next_eval_us[p] = sample_us + (uint64_t)p * kRuntimeTickUs / kNumPhases;
			}

			if (sample_us < _next_eval_us[p]) { continue; }

			_next_eval_us[p] += kRuntimeTickUs;

			if (_start_us < 0) { _start_us = (double)sample_us; }

			const double elapsed_s = ((double)sample_us - _start_us) / 1e6;

			_det[p].update(kMM, (hrt_abstime)sample_us, command.data(), current.data(), reversible.data());

			for (int m = 0; m < kMM; ++m) {
				if (!_seen[m]) { continue; }

				const float abs_residual_lpf = std::fabs(_det[p].status(m).residual_lpf);

				if (!std::isnan(abs_residual_lpf) && abs_residual_lpf > _peak[p][m]) { _peak[p][m] = abs_residual_lpf; }

				if (_det[p].status(m).failed && _trip_time[p][m] < 0) { _trip_time[p][m] = elapsed_s; }
			}

			if (p == 0) { ++_evaluated; }
		}
	}

	// Results, read after the whole log has been parsed.
	bool seen(int m) const { return _seen[m]; }
	float peak(int phase, int m) const { return _peak[phase][m]; }
	double tripTime(int phase, int m) const { return _trip_time[phase][m]; }
	bool haveStatus() const { return _have_status; }
	int paramsFromLog() const { return _params_from_log; }
	long evaluated() const { return _evaluated; }
	const MotorFailureDetector::Config &config() const { return _cfg; }

private:
	MotorFailureDetector::Config _cli_cfg {};
	std::array<bool, kNumParams> _cli_set {};

	// One detector instance per sampling phase; a single parse feeds all of them.
	std::array<MotorFailureDetector, kNumPhases> _det;
	MotorFailureDetector::Config _cfg = makeConfig();
	std::array<bool, kNumParams> _log_set {};
	int _params_from_log = 0;
	bool _configured = false;

	bool _armed = false;
	bool _have_status = false;
	std::array<float, kMM> _command_latest {};
	std::array<bool, kMM> _reversible_latest {};
	std::array<std::array<float, kMM>, kNumPhases> _peak {};
	std::array<std::array<double, kMM>, kNumPhases> _trip_time;   // -1 until a trip (filled in the ctor)
	std::array<bool, kMM> _seen {};
	double _start_us = -1.0;
	long _evaluated = 0;   // phase-0 evaluation count (== per-phase sample count)

	// Cached ULog "legend" (filled in addLoggedMessage): each message's id + layout, and handles to the
	// fields we read -- resolved once so the per-record loop reads by handle, not by name.
	uint16_t _esc_id = 0xffff;        // 0xffff = message type not seen yet
	uint16_t _actuator_id = 0xffff;
	uint16_t _status_id = 0xffff;
	std::shared_ptr<ulog_cpp::MessageFormat> _esc_fmt;
	std::shared_ptr<ulog_cpp::MessageFormat> _actuator_fmt;
	std::shared_ptr<ulog_cpp::MessageFormat> _status_fmt;
	int _esc_slots = 0;         // esc[] slots from the layout
	int _control_channels = 0;  // control[] channels from the layout

	std::vector<char> _esc_current_seen;   // per-ESC-slot "has reported current" latch (mirrors escCheck), never reset
	std::array<uint64_t, kNumPhases> _next_eval_us {};   // per-phase next 10 Hz evaluation time (runtime-cadence match)

	// esc_status field handles (suffix _f)
	std::shared_ptr<ulog_cpp::Field> _esc_timestamp_f;
	std::shared_ptr<ulog_cpp::Field> _esc_array_f;
	std::shared_ptr<ulog_cpp::Field> _esc_function_f;
	std::shared_ptr<ulog_cpp::Field> _esc_current_f;
	std::shared_ptr<ulog_cpp::Field> _esc_count_f;
	// actuator_motors field handles
	std::shared_ptr<ulog_cpp::Field> _actuator_control_f;
	std::shared_ptr<ulog_cpp::Field> _actuator_reversible_f;
	// vehicle_status field handles
	std::shared_ptr<ulog_cpp::Field> _status_arming_f;
};

// Parse the whole log once through a fresh Replay (which runs all kNumPhases detectors internally).
// Returns the populated Replay, or nullptr on a fatal error (already reported to stderr).
std::shared_ptr<Replay> parseLog(const char *ulog_path, const MotorFailureDetector::Config &cli_cfg,
				 const std::array<bool, kNumParams> &cli_set)
{
	auto replay = std::make_shared<Replay>(cli_cfg, cli_set);

	FILE *file = fopen(ulog_path, "rb");

	if (!file) { std::fprintf(stderr, "cannot open %s\n", ulog_path); return nullptr; }

	std::array<uint8_t, 4096> buf;
	int bytes_read;
	ulog_cpp::Reader reader{replay};

	while ((bytes_read = fread(buf.data(), 1, buf.size(), file)) > 0) {
		try {
			reader.readChunk(buf.data(), bytes_read);

		} catch (const ulog_cpp::ExceptionBase &e) {
			std::fprintf(stderr, "%s: %s\n  (a needed field is missing -- the log's message layout differs from what this tool expects)\n",
				     ulog_path, e.what());
			fclose(file);
			return nullptr;
		}

		if (replay->hadFatalError()) { break; }
	}

	fclose(file);

	if (replay->hadFatalError()) { std::fprintf(stderr, "fatal parse error in %s\n", ulog_path); return nullptr; }

	return replay;
}

int main(int argc, char **argv)
{
	const char *usage = "usage: %s <log.ulg> [--set MOTFAIL_NAME=value ...]\n";
	const char *ulog_path = nullptr;
	MotorFailureDetector::Config cli_cfg{};
	std::array<bool, kNumParams> cli_set{};
	int cli_overrides = 0;

	for (int i = 1; i < argc; ++i) {
		if (!std::strcmp(argv[i], "--set") && i + 1 < argc) {
			const char *kv = argv[++i];
			const char *eq = std::strchr(kv, '=');
			char *end = nullptr;
			const float val = eq ? std::strtof(eq + 1, &end) : 0.f;
			const int idx = (eq && end != eq + 1) ? setParam(cli_cfg, kv, (int)(eq - kv), val) : -1;

			if (idx < 0) {
				std::fprintf(stderr, "bad --set '%s' (expected MOTFAIL_NAME=value)\n", kv);
				return 2;
			}

			if (!cli_set[idx]) { ++cli_overrides; }

			cli_set[idx] = true;

		} else if (!ulog_path && argv[i][0] != '-') {
			ulog_path = argv[i];

		} else {
			std::fprintf(stderr, usage, argv[0]);
			return 2;
		}
	}

	if (!ulog_path) { std::fprintf(stderr, usage, argv[0]); return 2; }

	auto replay = parseLog(ulog_path, cli_cfg, cli_set);

	if (!replay) { return 2; }

	// Aggregate the per-phase results into a peak band + trip fraction per motor.
	std::array<bool, kMM> seen_any{};
	std::array<float, kMM> peak_min;
	std::array<float, kMM> peak_max;
	std::array<int, kMM> trip_count{};

	peak_min.fill(FLT_MAX);
	peak_max.fill(0.f);

	for (int m = 0; m < kMM; ++m) {
		if (!replay->seen(m)) { continue; }

		seen_any[m] = true;

		for (int p = 0; p < kNumPhases; ++p) {
			peak_min[m] = std::min(peak_min[m], replay->peak(p, m));
			peak_max[m] = std::max(peak_max[m], replay->peak(p, m));

			if (replay->tripTime(p, m) >= 0) { ++trip_count[m]; }
		}
	}

	for (const std::string &err : replay->parsingErrors()) { std::fprintf(stderr, "parse: %s\n", err.c_str()); }

	bool any_motor = false;

	for (int m = 0; m < kMM; ++m) { any_motor |= seen_any[m]; }

	if (!any_motor) {
		std::fprintf(stderr, "no armed esc_status samples in %s\n", ulog_path);
		return 2;
	}

	if (!replay->haveStatus()) {
		std::fprintf(stderr, "warning: no vehicle_status in %s -- arming gate disabled, evaluating all esc samples\n", ulog_path);
	}

	std::array<char, 160> config_source {};
	int n = replay->paramsFromLog() > 0
		? std::snprintf(config_source.data(), config_source.size(), "from log")
		: std::snprintf(config_source.data(), config_source.size(), "built-in calibration");
	n = std::min(n, (int)config_source.size() - 1);

	if (cli_overrides > 0) {
		n += std::snprintf(config_source.data() + n, config_source.size() - n, ", override%s:", cli_overrides > 1 ? "s" : "");

		for (int i = 0; i < kNumParams && n < (int)config_source.size() - 1; ++i) {
			if (cli_set[i]) { n += std::snprintf(config_source.data() + n, config_source.size() - n, " %s", kParamBinds[i].name); }
		}
	}

	const MotorFailureDetector::Config &cfg = replay->config();
	const char *count_label = replay->haveStatus() ? "armed samples each" : "samples each, arming gate OFF";
	std::printf("# %s  (%d phases, %ld %s)\n"
		    "# config (%s):  I_exp = %.2f*u + %.2f A,  trip if |LPF(I-I_exp)| > %.2f for %.2f s\n",
		    ulog_path, kNumPhases, replay->evaluated(), count_label, config_source.data(),
		    (double)cfg.model_b, (double)cfg.model_c,
		    (double)cfg.threshold_a, (double)cfg.persistence_s);
	std::printf("# %-5s  %-15s  %-5s  %s\n", "motor", "peak|LPF(r)|[A]", "trips", "verdict");

	bool any_failed = false;
	bool phase_dependent = false;

	for (int m = 0; m < kMM; ++m) {
		if (!seen_any[m]) { continue; }

		const char *verdict;

		if (trip_count[m] == kNumPhases) {
			verdict = "FAILED (all phases)";
			any_failed = true;

		} else if (trip_count[m] > 0) {
			verdict = "FAILED (sampling-dependent!)";
			any_failed = true;
			phase_dependent = true;

		} else {
			verdict = "ok";
		}

		if (peak_max[m] > 1.3f * peak_min[m]) { phase_dependent = true; }   // wide band => phase-sensitive

		std::array<char, 16> trips_str {};
		std::snprintf(trips_str.data(), trips_str.size(), "%d/%d", trip_count[m], kNumPhases);
		std::printf("  %5d  %6.2f - %-6.2f  %-5s  %s\n",
			    m, (double)peak_min[m], (double)peak_max[m], trips_str.data(), verdict);
	}

	if (phase_dependent) {
		std::printf("# NOTE: the verdict depends on which samples are used -- treat any trip as a\n"
			    "#       possible failure on the vehicle, not a pass.\n");
	}

	return any_failed ? 1 : 0;
}
