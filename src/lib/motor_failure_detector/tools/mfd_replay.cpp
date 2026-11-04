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
 * log it runs kNumPhases detector instances on different grid offsets and reports each motor's
 * residual extremes + trip fraction -- a single-phase verdict is only a lower bound.
 */

#include "EscLogReader.hpp"

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
constexpr uint64_t kRuntimeTickUs = 100000;   // runtime runs the check at 10 Hz (Commander.cpp) -- decimate to match
constexpr int kNumPhases = 5;   // phase-sweep: run N detectors on staggered grid offsets to expose sampling-phase sensitivity

// Default model fit; the FALLBACK used when the log carries no MOTFAIL_* params.
// Threshold band calibrated at the runtime's 10 Hz cadence (phase-swept):
// clean across the reference multirotor corpus. Illustrative for this airframe, not a shipped default.
MotorFailureDetector::Config makeConfig()
{
	MotorFailureDetector::Config cfg{};
	cfg.current_slope_a = 34.8f;
	cfg.current_idle_a = 2.88f;
	cfg.overcurrent_threshold_a = 5.0f;
	cfg.overcurrent_trigger_time_s = 0.4f;
	cfg.undercurrent_threshold_a = 5.0f;
	cfg.undercurrent_trigger_time_s = 0.4f;
	return cfg;
}

// MOTFAIL_* params -> Config fields, mirroring escCheck.cpp.
struct ParamBind {
	const char *name;
	float MotorFailureDetector::Config::*field;
};

constexpr std::array kParamBinds {
	ParamBind{"MOTFAIL_C2T",  &MotorFailureDetector::Config::current_slope_a},
	ParamBind{"MOTFAIL_IDLE", &MotorFailureDetector::Config::current_idle_a},
	ParamBind{"MOTFAIL_OVER", &MotorFailureDetector::Config::overcurrent_threshold_a},
	ParamBind{"MOTFAIL_OVR_TIME", &MotorFailureDetector::Config::overcurrent_trigger_time_s},
	ParamBind{"MOTFAIL_UNDER", &MotorFailureDetector::Config::undercurrent_threshold_a},
	ParamBind{"MOTFAIL_UND_TIME", &MotorFailureDetector::Config::undercurrent_trigger_time_s},
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

class Replay : public EscLogReader
{
public:
	// CLI overrides (from --set) are captured here and applied on top of the log's params in
	// headerComplete(), once the log's own MOTFAIL_* values are known.
	Replay(const MotorFailureDetector::Config &cli_cfg, const std::array<bool, kNumParams> &cli_set)
		: _cli_cfg(cli_cfg), _cli_set(cli_set)
	{
		for (auto &phase : _trip_time) { phase.fill(-1.0); }

		for (auto &phase : _rmin) { phase.fill(0.f); }

		for (auto &phase : _rmax) { phase.fill(0.f); }
	}

	// Results per ESC slot, read after the whole log has been parsed.
	bool seen(int i) const { return _seen[i]; }
	int motor(int i) const { return _motor[i]; }   ///< 0-based motor index the slot was last mapped to
	float residualMin(int phase, int i) const { return _rmin[phase][i]; }
	float residualMax(int phase, int i) const { return _rmax[phase][i]; }
	double tripTime(int phase, int i) const { return _trip_time[phase][i]; }
	int paramsFromLog() const { return _params_from_log; }
	long evaluated() const { return _evaluated; }
	const MotorFailureDetector::Config &config() const { return _cfg; }

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

protected:
	void disarmed() override
	{
		for (int p = 0; p < kNumPhases; ++p) {
			_det[p].reset();
			_next_eval_us[p] = 0;
		}
	}

	void escRecord(uint64_t record_us, const EscSample *samples, int count) override
	{
		for (int k = 0; k < count; ++k) {
			_seen[samples[k].slot] = true;
			_motor[samples[k].slot] = samples[k].motor;
		}

		// One parse, kNumPhases detectors: feed each phase whose 10 Hz grid this record crosses. Each
		// phase's grid is offset by p/kNumPhases of a tick, so a single pass over the log covers the
		// sampling-phase spread the runtime's unknown tick alignment could land on (no file re-read).
		for (int p = 0; p < kNumPhases; ++p) {
			if (_next_eval_us[p] == 0) {   // first tick: anchor this phase's grid at its offset
				_next_eval_us[p] = record_us + (uint64_t)p * kRuntimeTickUs / kNumPhases;
			}

			if (record_us < _next_eval_us[p]) { continue; }

			_next_eval_us[p] += kRuntimeTickUs;

			if (_start_us < 0) { _start_us = (double)record_us; }

			const double elapsed_s = ((double)record_us - _start_us) / 1e6;

			for (int k = 0; k < count; ++k) {
				const EscSample &s = samples[k];
				_det[p].update(s.slot, s.timestamp_us, s.command, s.current);

				const float residual_lpf = _det[p].residualFiltered(s.slot);

				if (!std::isnan(residual_lpf)) {
					_rmin[p][s.slot] = std::min(_rmin[p][s.slot], residual_lpf);
					_rmax[p][s.slot] = std::max(_rmax[p][s.slot], residual_lpf);
				}

				if (_det[p].failure(s.slot) != MotorFailureDetector::Failure::None && _trip_time[p][s.slot] < 0) {
					_trip_time[p][s.slot] = elapsed_s;
				}
			}

			if (p == 0) { ++_evaluated; }
		}
	}

private:
	MotorFailureDetector::Config _cli_cfg {};
	std::array<bool, kNumParams> _cli_set {};

	// One detector instance per sampling phase; a single parse feeds all of them.
	std::array<MotorFailureDetector, kNumPhases> _det;
	MotorFailureDetector::Config _cfg = makeConfig();
	std::array<bool, kNumParams> _log_set {};
	int _params_from_log = 0;
	bool _configured = false;

	std::array<std::array<float, kMM>, kNumPhases> _rmin;
	std::array<std::array<float, kMM>, kNumPhases> _rmax;
	std::array<std::array<double, kMM>, kNumPhases> _trip_time;   // -1 until a trip (filled in the ctor)
	std::array<bool, kMM> _seen {};    // per ESC slot
	std::array<int, kMM> _motor {};    // per ESC slot
	double _start_us = -1.0;
	long _evaluated = 0;   // phase-0 evaluation count (== per-phase sample count)
	std::array<uint64_t, kNumPhases> _next_eval_us {};   // per-phase next 10 Hz evaluation time (runtime-cadence match)
};

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

	auto replay = std::make_shared<Replay>(cli_cfg, cli_set);

	if (!readLog(ulog_path, replay)) { return 2; }

	// Aggregate the per-phase results into each ESC's residual extremes + trip fraction. The
	// envelope (deepest and highest any phase saw) is reported; the per-phase spread of each extreme
	// is what says whether the answer depends on sampling.
	std::array<bool, kMM> seen_any{};
	std::array<float, kMM> res_min;
	std::array<float, kMM> res_max;
	std::array<float, kMM> res_min_shallowest;
	std::array<float, kMM> res_max_lowest;
	std::array<int, kMM> trip_count{};

	res_min.fill(0.f);
	res_max.fill(0.f);
	res_min_shallowest.fill(-FLT_MAX);
	res_max_lowest.fill(FLT_MAX);

	for (int i = 0; i < kMM; ++i) {
		if (!replay->seen(i)) { continue; }

		seen_any[i] = true;

		for (int p = 0; p < kNumPhases; ++p) {
			res_min[i] = std::min(res_min[i], replay->residualMin(p, i));
			res_max[i] = std::max(res_max[i], replay->residualMax(p, i));
			res_min_shallowest[i] = std::max(res_min_shallowest[i], replay->residualMin(p, i));
			res_max_lowest[i] = std::min(res_max_lowest[i], replay->residualMax(p, i));

			if (replay->tripTime(p, i) >= 0) { ++trip_count[i]; }
		}
	}

	for (const std::string &err : replay->parsingErrors()) { std::fprintf(stderr, "parse: %s\n", err.c_str()); }

	bool any_motor = false;

	for (int i = 0; i < kMM; ++i) { any_motor |= seen_any[i]; }

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
	std::array<char, 48> under_desc {};
	std::array<char, 48> over_desc {};

	if (cfg.undercurrent_threshold_a > 0.f) {
		std::snprintf(under_desc.data(), under_desc.size(), "< -%.2f for %.2f s",
			      (double)cfg.undercurrent_threshold_a, (double)cfg.undercurrent_trigger_time_s);

	} else {
		std::snprintf(under_desc.data(), under_desc.size(), "undercurrent off");
	}

	if (cfg.overcurrent_threshold_a > 0.f) {
		std::snprintf(over_desc.data(), over_desc.size(), "> %.2f for %.2f s",
			      (double)cfg.overcurrent_threshold_a, (double)cfg.overcurrent_trigger_time_s);

	} else {
		std::snprintf(over_desc.data(), over_desc.size(), "overcurrent off");
	}

	std::printf("# %s  (%d phases, %ld %s)\n"
		    "# config (%s):  I_exp = %.2f*u + %.2f A,  trip if LPF(I-I_exp) %s or %s\n",
		    ulog_path, kNumPhases, replay->evaluated(), count_label, config_source.data(),
		    (double)cfg.current_slope_a, (double)cfg.current_idle_a, under_desc.data(), over_desc.data());
	std::printf("# %-5s  %8s  %8s  %-5s  %s\n", "motor", "min[A]", "max[A]", "trips", "verdict");

	bool any_failed = false;
	bool phase_dependent = false;

	for (int i = 0; i < kMM; ++i) {
		if (!seen_any[i]) { continue; }

		const char *verdict;

		if (trip_count[i] == kNumPhases) {
			verdict = "FAILED (all phases)";
			any_failed = true;

		} else if (trip_count[i] > 0) {
			verdict = "FAILED (sampling-dependent!)";
			any_failed = true;
			phase_dependent = true;

		} else {
			verdict = "ok";
		}

		// One side moving by >30% between phases means the sampling grid, not the flight, picked the answer.
		if (std::fabs(res_min[i]) > 1.3f * std::fabs(res_min_shallowest[i])
		    || res_max[i] > 1.3f * res_max_lowest[i]) {
			phase_dependent = true;
		}

		std::array<char, 16> trips_str {};
		std::snprintf(trips_str.data(), trips_str.size(), "%d/%d", trip_count[i], kNumPhases);
		// 1-based, matching the "Motor N ..." the vehicle reports; the arrays stay 0-based.
		std::printf("  %5d  %+8.2f  %+8.2f  %-5s  %s\n",
			    replay->motor(i) + 1, (double)res_min[i], (double)res_max[i], trips_str.data(), verdict);
	}

	if (phase_dependent) {
		std::printf("# NOTE: the verdict depends on which samples are used -- treat any trip as a\n"
			    "#       possible failure on the vehicle, not a pass.\n");
	}

	return any_failed ? 1 : 0;
}
