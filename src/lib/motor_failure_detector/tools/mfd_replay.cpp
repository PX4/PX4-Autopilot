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
 * Offline replay of the real MotorFailureDetector over a recorded .ulg, in the
 * spirit of EKF2 replay: the actual library is driven by recorded data so the
 * exact estimator that runs on the vehicle can be evaluated against a log.
 *
 * Unlike the EKF2 replay module it needs no uORB or SITL: the detector is a
 * uORB-free library, so this is a single self-contained host binary that parses
 * the ulog directly in one streaming pass (no ulog2csv detour, nothing buffered).
 * Fields are located from the log's own message FORMAT definitions and read by
 * offset, so the tool is independent of the firmware's current message layout
 * (e.g. it reads both 8- and 12-ESC esc_status logs). Only esc_status and
 * actuator_motors are decoded; every other message is skipped by seek.
 *
 * Per esc_status sample it feeds the latest actuator_motors command (zero-order
 * hold) to the detector and reports the per-motor trip verdict; exit code is
 * non-zero if any motor tripped.
 * --inject-* synthesizes a fault (no real failure log exists); --dump writes the
 * per-sample residual series for tools/plot_replay.py.
 */

#include "MotorFailureDetector.hpp"

#include <logger/messages.h>                   // ULog framing: ULogMessageType, ulog_*_s, ULOG_MSG_HEADER_LEN

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace
{

constexpr int kMM = MotorFailureDetector::kMaxMotors;

// One field of a ULog message FORMAT: "type[len] name" (len 1 if not an array).
struct Field {
	std::string type;
	int len;
	std::string name;
};

using FormatMap = std::map<std::string, std::vector<Field>>;

int baseSize(const std::string &t)
{
	if (t == "int8_t" || t == "uint8_t" || t == "char" || t == "bool") { return 1; }

	if (t == "int16_t" || t == "uint16_t") { return 2; }

	if (t == "int32_t" || t == "uint32_t" || t == "float") { return 4; }

	if (t == "int64_t" || t == "uint64_t" || t == "double") { return 8; }

	return -1; // nested message type
}

// Logged size of a type in bytes (recursively summing nested fields, incl. padding fields).
int typeSize(const FormatMap &formats, const std::string &type)
{
	const int b = baseSize(type);

	if (b > 0) { return b; }

	auto it = formats.find(type);

	if (it == formats.end()) { return -1; }

	int size = 0;

	for (const Field &f : it->second) {
		const int fs = typeSize(formats, f.type);

		if (fs < 0) { return -1; }

		size += fs * f.len;
	}

	return size;
}

// Byte offset of `field` within `type` (-1 if not found), and (optionally) its array length.
int fieldOffset(const FormatMap &formats, const std::string &type, const std::string &field, int *len = nullptr)
{
	auto it = formats.find(type);

	if (it == formats.end()) { return -1; }

	int off = 0;

	for (const Field &f : it->second) {
		if (f.name == field) {
			if (len) { *len = f.len; }

			return off;
		}

		const int fs = typeSize(formats, f.type);

		if (fs < 0) { return -1; }

		off += fs * f.len;
	}

	return -1;
}

// Parse a FORMAT payload "msg_name:type0 f0;type1[N] f1;..." into the format map.
void parseFormat(const char *payload, FormatMap &formats)
{
	std::string s(payload);
	const size_t colon = s.find(':');

	if (colon == std::string::npos) { return; }

	const std::string name = s.substr(0, colon);
	std::vector<Field> fields;
	std::stringstream fs(s.substr(colon + 1));
	std::string decl;

	while (std::getline(fs, decl, ';')) {
		if (decl.empty()) { continue; }

		const size_t sp = decl.find(' ');

		if (sp == std::string::npos) { continue; }

		std::string type = decl.substr(0, sp);
		const std::string fname = decl.substr(sp + 1);
		int len = 1;
		const size_t br = type.find('[');

		if (br != std::string::npos) {
			len = std::atoi(type.c_str() + br + 1);
			type = type.substr(0, br);
		}

		fields.push_back({type, len, fname});
	}

	formats[name] = fields;
}

template<typename T> T readAt(const std::vector<uint8_t> &buf, int off)
{
	T v{};

	if (off >= 0 && off + (int)sizeof(T) <= (int)buf.size()) {
		std::memcpy(&v, buf.data() + off, sizeof(T));
	}

	return v;
}

} // namespace

int main(int argc, char **argv)
{
	if (argc < 2) {
		std::fprintf(stderr,
			     "usage: %s <log.ulg> "
			     "[--a A][--b B][--c C][--coeffs F][--thr T][--rel R][--tau S][--persist S]"
			     "[--inject-motor M][--inject-t T][--inject-scale S][--dump F]\n",
			     argv[0]);
		return 2;
	}

	const char *ulog_path = argv[1];

	// Defaults from the DoorDash healthy hexa fit (re-derive per airframe).
	float a = 71.97f, b = 0.f, c = 2.88f;
	float thr = 2.5f, rel = 0.17f, tau = 0.2f, persist = 0.4f;

	int inject_motor = -1;
	double inject_t = 1e30;
	float inject_scale = 0.f;

	const char *dump_path = nullptr;
	const char *coeffs_path = nullptr;

	for (int i = 2; i + 1 < argc; i += 2) {
		const char *k = argv[i];
		const char *vs = argv[i + 1];
		const double v = std::strtod(vs, nullptr);

		if (!std::strcmp(k, "--dump")) { dump_path = vs; }

		else if (!std::strcmp(k, "--coeffs")) { coeffs_path = vs; }

		else if (!std::strcmp(k, "--a")) { a = v; }

		else if (!std::strcmp(k, "--b")) { b = v; }

		else if (!std::strcmp(k, "--c")) { c = v; }

		else if (!std::strcmp(k, "--thr")) { thr = v; }

		else if (!std::strcmp(k, "--rel")) { rel = v; }

		else if (!std::strcmp(k, "--tau")) { tau = v; }

		else if (!std::strcmp(k, "--persist")) { persist = v; }

		else if (!std::strcmp(k, "--inject-motor")) { inject_motor = (int)v; }

		else if (!std::strcmp(k, "--inject-t")) { inject_t = v; }

		else if (!std::strcmp(k, "--inject-scale")) { inject_scale = v; }

		else { std::fprintf(stderr, "unknown option %s\n", k); return 2; }
	}

	if (coeffs_path) {
		std::ifstream cf(coeffs_path);
		std::string ln;

		while (std::getline(cf, ln)) {
			if (ln.empty() || ln[0] == '#') { continue; }

			std::vector<float> vals;
			std::stringstream ss(ln);
			float x;

			while (ss >> x) { vals.push_back(x); }

			if (vals.size() >= 4) { a = vals[1]; b = vals[2]; c = vals[3]; }

			else if (vals.size() == 3) { a = vals[0]; b = vals[1]; c = vals[2]; }

			break;
		}
	}

	std::ifstream file(ulog_path, std::ios::binary);

	if (!file) {
		std::fprintf(stderr, "cannot open %s\n", ulog_path);
		return 2;
	}

	ulog_file_header_s file_header{};
	file.read((char *)&file_header, sizeof(file_header));
	const uint8_t magic[7] = {'U', 'L', 'o', 'g', 0x01, 0x12, 0x35};

	if (!file || std::memcmp(magic, file_header.magic, 7) != 0) {
		std::fprintf(stderr, "%s is not a ULog file\n", ulog_path);
		return 2;
	}

	MotorFailureDetector::Config cfg{};
	cfg.model_a = a;
	cfg.model_b = b;
	cfg.model_c = c;
	cfg.residual_lpf_tau_s = tau;
	cfg.threshold_a = thr;
	cfg.threshold_rel = rel;
	cfg.persistence_s = persist;

	MotorFailureDetector det;
	det.configure(cfg);

	FormatMap formats;

	enum class Topic : int8_t { kNone = -1, kEsc, kActuator };
	std::vector<Topic> id2topic;
	auto mapTopic = [&](uint16_t id, Topic t) {
		if (id >= id2topic.size()) { id2topic.resize(id + 1, Topic::kNone); }

		id2topic[id] = t;
	};

	// Field offsets, resolved from the log's FORMAT once the definitions are read.
	bool offsets_ready = false;
	int esc_count_max = 0, esc_arr_off = 0, esc_rep_size = 0, escr_cur_off = 0, escr_fn_off = 0;
	int act_ctrl_off = 0, act_ctrl_len = 0, act_rev_off = 0;
	auto resolveOffsets = [&]() {
		esc_arr_off = fieldOffset(formats, "esc_status", "esc", &esc_count_max);
		esc_rep_size = typeSize(formats, "esc_report");
		escr_cur_off = fieldOffset(formats, "esc_report", "esc_current");
		escr_fn_off = fieldOffset(formats, "esc_report", "actuator_function");
		act_ctrl_off = fieldOffset(formats, "actuator_motors", "control", &act_ctrl_len);
		act_rev_off = fieldOffset(formats, "actuator_motors", "reversible_flags");
		offsets_ready = true;
	};

	// ZOH cache: the latest actuator command vector (kept across esc samples).
	float command_cache[kMM];

	for (int m = 0; m < kMM; ++m) { command_cache[m] = NAN; }

	uint16_t reversible_cache = 0;

	float peak_resid[kMM] = {0};
	double trip_time[kMM];

	for (int m = 0; m < kMM; ++m) { trip_time[m] = -1.0; }

	std::vector<double> dump_t;
	std::vector<std::array<float, kMM>> dump_I, dump_rr, dump_rl;
	bool motor_seen[kMM] = {false};

	double t0 = -1.0, t_prev = -1.0;
	long evaluated = 0;

	std::vector<uint8_t> buf;
	ulog_message_header_s h;

	while (file.read((char *)&h, ULOG_MSG_HEADER_LEN)) {
		switch ((ULogMessageType)h.msg_type) {
		case ULogMessageType::FORMAT: {
				buf.resize(h.msg_size + 1);
				file.read((char *)buf.data(), h.msg_size);
				buf[h.msg_size] = 0;
				parseFormat((const char *)buf.data(), formats);
				break;
			}

		case ULogMessageType::ADD_LOGGED_MSG: {
				buf.resize(h.msg_size + 1);
				file.read((char *)buf.data(), h.msg_size);
				buf[h.msg_size] = 0;
				const uint8_t multi_id = buf[0];
				const uint16_t msg_id = buf[1] | ((uint16_t)buf[2] << 8);
				const char *name = (const char *)buf.data() + 3;

				if (multi_id == 0) {
					if (!std::strcmp(name, "esc_status")) { mapTopic(msg_id, Topic::kEsc); }

					else if (!std::strcmp(name, "actuator_motors")) { mapTopic(msg_id, Topic::kActuator); }
				}

				break;
			}

		case ULogMessageType::DATA: {
				uint16_t msg_id;
				file.read((char *)&msg_id, sizeof(msg_id));
				const int payload = h.msg_size - (int)sizeof(msg_id);
				const Topic topic = (msg_id < id2topic.size()) ? id2topic[msg_id] : Topic::kNone;

				if (topic == Topic::kNone || payload < 0) {
					file.seekg(payload, std::ios::cur);
					break;
				}

				buf.resize(payload);
				file.read((char *)buf.data(), payload);

				if (!offsets_ready) { resolveOffsets(); }

				if (topic == Topic::kActuator) {
					for (int m = 0; m < kMM && m < act_ctrl_len; ++m) {
						command_cache[m] = readAt<float>(buf, act_ctrl_off + 4 * m);
					}

					reversible_cache = readAt<uint16_t>(buf, act_rev_off);
					break;
				}

				// --- esc_status sample: one detector step (timestamp is the first field) ---
				const double t_us = (double)readAt<uint64_t>(buf, 0);

				if (t0 < 0) { t0 = t_us; }

				const double t = (t_us - t0) / 1e6;
				const float dt = (t_prev < 0) ? (1.f / 184.f) : (float)(t - t_prev);
				t_prev = t;

				if (dt <= 0.f || dt > MotorFailureDetector::kMaxGap) { break; } // gap / out of order -> skip

				float command[kMM], current[kMM];
				bool reversible[kMM], enabled[kMM];

				for (int m = 0; m < kMM; ++m) {
					command[m] = NAN; current[m] = 0.f; reversible[m] = false; enabled[m] = false;
				}

				for (int e = 0; e < esc_count_max; ++e) {
					const int base = esc_arr_off + esc_rep_size * e;
					const int fn = readAt<uint8_t>(buf, base + escr_fn_off);

					if (fn < 101 || fn >= 101 + kMM) { continue; } // unmapped ESC slot

					const int m = fn - 101;
					enabled[m] = true;
					motor_seen[m] = true;
					current[m] = readAt<float>(buf, base + escr_cur_off);

					if (m == inject_motor && t >= inject_t) { current[m] *= inject_scale; }

					command[m] = command_cache[m];
					reversible[m] = (reversible_cache >> m) & 1;
				}

				det.update(kMM, (hrt_abstime)readAt<uint64_t>(buf, 0), command, current, reversible, enabled);
				++evaluated;

				if (dump_path) {
					std::array<float, kMM> mi, mr, ml;

					for (int m = 0; m < kMM; ++m) {
						if (det.status(m).excluded) {
							mi[m] = mr[m] = ml[m] = NAN;

						} else {
							mi[m] = current[m];
							mr[m] = det.status(m).residual;
							ml[m] = det.status(m).residual_lpf;
						}
					}

					dump_t.push_back(t);
					dump_I.push_back(mi);
					dump_rr.push_back(mr);
					dump_rl.push_back(ml);
				}

				for (int m = 0; m < kMM; ++m) {
					if (!motor_seen[m]) { continue; }

					const float ar = std::fabs(det.status(m).residual_lpf);

					if (!std::isnan(ar) && ar > peak_resid[m]) { peak_resid[m] = ar; }

					if (det.status(m).failed && trip_time[m] < 0) { trip_time[m] = t; }
				}

				break;
			}

		default:
			file.seekg(h.msg_size, std::ios::cur);
			break;
		}
	}

	std::printf("# %s\n", ulog_path);
	std::printf("# samples=%ld  model I=%.2f*u^2%+.2f*u%+.2f  thr=%.2f%+.2f*I_exp  tau=%.2fs persist=%.2fs\n",
		    evaluated, (double)a, (double)b, (double)c, (double)thr, (double)rel,
		    (double)tau, (double)persist);
	std::printf("# motor   peak|LPF(r)|[A]   verdict\n");

	bool any = false;

	for (int m = 0; m < kMM; ++m) {
		if (!motor_seen[m]) { continue; }

		if (trip_time[m] >= 0) {
			std::printf("  %3d   %14.2f    FAILED @ %.2fs\n", m, (double)peak_resid[m], trip_time[m]);
			any = true;

		} else {
			std::printf("  %3d   %14.2f    ok\n", m, (double)peak_resid[m]);
		}
	}

	if (dump_path) {
		std::ofstream df(dump_path);

		if (!df) {
			std::fprintf(stderr, "cannot write dump %s\n", dump_path);

		} else {
			df << "# mfd_replay dump: per motor I (measured), r (residual), rlpf (LPF residual)\n";
			df << "# a=" << a << " b=" << b << " c=" << c
			   << " thr=" << thr << " rel=" << rel << " tau=" << tau
			   << " persist=" << persist << "\n";

			for (int m = 0; m < kMM; ++m) {
				if (!motor_seen[m]) { continue; }

				if (trip_time[m] >= 0) {
					df << "# motor " << m << " FAILED @ " << trip_time[m] << " peak " << peak_resid[m] << "\n";

				} else {
					df << "# motor " << m << " ok peak " << peak_resid[m] << "\n";
				}
			}

			df << "t";

			for (int m = 0; m < kMM; ++m) {
				if (motor_seen[m]) { df << ",m" << m << "_I,m" << m << "_r,m" << m << "_rlpf"; }
			}

			df << "\n";

			for (size_t r = 0; r < dump_t.size(); ++r) {
				df << dump_t[r];

				for (int m = 0; m < kMM; ++m) {
					if (motor_seen[m]) {
						df << "," << dump_I[r][m] << "," << dump_rr[r][m] << "," << dump_rl[r][m];
					}
				}

				df << "\n";
			}

			std::printf("# wrote per-sample dump: %s (%zu rows)\n", dump_path, dump_t.size());
		}
	}

	return any ? 1 : 0;
}
