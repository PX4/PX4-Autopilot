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
 * Reads the logs through EscLogReader like mfd_replay. Every log is a single streaming pass that only
 * accumulates the six sums a least-squares line needs, so memory is constant and unrelated to log size
 * or corpus size: pass the whole corpus at once and the fit is pooled over all of it, which is what
 * the detector wants (one model shared by every motor).
 */

#include "EscLogReader.hpp"

#include <array>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <memory>

namespace
{
constexpr int kMM = MotorFailureDetector::kMaxMotors;

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

class Fit : public EscLogReader
{
public:
	bool seen(int m) const { return _seen[m]; }
	const Sums &motor(int m) const { return _per_motor[m]; }

protected:
	void escRecord(uint64_t /*record_us*/, const EscSample *samples, int count) override
	{
		for (int k = 0; k < count; ++k) {
			const EscSample &s = samples[k];

			// Count each ESC sample once. A motor the allocation switched off or in reversible mode (NAN),
			// or an ESC reporting no current describes nothing about the healthy forward model.
			if (!s.fresh || !std::isfinite(s.current) || s.current <= FLT_EPSILON || !std::isfinite(s.command)) {
				continue;
			}

			_seen[s.motor] = true;
			_per_motor[s.motor].add(s.command, s.current);
		}
	}

private:
	std::array<Sums, kMM> _per_motor {};
	std::array<bool, kMM> _seen {};
};
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

		auto fit = std::make_shared<Fit>();

		if (!readLog(argv[arg], fit)) { continue; } // reported; keep going so one bad log does not lose the corpus

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
