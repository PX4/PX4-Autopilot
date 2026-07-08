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
 * @file FailureInjectionGnss.cpp
 *
 * GNSS Wrong/Garbage failure mutators: the pure cores (apply_gnss_wrong / apply_gnss_garbage)
 * plus the thin param-reading wrappers (gnss_wrong / gnss_garbage). The cores take an explicit
 * profile + now_us so they are deterministic; the wrappers read the SYS_FAIL_GPS_* params.
 * Exercised by FailureInjectionGnssTest, a functional gtest (functional gtests link the
 * parameters lib, so the wrappers resolve).
 */

#include "FailureInjection.hpp"

#include <cmath>

#include <lib/geo/geo.h>
#include <lib/parameters/param.h>
#include <mathlib/mathlib.h>

namespace failure_injection
{

namespace
{
// Wrong-failure (false position) hardcoded magnitudes.
constexpr float kWrongDriftRateMs  = 2.0f;   // drift speed for the drift profiles [m/s]
constexpr float kWrongBearingDeg   = 90.0f;  // drift/jump direction (0=N, 90=E)
constexpr float kWrongStaticJumpM  = 100.0f; // jump distance for the static profile [m]
// Onset ramp for the smooth-drift profile [s]. Chosen so the injected position and velocity
// offsets both start at zero, giving a C1-continuous handover the EKF adopts instead of gating.
constexpr float kWrongBlendTau     = 5.0f;

// Garbage-failure constants anchored to real degraded u-blox behaviour seen in flight logs
// (nominal noise_per_ms ~0 / jamming_indicator ~0; degraded ~110 / ~90).
constexpr int32_t  kGarbageNoisePerMs  = 110;
constexpr int32_t  kGarbageIndicator   = 90;
constexpr float    kGarbageSacc        = 100.0f; // speed-accuracy spike; the common EKF trip
constexpr float    kGarbageEphInvalid  = 1.0e6f; // horizontal/vertical accuracy on total loss
constexpr float    kGarbageDop         = 99.99f;
constexpr float    kGarbageEphDegraded = 20.0f;
constexpr float    kGarbageEpvDegraded = 30.0f;
constexpr uint8_t  kGarbageSatsDegraded = 4;
constexpr uint16_t kGarbageAgc          = 6000;  // elevated automatic gain control under interference
constexpr float    kGarbageVelNoiseMs   = 2.0f;  // velocity jitter on a degraded fix [m/s]
constexpr uint64_t kGarbageBlackoutBaseUs = 1000000; // 1 s
constexpr uint64_t kGarbageBlackoutRandUs = 5000000; // + onset_us % this  => 1..6 s dropout

// xorshift32 PRNG for the degraded-fix jitter. Seeded per call from now_us, so it needs no
// persistent state and varies every cycle.
inline uint32_t xorshift32(uint32_t &s)
{
	s ^= s << 13;
	s ^= s >> 17;
	s ^= s << 5;
	return s;
}

// Uniform pseudo-random float in [-1, 1).
inline float rand_sym(uint32_t &s)
{
	return (float)(xorshift32(s) >> 8) * (1.0f / 8388608.0f) - 1.0f;
}

// Capture the onset time on the rising edge of a failure mode.
void arm_onset(GnssFailureState &state, Mode mode, uint64_t now_us)
{
	if (state.active != mode) {
		state.active = mode;
		state.onset_us = now_us;
	}
}

} // namespace

void apply_gnss_wrong(sensor_gps_s &gps, GnssFailureState &state, int32_t profile, uint64_t now_us)
{
	arm_onset(state, Mode::Wrong, now_us);

	const float t = (float)((double)(now_us - state.onset_us) * 1e-6);
	const float R = kWrongDriftRateMs;
	const float brg = math::radians(kWrongBearingDeg);
	const float cb = cosf(brg);
	const float sb = sinf(brg);

	float offset_m = 0.f; // accumulated position offset along the bearing [m]
	float vel_m_s = 0.f;  // coherent velocity offset along the bearing [m/s]

	switch (profile) {
	case 0: // static jump / teleport
		offset_m = kWrongStaticJumpM;
		break;

	case 1: // constant drift
		offset_m = R * t;
		vel_m_s = R;
		break;

	case 3: { // altitude drift -- vertical ramp (coherent vel_d); leaves offset_m 0 (no horizontal move)
			const float e = expf(-t / kWrongBlendTau);
			const float d = R * (t - kWrongBlendTau * (1.f - e));
			gps.altitude_msl_m       -= (double)d; // walk the reported altitude down
			gps.altitude_ellipsoid_m -= (double)d;
			gps.vel_d_m_s += R * (1.f - e);        // NED, down positive
			break;
		}

	default: { // 2: smooth drift -- expanding, C1-continuous at onset
			const float e = expf(-t / kWrongBlendTau);
			vel_m_s = R * (1.f - e);
			offset_m = R * (t - kWrongBlendTau * (1.f - e));
			break;
		}
	}

	// Position offset in double (lat/lon are float64); pole-guard the longitude scaling.
	// For the altitude profile offset_m/vel_m_s are 0, so this leaves the horizontal state intact.
	double clat = cos(math::radians(gps.latitude_deg));

	if (fabs(clat) < 1e-6) {
		clat = (clat < 0.0) ? -1e-6 : 1e-6;
	}

	gps.latitude_deg  += math::degrees((double)(offset_m * cb) / CONSTANTS_RADIUS_OF_EARTH);
	gps.longitude_deg += math::degrees((double)(offset_m * sb) / CONSTANTS_RADIUS_OF_EARTH) / clat;

	// Coherent velocity so GPS position-rate matches GPS velocity (keeps EKF innovations small).
	gps.vel_n_m_s += vel_m_s * cb;
	gps.vel_e_m_s += vel_m_s * sb;
	gps.vel_m_s = sqrtf(gps.vel_n_m_s * gps.vel_n_m_s + gps.vel_e_m_s * gps.vel_e_m_s);
	gps.cog_rad = atan2f(gps.vel_e_m_s, gps.vel_n_m_s);

	// Fix quality (fix_type/eph/sats) and the report flags are left untouched: the false
	// position is delivered while the fix still looks perfectly healthy.
}

void apply_gnss_garbage(sensor_gps_s &gps, GnssFailureState &state, int32_t mode, uint64_t now_us)
{
	arm_onset(state, Mode::Garbage, now_us);

	// Phase-1 (loss) duration: 1..6 s. Derived from the onset timestamp so it varies
	// naturally between triggers without any PRNG state to carry.
	const uint64_t blackout_us = kGarbageBlackoutBaseUs + (state.onset_us % kGarbageBlackoutRandUs);
	const uint64_t elapsed = now_us - state.onset_us;

	// Full-loss mode, or phase 1 of the two-phase mode: no usable fix.
	const bool loss = (mode == 1) || (mode == 2 && elapsed < blackout_us);

	// Interference indicators fluctuate frame-to-frame around the jammed baseline.
	uint32_t rng = (uint32_t)now_us | 1u;
	gps.noise_per_ms = kGarbageNoisePerMs + (int32_t)(20.f * rand_sym(rng));
	gps.jamming_indicator = kGarbageIndicator + (int32_t)(15.f * rand_sym(rng));
	gps.automatic_gain_control = (uint16_t)(kGarbageAgc + (int32_t)(500.f * rand_sym(rng)));

	if (loss) {
		gps.fix_type = 0; // no fix (0 is a valid no-fix value)
		gps.satellites_used = 0;
		gps.eph = kGarbageEphInvalid;
		gps.epv = kGarbageEphInvalid;
		gps.hdop = kGarbageDop;
		gps.vdop = kGarbageDop;
		gps.s_variance_m_s = kGarbageSacc;
		gps.c_variance_rad = kGarbageSacc;
		gps.vel_ned_valid = false;
		gps.vel_m_s = 0.f;
		gps.vel_n_m_s = 0.f;
		gps.vel_e_m_s = 0.f;
		gps.vel_d_m_s = 0.f;

	} else {
		// Degraded-but-present fix: the quality metrics collapse and fluctuate each cycle, and
		// the position/velocity jitter within that degraded accuracy. Bounded noise, not a
		// coherent teleport -- moving the position elsewhere is the Wrong failure.
		gps.fix_type = sensor_gps_s::FIX_TYPE_3D;

		const int32_t sats = kGarbageSatsDegraded + (int32_t)(2.5f * rand_sym(rng)); // ~2..6
		gps.satellites_used = (uint8_t)math::max(sats, 1);

		gps.s_variance_m_s = kGarbageSacc + 20.f * rand_sym(rng);            // ~80..120
		gps.eph = kGarbageEphDegraded + 20.f * fabsf(rand_sym(rng));         // ~20..40 m
		gps.epv = kGarbageEpvDegraded + 24.f * fabsf(rand_sym(rng));         // ~30..54 m
		gps.hdop = 15.f + 14.f * fabsf(rand_sym(rng));
		gps.vdop = 16.f + 16.f * fabsf(rand_sym(rng));

		// Jitter position (~eph) and velocity so the sample is genuinely noisy, consistent with
		// the reported accuracy. Pole-guard the longitude scaling.
		double clat = cos(math::radians(gps.latitude_deg));

		if (fabs(clat) < 1e-6) {
			clat = (clat < 0.0) ? -1e-6 : 1e-6;
		}

		gps.latitude_deg  += math::degrees((double)(gps.eph * rand_sym(rng)) / CONSTANTS_RADIUS_OF_EARTH);
		gps.longitude_deg += math::degrees((double)(gps.eph * rand_sym(rng)) / CONSTANTS_RADIUS_OF_EARTH) / clat;
		gps.vel_n_m_s += kGarbageVelNoiseMs * rand_sym(rng);
		gps.vel_e_m_s += kGarbageVelNoiseMs * rand_sym(rng);
		gps.vel_m_s = sqrtf(gps.vel_n_m_s * gps.vel_n_m_s + gps.vel_e_m_s * gps.vel_e_m_s);
	}

	gps.jamming_state = sensor_gps_s::JAMMING_STATE_DETECTED;
}

void gnss_wrong(sensor_gps_s &gps, GnssFailureState &state, uint64_t now_us)
{
	// Cache the handle once; param_get leaves the default untouched if the param is missing.
	static const param_t handle = param_find("SYS_FAIL_GPS_WRG");
	int32_t profile = 2;
	param_get(handle, &profile);
	apply_gnss_wrong(gps, state, profile, now_us);
}

void gnss_garbage(sensor_gps_s &gps, GnssFailureState &state, uint64_t now_us)
{
	static const param_t handle = param_find("SYS_FAIL_GPS_GRB");
	int32_t mode = 2;
	param_get(handle, &mode);
	apply_gnss_garbage(gps, state, mode, now_us);
}

bool process_gnss(const Config &config, uint8_t uorb_instance, sensor_gps_s &gps,
		  Stuck<sensor_gps_s> &stuck, GnssFailureState &state, uint64_t now_us)
{
	const Mode m = config.mode(failure_injection_s::FAILURE_UNIT_SENSOR_GPS, uorb_instance + 1);

	if (!process(m, gps, stuck)) {
		return false;
	}

	switch (m) {
	case Mode::Wrong:   gnss_wrong(gps, state, now_us); break;

	case Mode::Garbage: gnss_garbage(gps, state, now_us); break;

	case Mode::Ok:      gnss_reset(state); break;

	default: break;
	}

	return true;
}

} // namespace failure_injection
