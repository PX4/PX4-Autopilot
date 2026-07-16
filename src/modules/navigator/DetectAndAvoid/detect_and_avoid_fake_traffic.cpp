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
 * @file detect_and_avoid_fake_traffic.cpp
 *
 * Generate synthetic traffic for manual Detect and Avoid testing
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "detect_and_avoid.h"

#include <containers/LockGuard.hpp>
#include <cstring>
#include <lib/geo/geo.h>

using namespace time_literals;

namespace
{
// Each fake-traffic mode is a deterministic script. `run_fake_traffic()` only
// stores the selected mode and an ownship position snapshot; `process_fake_traffic()`
// later publishes the scripted reports that are due from the navigator update loop.
static constexpr uint16_t kDefaultFakeTrafficFlags = DetectAndAvoid::kFakeTrafficDefaultFlags;
static constexpr uint16_t kCallsignNotValidFlags = kDefaultFakeTrafficFlags & ~transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN;
static constexpr uint16_t kVelocityNotValidFlags = kDefaultFakeTrafficFlags & ~transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;

// Publish one final far-away sample to clear the same traffic entry cleanly.
static constexpr float kFakeTrafficResolveDistance = 5000.f;

struct fake_traffic_script_step_s {
	uint32_t icao_address;
	const char *callsign;
	float distance;
	float direction;
	float traffic_heading;
	float altitude_diff;
	float hor_velocity;
	float ver_velocity;
	int emitter_type;
	uint16_t flags;
	hrt_abstime delay_to_next;
};

// Three isolated sequences that exercise ICAO, callsign, and UAS-ID fallback.
static constexpr fake_traffic_script_step_s kUniqueIdScript[] {
	{
		7249787, "6E9F7B", 1500.f, 1.0f, 0.0f, 0.0f, 100.0f, 10.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 2_s
	},
	{
		7249787, "6E9F7B", 800.f, 1.0f, 0.0f, 0.0f, 100.0f, 10.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 2_s
	},
	{
		7249787, "6E9F7B", 200.f, 1.0f, 0.0f, 0.0f, 100.0f, 10.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 2_s
	},
	{
		7249787, "6E9F7B", kFakeTrafficResolveDistance, 1.0f, 0.0f, 0.0f, 100.0f, 10.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 2_s
	},
	{
		0, "LX00777A", 1500.f, 1.0f, 0.0f, 0.0f, 100.0f, 10.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 2_s
	},
	{
		0, "LX00777A", 800.f, 1.0f, 0.0f, 0.0f, 100.0f, 10.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 2_s
	},
	{
		0, "LX00777A", 200.f, 1.0f, 0.0f, 0.0f, 100.0f, 10.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 2_s
	},
	{
		0, "LX00777A", kFakeTrafficResolveDistance, 1.0f, 0.0f, 0.0f, 100.0f, 10.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 2_s
	},
	{
		0, "NOCALL", 1500.f, 1.0f, 0.0f, 0.0f, 100.0f, 10.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kCallsignNotValidFlags, 2_s
	},
	{
		0, "NOCALL", 800.f, 1.0f, 0.0f, 0.0f, 100.0f, 10.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kCallsignNotValidFlags, 2_s
	},
	{
		0, "NOCALL", 200.f, 1.0f, 0.0f, 0.0f, 100.0f, 10.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kCallsignNotValidFlags, 2_s
	},
	{
		0, "NOCALL", kFakeTrafficResolveDistance, 1.0f, 0.0f, 0.0f, 100.0f, 10.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kCallsignNotValidFlags, 2_s
	},
};

// Deterministic ranges chosen to fill, evict, and ignore entries in the traffic buffer.
static constexpr fake_traffic_script_step_s kSpamNewScript[] {
	{
		3673751, "", 1200.f, 1.0f, 0.0f, 0.0f, 20.0f, 0.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 1_s
	}, // 380E97 LOW
	{
		6813217, "", 900.f, 1.0f, 0.0f, 0.0f, 20.0f, 0.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 1_s
	}, // 67F621 LOW
	{
		11329143, "", 700.f, 1.0f, 0.0f, 0.0f, 20.0f, 0.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 1_s
	}, // ACDE77 MEDIUM
	{
		4584054, "", 500.f, 1.0f, 0.0f, 0.0f, 20.0f, 0.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 1_s
	}, // 45F276 HIGH
	{
		3447055, "", 100.f, 1.0f, 0.0f, 0.0f, 20.0f, 0.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 1_s
	}, // 34990F CRITICAL
	{
		15426008, "", 130.f, 1.0f, 0.0f, 0.0f, 20.0f, 0.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 1_s
	}, // EB61D8 CRITICAL
	{
		10611378, "", 550.f, 1.0f, 0.0f, 0.0f, 20.0f, 0.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 1_s
	}, // A1EAB2 HIGH
	{
		12971298, "", 1100.f, 1.0f, 0.0f, 0.0f, 20.0f, 0.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 1_s
	}, // C5ED22 LOW -> ignored
	{
		11728980, "", 750.f, 1.0f, 0.0f, 0.0f, 20.0f, 0.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 1_s
	}, // B2F854 MEDIUM -> ignored
	{
		14771352, "", 140.f, 1.0f, 0.0f, 0.0f, 20.0f, 0.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, 1_s
	}, // E16498 CRITICAL
};

static constexpr fake_traffic_script_step_s kFlagsScript[] {
	{
		0, "L07NOVEL", 100.f, 1.0f, 0.0f, 0.0f, 100.0f, 10.0f,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kVelocityNotValidFlags, 0
	}
};

template <size_t N>
bool get_script_step(const fake_traffic_script_step_s(&script)[N], const uint8_t step_idx, fake_traffic_script_step_s &step)
{
	if (step_idx >= N) {
		return false;
	}

	step = script[step_idx];
	return true;
}

bool get_queue_fill_step(const uint8_t report_idx, fake_traffic_script_step_s &step)
{
	constexpr uint8_t batch_count = 3;
	constexpr uint8_t reports_per_batch = transponder_report_s::ORB_QUEUE_LENGTH;
	constexpr uint8_t total_report_count = batch_count * reports_per_batch;
	constexpr uint8_t replace_count = (reports_per_batch < (kDaaMaxTraffic - 1))
					  ? reports_per_batch : (kDaaMaxTraffic - 1);
	constexpr hrt_abstime batch_delay = 2_s;
	constexpr uint32_t main_icao = 0xDDF0A1u;
	constexpr uint32_t step_2_icao_base = 0x510000u;
	constexpr uint32_t step_3_icao_base = 0x520000u;
	constexpr float main_distance = 100.f;
	constexpr float step_2_first_distance = 800.f;
	constexpr float step_2_distance_delta = 10.f;
	constexpr float step_3_distance_delta = 20.f;
	constexpr float step_2_closest_distance = step_2_first_distance - ((reports_per_batch - 1u) * step_2_distance_delta);
	constexpr float step_3_first_distance = step_2_closest_distance - (replace_count * step_3_distance_delta);

	if (report_idx >= total_report_count) {
		return false;
	}

	// `queue_fill` is a three-batch scenario. Each batch publishes exactly
	// `reports_per_batch` reports, but only one batch is allowed per navigator
	// cycle so the queue drain and the scripted step count stay distinct.
	const uint8_t batch_idx = report_idx / reports_per_batch;
	const uint8_t report_idx_in_batch = report_idx % reports_per_batch;
	const bool end_of_batch = (report_idx_in_batch + 1u) == reports_per_batch;
	const hrt_abstime delay_to_next = (end_of_batch && (batch_idx + 1u) < batch_count) ? batch_delay : 0;

	switch (batch_idx) {
	case 0:
		step = {
			main_icao, "DDF0A1", main_distance, 0.0f, 0.0f, 0.0f, 20.0f, 0.0f,
			transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, delay_to_next
		};
		return true;

	case 1:
		step = {
			static_cast<uint32_t>(step_2_icao_base + report_idx_in_batch + 1u), "",
			step_2_first_distance - (report_idx_in_batch * step_2_distance_delta),
			0.1f * static_cast<float>(report_idx_in_batch), 0.0f, 0.0f, 20.0f, 0.0f,
			transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, delay_to_next
		};
		return true;

	case 2:
		step = {
			static_cast<uint32_t>(step_3_icao_base + report_idx_in_batch + 1u), "",
			step_3_first_distance + (report_idx_in_batch * step_3_distance_delta),
			0.1f * static_cast<float>(report_idx_in_batch), 0.0f, 0.0f, 20.0f, 0.0f,
			transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, delay_to_next
		};
		return true;

	default:
		return false;
	}
}

const char *fake_traffic_mode_to_string(const DetectAndAvoid::FakeTraffMode mode)
{
	switch (mode) {
	case DetectAndAvoid::FakeTraffMode::kUniqueIds:
		return "unique_ids";

	case DetectAndAvoid::FakeTraffMode::kEscalation:
		return "escalation";

	case DetectAndAvoid::FakeTraffMode::kSpamSame:
		return "spam_same";

	case DetectAndAvoid::FakeTraffMode::kSpamNew:
		return "spam_new";

	case DetectAndAvoid::FakeTraffMode::kFlags:
		return "flags";

	case DetectAndAvoid::FakeTraffMode::kQueueFill:
		return "queue_fill";

	default:
		return "";
	}
}

bool get_fake_traffic_step(const DetectAndAvoid::FakeTraffMode mode, const uint8_t step_idx, fake_traffic_script_step_s &step)
{
	switch (mode) {
	case DetectAndAvoid::FakeTraffMode::kUniqueIds:
		return get_script_step(kUniqueIdScript, step_idx, step);

	case DetectAndAvoid::FakeTraffMode::kSpamNew:
		return get_script_step(kSpamNewScript, step_idx, step);

	case DetectAndAvoid::FakeTraffMode::kFlags:
		return get_script_step(kFlagsScript, step_idx, step);

	case DetectAndAvoid::FakeTraffMode::kEscalation: {
			static constexpr uint8_t kStepCount = 30;
			static constexpr hrt_abstime kDelayEsc = 2_s;

			if (step_idx >= kStepCount) {
				return false;
			}

			step = {
				10436515, "9F3FA3", 3000.f - (100.f * step_idx), 1.0f, 0.0f, 0.0f, 50.0f, 0.0f,
				transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, kDelayEsc
			};
			return true;
		}

	case DetectAndAvoid::FakeTraffMode::kSpamSame: {
			static constexpr uint8_t kStepCount = 40;
			static constexpr hrt_abstime kDelaySpam = 100_ms;

			if (step_idx >= kStepCount) {
				return false;
			}

			step = {
				6420348, "61F77C", 200.f, 1.0f, 0.0f, 0.0f, 20.0f, 0.0f,
				transponder_report_s::ADSB_EMITTER_TYPE_LIGHT, kDefaultFakeTrafficFlags, kDelaySpam
			};
			return true;
		}

	case DetectAndAvoid::FakeTraffMode::kQueueFill:
		return get_queue_fill_step(step_idx, step);

	default:
		return false;
	}
}
} // namespace

void DetectAndAvoid::run_fake_traffic(const FakeTraffMode mode, const double lat_uav, const double lon_uav,
				      const float alt_uav)
{
	const char *const mode_name = fake_traffic_mode_to_string(mode);

	if (mode_name[0] == '\0') {
		PX4_ERR("DAA: unknown fake traffic mode %u", static_cast<unsigned>(mode));
		return;
	}

	{
		const LockGuard lg{_fake_traffic_mutex};
		_fake_traffic_state.clear();
		_fake_traffic_state.active = true;
		_fake_traffic_state.mode = mode;
		_fake_traffic_state.origin.lat = lat_uav;
		_fake_traffic_state.origin.lon = lon_uav;
		_fake_traffic_state.origin.alt = alt_uav;
	}

	PX4_WARN("DAA: fake traffic can trigger configured actions, including terminate");
	PX4_INFO("DAA: fake traffic '%s' scheduled", mode_name);
}

void DetectAndAvoid::stop_fake_traffic()
{
	const LockGuard lg{_fake_traffic_mutex};
	_fake_traffic_state.clear();
}

void DetectAndAvoid::process_fake_traffic()
{
	const hrt_abstime now = hrt_absolute_time();
	uint8_t max_reports_per_cycle = 1;

	{
		const LockGuard lg{_fake_traffic_mutex};

		if (!_fake_traffic_state.active) {
			return;
		}

		// `queue_fill` is the only burst mode. All other modes publish at most one
		// report per navigator update, even if a future script adds zero-delay steps.
		max_reports_per_cycle = (_fake_traffic_state.mode == DetectAndAvoid::FakeTraffMode::kQueueFill) ?
					transponder_report_s::ORB_QUEUE_LENGTH : 1u;
	}

	for (uint8_t published_reports = 0; published_reports < max_reports_per_cycle; ++published_reports) {
		fake_traffic_script_step_s step{};
		fake_traffic_origin_s origin{};

		{
			const LockGuard lg{_fake_traffic_mutex};

			if (!_fake_traffic_state.active) {
				return;
			}

			if ((_fake_traffic_state.next_publish_at != 0) && (now < _fake_traffic_state.next_publish_at)) {
				return;
			}

			if (!get_fake_traffic_step(_fake_traffic_state.mode, _fake_traffic_state.next_step_idx, step)) {
				_fake_traffic_state.clear();
				return;
			}

			origin = _fake_traffic_state.origin;

			++_fake_traffic_state.next_step_idx;
			_fake_traffic_state.next_publish_at = now + step.delay_to_next;
		}

		SyntheticTrafficReport report{};
		report.icao_address = step.icao_address;
		report.callsign = step.callsign;
		report.distance = step.distance;
		report.direction = step.direction;
		report.traffic_heading = step.traffic_heading;
		report.altitude_diff = step.altitude_diff;
		report.hor_velocity = step.hor_velocity;
		report.ver_velocity = step.ver_velocity;
		report.emitter_type = step.emitter_type;
		report.lat_uav = origin.lat;
		report.lon_uav = origin.lon;
		report.alt_uav = origin.alt;
		report.flags = step.flags;

		fake_traffic(report);

		if (step.delay_to_next != 0) {
			return;
		}
	}
}

void DetectAndAvoid::fake_traffic(const SyntheticTrafficReport &report)
{
	double lat{0.0};
	double lon{0.0};

	waypoint_from_heading_and_distance(report.lat_uav, report.lon_uav, report.direction, report.distance, &lat,
					   &lon);
	float alt = report.alt_uav + report.altitude_diff;

	transponder_report_s tr{};

	tr.timestamp = hrt_absolute_time();
	tr.icao_address = report.icao_address;
	tr.lat = lat;
	tr.lon = lon;
	tr.altitude_type = 0;
	tr.altitude = alt;
	tr.heading = report.traffic_heading;
	tr.hor_velocity	= report.hor_velocity;
	tr.ver_velocity = report.ver_velocity;
	strncpy(&tr.callsign[0], report.callsign != nullptr ? report.callsign : "", sizeof(tr.callsign) - 1);
	tr.callsign[sizeof(tr.callsign) - 1] = 0;
	tr.emitter_type = report.emitter_type;
	tr.tslc = 2; // seconds since last communication
	tr.flags = report.flags;
	tr.squawk = 6667;

#ifndef BOARD_HAS_NO_UUID
	px4_guid_t px4_guid {};

	if (board_get_px4_guid(px4_guid) == PX4_GUID_BYTE_LENGTH) {
		memcpy(tr.uas_id, px4_guid, sizeof(px4_guid_t));
		tr.uas_id[PX4_GUID_BYTE_LENGTH - 1] ^= 0x01;

	} else {
		for (int i = 0; i < PX4_GUID_BYTE_LENGTH; ++i) {
			tr.uas_id[i] = 0xe0 + i;
		}
	}

#else

	for (int i = 0; i < PX4_GUID_BYTE_LENGTH ; i++) {
		tr.uas_id[i] = 0xe0 + i;
	}

#endif /* BOARD_HAS_NO_UUID */

	PX4_DEBUG("Publish fake traffic");
	_fake_traffic_pub.publish(tr);
}
