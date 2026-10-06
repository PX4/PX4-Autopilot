/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file GnssSelector.hpp
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <px4_platform_common/defines.h>
#include <uORB/topics/sensor_gnss.h>
#include <uORB/topics/vehicle_gnss.h>

using namespace time_literals;

/*
 * Selects the receiver whose samples vehicle_gnss carries; the caller keeps the samples and publishes the selected
 * receiver's. Every switch makes EKF2 reset or restart its GNSS position, so short changes in either receiver ride
 * through a hold before the selection moves.
 *
 * A receiver is usable while its latest sample passed its checks and came without a gap in its output. It has failed
 * when it had no usable sample for FAIL_TIME_US, which covers a lost fix, sustained check failures, a receiver that
 * stopped publishing and an update rate that dropped below about 1 Hz. Intermittent failures are caught by the
 * availability: the fraction of recent time it was usable. A failed receiver is replaced at once. While armed, a receiver that was
 * left because it failed is selected again only when the selected one fails, as one that failed is likely to fail
 * again in the same flight.
 *
 * Receivers rank by being usable, then by meeting the accuracy requirements (the strict checks), then, without a
 * preferred receiver, by an RTK fixed solution. A receiver that meets the requirements replaces the selected one when
 * it ranks higher through the hold time. The preferred receiver ranks above another one of the same rank that meets
 * the requirements: it is kept while it meets them, whatever the other one reports, and left when it hasn't met them
 * through the hold while the other one has. A moving-base rover reports a better fix and accuracy while being the
 * worse position source, so an RTK fixed solution never outranks the preferred receiver. While disarmed the preferred
 * receiver is selected whenever it publishes: a vehicle whose preferred receiver fails its checks shouldn't take off
 * on the other one.
 */
class GnssSelector
{
public:
	// A receiver that hasn't published for this long is dropped
	static constexpr hrt_abstime GNSS_TIMEOUT_US = 2_s;

	// Without a usable sample for this long the selected receiver has failed, and any usable receiver replaces it
	static constexpr hrt_abstime FAIL_TIME_US = 2_s;

	// A single failed sample makes a receiver unusable for about a second in flight, which costs it 0.1 availability
	// with this time constant; a receiver usable a third of the time costs the margin within a few seconds
	static constexpr hrt_abstime AVAILABILITY_TIME_CONSTANT_US = 10_s;
	// A usable receiver whose availability is higher by this margin replaces the selected one. Moves to the preferred
	// or a higher ranked receiver accept half of it, so that the selected receiver doesn't take over again right after.
	static constexpr float AVAILABILITY_MARGIN = 0.2f;

	// How long a receiver must rank higher before the selection moves to it. While disarmed a receiver is usable only
	// once the strict checks held for GNSS_REQ_TIME. While armed the hold doubles with every such switch, so that
	// receivers going in and out of the requirements for longer than the hold can't keep resetting EKF2.
	static constexpr hrt_abstime SWITCH_HOLD_ARMED_US = 10_s;
	static constexpr hrt_abstime SWITCH_HOLD_DISARMED_US = 2_s;
	static constexpr uint8_t SWITCH_HOLD_MAX_DOUBLINGS = 4;

	// A sample is late, a gap in the receiver's output, when its interval exceeds this many times the receiver's recent
	// interval, and at least LATE_MIN_US. The recent interval follows a lower rate within a sample, as receivers slow
	// down for reasons that don't make their samples worse, such as processing corrections: only the first sample at
	// the lower rate is late. Below about 1 Hz, the first two intervals at the lower rate add up to FAIL_TIME_US.
	static constexpr float LATE_INTERVAL_RATIO = 3.f;
	static constexpr hrt_abstime LATE_MIN_US = 300_ms;
	// The recent interval filter is used once it has this many samples. It weights intervals by their duration, so
	// that samples delivered in bursts don't shorten it.
	static constexpr float INTERVAL_TIME_CONSTANT_S = 1.f;
	static constexpr uint8_t INTERVAL_SETTLE_SAMPLES = 10;

	static constexpr int GNSS_MAX_RECEIVERS = 2;

	GnssSelector();
	~GnssSelector() = default;

	// checks_passed and meets_requirements are the results of the receiver's own GnssChecks for this sample
	void setGnssData(const sensor_gnss_s &gnss_data, bool checks_passed, bool meets_requirements, uint8_t instance)
	{
		if (instance < GNSS_MAX_RECEIVERS) {
			_sample[instance] = {gnss_data.timestamp, checks_passed, meets_requirements,
					     gnss_data.fix_type == sensor_gnss_s::FIX_TYPE_RTK_FIXED
					    };
			_updated[instance] = true;
			_has_published[instance] = true;
		}
	}

	// -1 for no preferred receiver
	void setPreferredInstance(int instance) { _preferred_instance = instance; }
	void setArmed(bool armed)
	{
		_armed = armed;

		if (!armed) {
			for (bool &failed : _failed_while_armed) {
				failed = false;
			}

			_rank_switches_while_armed = 0;
		}
	}

	// Also call it periodically while no receiver publishes, so that receivers time out and lose availability
	void update(uint64_t hrt_now_us);

	int getSelectedInstance() const { return _selected_instance; }

	// The selected receiver delivered a sample since the previous update, for the caller to publish
	bool selectedHasNewSample() const { return _selected_has_new_sample; }

	// Increments when the published samples change to another receiver, which steps the position that consumers see
	uint8_t getSelectionCount() const { return _selection_count; }

	// Why the selected receiver is selected, as vehicle_gnss_s::SELECTION_*
	uint8_t getSelectionReason() const { return _selection_reason; }

	// The latest sample passed its checks and neither it nor the next one is late
	bool isUsable(int instance, uint64_t hrt_now_us) const;

	float getAvailability(int instance) const
	{
		return ((instance >= 0) && (instance < GNSS_MAX_RECEIVERS)) ? _availability[instance].getState() : 0.f;
	}

private:
	// What the selection uses of a receiver's latest sample
	struct Sample {
		uint64_t timestamp{0}; ///< 0 when never published or timed out
		bool checks_passed{false};
		bool meets_requirements{false};
		bool rtk_fixed{false};
	};

	struct UpdateInterval {
		float filtered_s{0.f}; ///< 0 until measured
		uint8_t samples{0};    ///< up to INTERVAL_SETTLE_SAMPLES
		bool late{false};      ///< the latest sample came late
	};

	// Track each receiver's update interval, and reset a receiver that stopped publishing
	void updateReceiverTimeouts(uint64_t hrt_now_us);

	// A receiver that timed out may come back at another rate. Its availability and the time it was last usable are
	// kept, so that the outage counts against it.
	void resetReceiver(int instance)
	{
		_sample[instance] = {};
		_interval[instance] = {};
	}

	void updateAvailability(uint64_t hrt_now_us);

	int selectReceiver(uint64_t hrt_now_us);

	uint8_t selectionReason() const;

	// Never published, or timed out
	bool isSilent(int instance) const { return _sample[instance].timestamp == 0; }


	hrt_abstime lateIntervalUs(int instance) const;

	bool hasFailed(int instance, uint64_t hrt_now_us) const
	{
		return isSilent(instance) || (_time_last_usable_us[instance] == 0)
		       || (hrt_now_us >= _time_last_usable_us[instance] + FAIL_TIME_US);
	}

	enum Rank : int8_t {
		RANK_UNUSABLE = -1,
		RANK_USABLE = 0,
		RANK_REQUIREMENTS = 1, ///< meets the accuracy requirements
		RANK_RTK_FIXED = 2,    ///< meets the accuracy requirements with an RTK fixed solution, without a preferred receiver
	};

	Rank rank(int instance, uint64_t hrt_now_us) const;

	// The receiver to replace the current one with in case the current is failing, -1 if none: any one that publishes, or only usable ones
	int bestReplacement(int current, bool usable_only, uint64_t hrt_now_us) const;

	// Find the best receiver available, -1 if none is usable
	int bestCandidate(uint64_t hrt_now_us) const;

	bool isBetterReceiver(int instance, int other, uint64_t hrt_now_us, float tiebreaker_availability_margin = 0.f) const;

	hrt_abstime switchHoldUs() const;

	bool hasPreferred() const { return (_preferred_instance >= 0) && (_preferred_instance < GNSS_MAX_RECEIVERS); }

	// Records the vehicle_gnss_s::SELECTION_* reason for switching to instance
	int switchTo(int instance, uint8_t reason);

	// Switches away from the selected receiver because it failed
	int failOver(int instance, uint8_t reason);

	// No switch led to the selected receiver: it was the first one selected
	static constexpr uint8_t REASON_INITIAL = UINT8_MAX;

	Sample _sample[GNSS_MAX_RECEIVERS] {};
	UpdateInterval _interval[GNSS_MAX_RECEIVERS] {};
	bool _updated[GNSS_MAX_RECEIVERS] {};
	bool _has_published[GNSS_MAX_RECEIVERS] {};
	bool _has_passed[GNSS_MAX_RECEIVERS] {};
	AlphaFilter<float> _availability[GNSS_MAX_RECEIVERS];
	uint64_t _time_last_update_us{0};

	uint64_t _time_last_usable_us[GNSS_MAX_RECEIVERS] {};
	uint64_t _time_usable_since_us[GNSS_MAX_RECEIVERS] {}; ///< start of the current usable period, 0 while unusable

	uint64_t _time_prev_us[GNSS_MAX_RECEIVERS] {};  ///< timestamp of the previous sample, to detect new data

	int _selected_instance{0};
	int _output_instance{-1};                         ///< receiver of the last published sample, -1 before the first one
	uint8_t _selection_count{0};
	uint8_t _switch_reason{REASON_INITIAL};          ///< why the last switch happened
	uint8_t _selection_reason{vehicle_gnss_s::SELECTION_ONLY};
	int _preferred_instance{-1};

	int _switch_candidate{-1};                        ///< higher ranked receiver, waiting for the hold time
	uint64_t _switch_candidate_since_us{0};
	bool _armed{false};
	uint8_t _rank_switches_while_armed{0};
	bool _failed_while_armed[GNSS_MAX_RECEIVERS] {}; ///< left because it failed, until disarmed

	bool _selected_has_new_sample{false};
};
