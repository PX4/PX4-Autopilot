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
 * receiver's. Every switch makes EKF2 reset or restart its GNSS position, so the selection only leaves a receiver that
 * failed, or moves to the preferred one or to a higher ranked one after a hold.
 *
 * A receiver is usable while its latest sample passed its checks and it delivers samples at its usual rate. It has
 * failed when it had no usable sample for FAIL_TIME_US, which covers a lost fix, sustained check failures, a receiver
 * that stopped publishing and an update rate that collapsed. Intermittent failures are caught by the availability:
 * the fraction of recent time it was usable. While armed, a receiver that was left because it failed is selected again
 * only when the selected one fails, as one that failed is likely to fail again in the same flight.
 *
 * While disarmed the preferred receiver is selected whenever it publishes: a vehicle whose preferred receiver fails its
 * checks shouldn't take off on the other one. In flight it is kept while usable, whatever the other one reports: a
 * moving-base rover reports a better fix and accuracy while being the worse position source. Without a preferred
 * receiver, receivers rank by meeting the accuracy requirements and then by an RTK fixed solution, and smaller
 * differences in reported accuracy never switch.
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

	// How long a receiver must rank higher, or while armed be preferred and usable, before the selection moves to it.
	// While armed the checks relax and pass after a second; while disarmed the strict checks require GNSS_REQ_TIME.
	static constexpr hrt_abstime SWITCH_HOLD_ARMED_US = 10_s;
	static constexpr hrt_abstime SWITCH_HOLD_DISARMED_US = 2_s;

	// A sample is late when its interval exceeds this many times the receiver's usual interval, and at least
	// LATE_MIN_US. Every sample of a receiver whose rate collapsed below a third is late.
	static constexpr float LATE_INTERVAL_RATIO = 3.f;
	static constexpr hrt_abstime LATE_MIN_US = 300_ms;
	// The usual interval is the shortest the interval filter reached after this many samples. The filter weights
	// intervals by their duration, so that samples delivered in bursts don't shorten it.
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
	uint8_t getSelectionReason() const;

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
		uint8_t samples{0};
		float usual_s{0.f};    ///< 0 until settled
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

	// Never published, or timed out
	bool isSilent(int instance) const { return _sample[instance].timestamp == 0; }

	// The latest sample passed its checks and neither it nor the next one is late
	bool isUsable(int instance, uint64_t hrt_now_us) const;

	hrt_abstime lateIntervalUs(int instance) const;

	bool hasFailed(int instance, uint64_t hrt_now_us) const
	{
		return isSilent(instance) || (_time_last_usable_us[instance] == 0)
		       || (hrt_now_us >= _time_last_usable_us[instance] + FAIL_TIME_US);
	}

	// Without a preferred receiver the selection moves to a receiver that ranks higher, and at least meets the accuracy
	// requirements
	enum Rank : int8_t {
		RANK_UNUSABLE = -1,
		RANK_USABLE = 0,
		RANK_REQUIREMENTS = 1, ///< meets the accuracy requirements
		RANK_RTK_FIXED = 2,    ///< meets the accuracy requirements with an RTK fixed solution
	};

	Rank rank(int instance, uint64_t hrt_now_us) const;

	// Is instance A similarly or better available than instance B ? (Note: order matters)
	bool isComparablyAvailable(int instance_a, int instance_b) const
	{
		return _availability[instance_a].getState()
		       >= _availability[instance_b].getState() - 0.5f * AVAILABILITY_MARGIN;
	}

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
	uint8_t _selection_reason{REASON_INITIAL};
	int _preferred_instance{-1};

	bool _armed{false};
	bool _failed_while_armed[GNSS_MAX_RECEIVERS] {}; ///< left because it failed, until disarmed

	bool _selected_has_new_sample{false};
};
