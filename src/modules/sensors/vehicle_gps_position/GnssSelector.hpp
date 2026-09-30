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
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <uORB/topics/sensor_gnss.h>
#include <uORB/topics/vehicle_gnss.h>

using matrix::Vector3f;

using namespace time_literals;

/*
 * Selects the receiver whose samples vehicle_gnss carries. Every switch resets the EKF2 position, so the selection only
 * leaves a receiver that failed, returns to the preferred one, or moves to a clearly more accurate one after a hold.
 *
 * A receiver is usable while its latest sample passed its checks and it delivers samples at its usual rate. It has
 * failed when it had no usable sample for FAIL_TIME_US, which covers a lost fix, sustained check failures, a receiver
 * that stopped publishing and an update rate that collapsed. Intermittent failures are caught by the availability:
 * the fraction of recent time it was usable.
 *
 * With a preferred receiver, reported accuracy never matters: a moving-base rover reports a better accuracy while
 * being the worse position source. Without one, a receiver is more accurate when its reported eph is at most half the
 * selected one's and its epv is no worse, both raised to a floor below which RTK receivers tie.
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
	// A usable receiver whose availability is higher by this margin replaces the selected one. Returns and accuracy
	// switches accept half of it, so that a receiver hovering at the margin doesn't flap.
	static constexpr float AVAILABILITY_MARGIN = 0.2f;

	// While armed, the checks relax and pass after a second, so the preferred receiver has to prove itself this long
	// before the selection returns to it. While disarmed the strict checks already required GNSS_REQ_TIME.
	static constexpr hrt_abstime RETURN_HOLD_ARMED_US = 10_s;

	static constexpr hrt_abstime ACCURACY_HOLD_US = 5_s;
	static constexpr float ACCURACY_RATIO = 0.5f;
	static constexpr float ACCURACY_FLOOR = 0.05f; // [m]

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

	// checks_passed is the result of the receiver's own GnssChecks for this sample
	void setGnssData(const sensor_gnss_s &gnss_data, bool checks_passed, uint8_t instance)
	{
		if (instance < GNSS_MAX_RECEIVERS) {
			_gnss_state[instance] = gnss_data;
			_checks_passed[instance] = checks_passed;
			_updated[instance] = true;
			_has_published[instance] = true;
		}
	}

	// -1 for no preferred receiver
	void setPreferredInstance(int instance) { _preferred_instance = instance; }
	void setArmed(bool armed) { _armed = armed; }
	void setAntennaOffset(const Vector3f &offset, uint8_t instance)
	{
		if (instance < GNSS_MAX_RECEIVERS) { _antenna_offset[instance] = offset; }
	}
	const Vector3f &getOutputAntennaOffset() const { return _output_antenna_offset; }

	void update(uint64_t hrt_now_us);

	bool isNewOutputDataAvailable() const { return _is_new_output_data_available; }
	const sensor_gnss_s &getOutputGnssData() const { return _gnss_state[_selected_instance]; }
	int getSelectedInstance() const { return _selected_instance; }

	// Increments when the output changes to another receiver, which steps the position that consumers see
	uint8_t getSelectionCount() const { return _selection_count; }

	// Why the selected receiver is selected, as vehicle_gnss_s::SELECTION_*
	uint8_t getSelectionReason() const;

	float getAvailability(int instance) const
	{
		return ((instance >= 0) && (instance < GNSS_MAX_RECEIVERS)) ? _availability[instance].getState() : 0.f;
	}

private:
	// Track each receiver's update interval, and drop the stored fix of a receiver that stopped publishing
	void updateReceiverTimeouts(uint64_t hrt_now_us);

	void updateAvailability(uint64_t hrt_now_us);

	int selectReceiver(uint64_t hrt_now_us);

	// Never published, or timed out
	bool isSilent(int instance) const { return _gnss_state[instance].timestamp == 0; }

	// The latest sample passed its checks and neither it nor the next one is late
	bool isUsable(int instance, uint64_t hrt_now_us) const;

	hrt_abstime lateIntervalUs(int instance) const;

	bool hasFailed(int instance, uint64_t hrt_now_us) const
	{
		return isSilent(instance) || (_time_last_usable_us[instance] == 0)
		       || (hrt_now_us >= _time_last_usable_us[instance] + FAIL_TIME_US);
	}

	bool isMoreAccurate(int instance, int than) const;

	bool hasPreferred() const { return (_preferred_instance >= 0) && (_preferred_instance < GNSS_MAX_RECEIVERS); }

	// Records the vehicle_gnss_s::SELECTION_* reason for switching to instance
	int switchTo(int instance, uint8_t reason);

	// No switch led to the selected receiver: it was the first one selected
	static constexpr uint8_t REASON_INITIAL = UINT8_MAX;

	sensor_gnss_s _gnss_state[GNSS_MAX_RECEIVERS] {};
	bool _checks_passed[GNSS_MAX_RECEIVERS] {};
	bool _updated[GNSS_MAX_RECEIVERS] {};
	bool _has_published[GNSS_MAX_RECEIVERS] {};
	bool _has_passed[GNSS_MAX_RECEIVERS] {};
	AlphaFilter<float> _availability[GNSS_MAX_RECEIVERS];
	uint64_t _time_last_update_us{0};

	uint64_t _time_last_usable_us[GNSS_MAX_RECEIVERS] {};
	uint64_t _time_usable_since_us[GNSS_MAX_RECEIVERS] {}; ///< start of the current usable period, 0 while unusable

	uint64_t _time_prev_us[GNSS_MAX_RECEIVERS] {};  ///< timestamp of the previous sample, to detect new data
	float _interval_s[GNSS_MAX_RECEIVERS] {};        ///< filtered update interval, 0 until measured
	uint8_t _interval_samples[GNSS_MAX_RECEIVERS] {};
	float _usual_interval_s[GNSS_MAX_RECEIVERS] {};  ///< 0 until settled
	bool _sample_late[GNSS_MAX_RECEIVERS] {};

	int _selected_instance{0};
	int _output_instance{-1};                         ///< receiver of the last output, -1 before the first one
	uint8_t _selection_count{0};
	uint8_t _selection_reason{REASON_INITIAL};
	int _preferred_instance{-1};

	int _switch_candidate{-1};                        ///< more accurate receiver, waiting for the hold time
	uint64_t _switch_candidate_since_us{0};
	bool _armed{false};

	bool _is_new_output_data_available{false};

	Vector3f _antenna_offset[GNSS_MAX_RECEIVERS] {};
	Vector3f _output_antenna_offset {};
};
