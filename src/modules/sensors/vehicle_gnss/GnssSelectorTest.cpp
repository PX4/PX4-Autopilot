/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
 * Test code for the GNSS receiver selection logic
 * Run this test only using make tests TESTFILTER=GnssSelector
 *
 */

#include <cmath>
#include <gtest/gtest.h>

#include "GnssSelector.hpp"

class GnssSelectorTest : public ::testing::Test
{
public:
	sensor_gnss_s getDefaultGnssData();
	void runSeconds(float duration_s, GnssSelector &selector, sensor_gnss_s &gnss_data, int instance);
	void runSeconds(float duration_s, GnssSelector &selector, sensor_gnss_s &gnss_data0, sensor_gnss_s &gnss_data1);

	// Runs both receivers at 10 Hz, except that gnss0 only publishes every gnss0_divider-th step
	void runSecondsSlowGnss0(float duration_s, GnssSelector &selector, sensor_gnss_s &gnss_data0, sensor_gnss_s &gnss_data1,
				 int gnss0_divider);

	uint64_t _time_now_us{1000000};

	// Results of each receiver's checks, which the sensors module passes along with every sample
	bool _checks_passed[2] {true, true};
	bool _meets_requirements[2] {true, true};
};

sensor_gnss_s GnssSelectorTest::getDefaultGnssData()
{
	sensor_gnss_s gnss_data{};
	gnss_data.timestamp = _time_now_us - 10e3;
	gnss_data.time_utc_usec = 0;
	gnss_data.latitude = 47.0;
	gnss_data.longitude = 9.0;
	gnss_data.altitude_msl = 800.0;
	gnss_data.altitude_ellipsoid = 800.0;
	gnss_data.speed_accuracy = 0.2f;
	gnss_data.course_accuracy = 0.5f;
	gnss_data.eph = 0.7f;
	gnss_data.epv = 1.2f;
	gnss_data.hdop = 1.f;
	gnss_data.vdop = 1.f;
	gnss_data.noise = 20;
	gnss_data.jamming_indicator = 40;
	gnss_data.ground_speed = 1.f;
	gnss_data.vel_north = 1.f;
	gnss_data.vel_east = 1.f;
	gnss_data.vel_down = 1.f;
	gnss_data.course = 0.f;
	gnss_data.timestamp_time_relative = 0;
	gnss_data.fix_type = 4;
	gnss_data.vel_ned_valid = true;
	gnss_data.satellites_used = 8;

	return gnss_data;
}

void GnssSelectorTest::runSeconds(float duration_s, GnssSelector &selector, sensor_gnss_s &gnss_data, int instance)
{
	const float dt = 0.1;
	const uint64_t dt_us = static_cast<uint64_t>(dt * 1e6f);

	for (int k = 0; k < lroundf(duration_s / dt); k++) {
		selector.setGnssData(gnss_data, _checks_passed[instance], _meets_requirements[instance], instance);
		selector.update(_time_now_us);

		_time_now_us += dt_us;
		gnss_data.timestamp += dt_us;
	}
}

void GnssSelectorTest::runSeconds(float duration_s, GnssSelector &selector, sensor_gnss_s &gnss_data0,
				  sensor_gnss_s &gnss_data1)
{
	runSecondsSlowGnss0(duration_s, selector, gnss_data0, gnss_data1, 1);
}

void GnssSelectorTest::runSecondsSlowGnss0(float duration_s, GnssSelector &selector, sensor_gnss_s &gnss_data0,
		sensor_gnss_s &gnss_data1, int gnss0_divider)
{
	const float dt = 0.1;
	const uint64_t dt_us = static_cast<uint64_t>(dt * 1e6f);

	for (int k = 0; k < lroundf(duration_s / dt); k++) {
		if ((k % gnss0_divider) == 0) {
			selector.setGnssData(gnss_data0, _checks_passed[0], _meets_requirements[0], 0);
		}

		selector.setGnssData(gnss_data1, _checks_passed[1], _meets_requirements[1], 1);

		selector.update(_time_now_us);

		_time_now_us += dt_us;
		gnss_data0.timestamp += dt_us;
		gnss_data1.timestamp += dt_us;
	}
}

TEST_F(GnssSelectorTest, noData)
{
	GnssSelector selector;

	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_FALSE(selector.selectedHasNewSample());

	selector.update(_time_now_us);

	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_FALSE(selector.selectedHasNewSample());
}

TEST_F(GnssSelectorTest, singleReceiver)
{
	GnssSelector selector;

	sensor_gnss_s gnss_data = getDefaultGnssData();

	selector.setGnssData(gnss_data, true, true, 1);
	selector.update(_time_now_us);

	_time_now_us += 200e3;
	gnss_data.timestamp = _time_now_us - 10e3;
	selector.setGnssData(gnss_data, true, true, 1);
	selector.update(_time_now_us);

	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_TRUE(selector.selectedHasNewSample());

	// BUT IF: a second update is called without data
	selector.update(_time_now_us);

	// THEN: no new data should be available
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_FALSE(selector.selectedHasNewSample());
}

TEST_F(GnssSelectorTest, timeoutWithoutAnyPublisher)
{
	GnssSelector selector;

	// GIVEN: a single receiver that is usable
	sensor_gnss_s gnss_data = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data, 0);

	EXPECT_FLOAT_EQ(selector.getAvailability(0), 1.f);

	// WHEN: it stops publishing, and the selector keeps updating without samples
	for (int k = 0; k < 10; k++) {
		_time_now_us += 300e3;
		selector.update(_time_now_us);
	}

	// THEN: its availability fell
	EXPECT_FALSE(selector.selectedHasNewSample());
	EXPECT_LT(selector.getAvailability(0), 0.8f);

	// WHEN: it publishes again
	_time_now_us += 100e3;
	gnss_data.timestamp = _time_now_us - 10e3;

	runSeconds(0.1f, selector, gnss_data, 0);

	// THEN: its output resumes, and its availability keeps the outage
	EXPECT_TRUE(selector.selectedHasNewSample());
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_LT(selector.getAvailability(0), 0.8f);
}

TEST_F(GnssSelectorTest, preferredWhateverTheOtherReports)
{
	GnssSelector selector;

	// GIVEN: gnss0 preferred, only gnss1 publishing
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(10.f, selector, gnss_data1, 1);

	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_ONLY);

	// WHEN: gnss0 starts publishing and passes its checks
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	gnss_data0.timestamp = gnss_data1.timestamp;

	runSeconds(0.2f, selector, gnss_data0, gnss_data1);

	// THEN: it is selected right away while disarmed
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_PREFERRED);
	EXPECT_EQ(selector.getSelectionCount(), 1);

	// WHEN: gnss1 reports a better fix type and accuracy
	gnss_data1.fix_type = sensor_gnss_s::FIX_TYPE_RTK_FIXED;
	gnss_data1.eph = 0.02f;
	gnss_data1.epv = 0.03f;

	runSeconds(20.f, selector, gnss_data0, gnss_data1);

	// THEN: the preferred receiver is kept
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionCount(), 1);
}

TEST_F(GnssSelectorTest, timeoutSwitchesImmediately)
{
	GnssSelector selector;

	// GIVEN: gnss0 preferred and selected
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 0);

	// WHEN: gnss0 stops publishing
	runSeconds(1.5f, selector, gnss_data1, 1);

	// THEN: it is kept until it times out, without output
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_FALSE(selector.selectedHasNewSample());

	runSeconds(1.f, selector, gnss_data1, 1);

	// THEN: the receiver that still publishes is selected
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_TIMEOUT);
	EXPECT_EQ(selector.getSelectionCount(), 1);

	// WHEN: gnss0 publishes again
	gnss_data0.timestamp = gnss_data1.timestamp;

	runSeconds(0.2f, selector, gnss_data0, gnss_data1);

	// THEN: the selection returns at once, as the vehicle is disarmed
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_PREFERRED);
	EXPECT_EQ(selector.getSelectionCount(), 2);
}

TEST_F(GnssSelectorTest, timeoutToFailingReceiver)
{
	GnssSelector selector;

	// GIVEN: gnss0 preferred, both receivers failing their checks
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();
	_checks_passed[0] = false;
	_checks_passed[1] = false;

	runSeconds(10.f, selector, gnss_data0, gnss_data1);

	// THEN: the preferred receiver is kept
	EXPECT_EQ(selector.getSelectedInstance(), 0);

	// WHEN: gnss0 stops publishing
	runSeconds(2.5f, selector, gnss_data1, 1);

	// THEN: the receiver that still publishes is selected, so that its samples keep coming
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_TRUE(selector.selectedHasNewSample());
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_TIMEOUT);
}

TEST_F(GnssSelectorTest, continuousFailure)
{
	GnssSelector selector;

	// GIVEN: an armed vehicle using gnss0, the preferred receiver
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	// WHEN: gnss0 fails its checks
	_checks_passed[0] = false;

	runSeconds(1.5f, selector, gnss_data0, gnss_data1);

	// THEN: it is left once it had no usable sample for 2 s
	EXPECT_EQ(selector.getSelectedInstance(), 0);

	runSeconds(1.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_UNHEALTHY);
	EXPECT_EQ(selector.getSelectionCount(), 1);
}

TEST_F(GnssSelectorTest, continuousFailureWithFlakyStandby)
{
	GnssSelector selector;

	// GIVEN: an armed vehicle using gnss0, the preferred receiver, and gnss1 that failed its checks for 5 s
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	_checks_passed[1] = false;
	runSeconds(5.f, selector, gnss_data0, gnss_data1);

	EXPECT_LT(selector.getAvailability(1), 0.7f);

	// WHEN: gnss1 recovers as gnss0 fails its checks
	_checks_passed[0] = false;
	_checks_passed[1] = true;

	runSeconds(2.5f, selector, gnss_data0, gnss_data1);

	// THEN: gnss1 replaces it after the same 2 s, although it was recently less available than gnss0
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_UNHEALTHY);
	EXPECT_EQ(selector.getSelectionCount(), 1);
}

TEST_F(GnssSelectorTest, noReturnToFailedReceiverOnSingleSample)
{
	GnssSelector selector;

	// GIVEN: an armed vehicle without a preferred receiver that left gnss0 for gnss1, which failed recently itself
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	_checks_passed[1] = false;
	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	_checks_passed[0] = false;
	_checks_passed[1] = true;
	runSeconds(2.5f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 1);

	// WHEN: gnss0 passes a single sample every second, still more available than gnss1
	for (int cycle = 0; cycle < 10; cycle++) {
		_checks_passed[0] = true;
		runSeconds(0.1f, selector, gnss_data0, gnss_data1);
		_checks_passed[0] = false;
		runSeconds(0.9f, selector, gnss_data0, gnss_data1);
	}

	// THEN: gnss1 is kept
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionCount(), 1);
}

TEST_F(GnssSelectorTest, intermittentFailure)
{
	GnssSelector selector;

	// GIVEN: an armed vehicle using gnss0, the preferred receiver
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	// WHEN: gnss0 fails one sample every 1.5 s, which makes it unusable for 1 s each time: it never fails continuously
	// for long, but only a third of its samples can be fused
	for (int cycle = 0; cycle < 10; cycle++) {
		_checks_passed[0] = false;
		runSeconds(1.f, selector, gnss_data0, gnss_data1);
		_checks_passed[0] = true;
		runSeconds(0.5f, selector, gnss_data0, gnss_data1);
	}

	// THEN: the selection moves to the receiver whose samples can all be fused
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_UNHEALTHY);
	EXPECT_EQ(selector.getSelectionCount(), 1);
}

TEST_F(GnssSelectorTest, occasionalFailureDoesNotSwitch)
{
	GnssSelector selector;

	// GIVEN: an armed vehicle using gnss0, the preferred receiver
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	// WHEN: gnss0 fails one sample every 10 s, unusable 10% of the time
	for (int cycle = 0; cycle < 6; cycle++) {
		_checks_passed[0] = false;
		runSeconds(1.f, selector, gnss_data0, gnss_data1);
		_checks_passed[0] = true;
		runSeconds(9.f, selector, gnss_data0, gnss_data1);
	}

	// THEN: it is kept. A switch would reset the EKF2 position.
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionCount(), 0);
}

TEST_F(GnssSelectorTest, rateCollapse)
{
	GnssSelector selector;

	// GIVEN: an armed vehicle using gnss0, the preferred receiver, both at 10 Hz
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	// WHEN: gnss0 drops to 1 Hz, its samples still passing their checks
	runSecondsSlowGnss0(1.5f, selector, gnss_data0, gnss_data1, 10);

	EXPECT_EQ(selector.getSelectedInstance(), 0);

	runSecondsSlowGnss0(1.5f, selector, gnss_data0, gnss_data1, 10);

	// THEN: it has failed, as every sample is late
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_UNHEALTHY);
}

TEST_F(GnssSelectorTest, halfRateStillUsable)
{
	GnssSelector selector;

	// GIVEN: an armed vehicle using gnss0, the preferred receiver, both at 10 Hz
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	// WHEN: gnss0 drops to 5 Hz, or has a single 1 s gap
	runSecondsSlowGnss0(20.f, selector, gnss_data0, gnss_data1, 2);
	runSecondsSlowGnss0(1.f, selector, gnss_data0, gnss_data1, 20);
	runSeconds(10.f, selector, gnss_data0, gnss_data1);

	// THEN: it is kept
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionCount(), 0);
}

TEST_F(GnssSelectorTest, bothFailingKeepsSelection)
{
	GnssSelector selector;

	// GIVEN: two receivers without a preferred one, gnss0 selected
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 0);

	// WHEN: both fail their checks
	_checks_passed[0] = false;
	_checks_passed[1] = false;

	runSeconds(20.f, selector, gnss_data0, gnss_data1);

	// THEN: neither is usable, so the selection stays
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionCount(), 0);
}

TEST_F(GnssSelectorTest, noReturnToFailedPreferred)
{
	GnssSelector selector;

	// GIVEN: an armed vehicle that failed over from gnss0, the preferred receiver
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	_checks_passed[0] = false;
	runSeconds(3.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 1);

	// WHEN: gnss0 passes its checks again
	_checks_passed[0] = true;

	runSeconds(30.f, selector, gnss_data0, gnss_data1);

	// THEN: gnss1 is kept, as gnss0 is likely to fail again
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionCount(), 1);

	// WHEN: gnss1 fails its checks
	_checks_passed[1] = false;

	runSeconds(2.5f, selector, gnss_data0, gnss_data1);

	// THEN: the selection returns to gnss0
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_PREFERRED);
	EXPECT_EQ(selector.getSelectionCount(), 2);
}

TEST_F(GnssSelectorTest, preferredStartingWhileArmed)
{
	GnssSelector selector;

	// GIVEN: an armed vehicle using gnss1, as gnss0, the preferred receiver, didn't publish yet
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data1, 1);
	selector.setArmed(true);

	EXPECT_EQ(selector.getSelectedInstance(), 1);

	// WHEN: gnss0 starts publishing and passes its checks
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	gnss_data0.timestamp = gnss_data1.timestamp;

	runSeconds(9.5f, selector, gnss_data0, gnss_data1);

	// THEN: the selection moves to it once it has been usable for the hold time, as it never failed
	EXPECT_EQ(selector.getSelectedInstance(), 1);

	runSeconds(1.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_PREFERRED);
	EXPECT_EQ(selector.getSelectionCount(), 1);
}

TEST_F(GnssSelectorTest, preferredReturnAfterDisarm)
{
	GnssSelector selector;

	// GIVEN: an armed vehicle that failed over from gnss0, the preferred receiver, which passes its checks again
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	_checks_passed[0] = false;
	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	_checks_passed[0] = true;
	runSeconds(5.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 1);

	// WHEN: the vehicle disarms while gnss0 is still clearly less available
	selector.setArmed(false);

	runSeconds(0.2f, selector, gnss_data0, gnss_data1);

	// THEN: gnss0 is selected at once
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_PREFERRED);
	EXPECT_EQ(selector.getSelectionCount(), 2);
}

TEST_F(GnssSelectorTest, preferredSelectedWhileDisarmed)
{
	GnssSelector selector;

	// GIVEN: gnss0 preferred, only gnss1 publishing
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(1.f, selector, gnss_data1, 1);

	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_ONLY);

	// WHEN: gnss0 publishes, but hasn't passed its checks yet
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	gnss_data0.timestamp = gnss_data1.timestamp;
	_checks_passed[0] = false;

	runSeconds(5.f, selector, gnss_data0, gnss_data1);

	// THEN: it is selected, so that the vehicle doesn't take off on gnss1
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_PREFERRED);
	EXPECT_EQ(selector.getSelectionCount(), 1);

	// WHEN: the vehicle arms anyway
	selector.setArmed(true);

	runSeconds(0.2f, selector, gnss_data0, gnss_data1);

	// THEN: gnss0 has failed, and gnss1 replaces it
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_UNHEALTHY);
	EXPECT_EQ(selector.getSelectionCount(), 2);

}

TEST_F(GnssSelectorTest, preferenceRemoved)
{
	GnssSelector selector;

	// GIVEN: the selection moved to gnss1, the preferred receiver, once it published
	selector.setPreferredInstance(1);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(1.f, selector, gnss_data0, 0);
	gnss_data1.timestamp = gnss_data0.timestamp;
	runSeconds(5.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionCount(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_PREFERRED);

	// WHEN: no receiver is preferred anymore
	selector.setPreferredInstance(-1);

	runSeconds(5.f, selector, gnss_data0, gnss_data1);

	// THEN: gnss1 is kept, as the other one is no more accurate
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_RANKED);
	EXPECT_EQ(selector.getSelectionCount(), 1);
}

TEST_F(GnssSelectorTest, rankedIgnoresFloatSatellitesAndRate)
{
	GnssSelector selector;

	// GIVEN: two receivers without a preferred one, gnss1 selected
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(1.f, selector, gnss_data1, 1);
	gnss_data0.timestamp = gnss_data1.timestamp;

	EXPECT_EQ(selector.getSelectedInstance(), 1);

	// WHEN: gnss0 reports an RTK float solution, more satellites and a better accuracy, and gnss1 publishes at twice
	// gnss0's rate
	gnss_data0.fix_type = sensor_gnss_s::FIX_TYPE_RTK_FLOAT;
	gnss_data0.satellites_used = gnss_data1.satellites_used + 10;
	gnss_data0.eph = 0.1f;
	gnss_data0.epv = 0.2f;

	runSecondsSlowGnss0(20.f, selector, gnss_data0, gnss_data1, 2);

	// THEN: gnss1 is kept, none of them ranks a receiver higher
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_RANKED);
	EXPECT_EQ(selector.getSelectionCount(), 0);
}

TEST_F(GnssSelectorTest, rankedSwitchOnFailure)
{
	GnssSelector selector;

	// GIVEN: two receivers without a preferred one, gnss0 selected
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_RANKED);

	// WHEN: gnss0 fails its checks
	_checks_passed[0] = false;

	runSeconds(3.f, selector, gnss_data0, gnss_data1);

	// THEN: the receiver that passes them is selected
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_UNHEALTHY);
	EXPECT_EQ(selector.getSelectionCount(), 1);

	// WHEN: gnss0 passes them again
	_checks_passed[0] = true;

	runSeconds(30.f, selector, gnss_data0, gnss_data1);

	// THEN: gnss1 is kept, as gnss0 doesn't rank higher
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionCount(), 1);
}

TEST_F(GnssSelectorTest, rankedRequirements)
{
	GnssSelector selector;

	// GIVEN: two receivers without a preferred one, gnss0 selected, on an armed vehicle
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	// WHEN: gnss0 no longer meets the accuracy requirements, which gnss1 does
	_meets_requirements[0] = false;

	runSeconds(9.5f, selector, gnss_data0, gnss_data1);

	// THEN: the switch waits for the hold time
	EXPECT_EQ(selector.getSelectedInstance(), 0);

	runSeconds(1.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_REQUIREMENTS);
	EXPECT_EQ(selector.getSelectionCount(), 1);

	// WHEN: gnss0 meets them again
	_meets_requirements[0] = true;

	runSeconds(20.f, selector, gnss_data0, gnss_data1);

	// THEN: gnss1 is kept, as gnss0 doesn't rank higher
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionCount(), 1);

	// WHEN: gnss1 no longer meets them
	_meets_requirements[1] = false;

	runSeconds(10.5f, selector, gnss_data0, gnss_data1);

	// THEN: the selection moves back to gnss0, which ranked lower but never failed
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_REQUIREMENTS);
	EXPECT_EQ(selector.getSelectionCount(), 2);
}

TEST_F(GnssSelectorTest, rankedHoldRestarts)
{
	GnssSelector selector;

	// GIVEN: two receivers without a preferred one, gnss0 selected, on an armed vehicle, and gnss0 not meeting the
	// accuracy requirements
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);
	_meets_requirements[0] = false;

	// WHEN: gnss1 meets them for 9 s at a time
	for (int cycle = 0; cycle < 5; cycle++) {
		_meets_requirements[1] = true;
		runSeconds(9.f, selector, gnss_data0, gnss_data1);
		_meets_requirements[1] = false;
		runSeconds(0.2f, selector, gnss_data0, gnss_data1);
	}

	// THEN: gnss0 is kept
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionCount(), 0);
}

TEST_F(GnssSelectorTest, rankedRtkFixed)
{
	GnssSelector selector;

	// GIVEN: two receivers without a preferred one, gnss0 selected, on an armed vehicle
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	// WHEN: gnss1 gets an RTK fixed solution
	gnss_data1.fix_type = sensor_gnss_s::FIX_TYPE_RTK_FIXED;

	runSeconds(10.5f, selector, gnss_data0, gnss_data1);

	// THEN: it is selected after the hold time
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_RTK_FIXED);
	EXPECT_EQ(selector.getSelectionCount(), 1);

	// WHEN: gnss0 gets one too, reporting a better accuracy
	gnss_data0.fix_type = sensor_gnss_s::FIX_TYPE_RTK_FIXED;
	gnss_data0.eph = 0.01f;
	gnss_data0.epv = 0.015f;

	runSeconds(20.f, selector, gnss_data0, gnss_data1);

	// THEN: gnss1 is kept
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionCount(), 1);

	// WHEN: gnss1 drops to RTK float
	gnss_data1.fix_type = sensor_gnss_s::FIX_TYPE_RTK_FLOAT;

	runSeconds(10.5f, selector, gnss_data0, gnss_data1);

	// THEN: the RTK fixed receiver is selected
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_RTK_FIXED);
	EXPECT_EQ(selector.getSelectionCount(), 2);
}

TEST_F(GnssSelectorTest, rankedRtkFixedNeedsRequirements)
{
	GnssSelector selector;

	// GIVEN: two receivers without a preferred one, gnss0 selected
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);

	// WHEN: gnss1 reports an RTK fixed solution, but doesn't meet the accuracy requirements
	gnss_data1.fix_type = sensor_gnss_s::FIX_TYPE_RTK_FIXED;
	_meets_requirements[1] = false;

	runSeconds(20.f, selector, gnss_data0, gnss_data1);

	// THEN: gnss0 is kept
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionCount(), 0);
}

TEST_F(GnssSelectorTest, rankedNoReturnToFailedReceiver)
{
	GnssSelector selector;

	// GIVEN: two receivers without a preferred one, gnss0 selected with an RTK fixed solution, on an armed vehicle
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();
	gnss_data0.fix_type = sensor_gnss_s::FIX_TYPE_RTK_FIXED;

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	EXPECT_EQ(selector.getSelectedInstance(), 0);

	// WHEN: gnss0 fails its checks, then recovers
	_checks_passed[0] = false;
	runSeconds(3.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 1);

	_checks_passed[0] = true;
	runSeconds(30.f, selector, gnss_data0, gnss_data1);

	// THEN: gnss1 is kept, although gnss0 ranks higher
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionCount(), 1);

	// WHEN: the vehicle disarms
	selector.setArmed(false);

	runSeconds(1.5f, selector, gnss_data0, gnss_data1);

	// THEN: gnss0 ranks again, with the shorter hold time
	EXPECT_EQ(selector.getSelectedInstance(), 1);

	runSeconds(1.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_RTK_FIXED);
	EXPECT_EQ(selector.getSelectionCount(), 2);
}

TEST_F(GnssSelectorTest, noOutputWithoutNewData)
{
	GnssSelector selector;

	// GIVEN: two receivers, gnss1 selected as it published first
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	selector.setGnssData(gnss_data1, true, true, 1);
	selector.update(_time_now_us);

	_time_now_us += 100e3;
	gnss_data0.timestamp = _time_now_us - 10e3;
	gnss_data1.timestamp = _time_now_us - 10e3;
	selector.setGnssData(gnss_data0, true, true, 0);
	selector.setGnssData(gnss_data1, true, true, 1);
	selector.update(_time_now_us);

	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_TRUE(selector.selectedHasNewSample());

	// WHEN: an update runs without new data from either receiver
	selector.update(_time_now_us);

	// THEN: no output is reported. The updated flag of the receiver that isn't selected must be cleared as well
	EXPECT_FALSE(selector.selectedHasNewSample());
}

TEST_F(GnssSelectorTest, availability)
{
	GnssSelector selector;

	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	// GIVEN: gnss1 hasn't passed its checks yet
	_checks_passed[1] = false;

	runSeconds(5.f, selector, gnss_data0, gnss_data1);

	// THEN: a receiver starts fully available once it first passes, and has no availability before
	EXPECT_FLOAT_EQ(selector.getAvailability(0), 1.f);
	EXPECT_FLOAT_EQ(selector.getAvailability(1), 0.f);

	// WHEN: gnss0 fails its checks for one time constant
	_checks_passed[0] = false;

	runSeconds(10.f, selector, gnss_data0, gnss_data1);

	// THEN: its availability decays like a first order filter
	EXPECT_NEAR(selector.getAvailability(0), expf(-1.f), 0.01f);
}
