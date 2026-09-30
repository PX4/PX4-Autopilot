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
 * Test code for the GPS receiver selection logic
 * Run this test only using make tests TESTFILTER=GnssSelector
 *
 */

#include <cmath>
#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>

#include "GnssSelector.hpp"

using matrix::Vector3f;

class GnssSelectorTest : public ::testing::Test
{
public:
	sensor_gnss_s getDefaultGnssData();
	void runSeconds(float duration_s, GnssSelector &selector, sensor_gnss_s &gnss_data, int instance);
	void runSeconds(float duration_s, GnssSelector &selector, sensor_gnss_s &gnss_data0, sensor_gnss_s &gnss_data1);

	// Runs both receivers at 10 Hz, except that gps0 only publishes every gps0_divider-th step
	void runSecondsSlowGps0(float duration_s, GnssSelector &selector, sensor_gnss_s &gnss_data0, sensor_gnss_s &gnss_data1,
				int gps0_divider);

	uint64_t _time_now_us{1000000};

	// Result of each receiver's checks, which the sensors module passes along with every sample
	bool _checks_passed[2] {true, true};
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
		selector.setGnssData(gnss_data, _checks_passed[instance], instance);
		selector.update(_time_now_us);

		_time_now_us += dt_us;
		gnss_data.timestamp += dt_us;
	}
}

void GnssSelectorTest::runSeconds(float duration_s, GnssSelector &selector, sensor_gnss_s &gnss_data0,
				  sensor_gnss_s &gnss_data1)
{
	runSecondsSlowGps0(duration_s, selector, gnss_data0, gnss_data1, 1);
}

void GnssSelectorTest::runSecondsSlowGps0(float duration_s, GnssSelector &selector, sensor_gnss_s &gnss_data0,
		sensor_gnss_s &gnss_data1, int gps0_divider)
{
	const float dt = 0.1;
	const uint64_t dt_us = static_cast<uint64_t>(dt * 1e6f);

	for (int k = 0; k < lroundf(duration_s / dt); k++) {
		if ((k % gps0_divider) == 0) {
			selector.setGnssData(gnss_data0, _checks_passed[0], 0);
		}

		selector.setGnssData(gnss_data1, _checks_passed[1], 1);

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
	EXPECT_FALSE(selector.isNewOutputDataAvailable());

	selector.update(_time_now_us);

	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_FALSE(selector.isNewOutputDataAvailable());
}

TEST_F(GnssSelectorTest, singleReceiver)
{
	GnssSelector selector;

	sensor_gnss_s gnss_data = getDefaultGnssData();

	selector.setGnssData(gnss_data, true, 1);
	selector.update(_time_now_us);

	_time_now_us += 200e3;
	gnss_data.timestamp = _time_now_us - 10e3;
	selector.setGnssData(gnss_data, true, 1);
	selector.update(_time_now_us);

	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_TRUE(selector.isNewOutputDataAvailable());

	// BUT IF: a second update is called without data
	selector.update(_time_now_us);

	// THEN: no new data should be available
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_FALSE(selector.isNewOutputDataAvailable());
}

TEST_F(GnssSelectorTest, preferredWhateverTheOtherReports)
{
	GnssSelector selector;

	// GIVEN: gps0 preferred, only gps1 publishing
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(10.f, selector, gnss_data1, 1);

	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_ONLY);

	// WHEN: gps0 starts publishing and passes its checks
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	gnss_data0.timestamp = gnss_data1.timestamp;

	runSeconds(0.2f, selector, gnss_data0, gnss_data1);

	// THEN: it is selected right away while disarmed, its checks having qualified it already
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_PREFERRED);
	EXPECT_EQ(selector.getSelectionCount(), 1);

	// WHEN: gps1 reports a better fix type and accuracy
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

	// GIVEN: gps0 preferred and selected
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 0);

	// WHEN: gps0 stops publishing
	runSeconds(1.5f, selector, gnss_data1, 1);

	// THEN: it is kept until it times out, without output
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_FALSE(selector.isNewOutputDataAvailable());

	runSeconds(1.f, selector, gnss_data1, 1);

	// THEN: the receiver that still publishes is selected
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_TIMEOUT);
	EXPECT_EQ(selector.getSelectionCount(), 1);

	// WHEN: gps0 publishes again, its availability reduced by the outage
	gnss_data0.timestamp = gnss_data1.timestamp;

	runSeconds(3.f, selector, gnss_data0, gnss_data1);

	// THEN: the selection returns once it is about as available as gps1
	EXPECT_EQ(selector.getSelectedInstance(), 1);

	runSeconds(7.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_PREFERRED);
	EXPECT_EQ(selector.getSelectionCount(), 2);
}

TEST_F(GnssSelectorTest, timeoutToFailingReceiver)
{
	GnssSelector selector;

	// GIVEN: gps0 preferred, both receivers failing their checks
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();
	_checks_passed[0] = false;
	_checks_passed[1] = false;

	runSeconds(10.f, selector, gnss_data0, gnss_data1);

	// THEN: the preferred receiver is kept, as the other one is no better
	EXPECT_EQ(selector.getSelectedInstance(), 0);

	// WHEN: gps0 stops publishing
	runSeconds(2.5f, selector, gnss_data1, 1);

	// THEN: the receiver that still publishes is selected, so that its samples keep coming
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_TRUE(selector.isNewOutputDataAvailable());
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_TIMEOUT);
}

TEST_F(GnssSelectorTest, continuousFailure)
{
	GnssSelector selector;

	// GIVEN: an armed vehicle using gps0, the preferred receiver
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	// WHEN: gps0 fails its checks
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

	// GIVEN: an armed vehicle using gps0, the preferred receiver, and gps1 that failed its checks for 5 s
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	_checks_passed[1] = false;
	runSeconds(5.f, selector, gnss_data0, gnss_data1);

	EXPECT_LT(selector.getAvailability(1), 0.7f);

	// WHEN: gps1 recovers as gps0 fails its checks
	_checks_passed[0] = false;
	_checks_passed[1] = true;

	runSeconds(2.5f, selector, gnss_data0, gnss_data1);

	// THEN: gps1 replaces it after the same 2 s, although it was recently less available than gps0
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_UNHEALTHY);
	EXPECT_EQ(selector.getSelectionCount(), 1);
}

TEST_F(GnssSelectorTest, noReturnToFailedReceiverOnSingleSample)
{
	GnssSelector selector;

	// GIVEN: an armed vehicle without a preferred receiver that left gps0 for gps1, which failed recently itself
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

	// WHEN: gps0 passes a single sample every second, still more available than gps1
	for (int cycle = 0; cycle < 10; cycle++) {
		_checks_passed[0] = true;
		runSeconds(0.1f, selector, gnss_data0, gnss_data1);
		_checks_passed[0] = false;
		runSeconds(0.9f, selector, gnss_data0, gnss_data1);
	}

	// THEN: gps1 is kept
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionCount(), 1);
}

TEST_F(GnssSelectorTest, intermittentFailure)
{
	GnssSelector selector;

	// GIVEN: an armed vehicle using gps0, the preferred receiver
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	// WHEN: gps0 fails one sample every 1.5 s, which makes it unusable for 1 s each time: it never fails continuously
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

	// GIVEN: an armed vehicle using gps0, the preferred receiver
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	// WHEN: gps0 fails one sample every 10 s, unusable 10% of the time
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

	// GIVEN: an armed vehicle using gps0, the preferred receiver, both at 10 Hz
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	// WHEN: gps0 drops to 1 Hz, its samples still passing their checks
	runSecondsSlowGps0(1.5f, selector, gnss_data0, gnss_data1, 10);

	EXPECT_EQ(selector.getSelectedInstance(), 0);

	runSecondsSlowGps0(1.5f, selector, gnss_data0, gnss_data1, 10);

	// THEN: it has failed, as every sample is late
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_UNHEALTHY);
}

TEST_F(GnssSelectorTest, halfRateStillUsable)
{
	GnssSelector selector;

	// GIVEN: an armed vehicle using gps0, the preferred receiver, both at 10 Hz
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	// WHEN: gps0 drops to 5 Hz, or has a single 1 s gap
	runSecondsSlowGps0(20.f, selector, gnss_data0, gnss_data1, 2);
	runSecondsSlowGps0(1.f, selector, gnss_data0, gnss_data1, 20);
	runSeconds(10.f, selector, gnss_data0, gnss_data1);

	// THEN: it is kept
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionCount(), 0);
}

TEST_F(GnssSelectorTest, bothFailingKeepsSelection)
{
	GnssSelector selector;

	// GIVEN: two receivers without a preferred one, gps0 selected
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

TEST_F(GnssSelectorTest, preferredArmedReturn)
{
	GnssSelector selector;

	// GIVEN: an armed vehicle that failed over from gps0, the preferred receiver
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	_checks_passed[0] = false;
	runSeconds(3.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 1);

	// WHEN: gps0 passes its checks again
	_checks_passed[0] = true;

	runSeconds(9.5f, selector, gnss_data0, gnss_data1);

	// THEN: the selection returns to it once it passed them for 10 s
	EXPECT_EQ(selector.getSelectedInstance(), 1);

	runSeconds(1.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_PREFERRED);
	EXPECT_EQ(selector.getSelectionCount(), 2);
}

TEST_F(GnssSelectorTest, preferredArmedReturnWaitsForAvailability)
{
	GnssSelector selector;

	// GIVEN: an armed vehicle that failed over from gps0, the preferred receiver, after a long failure
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	_checks_passed[0] = false;
	runSeconds(10.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 1);

	// WHEN: gps0 passes its checks again
	_checks_passed[0] = true;

	runSeconds(12.f, selector, gnss_data0, gnss_data1);

	// THEN: the return also waits until it is about as available as gps1, so that gps1 doesn't take over again
	EXPECT_EQ(selector.getSelectedInstance(), 1);

	runSeconds(10.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionCount(), 2);

	runSeconds(20.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectionCount(), 2);
}

TEST_F(GnssSelectorTest, preferredReturnAfterDisarm)
{
	GnssSelector selector;

	// GIVEN: an armed vehicle that failed over from gps0, the preferred receiver, which passes its checks again
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

	// WHEN: the vehicle disarms while gps0 is still clearly less available
	selector.setArmed(false);

	runSeconds(1.f, selector, gnss_data0, gnss_data1);

	// THEN: gps1 is kept until gps0 has caught up, without the armed hold
	EXPECT_EQ(selector.getSelectedInstance(), 1);

	runSeconds(20.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_PREFERRED);
	EXPECT_EQ(selector.getSelectionCount(), 2);
}

TEST_F(GnssSelectorTest, preferredQualifyingReason)
{
	GnssSelector selector;

	// GIVEN: gps0 preferred, only gps1 publishing
	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(1.f, selector, gnss_data1, 1);

	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_ONLY);

	// WHEN: gps0 publishes, but hasn't passed its checks yet
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	gnss_data0.timestamp = gnss_data1.timestamp;
	_checks_passed[0] = false;

	runSeconds(5.f, selector, gnss_data0, gnss_data1);

	// THEN: gps1 is kept, because the preferred receiver fails its checks
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_UNHEALTHY);
	EXPECT_EQ(selector.getSelectionCount(), 0);

	// WHEN: gps0 stops publishing
	runSeconds(2.5f, selector, gnss_data1, 1);

	// THEN: gps1 is kept, because the preferred receiver timed out
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_TIMEOUT);
}

TEST_F(GnssSelectorTest, preferenceRemoved)
{
	GnssSelector selector;

	// GIVEN: the selection moved to gps1, the preferred receiver, once it published
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

	// THEN: gps1 is kept, as the other one is no more accurate
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_RANKED);
	EXPECT_EQ(selector.getSelectionCount(), 1);
}

TEST_F(GnssSelectorTest, rankedIgnoresFixTypeSatellitesAndRate)
{
	GnssSelector selector;

	// GIVEN: two receivers without a preferred one, gps1 selected
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(1.f, selector, gnss_data1, 1);
	gnss_data0.timestamp = gnss_data1.timestamp;

	EXPECT_EQ(selector.getSelectedInstance(), 1);

	// WHEN: gps0 reports a better fix type and more satellites at the same accuracy, and gps1 publishes at twice
	// gps0's rate
	gnss_data0.fix_type = sensor_gnss_s::FIX_TYPE_RTK_FIXED;
	gnss_data0.satellites_used = gnss_data1.satellites_used + 10;

	runSecondsSlowGps0(20.f, selector, gnss_data0, gnss_data1, 2);

	// THEN: gps1 is kept, none of them is used for ranking
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_RANKED);
	EXPECT_EQ(selector.getSelectionCount(), 0);
}

TEST_F(GnssSelectorTest, rankedSwitchOnFailure)
{
	GnssSelector selector;

	// GIVEN: two receivers without a preferred one, gps0 selected
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_RANKED);

	// WHEN: gps0 fails its checks
	_checks_passed[0] = false;

	runSeconds(3.f, selector, gnss_data0, gnss_data1);

	// THEN: the receiver that passes them is selected
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_UNHEALTHY);
	EXPECT_EQ(selector.getSelectionCount(), 1);

	// WHEN: gps0 passes them again
	_checks_passed[0] = true;

	runSeconds(30.f, selector, gnss_data0, gnss_data1);

	// THEN: gps1 is kept, as gps0 is no more accurate
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionCount(), 1);
}

TEST_F(GnssSelectorTest, rankedAccuracy)
{
	GnssSelector selector;

	// GIVEN: two receivers without a preferred one, gps0 selected, on an armed vehicle
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);
	selector.setArmed(true);

	// WHEN: gps1 reports a better eph, but not half of gps0's
	gnss_data1.eph = 0.4f;

	runSeconds(20.f, selector, gnss_data0, gnss_data1);

	// THEN: gps0 is kept
	EXPECT_EQ(selector.getSelectedInstance(), 0);

	// WHEN: gps1 reports less than half of gps0's eph
	gnss_data1.eph = 0.3f;

	runSeconds(4.5f, selector, gnss_data0, gnss_data1);

	// THEN: the switch waits for the hold time
	EXPECT_EQ(selector.getSelectedInstance(), 0);

	runSeconds(1.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_ACCURACY);
	EXPECT_EQ(selector.getSelectionCount(), 1);

	// WHEN: gps0 catches up
	gnss_data0.eph = 0.2f;

	runSeconds(20.f, selector, gnss_data0, gnss_data1);

	// THEN: gps1 is kept, as gps0 isn't twice as accurate
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionCount(), 1);
}

TEST_F(GnssSelectorTest, rankedAccuracyHoldRestarts)
{
	GnssSelector selector;

	// GIVEN: two receivers without a preferred one, gps0 selected
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);

	// WHEN: gps1 is twice as accurate for 4 s at a time
	for (int cycle = 0; cycle < 5; cycle++) {
		gnss_data1.eph = 0.3f;
		runSeconds(4.f, selector, gnss_data0, gnss_data1);
		gnss_data1.eph = 0.7f;
		runSeconds(0.2f, selector, gnss_data0, gnss_data1);
	}

	// THEN: gps0 is kept
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionCount(), 0);
}

TEST_F(GnssSelectorTest, rankedAccuracyFloor)
{
	GnssSelector selector;

	// GIVEN: two RTK fixed receivers without a preferred one, gps0 selected
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();
	gnss_data0.eph = 0.03f;
	gnss_data0.epv = 0.04f;
	gnss_data1.eph = 0.01f;
	gnss_data1.epv = 0.015f;

	runSeconds(20.f, selector, gnss_data0, gnss_data1);

	// THEN: gps0 is kept, both are below the floor
	EXPECT_EQ(selector.getSelectedInstance(), 0);

	// WHEN: gps0 drops to RTK float
	gnss_data0.eph = 0.2f;
	gnss_data0.epv = 0.3f;

	runSeconds(6.f, selector, gnss_data0, gnss_data1);

	// THEN: the RTK fixed receiver is selected
	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_EQ(selector.getSelectionReason(), vehicle_gnss_s::SELECTION_ACCURACY);
}

TEST_F(GnssSelectorTest, rankedAccuracyNeedsVertical)
{
	GnssSelector selector;

	// GIVEN: two receivers without a preferred one, gps0 selected
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(5.f, selector, gnss_data0, gnss_data1);

	// WHEN: gps1 reports a far better eph, but a worse epv, or no accuracy at all
	gnss_data1.eph = 0.1f;
	gnss_data1.epv = 2.f;

	runSeconds(20.f, selector, gnss_data0, gnss_data1);

	gnss_data1.eph = 0.f;
	gnss_data1.epv = 0.f;

	runSeconds(20.f, selector, gnss_data0, gnss_data1);

	// THEN: gps0 is kept
	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_EQ(selector.getSelectionCount(), 0);
}

TEST_F(GnssSelectorTest, singleReceiverAntennaOffset)
{
	GnssSelector selector;

	sensor_gnss_s gnss_data = getDefaultGnssData();

	const Vector3f offset0(0.1f, 0.0f, -0.05f);
	selector.setAntennaOffset(offset0, 1);

	selector.setGnssData(gnss_data, true, 1);
	selector.update(_time_now_us);

	_time_now_us += 200e3;
	gnss_data.timestamp = _time_now_us - 10e3;
	selector.setGnssData(gnss_data, true, 1);
	selector.update(_time_now_us);

	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_TRUE(selector.isNewOutputDataAvailable());

	const Vector3f &out = selector.getOutputAntennaOffset();
	EXPECT_FLOAT_EQ(out(0), offset0(0));
	EXPECT_FLOAT_EQ(out(1), offset0(1));
	EXPECT_FLOAT_EQ(out(2), offset0(2));
}

TEST_F(GnssSelectorTest, failoverAntennaOffset)
{
	GnssSelector selector;

	selector.setPreferredInstance(0);

	const Vector3f offset0(0.1f, 0.0f, 0.0f);
	const Vector3f offset1(-0.1f, 0.0f, 0.0f);
	selector.setAntennaOffset(offset0, 0);
	selector.setAntennaOffset(offset1, 1);

	// Only secondary available
	sensor_gnss_s gnss_data1 = getDefaultGnssData();
	runSeconds(10.f, selector, gnss_data1, 1);

	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_FLOAT_EQ(selector.getOutputAntennaOffset()(0), offset1(0));

	// Now the preferred receiver publishes
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	gnss_data0.timestamp = gnss_data1.timestamp;
	runSeconds(1.f, selector, gnss_data0, gnss_data1);

	EXPECT_EQ(selector.getSelectedInstance(), 0);
	EXPECT_FLOAT_EQ(selector.getOutputAntennaOffset()(0), offset0(0));
}

TEST_F(GnssSelectorTest, noOutputWithoutNewData)
{
	GnssSelector selector;

	// GIVEN: two receivers, gps1 selected as it published first
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	selector.setGnssData(gnss_data1, true, 1);
	selector.update(_time_now_us);

	_time_now_us += 100e3;
	gnss_data0.timestamp = _time_now_us - 10e3;
	gnss_data1.timestamp = _time_now_us - 10e3;
	selector.setGnssData(gnss_data0, true, 0);
	selector.setGnssData(gnss_data1, true, 1);
	selector.update(_time_now_us);

	EXPECT_EQ(selector.getSelectedInstance(), 1);
	EXPECT_TRUE(selector.isNewOutputDataAvailable());

	// WHEN: an update runs without new data from either receiver
	selector.update(_time_now_us);

	// THEN: no output is reported. The updated flag of the receiver that isn't selected must be cleared as well
	EXPECT_FALSE(selector.isNewOutputDataAvailable());
}

TEST_F(GnssSelectorTest, availability)
{
	GnssSelector selector;

	selector.setPreferredInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	// GIVEN: gps1 hasn't passed its checks yet
	_checks_passed[1] = false;

	runSeconds(5.f, selector, gnss_data0, gnss_data1);

	// THEN: a receiver starts fully available once it first passes, and has no availability before
	EXPECT_FLOAT_EQ(selector.getAvailability(0), 1.f);
	EXPECT_FLOAT_EQ(selector.getAvailability(1), 0.f);

	// WHEN: gps0 fails its checks for one time constant
	_checks_passed[0] = false;

	runSeconds(10.f, selector, gnss_data0, gnss_data1);

	// THEN: its availability decays like a first order filter
	EXPECT_NEAR(selector.getAvailability(0), expf(-1.f), 0.01f);
}
