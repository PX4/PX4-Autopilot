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

#include <gtest/gtest.h>

#include "checks/geofenceCheck.hpp"

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/geofence_result.h>

// to run: make tests TESTFILTER=geofenceCheck

/* EVENT
 * @skip-file
 */

class GeofenceChecksTest : public ::testing::Test
{
public:
	// Publish what Navigator found, run the check and store its results.
	void runCheck(const geofence_result_s &result, bool home_position_invalid = false)
	{
		geofence_result_s published = result;
		published.timestamp = hrt_absolute_time();
		_geofence_result_pub.publish(published);

		vehicle_status_s status{};
		Context context{status};
		_failsafe_flags = {};
		_failsafe_flags.home_position_invalid = home_position_invalid;
		Report reporter{_failsafe_flags, 0};
		_check.checkAndReport(context, reporter);
		_can_arm_all_modes = reporter.armingCheckResults().can_arm == NavModes::All;
	}

	static geofence_result_s resultWithAction(uint8_t action)
	{
		geofence_result_s result{};
		result.geofence_action = action;
		return result;
	}

	failsafe_flags_s _failsafe_flags{};
	bool             _can_arm_all_modes{false};
	GeofenceChecks   _check;
	uORB::Publication<geofence_result_s> _geofence_result_pub{ORB_ID(geofence_result)};
};

// A pre-geofence violation sets its own flag and denies arming before the hard fence is breached.
TEST_F(GeofenceChecksTest, PreGeofenceSetsItsOwnFlagAndDeniesArming)
{
	geofence_result_s result = resultWithAction(geofence_result_s::GF_ACTION_TERMINATE);
	result.pre_geofence_action = geofence_result_s::GF_ACTION_LOITER;
	runCheck(result);
	EXPECT_FALSE(_failsafe_flags.geofence_breached);
	EXPECT_FALSE(_failsafe_flags.pre_geofence);
	EXPECT_TRUE(_can_arm_all_modes);

	result.pre_geofence_triggered = true;
	runCheck(result);
	EXPECT_FALSE(_failsafe_flags.geofence_breached);
	EXPECT_TRUE(_failsafe_flags.pre_geofence);
	EXPECT_FALSE(_can_arm_all_modes);
}

// The pre-geofence does not depend on GF_ACTION.
TEST_F(GeofenceChecksTest, PreGeofenceWorksWithoutAGeofenceAction)
{
	geofence_result_s result = resultWithAction(geofence_result_s::GF_ACTION_NONE);
	result.pre_geofence_action = geofence_result_s::GF_ACTION_LOITER;
	result.pre_geofence_triggered = true;
	result.geofence_custom_fence_triggered = true;
	runCheck(result);
	EXPECT_FALSE(_failsafe_flags.geofence_breached);
	EXPECT_TRUE(_failsafe_flags.pre_geofence);
	EXPECT_FALSE(_can_arm_all_modes);
}

TEST_F(GeofenceChecksTest, HardAndPreGeofenceBreachesSetIndependentFlags)
{
	geofence_result_s result = resultWithAction(geofence_result_s::GF_ACTION_TERMINATE);
	result.pre_geofence_action = geofence_result_s::GF_ACTION_LOITER;
	result.geofence_custom_fence_triggered = true;
	runCheck(result);
	EXPECT_TRUE(_failsafe_flags.geofence_breached);
	EXPECT_FALSE(_failsafe_flags.pre_geofence);
	EXPECT_FALSE(_can_arm_all_modes);

	result.pre_geofence_triggered = true;
	runCheck(result);
	EXPECT_TRUE(_failsafe_flags.geofence_breached);
	EXPECT_TRUE(_failsafe_flags.pre_geofence);
	EXPECT_FALSE(_can_arm_all_modes);
}

// A loaded pre-geofence with a Return action needs a valid Home to arm.
TEST_F(GeofenceChecksTest, PreGeofenceReturnNeedsAValidHome)
{
	geofence_result_s result = resultWithAction(geofence_result_s::GF_ACTION_LOITER);
	result.pre_geofence_action = geofence_result_s::GF_ACTION_RTL;
	runCheck(result);
	EXPECT_TRUE(_can_arm_all_modes);
	runCheck(result, true);
	EXPECT_FALSE(_can_arm_all_modes);

	result.pre_geofence_action = geofence_result_s::GF_ACTION_LOITER;
	runCheck(result, true);
	EXPECT_TRUE(_can_arm_all_modes);
}

// No pre-geofence means no action, even if an old trigger remains in a received result.
TEST_F(GeofenceChecksTest, AbsentPreGeofenceIgnoresStaleTrigger)
{
	geofence_result_s result = resultWithAction(geofence_result_s::GF_ACTION_NONE);
	result.pre_geofence_triggered = true;
	runCheck(result, true);
	EXPECT_FALSE(_failsafe_flags.geofence_breached);
	EXPECT_FALSE(_failsafe_flags.pre_geofence);
	EXPECT_TRUE(_can_arm_all_modes);
}
