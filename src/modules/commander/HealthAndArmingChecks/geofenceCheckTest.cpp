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

	static geofence_result_s resultWithActions(uint8_t geofence_emer_action, uint8_t geofence_cont_action)
	{
		geofence_result_s result{};
		result.geofence_emer_action = geofence_emer_action;
		result.geofence_cont_action = geofence_cont_action;
		return result;
	}

	failsafe_flags_s _failsafe_flags{};
	bool             _can_arm_all_modes{false};
	GeofenceChecks   _check;
	uORB::Publication<geofence_result_s> _geofence_result_pub{ORB_ID(geofence_result)};
};

// Each trigger sets the flag of its fence and denies arming, unless that fence's action is None.
TEST_F(GeofenceChecksTest, EachTriggerSetsItsOwnFlagAndDeniesArming)
{
	const struct {
		const char *name;
		bool geofence_result_s::*trigger;
		bool contingency;
	} triggers[] = {
		{"cont_max_dist", &geofence_result_s::geofence_cont_max_dist_triggered, true},
		{"cont_custom_fence", &geofence_result_s::geofence_cont_custom_fence_triggered, true},
		{"emer_max_dist", &geofence_result_s::geofence_emer_max_dist_triggered, false},
		{"emer_max_alt", &geofence_result_s::geofence_emer_max_alt_triggered, false},
		{"emer_custom_fence", &geofence_result_s::geofence_emer_custom_fence_triggered, false},
	};

	for (const auto &c : triggers) {
		SCOPED_TRACE(c.name);

		for (const auto action : {
			     geofence_result_s::GF_ACTION_NONE, geofence_result_s::GF_ACTION_WARN,
			     geofence_result_s::GF_ACTION_LOITER
		     }) {
			SCOPED_TRACE(action);
			geofence_result_s result = resultWithActions(action, action);
			runCheck(result);
			EXPECT_FALSE(_failsafe_flags.gf_emer_breach);
			EXPECT_FALSE(_failsafe_flags.gf_cont_breach);
			EXPECT_TRUE(_can_arm_all_modes);

			result.*(c.trigger) = true;
			runCheck(result);
			const bool enabled = action != geofence_result_s::GF_ACTION_NONE;
			EXPECT_EQ(_failsafe_flags.gf_emer_breach, enabled && !c.contingency);
			EXPECT_EQ(_failsafe_flags.gf_cont_breach, enabled && c.contingency);
			EXPECT_EQ(_can_arm_all_modes, !enabled); // Warning also prevents arming during a breach.
		}
	}
}

// The contingency geofence does not depend on GF_ACTION.
TEST_F(GeofenceChecksTest, ContGeofenceWorksWithoutAGeofenceAction)
{
	geofence_result_s result = resultWithActions(geofence_result_s::GF_ACTION_NONE, geofence_result_s::GF_ACTION_LOITER);
	result.geofence_cont_max_dist_triggered = true;
	result.geofence_emer_max_dist_triggered = true;
	runCheck(result);
	EXPECT_FALSE(_failsafe_flags.gf_emer_breach);
	EXPECT_TRUE(_failsafe_flags.gf_cont_breach);
	EXPECT_FALSE(_can_arm_all_modes);
}

TEST_F(GeofenceChecksTest, GeofenceAndContGeofenceSetIndependentFlags)
{
	geofence_result_s result = resultWithActions(geofence_result_s::GF_ACTION_TERMINATE,
				   geofence_result_s::GF_ACTION_LOITER);
	result.geofence_emer_custom_fence_triggered = true;
	runCheck(result);
	EXPECT_TRUE(_failsafe_flags.gf_emer_breach);
	EXPECT_FALSE(_failsafe_flags.gf_cont_breach);
	EXPECT_FALSE(_can_arm_all_modes);

	result.geofence_cont_custom_fence_triggered = true;
	runCheck(result);
	EXPECT_TRUE(_failsafe_flags.gf_emer_breach);
	EXPECT_TRUE(_failsafe_flags.gf_cont_breach);
	EXPECT_FALSE(_can_arm_all_modes);
}

// Either geofence with a Return action needs a valid Home to arm.
TEST_F(GeofenceChecksTest, EitherGeofenceReturnNeedsAValidHome)
{
	for (const bool contingency : {false, true}) {
		SCOPED_TRACE(contingency);
		geofence_result_s result = resultWithActions(
						   contingency ? geofence_result_s::GF_ACTION_LOITER : geofence_result_s::GF_ACTION_RTL,
						   contingency ? geofence_result_s::GF_ACTION_RTL : geofence_result_s::GF_ACTION_LOITER);
		runCheck(result);
		EXPECT_TRUE(_can_arm_all_modes);
		runCheck(result, true);
		EXPECT_FALSE(_can_arm_all_modes);

		result.geofence_emer_action = geofence_result_s::GF_ACTION_LOITER;
		result.geofence_cont_action = geofence_result_s::GF_ACTION_LOITER;
		runCheck(result, true);
		EXPECT_TRUE(_can_arm_all_modes);
	}
}
