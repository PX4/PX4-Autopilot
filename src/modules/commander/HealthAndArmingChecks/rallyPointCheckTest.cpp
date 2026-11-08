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

#include "checks/rallyPointCheck.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/event.h>

// to run: make tests TESTFILTER=rallyPointCheck

/* EVENT
 * @skip-file
 */

class RallyPointChecksTest : public ::testing::Test
{
public:
	void SetUp() override { param_control_autosave(false); }
	void TearDown() override { param_reset(param_find("RTL_TYPE")); }

	struct Result {
		bool warned;
		bool return_can_run;
		bool return_required;
	};

	Result check(int32_t rtl_type, uint8_t nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL,
		     uint8_t user_intention = vehicle_status_s::NAVIGATION_STATE_MANUAL)
	{
		param_set_no_notification(param_find("RTL_TYPE"), &rtl_type);
		vehicle_status_s status{};
		status.nav_state = nav_state;
		status.nav_state_user_intention = user_intention;
		Context context{status};
		failsafe_flags_s failsafe_flags{};
		Report reporter{failsafe_flags, 0};
		RallyPointChecks check;
		check.checkAndReport(context, reporter);

		EXPECT_EQ(static_cast<uint64_t>(reporter.armingCheckResults().error), 0u);
		const uint32_t return_mode = 1u << vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		return {
			.warned = static_cast<bool>(reporter.armingCheckResults().warning & health_component_t::system),
			.return_can_run = (static_cast<uint32_t>(reporter.armingCheckResults().can_run) & return_mode) != 0,
			.return_required = (failsafe_flags.mode_req_other & return_mode) != 0,
		};
	}

	void publishRtlStatus(uint8_t rtl_type, bool pending, uint8_t safe_point_index = UINT8_MAX,
			      hrt_abstime timestamp = hrt_absolute_time())
	{
		rtl_status_s rtl_status{};
		rtl_status.timestamp = timestamp;
		rtl_status.rtl_type = rtl_type;
		rtl_status.is_evaluation_pending = pending;
		rtl_status.safe_point_index = safe_point_index;
		_rtl_status_pub.publish(rtl_status);
	}

private:
	uORB::Publication<rtl_status_s> _rtl_status_pub{ORB_ID(rtl_status)};
};

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
// Each row publishes a route status, then checks RTL_TYPE in a mode with a user intention.
TEST_F(RallyPointChecksTest, RouteStatusDecidesWhetherReturnCanRun)
{
	static constexpr uint8_t kDirect = rtl_status_s::RTL_STATUS_TYPE_DIRECT_SAFE_POINT;
	static constexpr uint8_t kRoute = rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION_SAFE_POINT;
	static constexpr uint8_t kManual = vehicle_status_s::NAVIGATION_STATE_MANUAL;
	static constexpr uint8_t kReturn = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
	static constexpr uint8_t kHold = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
	static constexpr uint8_t kMission = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	static constexpr uint8_t kNone = UINT8_MAX;

	const struct {
		const char *name;
		int32_t rtl_type;
		uint8_t status_type;
		bool pending;
		uint8_t safe_point_index;
		int age_s; // -1: no timestamp
		uint8_t nav_state;
		uint8_t user_intention;
		Result expected;
	} cases[] {
		{"no route", 7, kDirect, false, kNone, 0, kManual, kManual, {true, false, true}},
		{"route", 7, kRoute, false, 0, 0, kManual, kManual, {false, true, false}},
		{"no timestamp", 7, kRoute, false, 0, -1, kManual, kManual, {true, false, true}},
		{"stale route", 7, kRoute, false, 0, 6, kManual, kManual, {true, false, true}},
		{"inputs loading", 7, kDirect, true, kNone, 0, kManual, kManual, {true, false, true}},
		// A running Return, also a failsafe one, is not interrupted and does not depend on the status age.
		{"active Return without route", 7, kDirect, false, kNone, 0, kReturn, kManual, {false, true, false}},
		{"active Return, old status", 7, kRoute, false, 0, 6, kReturn, kManual, {false, true, false}},
		// A failsafe Hold replaces the selected Return: it warns, but Return must not fall back to Land.
		{"Hold in Return, old route", 7, kRoute, false, 0, 6, kHold, kReturn, {true, true, false}},
		{"Hold in Return, loading", 7, kDirect, true, kNone, 0, kHold, kReturn, {true, true, false}},
		{"Hold in Return, no route", 7, kDirect, false, kNone, 0, kHold, kReturn, {true, true, false}},
		{"Hold in Mission", 7, kDirect, false, kNone, 0, kHold, kMission, {true, false, true}},
		// Other types ignore the route; type 5 only warns about the missing rally point.
		{"type 3", 3, kDirect, false, kNone, 0, kManual, kManual, {false, true, false}},
		{"type 5", 5, kDirect, false, kNone, 0, kManual, kManual, {true, true, false}},
	};

	for (const auto &row : cases) {
		SCOPED_TRACE(row.name);
		const hrt_abstime timestamp = row.age_s < 0 ? 0 : hrt_absolute_time() - static_cast<hrt_abstime>(row.age_s) * 1_s;
		publishRtlStatus(row.status_type, row.pending, row.safe_point_index, timestamp);
		const Result result = check(row.rtl_type, row.nav_state, row.user_intention);
		EXPECT_EQ(result.warned, row.expected.warned);
		EXPECT_EQ(result.return_can_run, row.expected.return_can_run);
		EXPECT_EQ(result.return_required, row.expected.return_required);
	}
}

TEST_F(RallyPointChecksTest, RouteReturnWarningIsReportedInEveryMode)
{
	// Ground stations list a check only in the modes it is reported for, the event's first argument.
	int32_t rtl_type = 7;
	param_set_no_notification(param_find("RTL_TYPE"), &rtl_type);
	publishRtlStatus(rtl_status_s::RTL_STATUS_TYPE_DIRECT_SAFE_POINT, false);
	// the event topic has to exist before the first send, or the queued events are lost
	orb_advertise(ORB_ID(event), nullptr);
	uORB::Subscription event_sub{ORB_ID(event)};
	event_s event{};

	const struct {
		const char *name;
		uint8_t nav_state;
		uint8_t user_intention;
	} cases[] {
		{"Position", vehicle_status_s::NAVIGATION_STATE_POSCTL, vehicle_status_s::NAVIGATION_STATE_POSCTL},
		{"Hold in Return", vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL},
	};

	for (const auto &c : cases) {
		SCOPED_TRACE(c.name);
		vehicle_status_s status{};
		status.nav_state = c.nav_state;
		status.nav_state_user_intention = c.user_intention;
		Context context{status};
		failsafe_flags_s failsafe_flags{};
		Report reporter{failsafe_flags, 0};
		RallyPointChecks check;

		while (event_sub.update(&event)) {}

		reporter.reset();
		check.checkAndReport(context, reporter);
		reporter.finalize();
		reporter.report(false);

		uint32_t reported_modes = 0;

		while (event_sub.update(&event)) {
			if (event.id == events::ID("check_rtl_route_unavailable")) {
				memcpy(&reported_modes, event.arguments, sizeof(reported_modes));
			}
		}

		EXPECT_EQ(reported_modes, static_cast<uint32_t>(NavModes::All));
		// Arming stays possible in the other modes, and the check still fails for Return.
		EXPECT_TRUE(reporter.canArm(vehicle_status_s::NAVIGATION_STATE_POSCTL));
		EXPECT_FALSE(reporter.canArm(vehicle_status_s::NAVIGATION_STATE_AUTO_RTL));
	}
}
#else
TEST_F(RallyPointChecksTest, RouteReturnWarnsWithoutRoutePlanning)
{
	const Result result = check(7);
	EXPECT_TRUE(result.warned);
	EXPECT_TRUE(result.return_can_run);
	EXPECT_FALSE(result.return_required);
}
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
