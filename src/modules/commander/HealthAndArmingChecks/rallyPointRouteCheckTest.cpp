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
 *    the distribution.
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
#include <uORB/topics/health_report.h>
#include <uORB/topics/rtl_status.h>

// to run: make tests TESTFILTER=rallyPointRouteCheck

/* EVENT
 * @skip-file
 */

using namespace time_literals;

namespace
{

enum class RouteStatus : uint8_t { Fresh, Stale, NoRoute };

struct RouteCheckCase {
	const char *test_name;
	uint8_t vehicle_type;
	bool is_vtol;
	RouteStatus route_status;
	uint8_t nav_state;
	uint8_t user_intention;
	bool route_warning; ///< "No mission route for Return": arming in Return fails
	bool return_denied; ///< no switch to Return, a failsafe falls back
	bool unavailable_warning; ///< "RTL_TYPE 7 not available": a warning only
};

} // namespace

class RallyPointRouteCheckTest : public ::testing::TestWithParam<RouteCheckCase>
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		const int32_t rtl_type = 7;
		param_set(param_find("RTL_TYPE"), &rtl_type);
	}

	void runCheck(const RouteCheckCase &c)
	{
		rtl_status_s rtl_status{};
		rtl_status.timestamp = (c.route_status == RouteStatus::Stale) ? hrt_absolute_time() - 6_s : hrt_absolute_time();
		rtl_status.rtl_type = (c.route_status == RouteStatus::NoRoute) ? rtl_status_s::RTL_STATUS_TYPE_NONE
				      : rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION_SAFE_POINT;
		_rtl_status_pub.publish(rtl_status);

		vehicle_status_s status{};
		status.vehicle_type = c.vehicle_type;
		status.is_vtol = c.is_vtol;
		status.nav_state = c.nav_state;
		status.nav_state_user_intention = c.user_intention;
		Context context{status};
		Report reporter{_failsafe_flags, 0};
		_check.checkAndReport(context, reporter);
		_return_can_arm = (reporter.armingCheckResults().can_arm & kReturn) != NavModes::None;
		_return_can_run = (reporter.armingCheckResults().can_run & kReturn) != NavModes::None;
		health_report_s report{};
		reporter.getHealthReport(report);
		_reported = ((report.arming_check_warning_flags | report.arming_check_error_flags)
			     & (uint64_t)health_component_t::system) != 0;
	}

	static constexpr NavModes kReturn = static_cast<NavModes>(1u << vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);

	failsafe_flags_s _failsafe_flags{};
	bool _return_can_arm{false};
	bool _return_can_run{false};
	bool _reported{false};
	RallyPointChecks _check;
	uORB::Publication<rtl_status_s> _rtl_status_pub{ORB_ID(rtl_status)};
};

// WHY: with RTL_TYPE 7 a Return without a route holds. A switch to Return then is denied and a failsafe lands instead,
// while a Return that already runs keeps holding. Where type 7 is not available, the operator learns what Return does.
// WHAT: the check for each vehicle and route status, in and out of Return.
TEST_P(RallyPointRouteCheckTest, ReturnNeedsARouteWithType7)
{
	const RouteCheckCase &c = GetParam();

	runCheck(c);

	EXPECT_EQ(_reported, c.route_warning || c.unavailable_warning);
	EXPECT_EQ(_return_can_arm, !c.route_warning);
	EXPECT_EQ(_return_can_run, !c.return_denied);
	EXPECT_EQ((_failsafe_flags.mode_req_other & static_cast<uint32_t>(kReturn)) != 0, c.return_denied);
}

static constexpr uint8_t kMulticopter = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
static constexpr uint8_t kPosition = vehicle_status_s::NAVIGATION_STATE_POSCTL;
static constexpr uint8_t kReturnMode = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

static const RouteCheckCase kRouteCheckCases[] = {
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	RouteCheckCase{"WithRoute", kMulticopter, false, RouteStatus::Fresh, kPosition, kPosition, false, false, false},
	RouteCheckCase{"WithoutRoute", kMulticopter, false, RouteStatus::NoRoute, kPosition, kPosition, true, true, false},
	RouteCheckCase{"WithOldRouteStatus", kMulticopter, false, RouteStatus::Stale, kPosition, kPosition, true, true, false},
	RouteCheckCase{"InReturn", kMulticopter, false, RouteStatus::NoRoute, kReturnMode, kReturnMode, false, false, false},
	RouteCheckCase{"FailsafeDuringASelectedReturn", kMulticopter, false, RouteStatus::NoRoute, vehicle_status_s::NAVIGATION_STATE_AUTO_LAND, kReturnMode, true, false, false},
	RouteCheckCase{"FixedWing", vehicle_status_s::VEHICLE_TYPE_FIXED_WING, false, RouteStatus::NoRoute, kPosition, kPosition, false, false, true},
	RouteCheckCase{"VtolInHover", kMulticopter, true, RouteStatus::NoRoute, kPosition, kPosition, false, false, true},
#else
	// Without the mission route cache, type 7 is not available on any vehicle.
	RouteCheckCase{"WithoutRouteCache", kMulticopter, false, RouteStatus::Fresh, kPosition, kPosition, false, false, true},
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
};

INSTANTIATE_TEST_SUITE_P(
	RTL,
	RallyPointRouteCheckTest,
	::testing::ValuesIn(kRouteCheckCases),
	[](const ::testing::TestParamInfo<RouteCheckCase> &param_info)
{
	return std::string(param_info.param.test_name);
});
