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

	Result check(int32_t rtl_type, uint8_t nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL)
	{
		param_set_no_notification(param_find("RTL_TYPE"), &rtl_type);
		vehicle_status_s status{};
		status.nav_state = nav_state;
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
TEST_F(RallyPointChecksTest, RouteReturnWithoutRouteIsUnavailable)
{
	publishRtlStatus(rtl_status_s::RTL_STATUS_TYPE_DIRECT_SAFE_POINT, false);
	const Result result = check(7);
	EXPECT_TRUE(result.warned);
	EXPECT_FALSE(result.return_can_run);
	EXPECT_TRUE(result.return_required);
}

TEST_F(RallyPointChecksTest, RouteReturnWithRouteIsAvailable)
{
	publishRtlStatus(rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION_SAFE_POINT, false, 0);
	const Result result = check(7);
	EXPECT_FALSE(result.warned);
	EXPECT_TRUE(result.return_can_run);
	EXPECT_FALSE(result.return_required);
}

TEST_F(RallyPointChecksTest, RouteReturnWithoutTimestampIsUnavailable)
{
	publishRtlStatus(rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION_SAFE_POINT, false, 0, 0);
	const Result result = check(7);
	EXPECT_TRUE(result.warned);
	EXPECT_FALSE(result.return_can_run);
	EXPECT_TRUE(result.return_required);
}

TEST_F(RallyPointChecksTest, RouteReturnWithStaleRouteIsUnavailable)
{
	publishRtlStatus(rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION_SAFE_POINT, false, 0,
			 hrt_absolute_time() - 6_s);
	const Result result = check(7);
	EXPECT_TRUE(result.warned);
	EXPECT_FALSE(result.return_can_run);
	EXPECT_TRUE(result.return_required);
}

TEST_F(RallyPointChecksTest, RouteReturnIsUnavailableWhileTheRouteInputsLoad)
{
	publishRtlStatus(rtl_status_s::RTL_STATUS_TYPE_DIRECT_SAFE_POINT, true);
	const Result result = check(7);
	EXPECT_TRUE(result.warned);
	EXPECT_FALSE(result.return_can_run);
	EXPECT_TRUE(result.return_required);
}

TEST_F(RallyPointChecksTest, ActiveRouteReturnIsNotInterrupted)
{
	publishRtlStatus(rtl_status_s::RTL_STATUS_TYPE_DIRECT_SAFE_POINT, false);
	const Result result = check(7, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
	EXPECT_FALSE(result.warned);
	EXPECT_TRUE(result.return_can_run);
	EXPECT_FALSE(result.return_required);
}

TEST_F(RallyPointChecksTest, ActiveRouteReturnDoesNotRequireFreshStatus)
{
	// Active Return publishes status on selection, not on every execution cycle.
	publishRtlStatus(rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION_SAFE_POINT, false, 0,
			 hrt_absolute_time() - 6_s);
	const Result result = check(7, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
	EXPECT_FALSE(result.warned);
	EXPECT_TRUE(result.return_can_run);
	EXPECT_FALSE(result.return_required);
}

TEST_F(RallyPointChecksTest, OtherReturnTypesIgnoreTheRoute)
{
	publishRtlStatus(rtl_status_s::RTL_STATUS_TYPE_DIRECT_SAFE_POINT, false);
	const Result direct = check(3);
	EXPECT_FALSE(direct.warned);
	EXPECT_TRUE(direct.return_can_run);
	EXPECT_FALSE(direct.return_required);

	// Type 5 only warns about the missing rally point.
	const Result rally = check(5);
	EXPECT_TRUE(rally.warned);
	EXPECT_TRUE(rally.return_can_run);
	EXPECT_FALSE(rally.return_required);
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
