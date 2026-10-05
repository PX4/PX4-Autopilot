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

#include <px4_platform_common/param.h>

// to run: make tests TESTFILTER=rallyPointCheck

/* EVENT
 * @skip-file
 */

class RallyPointChecksTest : public ::testing::Test
{
public:
	void SetUp() override { param_control_autosave(false); }
	void TearDown() override { param_reset(param_find("RTL_TYPE")); }

	bool warns(int32_t rtl_type)
	{
		param_set_no_notification(param_find("RTL_TYPE"), &rtl_type);
		vehicle_status_s status{};
		Context context{status};
		failsafe_flags_s failsafe_flags{};
		Report reporter{failsafe_flags, 0};
		RallyPointChecks check;
		check.checkAndReport(context, reporter);

		EXPECT_EQ(static_cast<uint64_t>(reporter.armingCheckResults().error), 0u);

		EXPECT_EQ(reporter.armingCheckResults().can_arm, NavModes::All);
		EXPECT_EQ(reporter.armingCheckResults().can_run, NavModes::All);

		return reporter.armingCheckResults().warning & health_component_t::system;
	}
};

TEST_F(RallyPointChecksTest, RouteReturnWarnsOnlyWithoutRoutePlanning)
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	EXPECT_FALSE(warns(7));
#else
	EXPECT_TRUE(warns(7));
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
	EXPECT_FALSE(warns(1));
}
