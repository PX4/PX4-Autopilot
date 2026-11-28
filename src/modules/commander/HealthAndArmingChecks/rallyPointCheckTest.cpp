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
#include <uORB/topics/health_report.h>
#include <uORB/topics/rtl_status.h>

// to run: make tests TESTFILTER=rallyPointCheck

/* EVENT
 * @skip-file
 */

class RallyPointChecksTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		param_reset(param_find("COM_ARM_RALLY_PT"));
		setParam("RTL_TYPE", kRtlTypeRallyPointOnly);
	}

	static void setParam(const char *name, int32_t value)
	{
		param_set(param_find(name), &value);
	}

	// Publish the rally point Return selected, run the check and keep its results for the assertions.
	void runCheck(uint8_t safe_point_index)
	{
		rtl_status_s rtl_status{};
		rtl_status.timestamp = hrt_absolute_time();
		rtl_status.safe_point_index = safe_point_index;
		_rtl_status_pub.publish(rtl_status);
		runCheck();
	}

	void runCheck()
	{
		vehicle_status_s status{};
		Context context{status};
		_check.updateParams();
		Report reporter{_failsafe_flags, 0};
		_check.checkAndReport(context, reporter);
		_can_arm = reporter.armingCheckResults().can_arm;
		reporter.getHealthReport(_report);
	}

	bool armingError() const
	{
		return _report.arming_check_error_flags & (uint64_t)health_component_t::system;
	}

	bool armingWarning() const
	{
		return _report.arming_check_warning_flags & (uint64_t)health_component_t::system;
	}

	static constexpr int32_t kRtlTypeRallyPointOnly = 5;
	static constexpr uint8_t kNoRallyPoint = UINT8_MAX; // safe_point_index when Return found none

	failsafe_flags_s _failsafe_flags{};
	health_report_s _report{};
	NavModes _can_arm{NavModes::None};
	RallyPointChecks _check;
	uORB::Publication<rtl_status_s> _rtl_status_pub{ORB_ID(rtl_status)};
};

// rtl_status stays advertised for the rest of the process, so this case runs before any that publishes it.
TEST_F(RallyPointChecksTest, WaitsForReturnToReport)
{
	runCheck();
	EXPECT_EQ(_can_arm, NavModes::All);
	EXPECT_FALSE(armingError());
	EXPECT_FALSE(armingWarning());
}

TEST_F(RallyPointChecksTest, MissingRallyPointDeniesArmingByDefault)
{
	runCheck(kNoRallyPoint);
	EXPECT_EQ(_can_arm, NavModes::None);
	EXPECT_TRUE(armingError());
	EXPECT_FALSE(armingWarning());
}

TEST_F(RallyPointChecksTest, MissingRallyPointOnlyWarnsWhenConfigured)
{
	setParam("COM_ARM_RALLY_PT", 0);
	runCheck(kNoRallyPoint);
	EXPECT_EQ(_can_arm, NavModes::All);
	EXPECT_FALSE(armingError());
	EXPECT_TRUE(armingWarning());
}

TEST_F(RallyPointChecksTest, ARallyPointAllowsArming)
{
	runCheck(0);
	EXPECT_EQ(_can_arm, NavModes::All);
	EXPECT_FALSE(armingError());
	EXPECT_FALSE(armingWarning());
}

TEST_F(RallyPointChecksTest, OnlyActiveForTheRallyPointReturnType)
{
	setParam("RTL_TYPE", 0);
	runCheck(kNoRallyPoint);
	EXPECT_EQ(_can_arm, NavModes::All);
	EXPECT_FALSE(armingError());
	EXPECT_FALSE(armingWarning());
}
