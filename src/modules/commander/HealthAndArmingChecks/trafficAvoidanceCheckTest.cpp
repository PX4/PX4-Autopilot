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

#include "checks/trafficAvoidanceCheck.hpp"

#include <px4_platform_common/param.h>

// to run: make tests TESTFILTER=trafficAvoidanceCheck

/* EVENT
 * @skip-file
 */

class TrafficAvoidanceChecksTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		param_reset(param_find("COM_TRAFF_AVOID"));
	}

	// Run the check and store results in _failsafe_flags and the health/arming result caches.
	void runCheck(bool system_present)
	{
		vehicle_status_s status{};
		status.traffic_avoidance_system_present = system_present;

		_check.updateParams();
		Context context{status};
		_failsafe_flags = {};
		Report reporter{_failsafe_flags, 0};
		_check.checkAndReport(context, reporter);
		_health_error_traffic_avoidance = (bool)(reporter.healthResults().error & health_component_t::traffic_avoidance);
		_health_warning_traffic_avoidance = (bool)(reporter.healthResults().warning & health_component_t::traffic_avoidance);
		_is_present_traffic_avoidance = (bool)(reporter.healthResults().is_present & health_component_t::traffic_avoidance);
		_can_arm_all_modes = reporter.armingCheckResults().can_arm == NavModes::All;
	}

	void setParam(int com_traff_avoid)
	{
		param_set(param_find("COM_TRAFF_AVOID"), &com_traff_avoid);
	}

	failsafe_flags_s  _failsafe_flags{};
	bool              _health_error_traffic_avoidance{false};
	bool              _health_warning_traffic_avoidance{false};
	bool              _is_present_traffic_avoidance{false};
	bool              _can_arm_all_modes{false};
	TrafficAvoidanceChecks _check;
};

// COM_TRAFF_AVOID = 0 (Disabled): missing system is ignored entirely, arming unaffected.
TEST_F(TrafficAvoidanceChecksTest, DisabledIgnoresMissingSystem)
{
	setParam(0);

	runCheck(false);
	EXPECT_FALSE(_failsafe_flags.traffic_avoidance_unhealthy);
	EXPECT_FALSE(_health_error_traffic_avoidance);
	EXPECT_FALSE(_health_warning_traffic_avoidance);
	EXPECT_TRUE(_can_arm_all_modes);
}

// COM_TRAFF_AVOID = 1 (Warning): missing system sets unhealthy + a warning, but does NOT block arming.
TEST_F(TrafficAvoidanceChecksTest, WarningMissingSystemDoesNotBlockArming)
{
	setParam(1);

	runCheck(false);
	EXPECT_TRUE(_failsafe_flags.traffic_avoidance_unhealthy);
	EXPECT_TRUE(_health_warning_traffic_avoidance);
	EXPECT_FALSE(_health_error_traffic_avoidance);
	EXPECT_TRUE(_can_arm_all_modes);
}

// COM_TRAFF_AVOID = 2 (WarningBlockArming): missing system sets unhealthy + an error, and blocks arming.
TEST_F(TrafficAvoidanceChecksTest, WarningBlockArmingMissingSystemBlocksArming)
{
	setParam(2);

	runCheck(false);
	EXPECT_TRUE(_failsafe_flags.traffic_avoidance_unhealthy);
	EXPECT_TRUE(_health_error_traffic_avoidance);
	EXPECT_FALSE(_can_arm_all_modes);
}

// COM_TRAFF_AVOID >= 1 and system present -> healthy, present, arming unaffected.
TEST_F(TrafficAvoidanceChecksTest, PresentSystemIsHealthy)
{
	setParam(2);

	runCheck(true);
	EXPECT_FALSE(_failsafe_flags.traffic_avoidance_unhealthy);
	EXPECT_FALSE(_health_error_traffic_avoidance);
	EXPECT_FALSE(_health_warning_traffic_avoidance);
	EXPECT_TRUE(_is_present_traffic_avoidance);
	EXPECT_TRUE(_can_arm_all_modes);
}
