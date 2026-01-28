/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "Common.hpp"
#include <uORB/topics/event.h>
#include <uORB/Subscription.hpp>

#include <stdint.h>

using namespace time_literals;

// to run: make tests TESTFILTER=HealthAndArmingChecks

/* EVENT
 * @skip-file
 */

class ReporterTest : public ::testing::Test

{
public:

	void SetUp() override
	{
		// ensure topic exists, otherwise we might lose first queued events
		orb_advertise(ORB_ID(event), nullptr);
	}

};


TEST_F(ReporterTest, basic_no_checks)
{
	failsafe_flags_s failsafe_flags{};
	Report reporter{failsafe_flags, 0_s};
	ASSERT_FALSE(reporter.canArm(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION));

	reporter.reset();
	reporter.finalize();
	reporter.report(false);

	ASSERT_TRUE(reporter.canArm(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION));
	ASSERT_EQ((uint8_t)reporter.armingCheckResults().can_arm, 0xff);
	ASSERT_EQ((uint64_t)reporter.armingCheckResults().error, 0);
	ASSERT_EQ((uint64_t)reporter.armingCheckResults().warning, 0);

	ASSERT_EQ((uint64_t)reporter.healthResults().is_present, 0);
	ASSERT_EQ((uint64_t)reporter.healthResults().error, 0);
	ASSERT_EQ((uint64_t)reporter.healthResults().warning, 0);
}

TEST_F(ReporterTest, basic_fail_all_modes)
{
	failsafe_flags_s failsafe_flags{};
	Report reporter{failsafe_flags, 0_s};

	// ensure arming is always denied with a NavModes::All failure
	for (uint8_t nav_state = 0; nav_state < vehicle_status_s::NAVIGATION_STATE_MAX; ++nav_state) {
		reporter.reset();
		reporter.armingCheckFailure(NavModes::All, health_component_t::remote_control,
					    events::ID("arming_test_basic_fail_all_modes_fail1"), events::Log::Info, "");
		reporter.finalize();
		reporter.report(false);

		ASSERT_FALSE(reporter.canArm(nav_state));
		ASSERT_TRUE(reporter.canRun(nav_state));
	}
}

TEST_F(ReporterTest, arming_checks_mode_category)
{
	failsafe_flags_s failsafe_flags{};
	Report reporter{failsafe_flags, 0_s};

	// arming must still be possible for non-relevant failures
	reporter.reset();
	reporter.armingCheckFailure(NavModes::PositionControl | NavModes::Stabilized, health_component_t::remote_control,
				    events::ID("arming_test_arming_checks_mode_category_fail1"), events::Log::Warning, "");
	reporter.clearCanRunBits(NavModes::PositionControl | NavModes::Stabilized);
	reporter.healthFailure(NavModes::PositionControl, health_component_t::local_position_estimate,
			       events::ID("arming_test_arming_checks_mode_category_fail2"), events::Log::Info, "");
	reporter.setIsPresent(health_component_t::battery);
	reporter.finalize();
	reporter.report(false);

	ASSERT_TRUE(reporter.canArm(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION));
	ASSERT_TRUE(reporter.canRun(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION));
	ASSERT_FALSE(reporter.canRun(vehicle_status_s::NAVIGATION_STATE_POSCTL));

	ASSERT_EQ((uint8_t)reporter.armingCheckResults().can_arm, (uint8_t)~(NavModes::PositionControl | NavModes::Stabilized));
	ASSERT_EQ((uint64_t)reporter.armingCheckResults().error, 0);
	ASSERT_EQ(reporter.armingCheckResults().warning, events::px4::enums::health_component_t::remote_control);

	ASSERT_EQ(reporter.healthResults().is_present, events::px4::enums::health_component_t::battery);
	ASSERT_EQ((uint64_t)reporter.healthResults().error, 0);
	ASSERT_EQ((uint64_t)reporter.healthResults().warning, 0);
}

TEST_F(ReporterTest, arming_checks_mode_category2)
{
	failsafe_flags_s failsafe_flags{};
	Report reporter{failsafe_flags, 0_s};

	// A matching mode category must deny arming
	reporter.reset();
	reporter.healthFailure(NavModes::Mission, health_component_t::remote_control,
			       events::ID("arming_test_arming_checks_mode_category2_fail1"), events::Log::Warning, "");
	reporter.finalize();
	reporter.report(false);

	ASSERT_FALSE(reporter.canArm(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION));

	ASSERT_EQ((uint8_t)reporter.armingCheckResults().can_arm, (uint8_t)~(NavModes::Mission));
	ASSERT_EQ((uint64_t)reporter.armingCheckResults().error, 0);
	ASSERT_EQ((uint64_t)reporter.armingCheckResults().warning, 0);

	ASSERT_EQ((uint64_t)reporter.healthResults().is_present, 0);
	ASSERT_EQ((uint64_t)reporter.healthResults().error, 0);
	ASSERT_EQ(reporter.healthResults().warning, events::px4::enums::health_component_t::remote_control);
}

TEST_F(ReporterTest, reporting)
{
	failsafe_flags_s failsafe_flags{};
	Report reporter{failsafe_flags, 0_s};

	uORB::Subscription event_sub{ORB_ID(event)};
	event_sub.subscribe();
	event_s event;

	while (event_sub.update(&event)); // clear all updates

	for (int j = 0; j < 2; ++j) { // test with and without additional report arguments
		const bool with_arg = j == 0;

		for (int i = 0; i < 3; ++i) { // repeat same report: we expect reporting only the first time
			reporter.reset();

			if (with_arg) {
				reporter.armingCheckFailure<uint16_t>(NavModes::All, health_component_t::remote_control,
								      events::ID("arming_test_reporting_fail1"), events::Log::Warning, "", 4938);

			} else {
				reporter.armingCheckFailure(NavModes::PositionControl, health_component_t::remote_control,
							    events::ID("arming_test_reporting_fail2"), events::Log::Warning, "");
			}

			reporter.finalize();
			reporter.report(false);
			ASSERT_FALSE(reporter.canArm(vehicle_status_s::NAVIGATION_STATE_POSCTL));

			if (i == 0) {
				ASSERT_TRUE(event_sub.update(&event));
				ASSERT_EQ(event.id, events::ID("commander_arming_check_summary"));
				ASSERT_TRUE(event_sub.update(&event));

				if (with_arg) {
					ASSERT_EQ(event.id, events::ID("arming_test_reporting_fail1"));

				} else {
					ASSERT_EQ(event.id, events::ID("arming_test_reporting_fail2"));
				}

				ASSERT_TRUE(event_sub.update(&event));
				ASSERT_EQ(event.id, events::ID("commander_health_summary"));

			} else {
				ASSERT_FALSE(event_sub.updated());
			}
		}
	}

	// now the same for health
	for (int j = 0; j < 2; ++j) {
		const bool with_arg = j == 0;

		for (int i = 0; i < 3; ++i) {
			reporter.reset();

			if (with_arg) {
				reporter.healthFailure<uint16_t>(NavModes::All, health_component_t::remote_control,
								 events::ID("arming_test_reporting_fail3"), events::Log::Warning, "", 4938);

			} else {
				reporter.healthFailure(NavModes::PositionControl, health_component_t::remote_control,
						       events::ID("arming_test_reporting_fail4"), events::Log::Warning, "");
			}

			reporter.finalize();
			reporter.report(false);
			ASSERT_FALSE(reporter.canArm(vehicle_status_s::NAVIGATION_STATE_POSCTL));

			if (i == 0) {
				ASSERT_TRUE(event_sub.update(&event));
				ASSERT_EQ(event.id, events::ID("commander_arming_check_summary"));
				ASSERT_TRUE(event_sub.update(&event));

				if (with_arg) {
					ASSERT_EQ(event.id, events::ID("arming_test_reporting_fail3"));

				} else {
					ASSERT_EQ(event.id, events::ID("arming_test_reporting_fail4"));
				}

				ASSERT_TRUE(event_sub.update(&event));
				ASSERT_EQ(event.id, events::ID("commander_health_summary"));

			} else {
				ASSERT_FALSE(event_sub.updated());
			}
		}
	}

}

TEST_F(ReporterTest, reporting_multiple)
{
	failsafe_flags_s failsafe_flags{};
	Report reporter{failsafe_flags, 0_s};

	uORB::Subscription event_sub{ORB_ID(event)};
	event_sub.subscribe();
	event_s event;

	while (event_sub.update(&event)); // clear all updates

	for (int i = 0; i < 3; ++i) {
		reporter.reset();
		reporter.armingCheckFailure<uint16_t>(NavModes::All, health_component_t::remote_control,
						      events::ID("arming_test_reporting_multiple_fail1"), events::Log::Warning, "", 4938);
		reporter.armingCheckFailure<float>(NavModes::All, health_component_t::remote_control,
						   events::ID("arming_test_reporting_multiple_fail2"), events::Log::Warning, "", 123.f);
		reporter.armingCheckFailure<uint8_t>(NavModes::All, health_component_t::remote_control,
						     events::ID("arming_test_reporting_multiple_fail3"), events::Log::Warning, "", 55);
		reporter.finalize();
		reporter.report(false);
		ASSERT_FALSE(reporter.canArm(vehicle_status_s::NAVIGATION_STATE_POSCTL));

		if (i == 0) {
			ASSERT_TRUE(event_sub.update(&event));
			ASSERT_EQ(event.id, events::ID("commander_arming_check_summary"));
			ASSERT_TRUE(event_sub.update(&event));
			ASSERT_EQ(event.id, events::ID("arming_test_reporting_multiple_fail1"));
			ASSERT_TRUE(event_sub.update(&event));
			ASSERT_EQ(event.id, events::ID("arming_test_reporting_multiple_fail2"));
			ASSERT_TRUE(event_sub.update(&event));
			ASSERT_EQ(event.id, events::ID("arming_test_reporting_multiple_fail3"));
			ASSERT_TRUE(event_sub.update(&event));
			ASSERT_EQ(event.id, events::ID("commander_health_summary"));

		} else {
			ASSERT_FALSE(event_sub.updated());
		}
	}
}
