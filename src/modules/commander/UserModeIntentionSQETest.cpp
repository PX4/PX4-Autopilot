/****************************************************************************
 * SE3002 Assignment 02 - Scope Expansion (UserModeIntention)
 * Target : UserModeIntention.cpp
 * Goal   : Maximum practically achievable statement and decision/branch coverage
 ****************************************************************************/

#include <gtest/gtest.h>
#include "UserModeIntention.hpp"

class TestModeChangeHandler : public ModeChangeHandler
{
public:
	uint8_t replaced_mode{vehicle_status_s::NAVIGATION_STATE_MAX};
	uint8_t disarm_return_mode{vehicle_status_s::NAVIGATION_STATE_MANUAL};

	uint8_t last_notified_state{0};
	ModeChangeSource last_notified_source{ModeChangeSource::User};
	int notification_count{0};

	void onUserIntendedNavStateChange(ModeChangeSource source, uint8_t user_intended_nav_state) override
	{
		last_notified_source = source;
		last_notified_state = user_intended_nav_state;
		notification_count++;
	}

	uint8_t getReplacedModeIfAny(uint8_t nav_state) override
	{
		if (replaced_mode != vehicle_status_s::NAVIGATION_STATE_MAX) {
			return replaced_mode;
		}

		return nav_state;
	}

	uint8_t onDisarm(uint8_t stored_nav_state) override
	{
		return disarm_return_mode;
	}
};

TEST(UserModeIntentionSQETest, InitialState)
{
	vehicle_status_s status{};
	status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	HealthAndArmingChecks checks(nullptr, status);

	UserModeIntention intention(status, checks, nullptr);

	EXPECT_EQ(intention.get(), vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
	EXPECT_FALSE(intention.everHadModeChange());
}

TEST(UserModeIntentionSQETest, ChangeDisarmedWithoutHandler)
{
	vehicle_status_s status{};
	status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	HealthAndArmingChecks checks(nullptr, status);

	UserModeIntention intention(status, checks, nullptr);

	EXPECT_TRUE(intention.change(vehicle_status_s::NAVIGATION_STATE_POSCTL));
	EXPECT_EQ(intention.get(), vehicle_status_s::NAVIGATION_STATE_POSCTL);
	EXPECT_TRUE(intention.everHadModeChange());
	EXPECT_TRUE(intention.getHadModeChangeAndClear());
	EXPECT_FALSE(intention.getHadModeChangeAndClear());
}

TEST(UserModeIntentionSQETest, HandlerReplacedModeNotificationAndDisarm)
{
	vehicle_status_s status{};
	status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	HealthAndArmingChecks checks(nullptr, status);
	TestModeChangeHandler handler;
	handler.replaced_mode = vehicle_status_s::NAVIGATION_STATE_ALTCTL;

	UserModeIntention intention(status, checks, &handler);

	EXPECT_TRUE(intention.change(vehicle_status_s::NAVIGATION_STATE_POSCTL, ModeChangeSource::ModeExecutor));
	EXPECT_EQ(intention.get(), vehicle_status_s::NAVIGATION_STATE_ALTCTL);
	EXPECT_EQ(handler.notification_count, 1);
	EXPECT_EQ(handler.last_notified_state, vehicle_status_s::NAVIGATION_STATE_ALTCTL);

	intention.onDisarm();
	EXPECT_EQ(intention.get(), vehicle_status_s::NAVIGATION_STATE_MANUAL);
}

TEST(UserModeIntentionSQETest, TerminationStateRejection)
{
	vehicle_status_s status{};
	status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	status.nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
	HealthAndArmingChecks checks(nullptr, status);

	UserModeIntention intention(status, checks, nullptr);

	EXPECT_FALSE(intention.change(vehicle_status_s::NAVIGATION_STATE_POSCTL));
	EXPECT_NE(intention.get(), vehicle_status_s::NAVIGATION_STATE_POSCTL);
}

TEST(UserModeIntentionSQETest, TakeoffIntendedAndDisarmWithoutHandler)
{
	vehicle_status_s status{};
	status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	HealthAndArmingChecks checks(nullptr, status);

	UserModeIntention intention(status, checks, nullptr);

	EXPECT_TRUE(intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF));
	EXPECT_EQ(intention.get(), vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF);

	EXPECT_TRUE(intention.change(vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF));
	EXPECT_EQ(intention.get(), vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF);

	intention.onDisarm();
	EXPECT_EQ(intention.get(), vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
}

TEST(UserModeIntentionSQETest, ArmedFallbackAndForcedChange)
{
	vehicle_status_s status{};
	status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	HealthAndArmingChecks checks(nullptr, status);

	UserModeIntention intention(status, checks, nullptr);

	// Armed without position/alt checks -> canRun returns false; test allow_fallback path
	(void)intention.change(vehicle_status_s::NAVIGATION_STATE_POSCTL, ModeChangeSource::User, true /*allow_fallback*/, false);

	// Forced change while armed
	EXPECT_TRUE(intention.change(vehicle_status_s::NAVIGATION_STATE_STAB, ModeChangeSource::User, false, true /*force*/));
	EXPECT_EQ(intention.get(), vehicle_status_s::NAVIGATION_STATE_STAB);
}
