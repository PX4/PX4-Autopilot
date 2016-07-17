#include <systemlib/hysteresis/hysteresis.h>

#include "gtest/gtest.h"

const static unsigned hysteresis_time_us = 5000;

TEST(HysteresisTest, InitFalse)
{
	systemlib::Hysteresis hysteresis(hysteresis_time_us, false);
	ASSERT_FALSE(hysteresis.get_state());
}

TEST(HysteresisTest, InitTrue)
{
	systemlib::Hysteresis hysteresis(hysteresis_time_us, true);
	ASSERT_TRUE(hysteresis.get_state());
}

TEST(HysteresisTest, ZeroCase)
{
	systemlib::Hysteresis hysteresis(0, false);
	ASSERT_FALSE(hysteresis.get_state());

	// Change and see result immediately.
	hysteresis.set_state_and_update(true);
	ASSERT_TRUE(hysteresis.get_state());
	hysteresis.set_state_and_update(false);
	ASSERT_FALSE(hysteresis.get_state());
	hysteresis.set_state_and_update(true);
	ASSERT_TRUE(hysteresis.get_state());

	// A wait won't change anything.
	usleep(1000);
	hysteresis.update();
	ASSERT_TRUE(hysteresis.get_state());
}

TEST(HysteresisTest, ChangeAfterTime)
{
	systemlib::Hysteresis hysteresis(hysteresis_time_us, false);

	// Change to true.
	hysteresis.set_state_and_update(true);
	ASSERT_FALSE(hysteresis.get_state());
	usleep(3000);
	hysteresis.update();
	ASSERT_FALSE(hysteresis.get_state());
	usleep(3000);
	hysteresis.update();
	ASSERT_TRUE(hysteresis.get_state());

	// Change back to false.
	hysteresis.set_state_and_update(false);
	ASSERT_TRUE(hysteresis.get_state());
	usleep(3000);
	hysteresis.update();
	ASSERT_TRUE(hysteresis.get_state());
	usleep(3000);
	hysteresis.update();
	ASSERT_FALSE(hysteresis.get_state());
}

TEST(HysteresisTest, ChangeAfterTimeMultipleSets)
{
	systemlib::Hysteresis hysteresis(hysteresis_time_us, false);

	// Change to true.
	hysteresis.set_state_and_update(true);
	ASSERT_FALSE(hysteresis.get_state());
	usleep(3000);
	hysteresis.set_state_and_update(true);
	ASSERT_FALSE(hysteresis.get_state());
	usleep(3000);
	hysteresis.set_state_and_update(true);
	ASSERT_TRUE(hysteresis.get_state());

	// Change to false.
	hysteresis.set_state_and_update(false);
	ASSERT_TRUE(hysteresis.get_state());
	usleep(3000);
	hysteresis.set_state_and_update(false);
	ASSERT_TRUE(hysteresis.get_state());
	usleep(3000);
	hysteresis.set_state_and_update(false);
	ASSERT_FALSE(hysteresis.get_state());
}

TEST(HysteresisTest, TakeChangeBack)
{
	systemlib::Hysteresis hysteresis(hysteresis_time_us, false);

	// Change to true.
	hysteresis.set_state_and_update(true);
	ASSERT_FALSE(hysteresis.get_state());
	usleep(3000);
	hysteresis.update();
	ASSERT_FALSE(hysteresis.get_state());
	// Change your mind to false.
	hysteresis.set_state_and_update(false);
	ASSERT_FALSE(hysteresis.get_state());
	usleep(6000);
	hysteresis.update();
	ASSERT_FALSE(hysteresis.get_state());

	// And true again
	hysteresis.set_state_and_update(true);
	ASSERT_FALSE(hysteresis.get_state());
	usleep(3000);
	hysteresis.update();
	ASSERT_FALSE(hysteresis.get_state());
	usleep(3000);
	hysteresis.update();
	ASSERT_TRUE(hysteresis.get_state());
}
