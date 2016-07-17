#include <systemlib/hysteresis/hysteresis.h>

#include "gtest/gtest.h"


TEST(HysteresisTest, InitFalse)
{
	systemlib::Hysteresis hysteresis(false);
	ASSERT_FALSE(hysteresis.get_state());
}

TEST(HysteresisTest, InitTrue)
{
	systemlib::Hysteresis hysteresis(true);
	ASSERT_TRUE(hysteresis.get_state());
}

TEST(HysteresisTest, ZeroCase)
{
	// Default is 0 hysteresis.
	systemlib::Hysteresis hysteresis(false);
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
	systemlib::Hysteresis hysteresis(false);
	hysteresis.set_hysteresis_time_from(false, 5000);
	hysteresis.set_hysteresis_time_from(true, 3000);

	// Change to true.
	hysteresis.set_state_and_update(true);
	ASSERT_FALSE(hysteresis.get_state());
	usleep(4000);
	hysteresis.update();
	ASSERT_FALSE(hysteresis.get_state());
	usleep(2000);
	hysteresis.update();
	ASSERT_TRUE(hysteresis.get_state());

	// Change back to false.
	hysteresis.set_state_and_update(false);
	ASSERT_TRUE(hysteresis.get_state());
	usleep(1000);
	hysteresis.update();
	ASSERT_TRUE(hysteresis.get_state());
	usleep(3000);
	hysteresis.update();
	ASSERT_FALSE(hysteresis.get_state());
}

TEST(HysteresisTest, HysteresisChanged)
{
	systemlib::Hysteresis hysteresis(false);
	hysteresis.set_hysteresis_time_from(true, 2000);
	hysteresis.set_hysteresis_time_from(false, 5000);

	// Change to true.
	hysteresis.set_state_and_update(true);
	ASSERT_FALSE(hysteresis.get_state());
	usleep(3000);
	hysteresis.update();
	ASSERT_FALSE(hysteresis.get_state());
	usleep(3000);
	hysteresis.update();
	ASSERT_TRUE(hysteresis.get_state());

	// Change hysteresis time.
	hysteresis.set_hysteresis_time_from(true, 10000);

	// Change back to false.
	hysteresis.set_state_and_update(false);
	ASSERT_TRUE(hysteresis.get_state());
	usleep(7000);
	hysteresis.update();
	ASSERT_TRUE(hysteresis.get_state());
	usleep(5000);
	hysteresis.update();
	ASSERT_FALSE(hysteresis.get_state());
}

TEST(HysteresisTest, ChangeAfterTimeMultipleSets)
{
	systemlib::Hysteresis hysteresis(false);
	hysteresis.set_hysteresis_time_from(true, 5000);
	hysteresis.set_hysteresis_time_from(false, 5000);

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
	systemlib::Hysteresis hysteresis(false);
	hysteresis.set_hysteresis_time_from(false, 5000);

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

	// The other directory is immediate.
	hysteresis.set_state_and_update(false);
	ASSERT_FALSE(hysteresis.get_state());
}
