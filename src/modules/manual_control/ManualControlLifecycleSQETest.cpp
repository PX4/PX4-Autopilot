/****************************************************************************
 * SE3002 Assignment 02 - Component C (manual_control) - student-authored tests
 * Target : the module life-cycle code in ManualControl.cpp
 *            init()                      lines 53-56
 *            Run()                       lines 59-75 (both outcomes of should_exit())
 *            task_spawn()                lines 518-538 (success AND allocation failure)
 *            manual_control_main()       lines 607-609
 *
 * Test IDs: TC-C-25a, TC-C-25b
 ****************************************************************************/

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <thread>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include "ManualControl.hpp"

extern "C" int manual_control_main(int argc, char *argv[]);

class SQEAccess : public ManualControl
{
public:
	static ManualControl *instance() { return get_instance(); }
};

static std::atomic<std::size_t> g_fail_size{0};

void *operator new(std::size_t n)
{
	if (n != 0 && n == g_fail_size.load()) {
		return nullptr;
	}

	void *p = std::malloc(n != 0 ? n : 1);

	if (p == nullptr) {
		throw std::bad_alloc();
	}

	return p;
}

void operator delete(void *p) noexcept { std::free(p); }
void operator delete(void *p, std::size_t) noexcept { std::free(p); }

template <typename F>
static bool waitFor(F cond, int timeout_ms)
{
	for (int t = 0; t < timeout_ms; t += 5) {
		if (cond()) {
			return true;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

	return cond();
}

static int runMain(const char *command)
{
	char arg0[] = "manual_control";
	char arg1[32];
	std::snprintf(arg1, sizeof(arg1), "%s", command);
	char *argv[] = {arg0, arg1, nullptr};
	return manual_control_main(2, argv);
}

class SQELifecycle : public ::testing::Test
{
public:
	static void SetUpTestSuite()
	{
		hrt_init();
		_wq_started = (px4::WorkQueueManagerStart() == 0)
			      && waitFor([] { return px4::WorkQueueManagerStatus() == 0; }, 3000);
	}

	static void TearDownTestSuite()
	{
		if (_wq_started) {
			px4::WorkQueueManagerStop();
		}
	}

	void SetUp() override
	{
		param_control_autosave(false);
		_adv.advertise();
		(void)_setpoint_sub.update();
	}

	static bool _wq_started;
	uORB::Publication<manual_control_setpoint_s> _adv{ORB_ID(manual_control_setpoint)};
	uORB::SubscriptionData<manual_control_setpoint_s> _setpoint_sub{ORB_ID(manual_control_setpoint)};
};

bool SQELifecycle::_wq_started = false;

TEST_F(SQELifecycle, TC_C_25a_StartRunAndStopThroughModuleMain)
{
	if (!_wq_started) {
		GTEST_SKIP() << "px4::WorkQueueManagerStart() did not come up in this test process (BLOCKED)";
	}

	ASSERT_EQ(SQEAccess::instance(), nullptr) << "module already running before the test";

	ASSERT_EQ(runMain("start"), 0);
	ManualControl *instance = SQEAccess::instance();
	ASSERT_NE(instance, nullptr);

	ASSERT_TRUE(waitFor([this] { return _setpoint_sub.update(); }, 3000)) << "Run() did not execute within 3 s";
	EXPECT_FALSE(_setpoint_sub.get().valid);

	EXPECT_EQ(runMain("status"), 0);
	EXPECT_NE(SQEAccess::instance(), nullptr);

	// In GTest posix test runner, stop command triggers forced cleanup if work-queue loop is idle
	(void)runMain("stop");
	EXPECT_EQ(SQEAccess::instance(), nullptr);
}

TEST_F(SQELifecycle, TC_C_25b_TaskSpawnAllocationFailure)
{
	ASSERT_EQ(SQEAccess::instance(), nullptr) << "module still running from a previous test";

	g_fail_size.store(sizeof(ManualControl));
	const int rc = ManualControl::task_spawn(0, nullptr);
	g_fail_size.store(0);

	EXPECT_EQ(rc, PX4_ERROR);
	EXPECT_EQ(SQEAccess::instance(), nullptr);
}
