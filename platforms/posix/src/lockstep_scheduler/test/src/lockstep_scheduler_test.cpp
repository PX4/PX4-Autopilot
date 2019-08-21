#include <lockstep_scheduler/lockstep_scheduler.h>
#include <gtest/gtest.h>
#include <thread>
#include <atomic>
#include <random>
#include <iostream>
#include <functional>
#include <chrono>

class TestThread
{
public:
	TestThread(const std::function<void()> &f)
		: _f(f)
	{
		_thread = std::thread(std::bind(&TestThread::execute, this));
	}

	void join(LockstepScheduler &ls)
	{
		// The unit-tests do not reflect the real usage, where
		// set_absolute_time() is called regularly and can do some
		// cleanup tasks. We simulate that here by waiting until
		// the given task returns (which is expected to happen quickly)
		// and then call set_absolute_time(), which can do the cleanup,
		// and _thread can then exit as well.
		while (!_done) {
			std::this_thread::yield(); // usleep is too slow here
		}

		ls.set_absolute_time(ls.get_absolute_time());
		_thread.join();
	}
private:
	void execute()
	{
		_f();
		_done = true;
	}
	std::function<void()> _f;
	std::atomic<bool> _done{false};
	std::thread _thread;
};

constexpr uint64_t some_time_us = 12345678;

void test_absolute_time()
{
	LockstepScheduler ls;
	ls.set_absolute_time(some_time_us);
	EXPECT_EQ(ls.get_absolute_time(),  some_time_us);
}

void test_condition_timing_out()
{
	// Create locked condition.
	pthread_cond_t cond;
	pthread_cond_init(&cond, NULL);

	// And a lock which needs to be locked
	pthread_mutex_t lock;
	pthread_mutex_init(&lock, NULL);

	LockstepScheduler ls;
	ls.set_absolute_time(some_time_us);

	std::atomic<bool> should_have_timed_out{false};
	pthread_mutex_lock(&lock);

	// Use a thread to wait for condition while we already have the lock.
	// This ensures the synchronization happens in the right order.
	TestThread thread([&ls, &cond, &lock, &should_have_timed_out]() {
		EXPECT_EQ(ls.cond_timedwait(&cond, &lock, some_time_us + 1000), ETIMEDOUT);
		EXPECT_TRUE(should_have_timed_out);
		// It should be re-locked afterwards, so we should be able to unlock it.
		EXPECT_EQ(pthread_mutex_unlock(&lock), 0);
	});

	ls.set_absolute_time(some_time_us + 500);
	should_have_timed_out = true;
	ls.set_absolute_time(some_time_us + 1500);

	thread.join(ls);

	pthread_mutex_destroy(&lock);
	pthread_cond_destroy(&cond);

}

void test_locked_semaphore_getting_unlocked()
{
	// Create locked condition.
	pthread_cond_t cond;
	pthread_cond_init(&cond, NULL);

	// And a lock which needs to be locked
	pthread_mutex_t lock;
	pthread_mutex_init(&lock, NULL);

	LockstepScheduler ls;
	ls.set_absolute_time(some_time_us);

	pthread_mutex_lock(&lock);
	// Use a thread to wait for condition while we already have the lock.
	// This ensures the synchronization happens in the right order.
	TestThread thread([&ls, &cond, &lock]() {

		ls.set_absolute_time(some_time_us + 500);
		EXPECT_EQ(ls.cond_timedwait(&cond, &lock, some_time_us + 1000), 0);
		// It should be re-locked afterwards, so we should be able to unlock it.
		EXPECT_EQ(pthread_mutex_unlock(&lock), 0);
	});

	pthread_mutex_lock(&lock);
	pthread_cond_broadcast(&cond);
	pthread_mutex_unlock(&lock);

	thread.join(ls);

	pthread_mutex_destroy(&lock);
	pthread_cond_destroy(&cond);
}

class TestCase
{
public:
	TestCase(unsigned timeout, unsigned unlocked_after, LockstepScheduler &ls) :
		_timeout(timeout + some_time_us),
		_unlocked_after(unlocked_after + some_time_us),
		_ls(ls)
	{
		pthread_mutex_init(&_lock, NULL);
		pthread_cond_init(&_cond, NULL);
	}

	~TestCase()
	{
		EXPECT_TRUE(_is_done);
		pthread_mutex_destroy(&_lock);
		pthread_cond_destroy(&_cond);
	}

	void run()
	{
		pthread_mutex_lock(&_lock);
		_thread = std::make_shared<TestThread>([this]() {
			_result = _ls.cond_timedwait(&_cond, &_lock, _timeout);
			pthread_mutex_unlock(&_lock);
		});
	}

	void check()
	{
		if (_is_done) {
			return;
		}

		uint64_t time_us = _ls.get_absolute_time();

		const bool unlock_reached = (time_us >= _unlocked_after);
		const bool unlock_is_before_timeout = (_unlocked_after <= _timeout);
		const bool timeout_reached = (time_us >= _timeout);

		if (unlock_reached && unlock_is_before_timeout && !(timeout_reached)) {
			pthread_mutex_lock(&_lock);
			pthread_cond_broadcast(&_cond);
			pthread_mutex_unlock(&_lock);
			_is_done = true;
			// We can be sure that this triggers.
			_thread->join(_ls);
			EXPECT_EQ(_result, 0);
		}

		else if (timeout_reached) {
			_is_done = true;
			_thread->join(_ls);
			EXPECT_EQ(_result, ETIMEDOUT);
		}
	}
private:
	static constexpr int INITIAL_RESULT = 42;

	unsigned _timeout;
	unsigned _unlocked_after;
	pthread_cond_t _cond;
	pthread_mutex_t _lock;
	LockstepScheduler &_ls;
	std::atomic<bool> _is_done{false};
	std::atomic<int> _result {INITIAL_RESULT};
	std::shared_ptr<TestThread> _thread{};
};

int random_number(int min, int max)
{
	// We want predictable test results, so we always
	// start with the seed 0.
	static int iteration = 0;

	std::seed_seq seed{iteration++};
	std::default_random_engine engine{seed};
	std::uniform_int_distribution<> distribution(min, max);

	const int random_number = distribution(engine);
	return random_number;
}

void test_multiple_semaphores_waiting()
{

	LockstepScheduler ls;
	ls.set_absolute_time(some_time_us);

	// Use different timeouts in random order.
	std::vector<std::shared_ptr<TestCase>> test_cases{};

	test_cases.push_back(
		std::make_shared<TestCase>(
			11111, 11111, ls));

	test_cases.push_back(
		std::make_shared<TestCase>(
			20000, 20000, ls));

	test_cases.push_back(
		std::make_shared<TestCase>(
			0, 20000, ls));

	test_cases.push_back(
		std::make_shared<TestCase>(
			20000, 10000, ls));

	test_cases.push_back(
		std::make_shared<TestCase>(
			0, 0, ls));

	const int num_additional_threads = random_number(1, 20);

	for (int i = 0; i < num_additional_threads; ++i) {
		const unsigned timeout = random_number(1, 20000);
		const unsigned unlocked_after = random_number(1, 20000);
		test_cases.push_back(
			std::make_shared<TestCase>(
				timeout, unlocked_after, ls));
	}

	for (auto &test_case : test_cases) {
		test_case->run();
	}

	const int min_step_size = 1;
	const int max_step_size = 100;

	// We need to go until the max plus max step size to make sure we trigger
	// all timeouts or semaphores.
	for (unsigned time_us = 1;
	     time_us <= (20000 + max_step_size);
	     time_us += random_number(min_step_size, max_step_size)) {

		ls.set_absolute_time(some_time_us + time_us);

		for (auto &test_case : test_cases) {
			test_case->check();
		}
	}

	test_cases.clear();
}

#define WAIT_FOR(condition_) \
	while (!(condition_)) { \
		std::this_thread::yield(); \
	}

void test_usleep()
{
	LockstepScheduler ls;
	ls.set_absolute_time(some_time_us);

	enum class Step {
		Init,
		ThreadStarted,
		BeforeUsleep,
		UsleepNotTriggeredYet,
		UsleepTriggered
	};

	std::atomic<Step> step{Step::Init};

	TestThread thread([&step, &ls]() {
		step = Step::ThreadStarted;

		WAIT_FOR(step == Step::BeforeUsleep);

		step = Step::UsleepNotTriggeredYet;
		ls.set_absolute_time(some_time_us + 500);

		step = Step::UsleepTriggered;
		ls.set_absolute_time(some_time_us + 1500);
	});

	WAIT_FOR(step == Step::ThreadStarted);

	step = Step::BeforeUsleep;

	EXPECT_EQ(ls.usleep_until(some_time_us + 1000), 0);
	EXPECT_EQ(step, Step::UsleepTriggered);
	thread.join(ls);
}

TEST(LockstepScheduler, All)
{
	for (unsigned iteration = 1; iteration <= 100; ++iteration) {
		//std::cout << "Test iteration: " << iteration << "\n";
		test_absolute_time();
		test_condition_timing_out();
		test_locked_semaphore_getting_unlocked();
		test_usleep();
		test_multiple_semaphores_waiting();
	}
}
