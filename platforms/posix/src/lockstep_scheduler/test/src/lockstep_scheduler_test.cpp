#include <lockstep_scheduler/lockstep_scheduler.h>
#include <cassert>
#include <thread>
#include <atomic>
#include <random>
#include <iostream>


constexpr uint64_t some_time_us = 12345678;

void test_absolute_time()
{
	LockstepScheduler ls;
	ls.set_absolute_time(some_time_us);
	assert(ls.get_absolute_time() == some_time_us);
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
	std::thread thread([&ls, &cond, &lock, &should_have_timed_out]() {
		assert(ls.cond_timedwait(&cond, &lock, some_time_us + 1000) == -1);
		assert(errno == ETIMEDOUT);
		assert(should_have_timed_out);
		// It should be re-locked afterwards, so we should be able to unlock it.
		assert(pthread_mutex_unlock(&lock) == 0);
	});

	ls.set_absolute_time(some_time_us + 500);
	should_have_timed_out = true;
	ls.set_absolute_time(some_time_us + 1500);

	thread.join();

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
	std::thread thread([&ls, &cond, &lock]() {

		ls.set_absolute_time(some_time_us + 500);
		assert(ls.cond_timedwait(&cond, &lock, some_time_us + 1000) == 0);
		// It should be re-locked afterwards, so we should be able to unlock it.
		assert(pthread_mutex_unlock(&lock) == 0);
	});

	pthread_mutex_lock(&lock);
	pthread_cond_broadcast(&cond);
	pthread_mutex_unlock(&lock);

	thread.join();

	pthread_mutex_destroy(&lock);
	pthread_cond_destroy(&cond);
}

class TestCase
{
public:
	TestCase(unsigned timeout, unsigned unlocked_after, LockstepScheduler &ls) :
		timeout_(timeout + some_time_us),
		unlocked_after_(unlocked_after + some_time_us),
		ls_(ls)
	{
		pthread_mutex_init(&lock_, NULL);
		pthread_cond_init(&cond_, NULL);
	}

	~TestCase()
	{
		assert(is_done_);
		pthread_mutex_destroy(&lock_);
		pthread_cond_destroy(&cond_);
	}

	void run()
	{
		pthread_mutex_lock(&lock_);
		thread_ = std::make_shared<std::thread>([this]() {
			result_ = ls_.cond_timedwait(&cond_, &lock_, timeout_);
			pthread_mutex_unlock(&lock_);
		});
	}

	void check()
	{
		if (is_done_) {
			return;
		}

		uint64_t time_us = ls_.get_absolute_time();

		const bool unlock_reached = (time_us >= unlocked_after_);
		const bool unlock_is_before_timeout = (unlocked_after_ <= timeout_);
		const bool timeout_reached = (time_us >= timeout_);

		if (unlock_reached && unlock_is_before_timeout && !(timeout_reached)) {
			pthread_mutex_lock(&lock_);
			pthread_cond_broadcast(&cond_);
			pthread_mutex_unlock(&lock_);
			is_done_ = true;
			// We can be sure that this triggers.
			thread_->join();
			assert(result_ == 0);
		}

		else if (timeout_reached) {
			is_done_ = true;
			thread_->join();
			assert(result_ == -1);
		}
	}
private:
	static constexpr int INITIAL_RESULT = 42;

	unsigned timeout_;
	unsigned unlocked_after_;
	pthread_cond_t cond_;
	pthread_mutex_t lock_;
	LockstepScheduler &ls_;
	std::atomic<bool> is_done_{false};
	std::atomic<int> result_ {INITIAL_RESULT};
	std::shared_ptr<std::thread> thread_{};
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

	std::thread thread([&step, &ls]() {
		step = Step::ThreadStarted;

		WAIT_FOR(step == Step::BeforeUsleep);

		step = Step::UsleepNotTriggeredYet;
		ls.set_absolute_time(some_time_us + 500);

		step = Step::UsleepTriggered;
		ls.set_absolute_time(some_time_us + 1500);
	});

	WAIT_FOR(step == Step::ThreadStarted);

	step = Step::BeforeUsleep;

	assert(ls.usleep_until(some_time_us + 1000) == 0);
	assert(step == Step::UsleepTriggered);
	thread.join();
}

int main(int /*argc*/, char ** /*argv*/)
{
	for (unsigned iteration = 1; iteration <= 10000; ++iteration) {
		std::cout << "Test iteration: " << iteration << "\n";
		test_absolute_time();
		test_condition_timing_out();
		test_locked_semaphore_getting_unlocked();
		test_usleep();
		test_multiple_semaphores_waiting();
	}

	return 0;
}
