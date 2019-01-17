#pragma once

#include <cstdint>
#include <mutex>
#include <vector>
#include <memory>
#include <atomic>
#include <pthread.h>

class LockstepScheduler
{
public:
	~LockstepScheduler();

	void set_absolute_time(uint64_t time_us);
	inline uint64_t get_absolute_time() const { return _time_us; }
	int cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *lock, uint64_t time_us);
	int usleep_until(uint64_t timed_us);

private:
	struct TimedWait {
		~TimedWait()
		{
			// If a thread quickly exits after a cond_timedwait(), the
			// thread_local object can still be in the linked list. In that case
			// we need to wait until it's removed.
			while (!removed) {
#ifndef UNIT_TESTS // unit tests don't define system_usleep and execute faster w/o sleeping here
				system_sleep(5000);
#endif
			}
		}

		pthread_cond_t *passed_cond{nullptr};
		pthread_mutex_t *passed_lock{nullptr};
		uint64_t time_us{0};
		bool timeout{false};
		std::atomic<bool> done{false};
		std::atomic<bool> removed{true};

		TimedWait *next{nullptr}; ///< linked list
	};

	std::atomic<uint64_t> _time_us{0};

	TimedWait *_timed_waits{nullptr}; ///< head of linked list
	std::mutex _timed_waits_mutex;
	std::atomic<bool> _setting_time{false}; ///< true if set_absolute_time() is currently being executed
};
