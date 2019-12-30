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
			if (!done) {
				// This can only happen when a thread gets canceled (e.g. via pthread_cancel), and since
				// pthread_cond_wait is a cancellation point, the rest of LockstepScheduler::cond_timedwait afterwards
				// might not be executed. Which means the mutex will not be unlocked either, so we unlock to avoid
				// a dead-lock in LockstepScheduler::set_absolute_time().
				// This destructor gets called as part of thread-local storage cleanup.
				// This is really only a work-around for non-proper thread stopping. Note that we also assume,
				// that we can still access the mutex.
				if (passed_lock) {
					pthread_mutex_unlock(passed_lock);
				}

				done = true;
			}

			// If a thread quickly exits after a cond_timedwait(), the
			// thread_local object can still be in the linked list. In that case
			// we need to wait until it's removed.
			while (!removed) {
				system_usleep(5000);
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
