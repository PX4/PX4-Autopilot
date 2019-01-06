#include "lockstep_scheduler/lockstep_scheduler.h"


void LockstepScheduler::set_absolute_time(uint64_t time_us)
{
	time_us_ = time_us;

	{
		std::unique_lock<std::mutex> lock_timed_waits(timed_waits_mutex_);
		setting_time_ = true;

		auto it = std::begin(timed_waits_);

		while (it != std::end(timed_waits_)) {

			std::shared_ptr<TimedWait> temp_timed_wait = *it;

			// Clean up the ones that are already done from last iteration.
			if (temp_timed_wait->done) {
				it = timed_waits_.erase(it);
				continue;
			}

			if (temp_timed_wait->time_us <= time_us &&
			    !temp_timed_wait->timeout) {
				// We are abusing the condition here to signal that the time
				// has passed.
				pthread_mutex_lock(temp_timed_wait->passed_lock);
				temp_timed_wait->timeout = true;
				pthread_cond_broadcast(temp_timed_wait->passed_cond);
				pthread_mutex_unlock(temp_timed_wait->passed_lock);
			}

			++it;
		}

		setting_time_ = false;
	}
}

int LockstepScheduler::cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *lock, uint64_t time_us)
{
	std::shared_ptr<TimedWait> new_timed_wait;
	{
		std::lock_guard<std::mutex> lock_timed_waits(timed_waits_mutex_);

		// The time has already passed.
		if (time_us <= time_us_) {
			return ETIMEDOUT;
		}

		new_timed_wait = std::make_shared<TimedWait>();
		new_timed_wait->time_us = time_us;
		new_timed_wait->passed_cond = cond;
		new_timed_wait->passed_lock = lock;
		timed_waits_.push_back(new_timed_wait);
	}

	int result = pthread_cond_wait(cond, lock);

	const bool timeout = new_timed_wait->timeout;

	if (result == 0 && timeout) {
		result = ETIMEDOUT;
	}

	new_timed_wait->done = true;

	if (!timeout && setting_time_) {
		// This is where it gets tricky: the timeout has not been triggered yet,
		// and another thread is in set_absolute_time().
		// If it already passed the 'done' check, it will access the mutex and
		// the condition variable next. However they might be invalid as soon as we
		// return here, so we wait until set_absolute_time() is done.
		// In addition we have to unlock 'lock', otherwise we risk a
		// deadlock due to a different locking order in set_absolute_time().
		// Note that this case does not happen too frequently, and thus can be
		// a bit more expensive.
		pthread_mutex_unlock(lock);
		timed_waits_mutex_.lock();
		timed_waits_mutex_.unlock();
		pthread_mutex_lock(lock);
	}

	return result;
}

int LockstepScheduler::usleep_until(uint64_t time_us)
{
	pthread_mutex_t lock;
	pthread_mutex_init(&lock, nullptr);
	pthread_cond_t cond;
	pthread_cond_init(&cond, nullptr);

	pthread_mutex_lock(&lock);

	int result = cond_timedwait(&cond, &lock, time_us);

	if (result == ETIMEDOUT) {
		// This is expected because we never notified to the condition.
		result = 0;
	}

	pthread_mutex_unlock(&lock);

	pthread_cond_destroy(&cond);
	pthread_mutex_destroy(&lock);

	return result;
}
