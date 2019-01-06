#include "lockstep_scheduler/lockstep_scheduler.h"


void LockstepScheduler::set_absolute_time(uint64_t time_us)
{
	time_us_ = time_us;

	{
		std::unique_lock<std::mutex> lock_timed_waits(timed_waits_mutex_);

		auto it = std::begin(timed_waits_);

		while (it != std::end(timed_waits_)) {

			std::shared_ptr<TimedWait> temp_timed_wait = *it;

			// Clean up the ones that are already done from last iteration.
			if (temp_timed_wait->done) {
				it = timed_waits_.erase(it);
				continue;
			}

			if (temp_timed_wait->time_us <= time_us &&
			    !temp_timed_wait->timeout &&
			    !temp_timed_wait->done) {
				temp_timed_wait->timeout = true;
				// We are abusing the condition here to signal that the time
				// has passed.
				timed_waits_iterator_invalidated_ = false;
				pthread_mutex_lock(temp_timed_wait->passed_lock);
				pthread_cond_broadcast(temp_timed_wait->passed_cond);
				pthread_mutex_unlock(temp_timed_wait->passed_lock);

				if (timed_waits_iterator_invalidated_) {
					// The vector might have changed, we need to start from the
					// beginning.
					it = std::begin(timed_waits_);
					continue;
				}
			}

			++it;
		}
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
		timed_waits_iterator_invalidated_ = true;
	}

	while (true) {
		int result = pthread_cond_wait(cond, lock);

		// We need to unlock before aqcuiring the timed_waits_mutex, otherwise
		// we are at rist of priority inversion.
		pthread_mutex_unlock(lock);

		{
			std::lock_guard<std::mutex> lock_timed_waits(timed_waits_mutex_);

			if (result == 0 && new_timed_wait->timeout) {
				result = ETIMEDOUT;
			}

			new_timed_wait->done = true;
		}

		// The lock needs to be locked on exit of this function
		pthread_mutex_lock(lock);
		return result;
	}
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
