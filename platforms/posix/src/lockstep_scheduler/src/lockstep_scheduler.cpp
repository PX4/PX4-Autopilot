#include <lockstep_scheduler/lockstep_scheduler.h>

LockstepScheduler::~LockstepScheduler()
{
	// cleanup the linked list
	std::unique_lock<std::mutex> lock_timed_waits(_timed_waits_mutex);

	while (_timed_waits) {
		TimedWait *tmp = _timed_waits;
		_timed_waits = _timed_waits->next;
		tmp->removed = true;
	}
}

void LockstepScheduler::set_absolute_time(uint64_t time_us)
{
	_time_us = time_us;

	{
		std::unique_lock<std::mutex> lock_timed_waits(_timed_waits_mutex);
		_setting_time = true;

		TimedWait *timed_wait = _timed_waits;
		TimedWait *timed_wait_prev = nullptr;

		while (timed_wait) {
			// Clean up the ones that are already done from last iteration.
			if (timed_wait->done) {
				// Erase from the linked list
				if (timed_wait_prev) {
					timed_wait_prev->next = timed_wait->next;

				} else {
					_timed_waits = timed_wait->next;
				}

				TimedWait *tmp = timed_wait;
				timed_wait = timed_wait->next;
				tmp->removed = true;
				continue;
			}

			if (timed_wait->time_us <= time_us &&
			    !timed_wait->timeout) {
				// We are abusing the condition here to signal that the time
				// has passed.
				pthread_mutex_lock(timed_wait->passed_lock);
				timed_wait->timeout = true;
				pthread_cond_broadcast(timed_wait->passed_cond);
				pthread_mutex_unlock(timed_wait->passed_lock);
			}

			timed_wait_prev = timed_wait;
			timed_wait = timed_wait->next;
		}

		_setting_time = false;
	}
}

int LockstepScheduler::cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *lock, uint64_t time_us)
{
	// A TimedWait object might still be in timed_waits_ after we return, so its lifetime needs to be
	// longer. And using thread_local is more efficient than malloc.
	static thread_local TimedWait timed_wait;
	{
		std::lock_guard<std::mutex> lock_timed_waits(_timed_waits_mutex);

		// The time has already passed.
		if (time_us <= _time_us) {
			return ETIMEDOUT;
		}

		timed_wait.time_us = time_us;
		timed_wait.passed_cond = cond;
		timed_wait.passed_lock = lock;
		timed_wait.timeout = false;
		timed_wait.done = false;

		// Add to linked list if not removed yet (otherwise just re-use the object)
		if (timed_wait.removed) {
			timed_wait.removed = false;
			timed_wait.next = _timed_waits;
			_timed_waits = &timed_wait;
		}
	}

	int result = pthread_cond_wait(cond, lock);

	const bool timeout = timed_wait.timeout;

	if (result == 0 && timeout) {
		result = ETIMEDOUT;
	}

	timed_wait.done = true;

	if (!timeout && _setting_time) {
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
		_timed_waits_mutex.lock();
		_timed_waits_mutex.unlock();
		pthread_mutex_lock(lock);
	}

	return result;
}

int LockstepScheduler::usleep_until(uint64_t time_us)
{
	pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
	pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

	pthread_mutex_lock(&lock);

	int result = cond_timedwait(&cond, &lock, time_us);

	if (result == ETIMEDOUT) {
		// This is expected because we never notified to the condition.
		result = 0;
	}

	pthread_mutex_unlock(&lock);

	return result;
}
