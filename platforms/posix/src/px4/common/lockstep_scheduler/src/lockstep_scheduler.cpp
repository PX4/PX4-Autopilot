/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <lockstep_scheduler/lockstep_scheduler.h>

#include <px4_platform_common/log.h>

#if defined(__PX4_WINDOWS) || defined(_WIN32)
#include <windows.h>
#endif

#if defined(__PX4_WINDOWS) || defined(_WIN32)
namespace
{
std::mutex &scheduler_registry_mutex()
{
	static std::mutex mutex;
	return mutex;
}

std::vector<LockstepScheduler *> &scheduler_registry()
{
	static std::vector<LockstepScheduler *> schedulers;
	return schedulers;
}

void px4_lockstep_pthread_cond_notify(pthread_cond_t *cond, int broadcast)
{
	LockstepScheduler::notify_pthread_condition(cond, broadcast != 0);
}
}
#endif

LockstepScheduler::LockstepScheduler(bool no_cleanup_on_destroy) :
	_components(no_cleanup_on_destroy)
{
#if defined(__PX4_WINDOWS) || defined(_WIN32)
	std::lock_guard<std::mutex> guard(scheduler_registry_mutex());
	scheduler_registry().push_back(this);

#if defined(_MSC_VER)
	px4_pthread_cond_set_notify_callback(px4_lockstep_pthread_cond_notify);
#endif
#endif
}

LockstepScheduler::~LockstepScheduler()
{
#if defined(__PX4_WINDOWS) || defined(_WIN32)
	{
		std::lock_guard<std::mutex> guard(scheduler_registry_mutex());
		auto &schedulers = scheduler_registry();

		for (auto it = schedulers.begin(); it != schedulers.end(); ++it)
		{
			if (*it == this) {
				schedulers.erase(it);
				break;
			}
		}
	}
#endif

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
	if (_time_us == 0 && time_us > 0) {
		PX4_INFO("setting initial absolute time to %" PRIu64 " us", time_us);
	}

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
#if defined(__PX4_WINDOWS) || defined(_WIN32)
				// Use the per-waiter Win32 CONDITION_VARIABLE/SRWLOCK
				// pair so the wake is kernel-managed and never dropped.
				// Avoid taking the user pthread mutex (winpthreads): it is
				// the broadcast that races and gets lost on that path.
				AcquireSRWLockExclusive(&timed_wait->wait_lock);
				timed_wait->timeout = true;
				WakeAllConditionVariable(&timed_wait->wait_cond);
				ReleaseSRWLockExclusive(&timed_wait->wait_lock);
#else
				pthread_mutex_lock(timed_wait->passed_lock);
				timed_wait->timeout = true;
				pthread_cond_broadcast(timed_wait->passed_cond);
				pthread_mutex_unlock(timed_wait->passed_lock);
#endif
			}

			timed_wait_prev = timed_wait;
			timed_wait = timed_wait->next;
		}

		_setting_time = false;
	}
}

#if defined(__PX4_WINDOWS) || defined(_WIN32)
void LockstepScheduler::notify_pthread_condition(pthread_cond_t *cond, bool broadcast)
{
	std::lock_guard<std::mutex> guard(scheduler_registry_mutex());

	for (LockstepScheduler *scheduler : scheduler_registry()) {
		if (scheduler->notify_pthread_condition_locked(cond, broadcast) && !broadcast) {
			break;
		}
	}
}

bool LockstepScheduler::notify_pthread_condition_locked(pthread_cond_t *cond, bool broadcast)
{
	bool notified = false;
	std::lock_guard<std::mutex> lock_timed_waits(_timed_waits_mutex);

	for (TimedWait *timed_wait = _timed_waits; timed_wait; timed_wait = timed_wait->next) {
		if (timed_wait->done || timed_wait->passed_cond != cond ||
		    timed_wait->timeout || timed_wait->signaled) {
			continue;
		}

		AcquireSRWLockExclusive(&timed_wait->wait_lock);
		timed_wait->signaled = true;
		WakeAllConditionVariable(&timed_wait->wait_cond);
		ReleaseSRWLockExclusive(&timed_wait->wait_lock);
		notified = true;

		if (!broadcast) {
			break;
		}
	}

	return notified;
}

extern "C" int px4_lockstep_pthread_cond_signal(pthread_cond_t *cond)
{
	const int result = (pthread_cond_signal)(cond);

	if (result == 0) {
		px4_lockstep_pthread_cond_notify(cond, 0);
	}

	return result;
}

extern "C" int px4_lockstep_pthread_cond_broadcast(pthread_cond_t *cond)
{
	const int result = (pthread_cond_broadcast)(cond);

	if (result == 0) {
		px4_lockstep_pthread_cond_notify(cond, 1);
	}

	return result;
}
#endif

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
#if defined(__PX4_WINDOWS) || defined(_WIN32)
		timed_wait.signaled = false;
#endif
		timed_wait.done = false;

		// Add to linked list if not removed yet (otherwise just re-use the object)
		if (timed_wait.removed) {
			timed_wait.removed = false;
			timed_wait.next = _timed_waits;
			_timed_waits = &timed_wait;
		}
	}

	// On most pthread implementations pthread_cond_wait here would
	// suffice — set_absolute_time broadcasts when our target is reached.
	// winpthreads (MinGW), however, occasionally drops the broadcast
	// when it races the waiter entering the wait, which leaves the
	// thread blocked forever.
	//
	// On Windows we therefore wait on a per-waiter native Win32
	// CONDITION_VARIABLE + SRWLOCK pair that the producer signals via
	// WakeAllConditionVariable (kernel-managed, never lost). The 500 ms
	// SleepConditionVariableSRW timeout is kept as a defensive watchdog
	// (under correct signaling it never fires).
	int result = 0;

#if defined(__PX4_WINDOWS) || defined(_WIN32)
	// pthread cond_timedwait contract: the user's `lock` is released for
	// the duration of the wait and re-acquired before return. Mirror that
	// while we sleep on the per-waiter Win32 primitive instead.
	pthread_mutex_unlock(lock);

	AcquireSRWLockExclusive(&timed_wait.wait_lock);

	while (!timed_wait.timeout && !timed_wait.signaled) {
		BOOL slept = SleepConditionVariableSRW(&timed_wait.wait_cond,
						       &timed_wait.wait_lock,
						       500, 0);

		if (timed_wait.timeout || timed_wait.signaled) {
			break;
		}

		if (!slept && GetLastError() == ERROR_TIMEOUT) {
			// Watchdog fired; re-check _time_us in case the
			// producer signalled before we entered the wait. With
			// native Win32 signaling this branch should never
			// flip the flag, but it's kept as belt-and-braces.
			ReleaseSRWLockExclusive(&timed_wait.wait_lock);
			_timed_waits_mutex.lock();

			if (time_us <= _time_us) {
				timed_wait.timeout = true;
			}

			_timed_waits_mutex.unlock();
			AcquireSRWLockExclusive(&timed_wait.wait_lock);
			continue;
		}

		// Spurious wakeup: loop to re-check.
	}

	ReleaseSRWLockExclusive(&timed_wait.wait_lock);

	pthread_mutex_lock(lock);
#else
	result = pthread_cond_wait(cond, lock);
#endif

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
#if defined(__PX4_WINDOWS) || defined(_WIN32)
	// On winpthreads PTHREAD_*_INITIALIZER triggers lazy allocation of
	// the underlying kernel object; without an explicit *_destroy() the
	// auto-storage variants leak on every tick, which at high speed
	// factors quickly accumulates. Keep a thread-local pair that is
	// created once per thread and destroyed at thread exit.
	static thread_local pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
	static thread_local pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
#else
	// Linux glibc treats PTHREAD_*_INITIALIZER as a static struct
	// initializer with no kernel-object allocation, so per-call
	// auto-storage matches main and avoids the cross-call cond
	// reuse semantics that interact badly with the LockstepScheduler
	// linked list when a thread re-enters usleep_until().
	pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
	pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
#endif

	pthread_mutex_lock(&lock);

	int result = cond_timedwait(&cond, &lock, time_us);

	if (result == ETIMEDOUT) {
		// This is expected because we never notified to the condition.
		result = 0;
	}

	pthread_mutex_unlock(&lock);

	return result;
}
