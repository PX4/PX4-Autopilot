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
	if (_time_us == 0 && time_us > 0) {
		PX4_INFO("setting initial absolute time to %" PRIu64 " us", time_us);
	}

	_time_us = time_us;

	// Hold _signaling_mutex across the whole operation so that waiters which
	// enter the dance cannot race past us by acquiring it before we do.
	// Lock order: _signaling_mutex -> _timed_waits_mutex, _signaling_mutex ->
	// passed_lock. cond_timedwait() uses passed_lock -> _timed_waits_mutex.
	// No cycle because _signaling_mutex is only ever acquired first here and
	// independently (without passed_lock held) in the waiter's dance.
	std::lock_guard<std::mutex> lock_signaling(_signaling_mutex);

	// Phase 1: under _timed_waits_mutex, clean up done entries, mark timeouts,
	// and thread waiters that need to be signaled onto a temporary list via
	// signal_next. We do NOT take any passed_lock here — that would create a
	// lock-order cycle with cond_timedwait (which holds passed_lock while
	// acquiring _timed_waits_mutex).
	TimedWait *to_signal_head = nullptr;

	{
		std::lock_guard<std::mutex> lock_timed_waits(_timed_waits_mutex);
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
				timed_wait->timeout = true;
				timed_wait->signal_next = to_signal_head;
				to_signal_head = timed_wait;
			}

			timed_wait_prev = timed_wait;
			timed_wait = timed_wait->next;
		}
	}

	// Phase 2: signal each waiter outside _timed_waits_mutex. _signaling_mutex
	// is still held, so any waiter that sees timeout via the wall-clock
	// fallback will block in the dance until we are done, preventing
	// use-after-free of their stack-local passed_lock/passed_cond.
	for (TimedWait *tw = to_signal_head; tw != nullptr;) {
		TimedWait *next = tw->signal_next;
		tw->signal_next = nullptr;
		pthread_mutex_lock(tw->passed_lock);
		pthread_cond_broadcast(tw->passed_cond);
		pthread_mutex_unlock(tw->passed_lock);
		tw = next;
	}

	// Phase 3: clear _setting_time under _timed_waits_mutex so waiters in
	// the dance observe a consistent state.
	{
		std::lock_guard<std::mutex> lock_timed_waits(_timed_waits_mutex);
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

	// Use a short wall-clock timeout instead of waiting indefinitely.
	// There is a race window between releasing _timed_waits_mutex (above)
	// and entering pthread_cond_wait: if set_absolute_time() broadcasts
	// during that window, the signal is lost and we'd block forever.
	// A periodic wake-up lets us re-check the timeout flag.
	int result;

	while (true) {
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		// Wake up every 10ms wall-clock to re-check
		ts.tv_nsec += 10000000; // 10ms

		if (ts.tv_nsec >= 1000000000) {
			ts.tv_sec += 1;
			ts.tv_nsec -= 1000000000;
		}

		result = pthread_cond_timedwait(cond, lock, &ts);

		if (timed_wait.timeout || result == 0) {
			break;
		}

		// ETIMEDOUT from the wall-clock timeout — just re-check
	}

	const bool timeout = timed_wait.timeout;

	if (result == 0 && timeout) {
		result = ETIMEDOUT;
	}

	timed_wait.done = true;

	if (_setting_time) {
		// Another thread is in set_absolute_time(). It may still hold a
		// pointer to our stack-local passed_lock / passed_cond in its
		// to-signal list. We must not return (and let those stack objects
		// go out of scope) until set_absolute_time() is finished.
		//
		// We also have to unlock 'lock' before acquiring the scheduler's
		// internal mutexes to avoid the ABBA with cond_timedwait()'s own
		// passed_lock -> _timed_waits_mutex order.
		//
		// _signaling_mutex is held by set_absolute_time() across its
		// signal phase, and _timed_waits_mutex guards the Phase 3 clear of
		// _setting_time. Acquiring both in order guarantees we outlive the
		// whole operation.
		pthread_mutex_unlock(lock);
		_signaling_mutex.lock();
		_signaling_mutex.unlock();
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
