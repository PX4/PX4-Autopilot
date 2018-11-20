/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

/**
 * @file px4_sem.cpp
 *
 * PX4 Middleware Wrapper Linux Implementation
 */

#include <px4_defines.h>
#include <px4_middleware.h>
#include <px4_workqueue.h>
#include <px4_time.h>
#include <stdint.h>
#include <stdio.h>
#include <pthread.h>
#include <errno.h>

#if (defined(__PX4_DARWIN) || defined(__PX4_CYGWIN) || defined(__PX4_POSIX)) && !defined(__PX4_QURT)

#include <px4_posix.h>

int px4_sem_init(px4_sem_t *s, int pshared, unsigned value)
{
	// We do not used the process shared arg
	(void)pshared;
	s->value = value;
	pthread_cond_init(&(s->wait), nullptr);
	pthread_mutex_init(&(s->lock), nullptr);

	return 0;
}

int px4_sem_setprotocol(px4_sem_t *s, int protocol)
{
	return 0;
}

int px4_sem_wait(px4_sem_t *s)
{
	int ret = pthread_mutex_lock(&(s->lock));

	if (ret) {
		return ret;
	}

	s->value--;

	if (s->value < 0) {
		ret = pthread_cond_wait(&(s->wait), &(s->lock));

	} else {
		ret = 0;
	}

	if (ret) {
		PX4_WARN("px4_sem_wait failure");
	}

	int mret = pthread_mutex_unlock(&(s->lock));

	return (ret) ? ret : mret;
}

int px4_sem_trywait(px4_sem_t *s)
{
	int ret = pthread_mutex_lock(&(s->lock));

	if (ret) {
		return ret;
	}

	if (s->value <= 0) {
		errno = EAGAIN;
		ret = -1;

	} else {
		s->value--;
	}

	int mret = pthread_mutex_unlock(&(s->lock));

	return (ret) ? ret : mret;
}

int px4_sem_timedwait(px4_sem_t *s, const struct timespec *abstime)
{
	int ret = pthread_mutex_lock(&(s->lock));

	if (ret) {
		return ret;
	}

	s->value--;
	errno = 0;

	if (s->value < 0) {
		ret = px4_pthread_cond_timedwait(&(s->wait), &(s->lock), abstime);

	} else {
		ret = 0;
	}

	int err = ret;

	if (err != 0 && errno != ETIMEDOUT) {
		setbuf(stdout, nullptr);
		setbuf(stderr, nullptr);
		const unsigned NAMELEN = 32;
		char thread_name[NAMELEN] = {};
		(void)pthread_getname_np(pthread_self(), thread_name, NAMELEN);
		PX4_WARN("%s: px4_sem_timedwait failure: ret: %d, %s", thread_name, ret, strerror(err));
	}

	int mret = pthread_mutex_unlock(&(s->lock));

	return (err) ? err : mret;
}

int px4_sem_post(px4_sem_t *s)
{
	int ret = pthread_mutex_lock(&(s->lock));

	if (ret) {
		return ret;
	}

	s->value++;

	if (s->value <= 0) {
		ret = pthread_cond_signal(&(s->wait));

	} else {
		ret = 0;
	}

	if (ret) {
		PX4_WARN("px4_sem_post failure");
	}

	int mret = pthread_mutex_unlock(&(s->lock));

	// return the cond signal failure if present,
	// else return the mutex status
	return (ret) ? ret : mret;
}

int px4_sem_getvalue(px4_sem_t *s, int *sval)
{
	int ret = pthread_mutex_lock(&(s->lock));

	if (ret) {
		PX4_WARN("px4_sem_getvalue failure");
	}

	if (ret) {
		return ret;
	}

	*sval = s->value;
	ret = pthread_mutex_unlock(&(s->lock));

	return ret;
}

int px4_sem_destroy(px4_sem_t *s)
{
	pthread_mutex_lock(&(s->lock));
	pthread_cond_destroy(&(s->wait));
	pthread_mutex_unlock(&(s->lock));
	pthread_mutex_destroy(&(s->lock));

	return 0;
}

#endif
