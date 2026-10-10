/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/sem.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>

#include <errno.h>

int px4_sem_init(px4_sem_t *s, int pshared, unsigned value)
{
	return k_sem_init(s, value, K_SEM_MAX_LIMIT);
}

int px4_sem_setprotocol(px4_sem_t *s, int protocol)
{
	return 0;
}

int px4_sem_wait(px4_sem_t *s)
{
	return k_sem_take(s, K_FOREVER);
}

int px4_sem_trywait(px4_sem_t *s)
{
	if (k_sem_take(s, K_NO_WAIT) != 0) {
		errno = EAGAIN;
		return -1;
	}

	return 0;
}

int px4_sem_timedwait(px4_sem_t *s, const struct timespec *abstime)
{
	const hrt_abstime deadline = ts_to_abstime(abstime);
	const hrt_abstime now = hrt_absolute_time();
	const k_timeout_t timeout = (deadline > now) ? K_USEC(deadline - now) : K_NO_WAIT;

	if (k_sem_take(s, timeout) != 0) {
		errno = ETIMEDOUT;
		return -1;
	}

	return 0;
}

int px4_sem_post(px4_sem_t *s)
{
	k_sem_give(s);
	return 0;
}

int px4_sem_getvalue(px4_sem_t *s, int *sval)
{
	*sval = k_sem_count_get(s);
	return 0;
}

int px4_sem_destroy(px4_sem_t *s)
{
	k_sem_reset(s);
	return 0;
}
