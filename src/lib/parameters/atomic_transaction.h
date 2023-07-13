/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#pragma once

#ifdef __PX4_NUTTX
#include "px4_platform_common/micro_hal.h"
#endif

#ifdef __PX4_POSIX
#include <pthread.h>

class _MutexHolder
{
public:
	pthread_mutex_t _mutex;
	pthread_mutexattr_t _mutex_attr;

	_MutexHolder()
	{
		pthread_mutexattr_init(&_mutex_attr);
		pthread_mutexattr_settype(&_mutex_attr, PTHREAD_MUTEX_RECURSIVE);
		pthread_mutex_init(&_mutex, &_mutex_attr);
	}

	~_MutexHolder()
	{
		pthread_mutex_destroy(&_mutex);
	}
};
#endif


class AtomicTransaction
{
private:
#ifdef __PX4_NUTTX
	irqstate_t _irq_state;
#endif

#ifdef __PX4_POSIX
	static _MutexHolder _mutex_holder;
#endif

public:
	AtomicTransaction()
	{
		lock();
	}

	~AtomicTransaction()
	{
		unlock();
	}

	void lock()
	{
#ifdef __PX4_NUTTX
		_irq_state = px4_enter_critical_section();
#endif
#ifdef __PX4_POSIX
		pthread_mutex_lock(&_mutex_holder._mutex);
#endif
	}

	void unlock()
	{
#ifdef __PX4_NUTTX
		px4_leave_critical_section(_irq_state);
#endif
#ifdef __PX4_POSIX
		pthread_mutex_unlock(&_mutex_holder._mutex);
#endif
	}
};
