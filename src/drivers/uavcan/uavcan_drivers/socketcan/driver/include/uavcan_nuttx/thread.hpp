/****************************************************************************
 *
 *   Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 *   Kinetis Port Author David Sidrane <david_s5@nscdg.com>
 *   NuttX SocketCAN port Copyright (C) 2022 NXP Semiconductors
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


#include <nuttx/config.h>
#include <nuttx/fs/fs.h>
#include <poll.h>
#include <errno.h>
#include <cstdio>
#include <ctime>
#include <cstring>

#include <uavcan/uavcan.hpp>

namespace uavcan_socketcan
{

class CanDriver;


/**
 * All bus events are reported as POLLIN.
 */
class BusEvent : uavcan::Noncopyable
{
	using SignalCallbackHandler = void(*)();

	SignalCallbackHandler signal_cb_{nullptr};
	sem_t sem_;
public:

	BusEvent(CanDriver &can_driver);
	~BusEvent();

	void registerSignalCallback(SignalCallbackHandler handler) { signal_cb_ = handler; }

	bool wait(uavcan::MonotonicDuration duration);

	void signalFromInterrupt();
};

class Mutex
{
	pthread_mutex_t mutex_;

public:
	Mutex()
	{
		init();
	}

	int init()
	{
		return pthread_mutex_init(&mutex_, UAVCAN_NULLPTR);
	}

	int deinit()
	{
		return pthread_mutex_destroy(&mutex_);
	}

	void lock()
	{
		(void)pthread_mutex_lock(&mutex_);
	}

	void unlock()
	{
		(void)pthread_mutex_unlock(&mutex_);
	}
};


class MutexLocker
{
	Mutex &mutex_;

public:
	MutexLocker(Mutex &mutex)
		: mutex_(mutex)
	{
		mutex_.lock();
	}
	~MutexLocker()
	{
		mutex_.unlock();
	}
};

}
