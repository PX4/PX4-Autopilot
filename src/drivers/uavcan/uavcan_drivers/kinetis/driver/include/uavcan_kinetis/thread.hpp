/*
 * Copyright (C) 2014, 2018 Pavel Kirienko <pavel.kirienko@gmail.com>
 * Kinetis Port Author David Sidrane <david_s5@nscdg.com>
 */

#pragma once

#include <uavcan_kinetis/build_config.hpp>

#include <nuttx/config.h>
#include <nuttx/fs/fs.h>
#include <poll.h>
#include <errno.h>
#include <cstdio>
#include <ctime>
#include <cstring>

#include <uavcan/uavcan.hpp>

namespace uavcan_kinetis
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
