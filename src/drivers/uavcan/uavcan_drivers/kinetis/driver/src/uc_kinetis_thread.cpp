/*
 * Copyright (C) 2014, 2018 Pavel Kirienko <pavel.kirienko@gmail.com>
 * Kinetis Port Author David Sidrane <david_s5@nscdg.com>
 */

#include <uavcan_kinetis/thread.hpp>
#include <uavcan_kinetis/clock.hpp>
#include <uavcan_kinetis/can.hpp>
#include "internal.hpp"

namespace uavcan_kinetis
{

BusEvent::BusEvent(CanDriver &can_driver)
{
	sem_init(&sem_, 0, 0);
	sem_setprotocol(&sem_, SEM_PRIO_NONE);
}

BusEvent::~BusEvent()
{
	sem_destroy(&sem_);
}

bool BusEvent::wait(uavcan::MonotonicDuration duration)
{
	if (duration.isPositive()) {
		timespec abstime;

		if (clock_gettime(CLOCK_REALTIME, &abstime) == 0) {
			const unsigned billion = 1000 * 1000 * 1000;
			uint64_t nsecs = abstime.tv_nsec + (uint64_t)duration.toUSec() * 1000;
			abstime.tv_sec += nsecs / billion;
			nsecs -= (nsecs / billion) * billion;
			abstime.tv_nsec = nsecs;

			int ret;

			while ((ret = sem_timedwait(&sem_, &abstime)) == -1 && errno == EINTR);

			if (ret == -1) { // timed out or error
				return false;
			}

			return true;
		}
	}

	return false;
}

void BusEvent::signalFromInterrupt()
{
	if (sem_.semcount <= 0) {
		(void)sem_post(&sem_);
	}

	if (signal_cb_) {
		signal_cb_();
	}
}

}
