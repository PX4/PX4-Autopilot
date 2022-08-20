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

#include <uavcan_nuttx/thread.hpp>
#include <uavcan_nuttx/socketcan.hpp>

namespace uavcan_socketcan
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
