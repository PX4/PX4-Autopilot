/*
 * Copyright (C) 2014, 2018 Pavel Kirienko <pavel.kirienko@gmail.com>
 * Kinetis Port Author David Sidrane <david_s5@nscdg.com>
 */

#include <uavcan_kinetis/clock.hpp>
#include <uavcan_kinetis/thread.hpp>
#include "internal.hpp"

#include <drivers/drv_hrt.h>

namespace uavcan_kinetis
{
namespace clock
{
namespace
{

Mutex mutex;
bool initialized = false;
uavcan_hrt_clock::Clock hrt_clock;

}

uavcan::MonotonicTime getMonotonic()
{
	return uavcan::MonotonicTime::fromUSec(hrt_absolute_time());
}

uavcan::UtcTime getUtc()
{
	return uavcan::UtcTime::fromUSec(hrt_clock.utcUsec());
}

uavcan::uint64_t getUtcUSecFromCanInterrupt()
{
	return hrt_clock.utcUsecFromInterrupt();
}

void setUtc(uavcan::UtcTime time)
{
	hrt_clock.setUtc(time.toUSec());
}

void adjustUtc(uavcan::UtcDuration adjustment)
{
	hrt_clock.adjustUtc(adjustment.toUSec());
}

uavcan_hrt_clock::SyncStatus getSyncStatus()
{
	return hrt_clock.status();
}

} // namespace clock

SystemClock &SystemClock::instance()
{
	static union SystemClockStorage {
		uavcan::uint8_t buffer[sizeof(SystemClock)];
		long long _aligner_1;
		long double _aligner_2;
	} storage;

	SystemClock *const ptr = reinterpret_cast<SystemClock *>(storage.buffer);

	if (!clock::initialized) {
		MutexLocker mlocker(clock::mutex);
		clock::initialized = true;
		new (ptr)SystemClock();
	}

	return *ptr;
}

} // namespace uavcan_kinetis
