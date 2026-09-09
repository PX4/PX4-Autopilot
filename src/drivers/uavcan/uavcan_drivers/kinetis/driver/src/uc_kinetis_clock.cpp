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

// Written under a critical section so the CAN interrupt, which reads them
// without locking, never sees a torn 64-bit offset.
bool utc_set = false;
uavcan::int64_t utc_offset_usec = 0;

uavcan::uint64_t utcNow()
{
	return utc_set ? uavcan::uint64_t(uavcan::int64_t(hrt_absolute_time()) + utc_offset_usec) : 0;
}

}

uavcan::MonotonicTime getMonotonic()
{
	return uavcan::MonotonicTime::fromUSec(hrt_absolute_time());
}

uavcan::UtcTime getUtc()
{
	CriticalSectionLocker locker;
	return uavcan::UtcTime::fromUSec(utcNow());
}

uavcan::uint64_t getUtcUSecFromCanInterrupt()
{
	return utcNow();
}

void setUtc(uavcan::UtcTime time)
{
	CriticalSectionLocker locker;
	utc_offset_usec = uavcan::int64_t(time.toUSec()) - uavcan::int64_t(hrt_absolute_time());
	utc_set = true;
}

void adjustUtc(uavcan::UtcDuration adjustment)
{
	CriticalSectionLocker locker;

	if (utc_set) {
		utc_offset_usec = utc_offset_usec + adjustment.toUSec();

	} else {
		utc_offset_usec = adjustment.toUSec() - uavcan::int64_t(hrt_absolute_time());
		utc_set = true;
	}
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
