/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan_stm32/clock.hpp>
#include <uavcan_stm32/thread.hpp>
#include "internal.hpp"

#include <drivers/drv_hrt.h>

namespace uavcan_stm32
{
namespace clock
{
namespace
{

Mutex mutex;

bool initialized = false;
}


void init()
{
	CriticalSectionLocker lock;

	if (initialized) {
		return;
	}

	initialized = true;
}

void setUtc(uavcan::UtcTime time)
{
	// DO NOTHING
}

uavcan::uint64_t getUtcUSecFromCanInterrupt()
{
	return hrt_absolute_time();
}

uavcan::MonotonicTime getMonotonic()
{
	uavcan::uint64_t usec = hrt_absolute_time();
	return uavcan::MonotonicTime::fromUSec(usec);
}

uavcan::UtcTime getUtc()
{
	uavcan::uint64_t usec = hrt_absolute_time();
	return uavcan::UtcTime::fromUSec(usec);
}

void adjustUtc(uavcan::UtcDuration adjustment)
{
	const float adj_usec = float(adjustment.toUSec());
	hrt_absolute_time_adjust(adj_usec);
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
		clock::init();
		new (ptr)SystemClock();
	}

	return *ptr;
}

} // namespace uavcan_stm32

