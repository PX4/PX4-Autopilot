/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan_stm32h7/build_config.hpp>
#include <uavcan/driver/system_clock.hpp>
#include <drivers/uavcan/uavcan_drivers/hrt_clock.hpp>

namespace uavcan_stm32h7
{

/*
 * Monotonic time is HRT. UTC is HRT plus a synced offset and rate (see
 * uavcan_hrt_clock::Clock), so the bus clock needs no hardware of its own.
 */
namespace clock
{
uavcan::MonotonicTime getMonotonic();

/**
 * Zero until set by setUtc() or the first adjustUtc().
 */
uavcan::UtcTime getUtc();

/**
 * UTC reads `time` from now on.
 */
void setUtc(uavcan::UtcTime time);

/**
 * Moves UTC by `adjustment`. While UTC is unset the adjustment is absolute:
 * UTC reads `adjustment` from now on.
 */
void adjustUtc(uavcan::UtcDuration adjustment);

uavcan_hrt_clock::SyncStatus getSyncStatus();
}

/**
 * Adapter for uavcan::ISystemClock.
 */
class SystemClock : public uavcan::ISystemClock, uavcan::Noncopyable
{
	SystemClock() { }

	virtual void adjustUtc(uavcan::UtcDuration adjustment) { clock::adjustUtc(adjustment); }

public:
	virtual uavcan::MonotonicTime getMonotonic() const { return clock::getMonotonic(); }
	virtual uavcan::UtcTime getUtc()             const { return clock::getUtc(); }

	static SystemClock &instance();
};

}
