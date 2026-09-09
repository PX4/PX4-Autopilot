/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan_stm32h7/build_config.hpp>
#include <uavcan/driver/system_clock.hpp>

namespace uavcan_stm32h7
{

/*
 * Monotonic time is HRT. UTC is HRT plus an offset that time sync moves, so
 * the bus time base shares HRT's rate and needs no hardware of its own.
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
