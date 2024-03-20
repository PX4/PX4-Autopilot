/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan_stm32/build_config.hpp>
#include <uavcan/driver/system_clock.hpp>

namespace uavcan_stm32
{

namespace clock
{
/**
 * Starts the clock.
 * Can be called multiple times, only the first call will be effective.
 */
void init();

/**
 * Returns current monotonic time since the moment when clock::init() was called.
 * This function is thread safe.
 */
uavcan::MonotonicTime getMonotonic();

/**
 * Sets the driver's notion of the system UTC. It should be called
 * at startup and any time the system clock is updated from an
 * external source that is not the UAVCAN Timesync master.
 * This function is thread safe.
 */
void setUtc(uavcan::UtcTime time);

/**
 * Returns UTC time if it has been set, otherwise returns zero time.
 * This function is thread safe.
 */
uavcan::UtcTime getUtc();

/**
 * Performs UTC phase and frequency adjustment.
 * The UTC time will be zero until first adjustment has been performed.
 * This function is thread safe.
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

	/**
	 * Calls clock::init() as needed.
	 * This function is thread safe.
	 */
	static SystemClock &instance();
};

}
