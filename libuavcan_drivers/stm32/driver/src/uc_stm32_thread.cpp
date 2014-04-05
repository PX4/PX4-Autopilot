/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan_stm32/thread.hpp>

namespace uavcan_stm32
{

#if UAVCAN_STM32_CHIBIOS
/*
 * Event
 */
bool Event::wait(uavcan::MonotonicDuration duration)
{
    static const uavcan::int64_t MaxDelayMSec = 0x000FFFFF;

    const uavcan::int64_t msec = duration.toMSec();
    msg_t ret = msg_t();

    if (msec <= 0)
    {
        ret = sem_.waitTimeout(TIME_IMMEDIATE);
    }
    else
    {
        ret = sem_.waitTimeout((msec > MaxDelayMSec) ? MS2ST(MaxDelayMSec) : MS2ST(msec));
    }
    return ret == RDY_OK;
}

void Event::signal()
{
    sem_.signal();
}

void Event::signalFromInterrupt()
{
    chSysLockFromIsr();
    sem_.signalI();
    chSysUnlockFromIsr();
}

/*
 * Mutex
 */
void Mutex::lock()
{
    mtx_.lock();
}

void Mutex::unlock()
{
    chibios_rt::BaseThread::unlockMutex();
}

#endif

}
