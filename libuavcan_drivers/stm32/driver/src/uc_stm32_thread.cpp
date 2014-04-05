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
    msg_t ret = msg_t();
    if (!duration.isPositive())
    {
        sem_.waitTimeout(TIME_IMMEDIATE);
    }
    else
    {
        sem_.waitTimeout(MS2ST(duration.toMSec()));
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
