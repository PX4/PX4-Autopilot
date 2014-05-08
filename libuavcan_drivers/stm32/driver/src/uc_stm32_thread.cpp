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

#elif UAVCAN_STM32_NUTTX

Event::Event()
    : sem_()
{
    (void)sem_init(&sem_, 0, 0);
}

Event::~Event()
{
    (void)sem_destroy(&sem_);
}

bool Event::wait(uavcan::MonotonicDuration duration)
{
    if (duration.isNegative())
    {
        duration = uavcan::MonotonicDuration();
    }

    timespec deadline = timespec();
    if (clock_gettime(CLOCK_REALTIME, &deadline) < 0)
    {
        ASSERT(0);
        return false;
    }
    deadline.tv_sec += duration.toUSec() / 1000000;
    deadline.tv_nsec += (duration.toUSec() % 1000000) * 1000;
    if (deadline.tv_nsec >= 1000000000L)
    {
        deadline.tv_sec += 1;
        deadline.tv_nsec -= 1000000000L;
    }

    const int result = sem_timedwait(&sem_, &deadline);
    return result >= 0;
}

void Event::signal()
{
    (void)sem_post(&sem_);
}

void Event::signalFromInterrupt()
{
    (void)sem_post(&sem_);  // Can be called from ISR http://nuttx.org/Documentation/NuttxUserGuide.html#sempost
}

#endif

}
