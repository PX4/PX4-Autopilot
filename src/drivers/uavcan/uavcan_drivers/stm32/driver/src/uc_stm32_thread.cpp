/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan_stm32/thread.hpp>
#include <uavcan_stm32/clock.hpp>
#include <uavcan_stm32/can.hpp>
#include "internal.hpp"


namespace uavcan_stm32
{

#if UAVCAN_STM32_CHIBIOS
/*
 * BusEvent
 */
bool BusEvent::wait(uavcan::MonotonicDuration duration)
{
    // set maximum time to allow for 16 bit timers running at 1MHz
    static const uavcan::int64_t MaxDelayUSec = 0x000FFFF;

    const uavcan::int64_t usec = duration.toUSec();
    msg_t ret = msg_t();

    if (usec <= 0)
    {
# if (CH_KERNEL_MAJOR == 2)
        ret = sem_.waitTimeout(TIME_IMMEDIATE);
# else // ChibiOS 3+
        ret = sem_.wait(TIME_IMMEDIATE);
# endif
    }
    else
    {
# if (CH_KERNEL_MAJOR == 2)
        ret = sem_.waitTimeout((usec > MaxDelayUSec) ? US2ST(MaxDelayUSec) : US2ST(usec));
# elif defined(MS2ST) // ChibiOS 3+
        ret = sem_.wait((usec > MaxDelayUSec) ? US2ST(MaxDelayUSec) : US2ST(usec));
# else // ChibiOS 17+
        ret = sem_.wait(::systime_t((usec > MaxDelayUSec) ? TIME_US2I(MaxDelayUSec) : TIME_US2I(usec)));
# endif
    }
# if (CH_KERNEL_MAJOR == 2)
    return ret == RDY_OK;
# else // ChibiOS 3+
    return ret == MSG_OK;
# endif
}

void BusEvent::signal()
{
    sem_.signal();
}

void BusEvent::signalFromInterrupt()
{
# if (CH_KERNEL_MAJOR == 2)
    chSysLockFromIsr();
    sem_.signalI();
    chSysUnlockFromIsr();
# else // ChibiOS 3+
    chSysLockFromISR();
    sem_.signalI();
    chSysUnlockFromISR();
# endif
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
# if (CH_KERNEL_MAJOR == 2)
    chibios_rt::BaseThread::unlockMutex();
# else // ChibiOS 3+
    mtx_.unlock();
# endif
}


#elif UAVCAN_STM32_FREERTOS

bool BusEvent::wait(uavcan::MonotonicDuration duration)
{
    static const uavcan::int64_t MaxDelayMSec = 0x000FFFFF;

    const uavcan::int64_t msec = duration.toMSec();

    BaseType_t ret;

    if (msec <= 0)
    {
        ret = xSemaphoreTake( sem_, ( TickType_t ) 0 );
    }
    else
    {
        ret = xSemaphoreTake( sem_, (msec > MaxDelayMSec) ? (MaxDelayMSec/portTICK_RATE_MS) : (msec/portTICK_RATE_MS));
    }
    return ret == pdTRUE;
}

void BusEvent::signal()
{
    xSemaphoreGive( sem_ );
}

void BusEvent::signalFromInterrupt()
{
    higher_priority_task_woken = pdFALSE;

    xSemaphoreGiveFromISR( sem_, &higher_priority_task_woken );
}

void BusEvent::yieldFromISR()
{
    portYIELD_FROM_ISR( higher_priority_task_woken );
}

/*
 * Mutex
 */
void Mutex::lock()
{
    xSemaphoreTake( mtx_, portMAX_DELAY );
}

void Mutex::unlock()
{
    xSemaphoreGive( mtx_ );
}


#elif UAVCAN_STM32_NUTTX

BusEvent::BusEvent(CanDriver& can_driver)
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
    if (sem_.semcount <= 0)
    {
        (void)sem_post(&sem_);
    }
    if (signal_cb_)
    {
        signal_cb_();
    }
}

#endif

}
