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
    static const uavcan::int64_t MaxDelayMSec = 0x000FFFFF;

    const uavcan::int64_t msec = duration.toMSec();
    msg_t ret = msg_t();

    if (msec <= 0)
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
        ret = sem_.waitTimeout((msec > MaxDelayMSec) ? MS2ST(MaxDelayMSec) : MS2ST(msec));
# else // ChibiOS 3+
        ret = sem_.wait((msec > MaxDelayMSec) ? MS2ST(MaxDelayMSec) : MS2ST(msec));
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

const unsigned BusEvent::MaxPollWaiters;
const char* const BusEvent::DevName = "/dev/uavcan/busevent";

int BusEvent::openTrampoline(::file* filp)
{
    return static_cast<BusEvent*>(filp->f_inode->i_private)->open(filp);
}

int BusEvent::closeTrampoline(::file* filp)
{
    return static_cast<BusEvent*>(filp->f_inode->i_private)->close(filp);
}

int BusEvent::pollTrampoline(::file* filp, ::pollfd* fds, bool setup)
{
    return static_cast<BusEvent*>(filp->f_inode->i_private)->poll(filp, fds, setup);
}

int BusEvent::open(::file* filp)
{
    (void)filp;
    return 0;
}

int BusEvent::close(::file* filp)
{
    (void)filp;
    return 0;
}

int BusEvent::poll(::file* filp, ::pollfd* fds, bool setup)
{
    CriticalSectionLocker locker;
    int ret = -1;

    if (setup)
    {
        ret = addPollWaiter(fds);
        if (ret == 0)
        {
            /*
             * Two events can be reported via POLLIN:
             *  - The RX queue is not empty. This event is level-triggered.
             *  - Transmission complete. This event is edge-triggered.
             * FIXME Since TX event is edge-triggered, it can be lost between poll() calls.
             */
            fds->revents |= fds->events & (can_driver_.hasReadableInterfaces() ? POLLIN : 0);
            if (fds->revents != 0)
            {
                (void)sem_post(fds->sem);
            }
        }
    }
    else
    {
        ret = removePollWaiter(fds);
    }

    return ret;
}

int BusEvent::addPollWaiter(::pollfd* fds)
{
    for (unsigned i = 0; i < MaxPollWaiters; i++)
    {
        if (pollset_[i] == UAVCAN_NULLPTR)
        {
            pollset_[i] = fds;
            return 0;
        }
    }
    return -ENOMEM;
}

int BusEvent::removePollWaiter(::pollfd* fds)
{
    for (unsigned i = 0; i < MaxPollWaiters; i++)
    {
        if (fds == pollset_[i])
        {
            pollset_[i] = UAVCAN_NULLPTR;
            return 0;
        }
    }
    return -EINVAL;
}

BusEvent::BusEvent(CanDriver& can_driver)
    : can_driver_(can_driver)
    , signal_(false)
{
    std::memset(&file_ops_, 0, sizeof(file_ops_));
    std::memset(pollset_, 0, sizeof(pollset_));
    file_ops_.open  = &BusEvent::openTrampoline;
    file_ops_.close = &BusEvent::closeTrampoline;
    file_ops_.poll  = &BusEvent::pollTrampoline;
    // TODO: move to init(), add proper error handling
    if (register_driver(DevName, &file_ops_, 0666, static_cast<void*>(this)) != 0)
    {
        std::abort();
    }
}

BusEvent::~BusEvent()
{
    (void)unregister_driver(DevName);
}

bool BusEvent::wait(uavcan::MonotonicDuration duration)
{
    // TODO blocking wait
    const uavcan::MonotonicTime deadline = clock::getMonotonic() + duration;
    while (clock::getMonotonic() < deadline)
    {
        {
            CriticalSectionLocker locker;
            if (signal_)
            {
                signal_ = false;
                return true;
            }
        }
        ::usleep(1000);
    }
    return false;
}

void BusEvent::signalFromInterrupt()
{
    signal_ = true;  // HACK
    for (unsigned i = 0; i < MaxPollWaiters; i++)
    {
        ::pollfd* const fd = pollset_[i];
        if (fd != UAVCAN_NULLPTR)
        {
            fd->revents |= fd->events & POLLIN;
            if ((fd->revents != 0) && (fd->sem->semcount <= 0))
            {
                (void)sem_post(fd->sem);
            }
        }
    }
}

#endif

}
