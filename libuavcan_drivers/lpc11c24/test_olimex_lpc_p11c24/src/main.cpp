/*
 * Pavel Kirienko, 2014 <pavel.kirienko@gmail.com>
 */

#include <board.hpp>
#include <chip.h>
#include <uavcan_lpc11c24/can.hpp>

int main()
{
    if (uavcan_lpc11c24::CanDriver::instance().init(1000000) < 0)
    {
        while (true) { }
    }

    while (true)
    {
        for (volatile int i = 0; i < 2000000; i++) { __asm volatile ("nop"); }

        board::setErrorLed(uavcan_lpc11c24::CanDriver::instance().getErrorCount() > 0);

        if (uavcan_lpc11c24::CanDriver::instance().hasPendingRx())
        {
            board::setStatusLed(true);
            uavcan::CanFrame frm;
            uavcan::UtcTime utc;
            uavcan::MonotonicTime mono;
            uavcan::CanIOFlags flags;
            uavcan_lpc11c24::CanDriver::instance().receive(frm, mono, utc, flags);
            asm volatile ("nop");
        }
        else
        {
            board::setStatusLed(false);
        }
    }
}
