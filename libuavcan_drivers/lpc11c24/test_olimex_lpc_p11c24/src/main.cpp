/*
 * Pavel Kirienko, 2014 <pavel.kirienko@gmail.com>
 */

#include <board.hpp>
#include <chip.h>
#include <uavcan_lpc11c24/can.hpp>
#include <uavcan_lpc11c24/clock.hpp>

#define ENFORCE(x)        \
    if ((x) == 0) {       \
        while (true) { board::setErrorLed(true); }  \
    }

int main()
{
    if (uavcan_lpc11c24::CanDriver::instance().init(1000000) < 0)
    {
        while (true) { }
    }

    uavcan_lpc11c24::clock::init();

    uavcan::MonotonicTime prev_mono;

    while (true)
    {
        const uavcan::MonotonicTime ts_mono = uavcan_lpc11c24::clock::getMonotonic();

        while (prev_mono >= ts_mono)
        {
        }
        prev_mono = ts_mono;

        board::setErrorLed(uavcan_lpc11c24::CanDriver::instance().getErrorCount() > 0 ||
                           uavcan_lpc11c24::CanDriver::instance().hadActivity());

        if (uavcan_lpc11c24::CanDriver::instance().hasPendingRx())
        {
            board::setStatusLed(true);
            uavcan::CanFrame frm;
            uavcan::UtcTime utc;
            uavcan::MonotonicTime mono;
            uavcan::CanIOFlags flags;
            ENFORCE(1 == uavcan_lpc11c24::CanDriver::instance().receive(frm, mono, utc, flags));
            asm volatile ("nop");

            frm.id += 0x100;
            ENFORCE(1 == uavcan_lpc11c24::CanDriver::instance().send(frm, mono, 0));
        }
        else
        {
            board::setStatusLed(false);
        }
    }
}
