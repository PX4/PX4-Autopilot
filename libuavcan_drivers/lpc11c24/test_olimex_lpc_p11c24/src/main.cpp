/*
 * Pavel Kirienko, 2014 <pavel.kirienko@gmail.com>
 */

#include <cstdio>
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
    uavcan::MonotonicTime prev_time_pub_at;

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

        if ((ts_mono - prev_time_pub_at).toMSec() >= 1000)
        {
            prev_time_pub_at = ts_mono;

            uavcan::CanFrame frm;
            frm.dlc = 8;
            std::fill_n(frm.data, 8, 0);
            snprintf(reinterpret_cast<char*>(frm.data), 8, "%u", unsigned(ts_mono.toMSec()));

            frm.id = ts_mono.toMSec() | uavcan::CanFrame::FlagEFF;

            ENFORCE(1 == uavcan_lpc11c24::CanDriver::instance().send(frm, ts_mono, 0));
        }
    }
}
