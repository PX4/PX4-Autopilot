/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <unistd.h>
#include <zubax_chibios/sys/sys.h>
#include <uavcan_stm32/uavcan_stm32.hpp>

namespace app
{
namespace
{

uavcan_stm32::CanInitHelper<128> can;

void ledSet(bool state)
{
    palWritePad(GPIO_PORT_LED, GPIO_PIN_LED, state);
}

int init()
{
    halInit();
    chibios_rt::System::init();
    sdStart(&STDOUT_SD, NULL);

    return can.init(1000000);
}

#if __GNUC__
__attribute__((noreturn))
#endif
void die(int status)
{
    lowsyslog("Now I am dead x_x %i\n", status);
    while (1)
    {
        ledSet(false);
        sleep(1);
        ledSet(true);
        sleep(1);
    }
}

}
}

int main()
{
    const int init_res = app::init();
    if (init_res != 0)
    {
        app::die(init_res);
    }

    lowsyslog("Bootup\n");

    struct IfaceStatus
    {
        unsigned peak_mbx_used = 0;
        unsigned tx_aborts = 0;
        unsigned errors = 0;
        unsigned lec = 0;

        IfaceStatus() { }

        IfaceStatus(uavcan_stm32::CanIface& iface) :
            peak_mbx_used(iface.getPeakNumTxMailboxesUsed()),
            tx_aborts(iface.getVoluntaryTxAbortCount()),
            errors(iface.getErrorCount()),
            lec(iface.yieldLastHardwareErrorCode())
        { }

        uavcan::MakeString<80>::Type toString() const
        {
            uavcan::MakeString<80>::Type s;
            s.appendFormatted("pmbx %u  ", peak_mbx_used);
            s.appendFormatted("txabrt %u  ", tx_aborts);
            s.appendFormatted("err %u  ", errors);
            s.appendFormatted("lec %u  ", lec);
            return s;
        }

        bool operator!=(const IfaceStatus& rhs) const { return !operator==(rhs); }
        bool operator==(const IfaceStatus& rhs) const
        {
            return std::memcmp(this, &rhs, sizeof(IfaceStatus)) == 0;
        }
    };

    unsigned msg_cnt = 0;

    while (true)
    {
        for (int i = 0; i < 100; i++)
        {
            ::usleep(1000);
            for (int i = 0; i < 128; i++)
            {
                uavcan::CanFrame frame;
                uavcan::MonotonicTime ts_mono;
                uavcan::UtcTime ts_utc;
                uavcan::CanIOFlags flags = 0;
                static_cast<uavcan::ICanIface*>(app::can.driver.getIface(0))->receive(frame, ts_mono, ts_utc, flags);
                static_cast<uavcan::ICanIface*>(app::can.driver.getIface(1))->receive(frame, ts_mono, ts_utc, flags);
            }
        }

        app::ledSet(app::can.driver.hadActivity());

        uint8_t data[8] = {};

        uavcan::CanFrame frame(0 | uavcan::CanFrame::FlagEFF, data, 0);

        const auto tx_deadline = uavcan_stm32::clock::getMonotonic() + uavcan::MonotonicDuration::fromMSec(10);

        static_cast<uavcan::ICanIface*>(app::can.driver.getIface(0))->send(frame, tx_deadline,
                                                                           uavcan::CanIOFlagAbortOnError);

        frame.id += 1;
        static_cast<uavcan::ICanIface*>(app::can.driver.getIface(0))->send(frame, tx_deadline,
                                                                           uavcan::CanIOFlagAbortOnError);

        const IfaceStatus if0(*app::can.driver.getIface(0));
        const IfaceStatus if1(*app::can.driver.getIface(1));

        static IfaceStatus old_if0;
        static IfaceStatus old_if1;

        if (old_if0 != if0 ||
            old_if1 != if1)
        {
            lowsyslog("report #%u\n", msg_cnt);
            lowsyslog("if0: %s\n", if0.toString().c_str());
            lowsyslog("if1: %s\n", if1.toString().c_str());
        }

        old_if0 = if0;
        old_if1 = if1;
    }
}
