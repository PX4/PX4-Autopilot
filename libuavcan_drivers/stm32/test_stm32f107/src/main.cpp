/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <unistd.h>
#include <crdr_chibios/sys/sys.h>
#include <uavcan_stm32/uavcan_stm32.hpp>

namespace app
{
namespace
{

uavcan_stm32::CanInitHelper<> can;

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

__attribute__((noreturn))
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


uavcan_stm32::Event led_turn_off_event;

class : public chibios_rt::BaseStaticThread<512>
{
    msg_t main()
    {
        while (true)
        {
            led_turn_off_event.wait(uavcan::MonotonicDuration::getInfinite());
            ledSet(false);
        }
        return msg_t();
    }
} led_turn_off_executor;

}
}

int main()
{
    const int init_res = app::init();
    if (init_res != 0)
    {
        app::die(init_res);
    }

    app::led_turn_off_executor.start(LOWPRIO);

    puts("Hello world");

    while (true)
    {
        sleep(1);
        app::ledSet(true);
        sleep(1);
        app::led_turn_off_event.signal();

        printf("Mono clock: %lu  %lu\n",
               static_cast<unsigned long>(uavcan_stm32::SystemClock::instance().getMonotonic().toUSec()),
               static_cast<unsigned long>(uavcan_stm32::SystemClock::instance().getMonotonic().toMSec()));

        printf("UTC  clock: %lu\n",
               static_cast<unsigned long>(uavcan_stm32::SystemClock::instance().getUtc().toUSec()));

        if (uavcan_stm32::SystemClock::instance().getMonotonic().toMSec() / 1000 == 10)
        {
            uavcan_stm32::SystemClock::instance().adjustUtc(uavcan::UtcDuration::fromMSec(10000));
        }

        uavcan::CanFrame frame;
        const uavcan::MonotonicTime deadline =
            uavcan_stm32::clock::getMonotonic() + uavcan::MonotonicDuration::fromMSec(10);

        frame.id = 123 | uavcan::CanFrame::FlagEFF;
        frame.data[0] = 42;
        frame.dlc = 2;

        const int send_res =
            static_cast<uavcan::ICanIface*>(app::can.driver.getIface(0))->send(frame, deadline,
                                                                               uavcan::CanIOFlagLoopback);
        printf("send_res=%i errcnt=%lu hwerr=%i\n", send_res,
               static_cast<unsigned long>(app::can.driver.getIface(0)->getErrorCount()),
               static_cast<int>(app::can.driver.getIface(0)->yieldLastHardwareErrorCode()));

        while (true)
        {
            uavcan::CanRxFrame rx_frame;
            uavcan::CanIOFlags flags = 0;

            const int recv_res =
                static_cast<uavcan::ICanIface*>(app::can.driver.getIface(0))->receive(rx_frame, rx_frame.ts_mono,
                                                                                      rx_frame.ts_utc, flags);

            printf("recv_res=%i flg=%u canid=%lu dlc=%u data=[%02x %02x %02x %02x %02x %02x %02x %02x]\n",
                   recv_res, unsigned(flags), rx_frame.id & uavcan::CanFrame::MaskExtID, int(rx_frame.dlc),
                   int(rx_frame.data[0]), int(rx_frame.data[1]), int(rx_frame.data[2]), int(rx_frame.data[3]),
                   int(rx_frame.data[4]), int(rx_frame.data[5]), int(rx_frame.data[6]), int(rx_frame.data[7]));

            if (recv_res <= 0)
            {
                break;
            }
        }
    }
}
