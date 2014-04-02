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

void ledSet(bool state)
{
    palWritePad(GPIO_PORT_LED, GPIO_PIN_LED, state);
}

int init()
{
    halInit();
    chibios_rt::System::init();
    sdStart(&STDOUT_SD, NULL);

    return 0;
}

__attribute__((noreturn))
void die(int status)
{
    lowsyslog("Now I am dead x_x %i\n", status);
    while (1) {
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
    }
}
