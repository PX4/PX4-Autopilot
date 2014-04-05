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

typedef uavcan::Node<4096> Node;

Node& getNode()
{
    static Node node(can.driver, uavcan_stm32::SystemClock::instance());
    return node;
}

void ledSet(bool state)
{
    palWritePad(GPIO_PORT_LED, GPIO_PIN_LED, state);
}

int init()
{
    int res = 0;

    halInit();
    chibios_rt::System::init();
    sdStart(&STDOUT_SD, NULL);

    res = can.init(1000000);
    if (res < 0)
    {
        goto leave;
    }

leave:
    return res;
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

class : public chibios_rt::BaseStaticThread<2048>
{
public:
    msg_t main()
    {
        /*
         * Setting up the node parameters
         */
        Node& node = app::getNode();

        node.setNodeID(64);
        node.setName("org.uavcan.stm32_test_stm32f107");

        /*
         * Initializing the UAVCAN node - this may take a while
         */
        while (true)
        {
            uavcan::NodeInitializationResult init_result;
            const int uavcan_start_res = node.start(init_result);

            if (uavcan_start_res < 0)
            {
                lowsyslog("Node initialization failure: %i, will try agin soon\n", uavcan_start_res);
            }
            else if (!init_result.isOk())
            {
                lowsyslog("Network conflict with %u, will try again soon\n", init_result.conflicting_node.get());
            }
            else
            {
                break;
            }
            ::sleep(3);
        }

        /*
         * Main loop
         */
        lowsyslog("UAVCAN node started\n");
        node.setStatusOk();
        while (true)
        {
            const int spin_res = node.spin(uavcan::MonotonicDuration::fromMSec(100));
            if (spin_res < 0)
            {
                lowsyslog("Spin failure: %i\n", spin_res);
            }
        }
        return msg_t();
    }
} uavcan_node_thread;

}
}

int main()
{
    const int init_res = app::init();
    if (init_res != 0)
    {
        app::die(init_res);
    }

    lowsyslog("Starting the UAVCAN thread\n");
    app::uavcan_node_thread.start(LOWPRIO);

    while (true)
    {
        app::ledSet(false);
        sleep(1);
        app::ledSet(true);
        sleep(1);
    }
}
