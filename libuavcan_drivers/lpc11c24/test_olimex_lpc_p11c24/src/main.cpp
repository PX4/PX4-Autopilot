/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdio>
#include <board.hpp>
#include <chip.h>
#include <uavcan_lpc11c24/uavcan_lpc11c24.hpp>

namespace
{

typedef uavcan::Node<3584> Node;

Node& getNode()
{
    static Node node(uavcan_lpc11c24::CanDriver::instance(), uavcan_lpc11c24::SystemClock::instance());
    return node;
}

__attribute__((noreturn))
void die()
{
    while (true) { }
}

void init()
{
    if (uavcan_lpc11c24::CanDriver::instance().init(1000000) < 0)
    {
        die();
    }

    getNode().setNodeID(72);
    getNode().setName("org.uavcan.lpc11c24_test");

    while (getNode().start() < 0)
    {
    }
}

}

int main()
{
    init();

    getNode().setStatusOk();

    while (true)
    {
        const int res = getNode().spin(uavcan::MonotonicDuration::fromMSec(25));
        board::setErrorLed(res < 0);
        board::setStatusLed(uavcan_lpc11c24::CanDriver::instance().hadActivity());
    }
}
