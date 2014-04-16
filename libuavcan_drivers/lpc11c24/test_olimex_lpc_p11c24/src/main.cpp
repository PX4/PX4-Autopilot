/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdio>
#include <board.hpp>
#include <chip.h>
#include <uavcan_lpc11c24/uavcan_lpc11c24.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include <uavcan/protocol/logger.hpp>

namespace
{

typedef uavcan::Node<3136> Node;

Node& getNode()
{
    static Node node(uavcan_lpc11c24::CanDriver::instance(), uavcan_lpc11c24::SystemClock::instance());
    return node;
}

uavcan::GlobalTimeSyncSlave& getTimeSyncSlave()
{
    static uavcan::GlobalTimeSyncSlave tss(getNode());
    return tss;
}

uavcan::Logger& getLogger()
{
    static uavcan::Logger logger(getNode());
    return logger;
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

    while (getTimeSyncSlave().start() < 0)
    {
    }

    while (getLogger().init() < 0)
    {
    }
    getLogger().setLevel(uavcan::protocol::debug::LogLevel::DEBUG);
}

void reverse(char* s)
{
    for (int i = 0, j = std::strlen(s) - 1; i < j; i++, j--)
    {
        const char c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}

void lltoa(long long n, char buf[24])
{
    const short sign = (n < 0) ? -1 : 1;
    if (sign < 0)
    {
        n = -n;
    }
    unsigned i = 0;
    do
    {
        buf[i++] = n % 10 + '0';
    }
    while ((n /= 10) > 0);
    if (sign < 0)
    {
        buf[i++] = '-';
    }
    buf[i] = '\0';
    reverse(buf);
}

}

int main()
{
    init();

    getNode().setStatusOk();

    uavcan::MonotonicTime prev_log_at;

    while (true)
    {
        const int res = getNode().spin(uavcan::MonotonicDuration::fromMSec(25));
        board::setErrorLed(res < 0);
        board::setStatusLed(uavcan_lpc11c24::CanDriver::instance().hadActivity());

        const auto ts = uavcan_lpc11c24::clock::getMonotonic();
        if ((ts - prev_log_at).toMSec() >= 1000)
        {
            prev_log_at = ts;

            // We don't want to use formatting functions provided by libuavcan because they rely on std::snprintf()
            char buf[24];
            lltoa(uavcan_lpc11c24::clock::getPrevUtcAdjustment().toUSec(), buf);
            buf[sizeof(buf) - 1] = '\0';

            // ...hence we need to construct the message manually:
            uavcan::protocol::debug::LogMessage logmsg;
            logmsg.level.value = uavcan::protocol::debug::LogLevel::INFO;
            logmsg.source = "app";
            logmsg.text = buf;
            (void)getLogger().log(logmsg);
        }
    }
}
