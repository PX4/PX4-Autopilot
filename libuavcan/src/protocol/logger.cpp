/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/protocol/logger.hpp>

namespace uavcan
{

const Logger::LogLevel Logger::LevelAboveAll;

int Logger::log(const protocol::debug::LogMessage& message)
{
    if (message.level.value >= level_)
    {
        const int res = logmsg_pub_.broadcast(message);
        if (external_sink_)
        {
            external_sink_->log(message);
        }
        return res;
    }
    return 0;
}

}
