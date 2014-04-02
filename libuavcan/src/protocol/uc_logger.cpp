/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdlib>
#include <uavcan/protocol/logger.hpp>

namespace uavcan
{

const Logger::LogLevel Logger::LevelAboveAll;

Logger::LogLevel Logger::getExternalSinkLevel() const
{
    return (external_sink_ == NULL) ? LevelAboveAll : external_sink_->getLogLevel();
}

int Logger::init()
{
    return logmsg_pub_.init();
}

int Logger::log(const protocol::debug::LogMessage& message)
{
    int retval = 0;
    if (message.level.value >= getExternalSinkLevel())
    {
        external_sink_->log(message);
    }
    if (message.level.value >= level_)
    {
        retval = logmsg_pub_.broadcast(message);
    }
    return retval;
}

}
