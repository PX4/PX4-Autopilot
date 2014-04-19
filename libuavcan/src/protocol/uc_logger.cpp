/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/protocol/logger.hpp>
#include <cstdlib>

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

#if UAVCAN_CPP_VERSION < UAVCAN_CPP11

int Logger::log(LogLevel level, const char* source, const char* text)
{
    if (level >= level_ || level >= getExternalSinkLevel())
    {
        msg_buf_.level.value = level;
        msg_buf_.source = source;
        msg_buf_.text = text;
        return log(msg_buf_);
    }
    return 0;
}

#endif

}
