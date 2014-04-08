/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/time.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>
#include <uavcan/marshal/char_array_formatter.hpp>
#include <uavcan/node/publisher.hpp>

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

namespace uavcan
{

class UAVCAN_EXPORT ILogSink
{
public:
    typedef typename StorageType<typename protocol::debug::LogLevel::FieldTypes::value>::Type LogLevel;

    virtual ~ILogSink() { }

    /**
     * Logger will not sink messages with level lower than returned by this method.
     */
    virtual LogLevel getLogLevel() const { return protocol::debug::LogLevel::DEBUG; }

    /**
     * Logger will call this method for every log message with level not less than the current level of this sink.
     */
    virtual void log(const protocol::debug::LogMessage& message) = 0;
};


class UAVCAN_EXPORT Logger
{
public:
    typedef ILogSink::LogLevel LogLevel;

    static const LogLevel LevelAboveAll = (1 << protocol::debug::LogLevel::FieldTypes::value::BitLen) - 1;

private:
    enum { DefaultTxTimeoutMs = 2000 };

    Publisher<protocol::debug::LogMessage> logmsg_pub_;
    protocol::debug::LogMessage msg_buf_;
    LogLevel level_;
    ILogSink* external_sink_;

    LogLevel getExternalSinkLevel() const;

public:
    explicit Logger(INode& node)
        : logmsg_pub_(node)
        , external_sink_(NULL)
    {
        level_ = protocol::debug::LogLevel::ERROR;
        setTxTimeout(MonotonicDuration::fromMSec(DefaultTxTimeoutMs));
        assert(getTxTimeout() == MonotonicDuration::fromMSec(DefaultTxTimeoutMs));
    }

    int init();

    int log(const protocol::debug::LogMessage& message);

    LogLevel getLevel() const { return level_; }
    void setLevel(LogLevel level) { level_ = level; }

    ILogSink* getExternalSink() const { return external_sink_; }
    void setExternalSink(ILogSink* sink) { external_sink_ = sink; }

    MonotonicDuration getTxTimeout() const { return logmsg_pub_.getTxTimeout(); }
    void setTxTimeout(MonotonicDuration val) { logmsg_pub_.setTxTimeout(val); }

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11

    template <typename... Args>
    int log(LogLevel level, const char* source, const char* format, Args... args);

    template <typename... Args>
    inline int logDebug(const char* source, const char* format, Args... args)
    {
        return log(protocol::debug::LogLevel::DEBUG, source, format, args...);
    }

    template <typename... Args>
    inline int logInfo(const char* source, const char* format, Args... args)
    {
        return log(protocol::debug::LogLevel::INFO, source, format, args...);
    }

    template <typename... Args>
    inline int logWarning(const char* source, const char* format, Args... args)
    {
        return log(protocol::debug::LogLevel::WARNING, source, format, args...);
    }

    template <typename... Args>
    inline int logError(const char* source, const char* format, Args... args)
    {
        return log(protocol::debug::LogLevel::ERROR, source, format, args...);
    }

#else

    int log(LogLevel level, const char* source, const char* text);

    int logDebug(const char* source, const char* text)
    {
        return log(protocol::debug::LogLevel::DEBUG, source, text);
    }

    int logInfo(const char* source, const char* text)
    {
        return log(protocol::debug::LogLevel::INFO, source, text);
    }

    int logWarning(const char* source, const char* text)
    {
        return log(protocol::debug::LogLevel::WARNING, source, text);
    }

    int logError(const char* source, const char* text)
    {
        return log(protocol::debug::LogLevel::ERROR, source, text);
    }

#endif
};

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11

template <typename... Args>
int Logger::log(LogLevel level, const char* source, const char* format, Args... args)
{
    if (level >= level_ || level >= getExternalSinkLevel())
    {
        msg_buf_.level.value = level;
        msg_buf_.source = source;
        msg_buf_.text.clear();
        CharArrayFormatter<typename protocol::debug::LogMessage::FieldTypes::text> formatter(msg_buf_.text);
        formatter.write(format, args...);
        return log(msg_buf_);
    }
    return 0;
}

#endif

}
