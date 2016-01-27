/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace uavcan_linux
{
/**
 * This is the root exception class for all exceptions that can be thrown from the libuavcan Linux driver.
 */
class Exception : public std::runtime_error
{
    const int errno_;

    static std::string makeErrorString(const std::string& descr)
    {
        return descr + " [errno " + std::to_string(errno) + " \"" + std::strerror(errno) + "\"]";
    }

public:
    explicit Exception(const std::string& descr)
        : std::runtime_error(makeErrorString(descr))
        , errno_(errno)
    { }

    /**
     * Returns standard UNIX errno value captured at the moment
     * when this exception object was constructed.
     */
    int getErrno() const { return errno_; }
};

/**
 * This exception is thrown when all available interfaces become down.
 */
class AllIfacesDownException : public Exception
{
public:
    AllIfacesDownException() : Exception("All ifaces are down") { }
};

}
