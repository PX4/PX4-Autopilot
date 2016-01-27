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

    static std::string makeErrorString(const std::string& descr, int use_errno)
    {
        return descr + " [errno " + std::to_string(use_errno) + " \"" + std::strerror(use_errno) + "\"]";
    }

public:
    explicit Exception(const std::string& descr, int use_errno = errno)
        : std::runtime_error(makeErrorString(descr, use_errno))
        , errno_(use_errno)
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
    AllIfacesDownException() : Exception("All ifaces are down", ENETDOWN) { }
};

}
