/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cerrno>
#include <stdexcept>

namespace uavcan_linux
{
/**
 * This is the root exception class for all exceptions that can be thrown from the libuavcan Linux driver.
 */
class Exception : public std::runtime_error
{
    const int errno_;

public:
    explicit Exception(const std::string& descr)
        : std::runtime_error(descr)
        , errno_(errno)
    { }

    /**
     * Returns standard UNIX errno value captured at the moment
     * when this exception object was constructed.
     */
    int getErrno() const { return errno_; }
};

}
