/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cerrno>
#include <stdexcept>

namespace uavcan_linux
{

class Exception : public std::runtime_error
{
    int errno_;

public:
    Exception(const char* descr)
        : std::runtime_error(descr)
        , errno_(errno)
    { }

    int getErrno() const { return errno_; }
};

}
