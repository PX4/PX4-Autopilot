/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/build_config.hpp>
#include <uavcan/stdint.hpp>

namespace uavcan
{
/**
 * API for transfer buffer users.
 */
class UAVCAN_EXPORT ITransferBuffer
{
public:
    virtual ~ITransferBuffer() { }

    virtual int read(unsigned offset, uint8_t* data, unsigned len) const = 0;
    virtual int write(unsigned offset, const uint8_t* data, unsigned len) = 0;
};

}
