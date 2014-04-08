/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/impl_constants.hpp>
#include <uavcan/transport/transfer.hpp>
#include <uavcan/transport/transfer_buffer.hpp>

namespace uavcan
{

class UAVCAN_EXPORT IMarshalBuffer : public ITransferBuffer
{
public:
    virtual const uint8_t* getDataPtr() const = 0;
    virtual unsigned getDataLength() const = 0;
};


class UAVCAN_EXPORT IMarshalBufferProvider
{
public:
    virtual ~IMarshalBufferProvider() { }
    virtual IMarshalBuffer* getBuffer(unsigned size) = 0;
};


template <unsigned MaxSize_ = MaxTransferPayloadLen>
class UAVCAN_EXPORT MarshalBufferProvider : public IMarshalBufferProvider
{
    class Buffer : public IMarshalBuffer
    {
        StaticTransferBuffer<MaxSize_> buf_;

        int read(unsigned offset, uint8_t* data, unsigned len) const
        {
            return buf_.read(offset, data, len);
        }

        int write(unsigned offset, const uint8_t* data, unsigned len)
        {
            return buf_.write(offset, data, len);
        }

        const uint8_t* getDataPtr() const { return buf_.getRawPtr(); }

        unsigned getDataLength() const { return buf_.getMaxWritePos(); }

    public:
        void reset() { buf_.reset(); }
    };

    Buffer buffer_;

public:
    enum { MaxSize = MaxSize_ };

    IMarshalBuffer* getBuffer(unsigned size)
    {
        if (size > MaxSize)
        {
            return NULL;
        }
        buffer_.reset();
        return &buffer_;
    }
};

}
