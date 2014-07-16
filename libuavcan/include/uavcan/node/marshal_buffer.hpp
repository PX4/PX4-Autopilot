/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/build_config.hpp>
#include <uavcan/transport/transfer.hpp>
#include <uavcan/transport/transfer_buffer.hpp>

namespace uavcan
{
/**
 * Abstract temporary buffer for data marshalling.
 */
class UAVCAN_EXPORT IMarshalBuffer : public ITransferBuffer
{
public:
    virtual const uint8_t* getDataPtr() const = 0;
    virtual unsigned getDataLength() const = 0;
};

/**
 * Abstract provider of abstract buffer for data marshalling.
 */
class UAVCAN_EXPORT IMarshalBufferProvider
{
public:
    virtual ~IMarshalBufferProvider() { }

    /**
     * Returns pointer to abstract buffer for data marshalling,
     * but does not transfer ownership.
     * If the requested buffer size is larger than available,
     * null pointer will be returned.
     * @param size  Maximum buffer size needed for marshaling.
     * @return      Pointer to the buffer, or null pointer if
     *              the requested size is too large.
     */
    virtual IMarshalBuffer* getBuffer(unsigned size) = 0;
};

/**
 * Default implementation of marshal buffer provider.
 * This implementation provides the buffer large enough to
 * serialize any UAVCAN data structure.
 */
template <unsigned MaxSize_ = MaxTransferPayloadLen>
class UAVCAN_EXPORT MarshalBufferProvider : public IMarshalBufferProvider
{
    class Buffer : public IMarshalBuffer
    {
        StaticTransferBuffer<MaxSize_> buf_;

        virtual int read(unsigned offset, uint8_t* data, unsigned len) const
        {
            return buf_.read(offset, data, len);
        }

        virtual int write(unsigned offset, const uint8_t* data, unsigned len)
        {
            return buf_.write(offset, data, len);
        }

        virtual const uint8_t* getDataPtr() const { return buf_.getRawPtr(); }

        virtual unsigned getDataLength() const { return buf_.getMaxWritePos(); }

    public:
        void reset() { buf_.reset(); }
    };

    Buffer buffer_;

public:
    enum { MaxSize = MaxSize_ };

    virtual IMarshalBuffer* getBuffer(unsigned size)
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
