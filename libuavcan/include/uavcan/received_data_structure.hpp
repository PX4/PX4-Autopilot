/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <uavcan/internal/transport/transfer_listener.hpp>

namespace uavcan
{

template <typename DataType_>
class ReceivedDataStructure : public DataType_
{
    const IncomingTransfer* transfer_;

    template <typename Ret, Ret (IncomingTransfer::*Fun)() const>
    Ret safeget() const
    {
        if (!transfer_)
        {
            assert(0);
            return Ret();
        }
        return (transfer_->*Fun)();
    }

protected:
    ReceivedDataStructure() : transfer_(NULL) { }

    void setTransfer(const IncomingTransfer* transfer)
    {
        assert(transfer);
        transfer_ = transfer;
    }

public:
    typedef DataType_ DataType;

    uint64_t getMonotonicTimestamp() const { return safeget<uint64_t, &IncomingTransfer::getMonotonicTimestamp>(); }
    uint64_t getUtcTimestamp()       const { return safeget<uint64_t, &IncomingTransfer::getUtcTimestamp>(); }
    TransferType getTransferType()   const { return safeget<TransferType, &IncomingTransfer::getTransferType>(); }
    TransferID getTransferID()       const { return safeget<TransferID, &IncomingTransfer::getTransferID>(); }
    NodeID getSrcNodeID()            const { return safeget<NodeID, &IncomingTransfer::getSrcNodeID>(); }
};

}
