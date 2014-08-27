/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cstdlib>
#include <cassert>
#include <uavcan/build_config.hpp>
#include <uavcan/error.hpp>
#include <uavcan/data_type.hpp>
#include <uavcan/transport/crc.hpp>
#include <uavcan/transport/transfer.hpp>
#include <uavcan/transport/dispatcher.hpp>

namespace uavcan
{

class UAVCAN_EXPORT TransferSender
{
    const MonotonicDuration max_transfer_interval_;
    const DataTypeDescriptor& data_type_;
    const CanTxQueue::Qos qos_;
    const TransferCRC crc_base_;
    CanIOFlags flags_;
    uint8_t iface_mask_;

    Dispatcher& dispatcher_;

    void registerError();

public:
    enum { AllIfacesMask = 0xFF };

    static MonotonicDuration getDefaultMaxTransferInterval()
    {
        return MonotonicDuration::fromMSec(60 * 1000);
    }

    TransferSender(Dispatcher& dispatcher, const DataTypeDescriptor& data_type, CanTxQueue::Qos qos,
                   MonotonicDuration max_transfer_interval = getDefaultMaxTransferInterval())
        : max_transfer_interval_(max_transfer_interval)
        , data_type_(data_type)
        , qos_(qos)
        , crc_base_(data_type.getSignature().toTransferCRC())
        , flags_(CanIOFlags(0))
        , iface_mask_(AllIfacesMask)
        , dispatcher_(dispatcher)
    { }

    CanIOFlags getCanIOFlags() const { return flags_; }
    void setCanIOFlags(CanIOFlags flags) { flags_ = flags; }

    uint8_t getIfaceMask() const { return iface_mask_; }
    void setIfaceMask(uint8_t iface_mask)
    {
        UAVCAN_ASSERT(iface_mask);
        iface_mask_ = iface_mask;
    }

    /**
     * Send with explicit Transfer ID.
     * Should be used only for service responses, where response TID should match request TID.
     */
    int send(const uint8_t* payload, unsigned payload_len, MonotonicTime tx_deadline,
             MonotonicTime blocking_deadline, TransferType transfer_type, NodeID dst_node_id,
             TransferID tid);

    /**
     * Send with automatic Transfer ID.
     * TID is managed by OutgoingTransferRegistry.
     */
    int send(const uint8_t* payload, unsigned payload_len, MonotonicTime tx_deadline,
             MonotonicTime blocking_deadline, TransferType transfer_type, NodeID dst_node_id);
};

}
