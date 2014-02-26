/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cstdlib>
#include <cassert>
#include <uavcan/internal/data_type.hpp>
#include <uavcan/internal/transport/crc.hpp>
#include <uavcan/internal/transport/transfer.hpp>
#include <uavcan/internal/transport/dispatcher.hpp>

namespace uavcan
{

class TransferSender
{
    static const uint64_t DefaultMaxTransferInterval = 60 * 1000 * 1000;

    const uint64_t max_transfer_interval_;
    const DataTypeDescriptor& data_type_;
    const CanTxQueue::Qos qos_;
    const TransferCRC crc_base_;

    Dispatcher& dispatcher_;

public:
    TransferSender(Dispatcher& dispatcher, const DataTypeDescriptor& data_type, CanTxQueue::Qos qos,
                   uint64_t max_transfer_interval = DefaultMaxTransferInterval)
    : max_transfer_interval_(max_transfer_interval)
    , data_type_(data_type)
    , qos_(qos)
    , crc_base_(data_type.signature.toTransferCRC())
    , dispatcher_(dispatcher)
    { }

    /**
     * Send with explicit Transfer ID.
     * Should be used only for service responses, where response TID should match request TID.
     */
    int send(const uint8_t* payload, int payload_len, uint64_t monotonic_tx_deadline,
             uint64_t monotonic_blocking_deadline, TransferType transfer_type, NodeID dst_node_id,
             TransferID tid);

    /**
     * Send with automatic Transfer ID.
     * TID is managed by OutgoingTransferRegistry.
     */
    int send(const uint8_t* payload, int payload_len, uint64_t monotonic_tx_deadline,
             uint64_t monotonic_blocking_deadline, TransferType transfer_type, NodeID dst_node_id);
};

}
