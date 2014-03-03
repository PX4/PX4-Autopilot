/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <stdint.h>
#include <uavcan/internal/map.hpp>
#include <uavcan/internal/debug.hpp>
#include <uavcan/internal/transport/transfer.hpp>

namespace uavcan
{

UAVCAN_PACKED_BEGIN
class OutgoingTransferRegistryKey
{
    uint16_t data_type_id_;
    uint8_t transfer_type_;
    NodeID destination_node_id_;  ///< Not applicable for message broadcasting

public:
    OutgoingTransferRegistryKey()
    : data_type_id_(0xFFFF)
    , transfer_type_(0xFF)
    { }

    OutgoingTransferRegistryKey(uint16_t data_type_id, TransferType transfer_type, NodeID destination_node_id)
    : data_type_id_(data_type_id)
    , transfer_type_(transfer_type)
    , destination_node_id_(destination_node_id)
    {
        assert((transfer_type == TransferTypeMessageBroadcast) == destination_node_id.isBroadcast());

        /* Service response transfers must use the same Transfer ID as matching service request transfer,
         * so this registry is not applicable for service response transfers at all.
         */
        assert(transfer_type != TransferTypeServiceResponse);
    }

    bool operator==(const OutgoingTransferRegistryKey& rhs) const
    {
        return
            (data_type_id_        == rhs.data_type_id_) &&
            (transfer_type_       == rhs.transfer_type_) &&
            (destination_node_id_ == rhs.destination_node_id_);
    }
};
UAVCAN_PACKED_END


class IOutgoingTransferRegistry
{
public:
    virtual ~IOutgoingTransferRegistry() { }
    virtual TransferID* accessOrCreate(const OutgoingTransferRegistryKey& key, uint64_t new_monotonic_deadline) = 0;
    virtual void cleanup(uint64_t monotonic_deadline) = 0;
};


template <int NumStaticEntries>
class OutgoingTransferRegistry : public IOutgoingTransferRegistry, Noncopyable
{
UAVCAN_PACKED_BEGIN
    struct Value
    {
        uint64_t monotonic_deadline;
        TransferID tid;
        Value() : monotonic_deadline(0) { }
    };
UAVCAN_PACKED_END

    class DeadlineExpiredPredicate
    {
        const uint64_t ts_monotonic_;

    public:
        DeadlineExpiredPredicate(uint64_t ts_monotonic)
        : ts_monotonic_(ts_monotonic)
        { }

        bool operator()(const OutgoingTransferRegistryKey& key, const Value& value) const
        {
            (void)key;
            assert(value.monotonic_deadline > 0);
            return value.monotonic_deadline <= ts_monotonic_;
        }
    };

    Map<OutgoingTransferRegistryKey, Value, NumStaticEntries> map_;

public:
    OutgoingTransferRegistry(IAllocator& allocator)
    : map_(allocator)
    { }

    TransferID* accessOrCreate(const OutgoingTransferRegistryKey& key, uint64_t new_monotonic_deadline)
    {
        assert(new_monotonic_deadline > 0);
        Value* p = map_.access(key);
        if (p == NULL)
            p = map_.insert(key, Value());
        if (p == NULL)
            return NULL;
        p->monotonic_deadline = new_monotonic_deadline;
        return &p->tid;
    }

    void cleanup(uint64_t ts_monotonic)
    {
        map_.removeWhere(DeadlineExpiredPredicate(ts_monotonic));
    }
};

}
