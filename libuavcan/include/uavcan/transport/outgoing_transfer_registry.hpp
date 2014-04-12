/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <uavcan/stdint.hpp>
#include <uavcan/impl_constants.hpp>
#include <uavcan/map.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/transport/transfer.hpp>
#include <uavcan/time.hpp>

namespace uavcan
{

UAVCAN_PACKED_BEGIN
class UAVCAN_EXPORT OutgoingTransferRegistryKey
{
    DataTypeID data_type_id_;
    uint8_t transfer_type_;
    NodeID destination_node_id_;  ///< Not applicable for message broadcasting

public:
    OutgoingTransferRegistryKey()
        : transfer_type_(0xFF)
    { }

    OutgoingTransferRegistryKey(DataTypeID data_type_id, TransferType transfer_type, NodeID destination_node_id)
        : data_type_id_(data_type_id)
        , transfer_type_(transfer_type)
        , destination_node_id_(destination_node_id)
    {
        assert((transfer_type == TransferTypeMessageBroadcast) == destination_node_id.isBroadcast());
        /*
         * Service response transfers must use the same Transfer ID as matching service request transfer,
         * so this registry is not applicable for service response transfers at all.
         */
        assert(transfer_type != TransferTypeServiceResponse);
    }

    DataTypeID getDataTypeID() const { return data_type_id_; }
    TransferType getTransferType() const { return TransferType(transfer_type_); }

    bool operator==(const OutgoingTransferRegistryKey& rhs) const
    {
        return
            (data_type_id_        == rhs.data_type_id_) &&
            (transfer_type_       == rhs.transfer_type_) &&
            (destination_node_id_ == rhs.destination_node_id_);
    }

#if UAVCAN_TOSTRING
    std::string toString() const;
#endif
};
UAVCAN_PACKED_END


class UAVCAN_EXPORT IOutgoingTransferRegistry
{
public:
    virtual ~IOutgoingTransferRegistry() { }
    virtual TransferID* accessOrCreate(const OutgoingTransferRegistryKey& key, MonotonicTime new_deadline) = 0;
    virtual bool exists(DataTypeID dtid, TransferType tt) const = 0;
    virtual void cleanup(MonotonicTime deadline) = 0;
};


template <int NumStaticEntries>
class UAVCAN_EXPORT OutgoingTransferRegistry : public IOutgoingTransferRegistry, Noncopyable
{
    UAVCAN_PACKED_BEGIN
    struct Value
    {
        MonotonicTime deadline;
        TransferID tid;
    };
    UAVCAN_PACKED_END

    class DeadlineExpiredPredicate
    {
        const MonotonicTime ts_;

    public:
        DeadlineExpiredPredicate(MonotonicTime ts)
            : ts_(ts)
        { }

        bool operator()(const OutgoingTransferRegistryKey& key, const Value& value) const
        {
            (void)key;
            assert(!value.deadline.isZero());
            const bool expired = value.deadline <= ts_;
            if (expired)
            {
                UAVCAN_TRACE("OutgoingTransferRegistry", "Expired %s tid=%i",
                             key.toString().c_str(), int(value.tid.get()));
            }
            return expired;
        }
    };

    class ExistenceCheckingPredicate
    {
        const DataTypeID dtid_;
        const TransferType tt_;

    public:
        ExistenceCheckingPredicate(DataTypeID dtid, TransferType tt)
            : dtid_(dtid)
            , tt_(tt)
        { }

        bool operator()(const OutgoingTransferRegistryKey& key, const Value&) const
        {
            return dtid_ == key.getDataTypeID() && tt_ == key.getTransferType();
        }
    };

    Map<OutgoingTransferRegistryKey, Value, NumStaticEntries> map_;

public:
    OutgoingTransferRegistry(IPoolAllocator& allocator)
        : map_(allocator)
    { }

    TransferID* accessOrCreate(const OutgoingTransferRegistryKey& key, MonotonicTime new_deadline);

    bool exists(DataTypeID dtid, TransferType tt) const;

    void cleanup(MonotonicTime ts);
};

// ----------------------------------------------------------------------------

/*
 * OutgoingTransferRegistry<>
 */
template <int NumStaticEntries>
TransferID* OutgoingTransferRegistry<NumStaticEntries>::accessOrCreate(const OutgoingTransferRegistryKey& key,
                                                                       MonotonicTime new_deadline)
{
    assert(!new_deadline.isZero());
    Value* p = map_.access(key);
    if (p == NULL)
    {
        p = map_.insert(key, Value());
        if (p == NULL)
        {
            return NULL;
        }
        UAVCAN_TRACE("OutgoingTransferRegistry", "Created %s", key.toString().c_str());
    }
    p->deadline = new_deadline;
    return &p->tid;
}

template <int NumStaticEntries>
bool OutgoingTransferRegistry<NumStaticEntries>::exists(DataTypeID dtid, TransferType tt) const
{
    return NULL != map_.findFirstKey(ExistenceCheckingPredicate(dtid, tt));
}

template <int NumStaticEntries>
void OutgoingTransferRegistry<NumStaticEntries>::cleanup(MonotonicTime ts)
{
    map_.removeWhere(DeadlineExpiredPredicate(ts));
}

}
