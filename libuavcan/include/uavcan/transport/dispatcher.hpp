/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <uavcan/stdint.hpp>
#include <uavcan/transport/transfer_listener.hpp>
#include <uavcan/transport/outgoing_transfer_registry.hpp>
#include <uavcan/transport/can_io.hpp>
#include <uavcan/linked_list.hpp>

namespace uavcan
{

class Dispatcher : Noncopyable
{
    CanIOManager canio_;
    ISystemClock& sysclock_;
    IOutgoingTransferRegistry& outgoing_transfer_reg_;

    class ListenerRegister
    {
        LinkedListRoot<TransferListenerBase> list_;

        class DataTypeIDInsertionComparator
        {
            const DataTypeID id_;
        public:
            DataTypeIDInsertionComparator(DataTypeID id) : id_(id) { }
            bool operator()(const TransferListenerBase* listener) const
            {
                assert(listener);
                return id_ > listener->getDataTypeDescriptor().getID();
            }
        };

    public:
        enum Mode { UniqueListener, ManyListeners };

        bool add(TransferListenerBase* listener, Mode mode);
        void remove(TransferListenerBase* listener);
        void cleanup(MonotonicTime ts);
        void handleFrame(const RxFrame& frame);

        int getNumEntries() const { return list_.getLength(); }
    };

    ListenerRegister lmsg_;
    ListenerRegister lsrv_req_;
    ListenerRegister lsrv_resp_;

    NodeID self_node_id_;

    void handleFrame(const CanRxFrame& can_frame);

public:
    Dispatcher(ICanDriver& driver, IAllocator& allocator, ISystemClock& sysclock, IOutgoingTransferRegistry& otr)
    : canio_(driver, allocator, sysclock)
    , sysclock_(sysclock)
    , outgoing_transfer_reg_(otr)
    { }

    int spin(MonotonicTime deadline);

    /**
     * Refer to CanIOManager::send() for the parameter description
     */
    int send(const Frame& frame, MonotonicTime tx_deadline, MonotonicTime blocking_deadline, CanTxQueue::Qos qos);

    void cleanup(MonotonicTime ts);

    bool registerMessageListener(TransferListenerBase* listener);
    bool registerServiceRequestListener(TransferListenerBase* listener);
    bool registerServiceResponseListener(TransferListenerBase* listener);

    void unregisterMessageListener(TransferListenerBase* listener);
    void unregisterServiceRequestListener(TransferListenerBase* listener);
    void unregisterServiceResponseListener(TransferListenerBase* listener);

    int getNumMessageListeners()         const { return lmsg_.getNumEntries(); }
    int getNumServiceRequestListeners()  const { return lsrv_req_.getNumEntries(); }
    int getNumServiceResponseListeners() const { return lsrv_resp_.getNumEntries(); }

    IOutgoingTransferRegistry& getOutgoingTransferRegistry() { return outgoing_transfer_reg_; }

    NodeID getNodeID() const { return self_node_id_; }
    bool setNodeID(NodeID nid);

    const ISystemClock& getSystemClock() const { return sysclock_; }
    ISystemClock& getSystemClock() { return sysclock_; }
};

}
