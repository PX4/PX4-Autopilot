/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <stdint.h>
#include <uavcan/internal/transport/transfer_listener.hpp>
#include <uavcan/internal/transport/outgoing_transfer_registry.hpp>
#include <uavcan/internal/transport/can_io.hpp>
#include <uavcan/internal/linked_list.hpp>

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
            const uint16_t id_;
        public:
            DataTypeIDInsertionComparator(uint16_t id) : id_(id) { }
            bool operator()(const TransferListenerBase* listener) const
            {
                assert(listener);
                return id_ > listener->getDataTypeDescriptor().id;
            }
        };

    public:
        enum Mode { UNIQUE_LISTENER, MANY_LISTENERS };

        bool add(TransferListenerBase* listener, Mode mode);
        void remove(TransferListenerBase* listener);
        void cleanup(uint64_t ts_monotonic);
        void handleFrame(const RxFrame& frame);

        int getNumEntries() const { return list_.getLength(); }
    };

    ListenerRegister lmsg_;
    ListenerRegister lsrv_req_;
    ListenerRegister lsrv_resp_;

    const NodeID self_node_id_;

    void handleFrame(const CanRxFrame& can_frame);

public:
    Dispatcher(ICanDriver& driver, IAllocator& allocator, ISystemClock& sysclock, IOutgoingTransferRegistry& otr,
               NodeID self_node_id)
    : canio_(driver, allocator, sysclock)
    , sysclock_(sysclock)
    , outgoing_transfer_reg_(otr)
    , self_node_id_(self_node_id)
    { }

    int spin(uint64_t monotonic_deadline);

    /**
     * Refer to CanIOManager::send() for the parameter description
     */
    int send(const Frame& frame, uint64_t monotonic_tx_deadline, uint64_t monotonic_blocking_deadline,
             CanTxQueue::Qos qos);

    void cleanup(uint64_t ts_monotonic);

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

    NodeID getSelfNodeID() const { return self_node_id_; }
};

}
