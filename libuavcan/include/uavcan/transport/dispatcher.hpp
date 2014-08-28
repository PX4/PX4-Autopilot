/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <uavcan/error.hpp>
#include <uavcan/stdint.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/transport/perf_counter.hpp>
#include <uavcan/transport/transfer_listener.hpp>
#include <uavcan/transport/outgoing_transfer_registry.hpp>
#include <uavcan/transport/can_io.hpp>
#include <uavcan/util/linked_list.hpp>

namespace uavcan
{

class UAVCAN_EXPORT Dispatcher;

/**
 * Inherit this class to receive notifications about all TX CAN frames that were transmitted with the loopback flag.
 */
class UAVCAN_EXPORT LoopbackFrameListenerBase : public LinkedListNode<LoopbackFrameListenerBase>, Noncopyable
{
    Dispatcher& dispatcher_;

protected:
    explicit LoopbackFrameListenerBase(Dispatcher& dispatcher)
        : dispatcher_(dispatcher)
    { }

    virtual ~LoopbackFrameListenerBase() { stopListening(); }

    void startListening();
    void stopListening();
    bool isListening() const;

    Dispatcher& getDispatcher() { return dispatcher_; }

public:
    virtual void handleLoopbackFrame(const RxFrame& frame) = 0;
};


class UAVCAN_EXPORT LoopbackFrameListenerRegistry : Noncopyable
{
    LinkedListRoot<LoopbackFrameListenerBase> listeners_;

public:
    void add(LoopbackFrameListenerBase* listener);
    void remove(LoopbackFrameListenerBase* listener);
    bool doesExist(const LoopbackFrameListenerBase* listener) const;
    unsigned getNumListeners() const { return listeners_.getLength(); }

    void invokeListeners(RxFrame& frame);
};

/**
 * This class performs low-level CAN frame routing.
 */
class UAVCAN_EXPORT Dispatcher : Noncopyable
{
    CanIOManager canio_;
    ISystemClock& sysclock_;
    IOutgoingTransferRegistry& outgoing_transfer_reg_;
    TransferPerfCounter perf_;

    class ListenerRegistry
    {
        LinkedListRoot<TransferListenerBase> list_;

        class DataTypeIDInsertionComparator
        {
            const DataTypeID id_;
        public:
            explicit DataTypeIDInsertionComparator(DataTypeID id) : id_(id) { }
            bool operator()(const TransferListenerBase* listener) const
            {
                UAVCAN_ASSERT(listener);
                return id_ > listener->getDataTypeDescriptor().getID();
            }
        };

    public:
        enum Mode { UniqueListener, ManyListeners };

        bool add(TransferListenerBase* listener, Mode mode);
        void remove(TransferListenerBase* listener);
        bool exists(DataTypeID dtid) const;
        void cleanup(MonotonicTime ts);
        void handleFrame(const RxFrame& frame);

        unsigned getNumEntries() const { return list_.getLength(); }

        const LinkedListRoot<TransferListenerBase>& getList() const { return list_; }
    };

    ListenerRegistry lmsg_;
    ListenerRegistry lsrv_req_;
    ListenerRegistry lsrv_resp_;

    LoopbackFrameListenerRegistry loopback_listeners_;

    NodeID self_node_id_;
    bool self_node_id_is_set_;

    void handleFrame(const CanRxFrame& can_frame);
    void handleLoopbackFrame(const CanRxFrame& can_frame);

public:
    Dispatcher(ICanDriver& driver, IPoolAllocator& allocator, ISystemClock& sysclock, IOutgoingTransferRegistry& otr)
        : canio_(driver, allocator, sysclock)
        , sysclock_(sysclock)
        , outgoing_transfer_reg_(otr)
        , self_node_id_is_set_(false)
    { }

    int spin(MonotonicTime deadline);

    /**
     * Refer to CanIOManager::send() for the parameter description
     */
    int send(const Frame& frame, MonotonicTime tx_deadline, MonotonicTime blocking_deadline, CanTxQueue::Qos qos,
             CanIOFlags flags, uint8_t iface_mask);

    void cleanup(MonotonicTime ts);

    bool registerMessageListener(TransferListenerBase* listener);
    bool registerServiceRequestListener(TransferListenerBase* listener);
    bool registerServiceResponseListener(TransferListenerBase* listener);

    void unregisterMessageListener(TransferListenerBase* listener);
    void unregisterServiceRequestListener(TransferListenerBase* listener);
    void unregisterServiceResponseListener(TransferListenerBase* listener);

    bool hasSubscriber(DataTypeID dtid) const;
    bool hasPublisher(DataTypeID dtid) const;
    bool hasServer(DataTypeID dtid) const;

    unsigned getNumMessageListeners()         const { return lmsg_.getNumEntries(); }
    unsigned getNumServiceRequestListeners()  const { return lsrv_req_.getNumEntries(); }
    unsigned getNumServiceResponseListeners() const { return lsrv_resp_.getNumEntries(); }

    /**
     * These methods can be used to retreive lists of messages, service requests and service responses the
     * dispatcher is currently listening to.
     * Note that the list of service response listeners is very volatile, because a response listener will be
     * removed from this list as soon as the corresponding service call is complete.
     * @{
     */
    const LinkedListRoot<TransferListenerBase>& getListOfMessageListeners() const
    {
        return lmsg_.getList();
    }
    const LinkedListRoot<TransferListenerBase>& getListOfServiceRequestListeners() const
    {
        return lsrv_req_.getList();
    }
    const LinkedListRoot<TransferListenerBase>& getListOfServiceResponseListeners() const
    {
        return lsrv_resp_.getList();
    }
    /**
     * @}
     */

    IOutgoingTransferRegistry& getOutgoingTransferRegistry() { return outgoing_transfer_reg_; }

    LoopbackFrameListenerRegistry& getLoopbackFrameListenerRegistry() { return loopback_listeners_; }

    /**
     * Node ID can be set only once.
     * Non-unicast Node ID puts the node into passive mode.
     */
    NodeID getNodeID() const { return self_node_id_; }
    bool setNodeID(NodeID nid);

    /**
     * Refer to the specs to learn more about passive mode.
     */
    bool isPassiveMode() const { return !getNodeID().isUnicast(); }

    const ISystemClock& getSystemClock() const { return sysclock_; }
    ISystemClock& getSystemClock() { return sysclock_; }

    const CanIOManager& getCanIOManager() const { return canio_; }

    const TransferPerfCounter& getTransferPerfCounter() const { return perf_; }
    TransferPerfCounter& getTransferPerfCounter() { return perf_; }
};

}
