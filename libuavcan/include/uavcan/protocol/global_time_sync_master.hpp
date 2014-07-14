/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/node/publisher.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/util/lazy_constructor.hpp>
#include <uavcan/protocol/GlobalTimeSync.hpp>

namespace uavcan
{
/**
 * Please read the specs to learn how the time synchronization works.
 *
 * No more than one object of this class is allowed per node; otherwise a disaster is bound to happen.
 *
 * NOTE: In order for this class to work, the platform driver must implement
 *       CAN bus TX loopback with both UTC and monotonic timestamping.
 *
 * Ref. M. Gergeleit, H. Streich - "Implementing a Distributed High-Resolution Real-Time Clock using the CAN-Bus"
 *
 * TODO: Enforce max one master per node
 */
class UAVCAN_EXPORT GlobalTimeSyncMaster : protected LoopbackFrameListenerBase
{
    class IfaceMaster
    {
        Publisher<protocol::GlobalTimeSync> pub_;
        MonotonicTime iface_prev_pub_mono_;
        UtcTime prev_tx_utc_;
        const uint8_t iface_index_;

    public:
        IfaceMaster(INode& node, uint8_t iface_index)
            : pub_(node)
            , iface_index_(iface_index)
        {
            UAVCAN_ASSERT(iface_index < MaxCanIfaces);
        }

        int init();

        void setTxTimestamp(UtcTime ts);

        int publish(TransferID tid, MonotonicTime current_time);
    };

    INode& node_;
    LazyConstructor<IfaceMaster> iface_masters_[MaxCanIfaces];
    MonotonicTime prev_pub_mono_;
    DataTypeID dtid_;
    bool initialized_;

    virtual void handleLoopbackFrame(const RxFrame& frame);

    int getNextTransferID(TransferID& tid);

public:
    explicit GlobalTimeSyncMaster(INode& node)
        : LoopbackFrameListenerBase(node.getDispatcher())
        , node_(node)
        , initialized_(false)
    { }

    /**
     * Merely prepares the class to work, doesn't do anything else.
     * Must be called before the master can be used.
     * Returns negative error code.
     */
    int init();

    /**
     * Whether the master instance has been initialized.
     */
    bool isInitialized() const { return initialized_; }

    /**
     * Publishes one sync message.
     *
     * Every call to this method hints the master to publish the next sync message once. Exact time will
     * be obtained from the TX loopback timestamp field.
     *
     * This method shall be called with a proper interval - refer to the time sync message definition
     * for min/max interval values.
     *
     * Returns negative error code.
     */
    int publish();
};

}
