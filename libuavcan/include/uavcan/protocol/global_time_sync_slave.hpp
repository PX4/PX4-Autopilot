/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/node/subscriber.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/protocol/GlobalTimeSync.hpp>

namespace uavcan
{

class GlobalTimeSyncSlave : Noncopyable
{
    typedef MethodBinder<GlobalTimeSyncSlave*,
        void (GlobalTimeSyncSlave::*)(const ReceivedDataStructure<protocol::GlobalTimeSync>&)> GlobalTimeSyncCallback;

    // Static buffers are explicitly disabled because time should never be unicasted.
    Subscriber<protocol::GlobalTimeSync, GlobalTimeSyncCallback, 2, 0> sub_;

    UtcTime prev_ts_utc_;
    MonotonicTime prev_ts_mono_;
    MonotonicTime last_adjustment_ts_;
    enum State { Update, Adjust } state_;
    NodeID master_nid_;
    TransferID prev_tid_;
    uint8_t prev_iface_index_;

    ISystemClock& getSystemClock() const { return sub_.getNode().getSystemClock(); }

    void adjustFromMsg(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg);

    void updateFromMsg(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg);

    void processMsg(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg);

    void handleGlobalTimeSync(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg);

public:
    GlobalTimeSyncSlave(INode& node)
    : sub_(node)
    , state_(Update)
    , prev_iface_index_(0xFF)
    { }

    int start();

    bool isActive() const;

    NodeID getMasterNodeID() const;

    MonotonicTime getLastAdjustmentTime() const { return last_adjustment_ts_; }
};

}
