/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include <uavcan/debug.hpp>

namespace uavcan
{

void GlobalTimeSyncSlave::adjustFromMsg(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg)
{
    assert(msg.prev_utc_usec > 0);
    const UtcDuration adjustment = UtcTime::fromUSec(msg.prev_utc_usec) - prev_ts_utc_;

    UAVCAN_TRACE("GlobalTimeSyncSlave", "Adjustment: usec=%lli snid=%i iface=%i",
                 static_cast<long long>(adjustment.toUSec()),
                 int(msg.getSrcNodeID().get()), int(msg.getIfaceIndex()));

    getSystemClock().adjustUtc(adjustment);
    last_adjustment_ts_ = msg.getMonotonicTimestamp();
    state_ = Update;
}

void GlobalTimeSyncSlave::updateFromMsg(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg)
{
    UAVCAN_TRACE("GlobalTimeSyncSlave", "Update: snid=%i iface=%i",
                 int(msg.getSrcNodeID().get()), int(msg.getIfaceIndex()));

    prev_ts_utc_      = msg.getUtcTimestamp();
    prev_ts_mono_     = msg.getMonotonicTimestamp();
    master_nid_       = msg.getSrcNodeID();
    prev_iface_index_ = msg.getIfaceIndex();
    state_             = Adjust;
}

void GlobalTimeSyncSlave::processMsg(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg)
{
    const MonotonicDuration time_since_prev_sync = msg.getMonotonicTimestamp() - prev_ts_mono_;
    assert(!time_since_prev_sync.isNegative());

    const bool needs_init = !master_nid_.isValid() || prev_ts_mono_.isZero();
    const bool switch_master = msg.getSrcNodeID() < master_nid_;
    const bool timeout = time_since_prev_sync.toMSec() > protocol::GlobalTimeSync::PUBLISHER_TIMEOUT_MS;

    if (switch_master || timeout || needs_init)
    {
        UAVCAN_TRACE("GlobalTimeSyncSlave", "Force update: needs_init=%i switch_master=%i timeout=%i",
                     int(needs_init), int(switch_master), int(timeout));
        updateFromMsg(msg);
    }
    else if (msg.getIfaceIndex() == prev_iface_index_ && msg.getSrcNodeID() == master_nid_)
    {
        if (state_ == Adjust && msg.prev_utc_usec > 0)
        {
            adjustFromMsg(msg);
        }
        else
        {
            updateFromMsg(msg);
        }
    }
    else
    {
        UAVCAN_TRACE("GlobalTimeSyncSlave", "Ignored: snid=%i iface=%i",
                     int(msg.getSrcNodeID().get()), int(msg.getIfaceIndex()));
    }
}

void GlobalTimeSyncSlave::handleGlobalTimeSync(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg)
{
    if (msg.getTransferType() == TransferTypeMessageBroadcast)
    {
        processMsg(msg);
    }
    else
    {
        UAVCAN_TRACE("GlobalTimeSyncSlave", "Invalid transfer type %i", int(msg.getTransferType()));
    }
}

int GlobalTimeSyncSlave::start()
{
    return sub_.start(GlobalTimeSyncCallback(this, &GlobalTimeSyncSlave::handleGlobalTimeSync));
}

bool GlobalTimeSyncSlave::isActive() const
{
    const MonotonicDuration since_prev_adj = getSystemClock().getMonotonic() - last_adjustment_ts_;
    return !last_adjustment_ts_.isZero() && since_prev_adj.toMSec() <= protocol::GlobalTimeSync::PUBLISHER_TIMEOUT_MS;
}

NodeID GlobalTimeSyncSlave::getMasterNodeID() const
{
    return isActive() ? master_nid_ : NodeID();
}

}
