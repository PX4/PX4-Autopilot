/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/protocol/global_time_sync_slave.hpp>
#include <uavcan/debug.hpp>
#include <cassert>

namespace uavcan
{

void GlobalTimeSyncSlave::adjustFromMsg(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg)
{
    UAVCAN_ASSERT(msg.prev_utc_usec > 0);
    const UtcDuration adjustment = UtcTime::fromUSec(msg.prev_utc_usec) - prev_ts_utc_;

    UAVCAN_TRACE("GlobalTimeSyncSlave", "Adjustment: usec=%lli snid=%i iface=%i suppress=%i",
                 static_cast<long long>(adjustment.toUSec()),
                 int(msg.getSrcNodeID().get()), int(msg.getIfaceIndex()), int(suppressed_));

    if (!suppressed_)
    {
        getSystemClock().adjustUtc(adjustment);
    }
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
    prev_tid_         = msg.getTransferID();
    state_            = Adjust;
}

void GlobalTimeSyncSlave::processMsg(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg)
{
    const MonotonicDuration since_prev_msg = msg.getMonotonicTimestamp() - prev_ts_mono_;
    UAVCAN_ASSERT(!since_prev_msg.isNegative());

    const bool needs_init = !master_nid_.isValid() || prev_ts_mono_.isZero();
    const bool switch_master = msg.getSrcNodeID() < master_nid_;
    const bool pub_timeout = since_prev_msg.toMSec() > protocol::GlobalTimeSync::PUBLISHER_TIMEOUT_MS;

    if (switch_master || pub_timeout || needs_init)
    {
        UAVCAN_TRACE("GlobalTimeSyncSlave", "Force update: needs_init=%i switch_master=%i pub_timeout=%i",
                     int(needs_init), int(switch_master), int(pub_timeout));
        updateFromMsg(msg);
    }
    else if (msg.getIfaceIndex() == prev_iface_index_ && msg.getSrcNodeID() == master_nid_)
    {
        if (state_ == Adjust)
        {
            const bool msg_invalid = msg.prev_utc_usec == 0;
            const bool wrong_tid = prev_tid_.computeForwardDistance(msg.getTransferID()) != 1;
            const bool wrong_timing = since_prev_msg.toMSec() > protocol::GlobalTimeSync::MAX_PUBLICATION_PERIOD_MS;
            if (msg_invalid || wrong_tid || wrong_timing)
            {
                UAVCAN_TRACE("GlobalTimeSyncSlave", "Adjustment skipped: msg_invalid=%i wrong_tid=%i wrong_timing=%i",
                             int(msg_invalid), int(wrong_tid), int(wrong_timing));
                state_ = Update;
            }
        }
        if (state_ == Adjust)
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
