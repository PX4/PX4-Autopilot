/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdlib>
#include <cassert>
#include <uavcan/protocol/global_time_sync_master.hpp>
#include <uavcan/debug.hpp>

namespace uavcan
{
/*
 * GlobalTimeSyncMaster::IfaceMaster
 */
int GlobalTimeSyncMaster::IfaceMaster::init()
{
    const int res = pub_.init();
    if (res >= 0)
    {
        TransferSender* const ts = pub_.getTransferSender();
        assert(ts != NULL);
        ts->setIfaceMask(1 << iface_index_);
        ts->setCanIOFlags(CanIOFlagLoopback);
    }
    return res;
}

void GlobalTimeSyncMaster::IfaceMaster::setTxTimestamp(UtcTime ts)
{
    prev_tx_utc_ = UtcTime();
    if (ts.isZero())
    {
        assert(0);
        pub_.getNode().registerInternalFailure("GlobalTimeSyncMaster got zero UTC TX timestamp");
        return;
    }
    if (!prev_tx_utc_.isZero())
    {
        assert(0);
        pub_.getNode().registerInternalFailure("GlobalTimeSyncMaster publication conflict");
        return;
    }
    prev_tx_utc_ = ts;
}

int GlobalTimeSyncMaster::IfaceMaster::publish()
{
    assert(pub_.getTransferSender()->getCanIOFlags() == CanIOFlagLoopback);
    assert(pub_.getTransferSender()->getIfaceMask() == (1 << iface_index_));

    const MonotonicTime ts_mono = pub_.getNode().getMonotonicTime();
    const MonotonicDuration since_prev_pub = ts_mono - prev_pub_mono_;
    assert(!since_prev_pub.isNegative());

    if (since_prev_pub.toMSec() > protocol::GlobalTimeSync::MIN_PUBLICATION_PERIOD_MS)
    {
        prev_pub_mono_ = ts_mono;
        protocol::GlobalTimeSync msg;
        if (since_prev_pub.toMSec() < protocol::GlobalTimeSync::MAX_PUBLICATION_PERIOD_MS)
        {
            msg.prev_utc_usec = prev_tx_utc_.toUSec();
        }
        else
        {
            msg.prev_utc_usec = 0;
        }
        prev_tx_utc_ = UtcTime();
        UAVCAN_TRACE("GlobalTimeSyncMaster", "Publishing %llu", static_cast<unsigned long long>(msg.prev_utc_usec));
        return pub_.broadcast(msg);
    }
    else
    {
        UAVCAN_TRACE("GlobalTimeSyncMaster", "Publication skipped");
        return 0;
    }
}

/*
 * GlobalTimeSyncMaster
 */
void GlobalTimeSyncMaster::handleLoopbackFrame(const RxFrame& frame)
{
    const uint8_t iface = frame.getIfaceIndex();
    if (initialized_ && iface < MaxCanIfaces)
    {
        if (frame.getDataTypeID() == dtid_ &&
            frame.getTransferType() == TransferTypeMessageBroadcast &&
            frame.isLast() && frame.isFirst())
        {
            iface_masters_[iface]->setTxTimestamp(frame.getUtcTimestamp());
        }
    }
    else
    {
        assert(0);
    }
}

int GlobalTimeSyncMaster::init()
{
    if (initialized_)
    {
        return 0;
    }

    // Data type ID
    const DataTypeDescriptor* const desc =
        GlobalDataTypeRegistry::instance().find(DataTypeKindMessage, protocol::GlobalTimeSync::getDataTypeFullName());
    if (desc == NULL)
    {
        return -ErrUnknownDataType;
    }
    dtid_ = desc->getID();

    // Iface master array
    int res = -ErrLogic;
    for (uint8_t i = 0; i < MaxCanIfaces; i++)
    {
        if (!iface_masters_[i].isConstructed())
        {
            iface_masters_[i].construct<INode&, uint8_t>(node_, i);
        }
        res = iface_masters_[i]->init();
        if (res < 0)
        {
            break;
        }
    }

    // Loopback listener
    initialized_ = res >= 0;
    if (initialized_)
    {
        LoopbackFrameListenerBase::startListening();
    }
    return res;
}

int GlobalTimeSyncMaster::publish()
{
    if (!initialized_)
    {
        const int res = init();
        if (res < 0)
        {
            return res;
        }
    }
    for (uint8_t i = 0; i < node_.getDispatcher().getCanIOManager().getNumIfaces(); i++)
    {
        const int res = iface_masters_[i]->publish();
        if (res < 0)
        {
            return res;
        }
    }
    return 0;
}

}
