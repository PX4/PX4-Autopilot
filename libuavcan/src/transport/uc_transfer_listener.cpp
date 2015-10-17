/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/transport/transfer_listener.hpp>
#include <uavcan/debug.hpp>
#include <cstdlib>
#include <cassert>

namespace uavcan
{
/*
 * IncomingTransfer
 */
int IncomingTransfer::write(unsigned, const uint8_t*, unsigned)
{
    UAVCAN_ASSERT(0);  // Incoming transfer container is read-only
    return -ErrLogic;
}

/*
 * SingleFrameIncomingTransfer
 */
SingleFrameIncomingTransfer::SingleFrameIncomingTransfer(const RxFrame& frm)
    : IncomingTransfer(frm.getMonotonicTimestamp(), frm.getUtcTimestamp(), frm.getPriority(),
                       frm.getTransferType(), frm.getTransferID(), frm.getSrcNodeID(), frm.getIfaceIndex())
    , payload_(frm.getPayloadPtr())
    , payload_len_(uint8_t(frm.getPayloadLen()))
{
    UAVCAN_ASSERT(frm.isValid());
}

int SingleFrameIncomingTransfer::read(unsigned offset, uint8_t* data, unsigned len) const
{
    if (data == NULL)
    {
        UAVCAN_ASSERT(0);
        return -ErrInvalidParam;
    }
    if (offset >= payload_len_)
    {
        return 0;
    }
    if ((offset + len) > payload_len_)
    {
        len = payload_len_ - offset;
    }
    UAVCAN_ASSERT((offset + len) <= payload_len_);
    (void)copy(payload_ + offset, payload_ + offset + len, data);
    return int(len);
}

bool SingleFrameIncomingTransfer::isAnonymousTransfer() const
{
    return (getTransferType() == TransferTypeMessageBroadcast) && getSrcNodeID().isBroadcast();
}

/*
 * MultiFrameIncomingTransfer
 */
MultiFrameIncomingTransfer::MultiFrameIncomingTransfer(MonotonicTime ts_mono, UtcTime ts_utc,
                                                       const RxFrame& last_frame, TransferBufferAccessor& tba)
    : IncomingTransfer(ts_mono, ts_utc, last_frame.getPriority(), last_frame.getTransferType(),
                       last_frame.getTransferID(), last_frame.getSrcNodeID(), last_frame.getIfaceIndex())
    , buf_acc_(tba)
{
    UAVCAN_ASSERT(last_frame.isValid());
    UAVCAN_ASSERT(last_frame.isEndOfTransfer());
}

int MultiFrameIncomingTransfer::read(unsigned offset, uint8_t* data, unsigned len) const
{
    const ITransferBuffer* const tbb = const_cast<TransferBufferAccessor&>(buf_acc_).access();
    if (tbb == NULL)
    {
        UAVCAN_TRACE("MultiFrameIncomingTransfer", "Read failed: no such buffer");
        return -ErrLogic;
    }
    return tbb->read(offset, data, len);
}

/*
 * TransferListener::TimedOutReceiverPredicate
 */
bool TransferListener::TimedOutReceiverPredicate::operator()(const TransferBufferManagerKey& key,
                                                             const TransferReceiver& value) const
{
    if (value.isTimedOut(ts_))
    {
        UAVCAN_TRACE("TransferListener", "Timed out receiver: %s", key.toString().c_str());
        /*
         * TransferReceivers do not own their buffers - this helps the Map<> container to copy them
         * around quickly and safely (using default assignment operator). Downside is that we need to
         * destroy the buffers manually.
         * Maybe it is not good that the predicate has side effects, but I ran out of better ideas.
         */
        parent_bufmgr_.remove(key);
        return true;
    }
    return false;
}

/*
 * TransferListener
 */
bool TransferListener::checkPayloadCrc(const uint16_t compare_with, const ITransferBuffer& tbb) const
{
    TransferCRC crc = crc_base_;
    unsigned offset = 0;
    while (true)
    {
        uint8_t buf[16];
        const int res = tbb.read(offset, buf, sizeof(buf));
        if (res < 0)
        {
            UAVCAN_TRACE("TransferListener", "Failed to check CRC: Buffer read failure %i", res);
            return false;
        }
        if (res == 0)
        {
            break;
        }
        offset += unsigned(res);
        crc.add(buf, unsigned(res));
    }
    if (crc.get() != compare_with)
    {
        UAVCAN_TRACE("TransferListener", "CRC mismatch, expected=0x%04x, got=0x%04x",
                     int(compare_with), int(crc.get()));
        return false;
    }
    return true;
}

void TransferListener::handleReception(TransferReceiver& receiver, const RxFrame& frame,
                                           TransferBufferAccessor& tba)
{
    switch (receiver.addFrame(frame, tba))
    {
    case TransferReceiver::ResultNotComplete:
    {
        perf_.addErrors(receiver.yieldErrorCount());
        break;
    }
    case TransferReceiver::ResultSingleFrame:
    {
        perf_.addRxTransfer();
        SingleFrameIncomingTransfer it(frame);
        handleIncomingTransfer(it);
        break;
    }
    case TransferReceiver::ResultComplete:
    {
        perf_.addRxTransfer();
        const ITransferBuffer* tbb = tba.access();
        if (tbb == NULL)
        {
            UAVCAN_TRACE("TransferListener", "Buffer access failure, last frame: %s", frame.toString().c_str());
            break;
        }
        if (!checkPayloadCrc(receiver.getLastTransferCrc(), *tbb))
        {
            UAVCAN_TRACE("TransferListener", "CRC error, last frame: %s", frame.toString().c_str());
            break;
        }
        MultiFrameIncomingTransfer it(receiver.getLastTransferTimestampMonotonic(),
                                      receiver.getLastTransferTimestampUtc(), frame, tba);
        handleIncomingTransfer(it);
        it.release();
        break;
    }
    default:
    {
        UAVCAN_ASSERT(0);
        break;
    }
    }
}

void TransferListener::handleAnonymousTransferReception(const RxFrame& frame)
{
    if (allow_anonymous_transfers_)
    {
        perf_.addRxTransfer();
        SingleFrameIncomingTransfer it(frame);
        handleIncomingTransfer(it);
    }
}

TransferListener::~TransferListener()
{
    // Map must be cleared before bufmgr is destroyed
    receivers_.clear();
}

void TransferListener::cleanup(MonotonicTime ts)
{
    receivers_.removeAllWhere(TimedOutReceiverPredicate(ts, bufmgr_));
    UAVCAN_ASSERT(receivers_.isEmpty() ? bufmgr_.isEmpty() : 1);
}

void TransferListener::handleFrame(const RxFrame& frame)
{
    if (frame.getSrcNodeID().isUnicast())       // Normal transfer
    {
        const TransferBufferManagerKey key(frame.getSrcNodeID(), frame.getTransferType());

        TransferReceiver* recv = receivers_.access(key);
        if (recv == NULL)
        {
            if (!frame.isStartOfTransfer())
            {
                return;
            }

            TransferReceiver new_recv;
            recv = receivers_.insert(key, new_recv);
            if (recv == NULL)
            {
                UAVCAN_TRACE("TransferListener", "Receiver registration failed; frame %s", frame.toString().c_str());
                return;
            }
        }
        TransferBufferAccessor tba(bufmgr_, key);
        handleReception(*recv, frame, tba);
    }
    else if (frame.getSrcNodeID().isBroadcast() &&
             frame.isStartOfTransfer() &&
             frame.isEndOfTransfer() &&
             frame.getDstNodeID().isBroadcast())        // Anonymous transfer
    {
        handleAnonymousTransferReception(frame);
    }
    else
    {
        UAVCAN_TRACE("TransferListener", "Invalid frame: %s", frame.toString().c_str()); // Invalid frame
    }
}

/*
 * TransferListenerWithFilter
 */
void TransferListenerWithFilter::handleFrame(const RxFrame& frame)
{
    if (filter_ != NULL)
    {
        if (filter_->shouldAcceptFrame(frame))
        {
            TransferListener::handleFrame(frame);
        }
    }
    else
    {
        UAVCAN_ASSERT(0);
    }
}

}
