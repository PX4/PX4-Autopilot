/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdlib>
#include <cassert>
#include <uavcan/transport/transfer_listener.hpp>
#include <uavcan/debug.hpp>

namespace uavcan
{
/*
 * IncomingTransfer
 */
int IncomingTransfer::write(unsigned int, const uint8_t*, unsigned int)
{
    assert(0);  // Incoming transfer container is read-only
    return -1;
}

/*
 * SingleFrameIncomingTransfer
 */
SingleFrameIncomingTransfer::SingleFrameIncomingTransfer(const RxFrame& frm)
    : IncomingTransfer(frm.getMonotonicTimestamp(), frm.getUtcTimestamp(), frm.getTransferType(),
                       frm.getTransferID(), frm.getSrcNodeID(), frm.getIfaceIndex())
    , payload_(frm.getPayloadPtr())
    , payload_len_(frm.getPayloadLen())
{
    assert(frm.isValid());
}

int SingleFrameIncomingTransfer::read(unsigned int offset, uint8_t* data, unsigned int len) const
{
    if (data == NULL)
    {
        assert(0);
        return -1;
    }
    if (offset >= payload_len_)
    {
        return 0;
    }
    if ((offset + len) > payload_len_)
    {
        len = payload_len_ - offset;
    }
    assert((offset + len) <= payload_len_);
    std::copy(payload_ + offset, payload_ + offset + len, data);
    return len;
}

/*
 * MultiFrameIncomingTransfer
 */
MultiFrameIncomingTransfer::MultiFrameIncomingTransfer(MonotonicTime ts_mono, UtcTime ts_utc,
                                                       const RxFrame& last_frame, TransferBufferAccessor& tba)
    : IncomingTransfer(ts_mono, ts_utc, last_frame.getTransferType(), last_frame.getTransferID(),
                       last_frame.getSrcNodeID(), last_frame.getIfaceIndex())
    , buf_acc_(tba)
{
    assert(last_frame.isValid());
    assert(last_frame.isLast());
}

int MultiFrameIncomingTransfer::read(unsigned int offset, uint8_t* data, unsigned int len) const
{
    const ITransferBuffer* const tbb = const_cast<TransferBufferAccessor&>(buf_acc_).access();
    if (tbb == NULL)
    {
        UAVCAN_TRACE("MultiFrameIncomingTransfer", "Read failed: no such buffer");
        return -1;
    }
    return tbb->read(offset, data, len);
}

/*
 * TransferListenerBase
 */
bool TransferListenerBase::checkPayloadCrc(const uint16_t compare_with, const ITransferBuffer& tbb) const
{
    TransferCRC crc = crc_base_;
    unsigned int offset = 0;
    while (true)
    {
        uint8_t buf[16];
        const int res = tbb.read(offset, buf, sizeof(buf));
        if (res < 0)
        {
            UAVCAN_TRACE("TransferListenerBase", "Failed to check CRC: Buffer read failure %i", res);
            return false;
        }
        if (res == 0)
        {
            break;
        }
        offset += res;
        crc.add(buf, res);
    }
    if (crc.get() != compare_with)
    {
        UAVCAN_TRACE("TransferListenerBase", "CRC mismatch, expected=0x%04x, got=0x%04x",
                     int(compare_with), int(crc.get()));
        return false;
    }
    return true;
}

void TransferListenerBase::handleReception(TransferReceiver& receiver, const RxFrame& frame,
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
            UAVCAN_TRACE("TransferListenerBase", "Buffer access failure, last frame: %s", frame.toString().c_str());
            break;
        }
        if (!checkPayloadCrc(receiver.getLastTransferCrc(), *tbb))
        {
            UAVCAN_TRACE("TransferListenerBase", "CRC error, last frame: %s", frame.toString().c_str());
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
        assert(0);
        break;
    }
    }
}

}
