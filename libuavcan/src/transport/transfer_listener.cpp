/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdlib>
#include <cassert>
#include <uavcan/internal/transport/transfer_listener.hpp>
#include <uavcan/internal/debug.hpp>

namespace uavcan
{

/*
 * SingleFrameIncomingTransfer
 */
SingleFrameIncomingTransfer::SingleFrameIncomingTransfer(const RxFrame& frm)
: IncomingTransfer(frm.getMonotonicTimestamp(), frm.getUtcTimestamp(), frm.getTransferType(),
                   frm.getTransferID(), frm.getSrcNodeID())
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
        return 0;
    if ((offset + len) > payload_len_)
        len = payload_len_ - offset;
    assert((offset + len) <= payload_len_);
    std::copy(payload_ + offset, payload_ + offset + len, data);
    return len;
}

/*
 * MultiFrameIncomingTransfer
 */
MultiFrameIncomingTransfer::MultiFrameIncomingTransfer(uint64_t ts_monotonic, uint64_t ts_utc,
                                                       const RxFrame& last_frame, TransferBufferAccessor& tba)
: IncomingTransfer(ts_monotonic, ts_utc, last_frame.getTransferType(), last_frame.getTransferID(),
                   last_frame.getSrcNodeID())
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
    Crc16 crc = crc_base_;
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
            break;
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
    case TransferReceiver::RESULT_NOT_COMPLETE:
        return;

    case TransferReceiver::RESULT_SINGLE_FRAME:
    {
        SingleFrameIncomingTransfer it(frame);
        handleIncomingTransfer(it);
        return;
    }

    case TransferReceiver::RESULT_COMPLETE:
    {
        const ITransferBuffer* tbb = tba.access();
        if (tbb == NULL)
        {
            UAVCAN_TRACE("TransferListenerBase", "Buffer access failure, last frame: %s", frame.toString().c_str());
            return;
        }

        if (!checkPayloadCrc(receiver.getLastTransferCrc(), *tbb))
        {
            UAVCAN_TRACE("TransferListenerBase", "CRC error, last frame: %s", frame.toString().c_str());
            return;
        }

        MultiFrameIncomingTransfer it(receiver.getLastTransferTimestampMonotonic(),
                                      receiver.getLastTransferTimestampUtc(), frame, tba);
        handleIncomingTransfer(it);
        it.release();
        return;
    }

    default:
        assert(0);
        break;
    }
}

}
