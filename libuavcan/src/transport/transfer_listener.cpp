/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdlib>
#include <uavcan/internal/transport/transfer_listener.hpp>

namespace uavcan
{

/*
 * SingleFrameIncomingTransfer
 */
SingleFrameIncomingTransfer::SingleFrameIncomingTransfer(const RxFrame& frm, const uint8_t* payload,
                                                         unsigned int payload_len)
: IncomingTransfer(frm.ts_monotonic, frm.ts_utc, frm.transfer_type, frm.transfer_id, frm.source_node_id)
, payload_(payload)
, payload_len_(payload_len)
{ }

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
: IncomingTransfer(ts_monotonic, ts_utc, last_frame.transfer_type, last_frame.transfer_id, last_frame.source_node_id)
, buf_acc_(tba)
{
    assert(last_frame.last_frame);
}

int MultiFrameIncomingTransfer::read(unsigned int offset, uint8_t* data, unsigned int len) const
{
    const TransferBufferBase* const tbb = const_cast<TransferBufferAccessor&>(buf_acc_).access();
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
bool TransferListenerBase::checkPayloadCrc(const uint16_t compare_with, const TransferBufferBase& tbb) const
{
    Crc16 crc = crc_base_;
    unsigned int offset = 0;
    while (true)
    {
        uint8_t buf[16];
        const int res = tbb.read(offset, buf, sizeof(buf));
        if (res == 0)
            break;
        crc.add(buf, res);
    }
    return crc.get() == compare_with;
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
        uint8_t payload[Frame::PAYLOAD_LEN_MAX];
        unsigned int payload_len = 0;
        const bool success = TransferReceiver::extractSingleFrameTransferPayload(frame, payload, payload_len);
        if (!success)
        {
            UAVCAN_TRACE("TransferListenerBase", "SFT payload extraction failed, frame: %s", frame.toString().c_str());
            return;
        }
        assert(payload_len <= Frame::PAYLOAD_LEN_MAX);

        SingleFrameIncomingTransfer it(frame, payload, payload_len);
        handleIncomingTransfer(it);
        return;
    }

    case TransferReceiver::RESULT_COMPLETE:
    {
        const TransferBufferBase* tbb = tba.access();
        if (tbb == NULL)
        {
            UAVCAN_TRACE("TransferListenerBase", "Buffer access failure, last frame: %s", frame.toString().c_str());
            return;
        }

        if (!checkPayloadCrc(receiver.getLastTransferCrc(), *tbb))
        {
            UAVCAN_TRACE("TransferListenerBase", "CRC mismatch, last frame: %s", frame.toString().c_str());
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
