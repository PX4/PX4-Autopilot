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
SingleFrameIncomingTransfer::SingleFrameIncomingTransfer(const RxFrame& frm)
: IncomingTransfer(frm.ts_monotonic, frm.ts_utc, frm.transfer_type, frm.transfer_id, frm.source_node_id)
, payload_len_(frm.payload_len)
{
    assert(frm.payload_len <= Frame::PAYLOAD_LEN_MAX);
    assert(frm.frame_index == 0);
    assert(frm.last_frame);
    std::copy(frm.payload, frm.payload + frm.payload_len, payload_);
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
MultiFrameIncomingTransfer::MultiFrameIncomingTransfer(const RxFrame& last_frame, uint8_t buffer_offset,
                                                       ITransferBufferManager* bufmgr,
                                                       const TransferBufferManagerKey& bufmgr_key)
: IncomingTransfer(last_frame.ts_monotonic, last_frame.ts_utc, last_frame.transfer_type,
                   last_frame.transfer_id, last_frame.source_node_id)
, bufmgr_(bufmgr)
, bufmgr_key_(bufmgr_key)
, buffer_offset_(buffer_offset)
{
    assert(bufmgr);
    assert(!bufmgr_key.isEmpty());
    assert(last_frame.last_frame);
}

int MultiFrameIncomingTransfer::read(unsigned int offset, uint8_t* data, unsigned int len) const
{
    const TransferBufferBase* const tbb = const_cast<ITransferBufferManager*>(bufmgr_)->access(bufmgr_key_);
    if (tbb == NULL)
    {
        UAVCAN_TRACE("MultiFrameIncomingTransfer", "Read failed: no such buffer %s",
            bufmgr_key_.toString().c_str());
        return -1;
    }
    return tbb->read(offset + buffer_offset_, data, len);
}

/*
 * TransferListenerBase
 */
void TransferListenerBase::handleReception(TransferReceiver& receiver, const RxFrame& frame,
                                           const TransferBufferManagerKey& bufmgr_key)
{
    const TransferReceiver::ResultCode result = receiver.addFrame(frame, bufmgr_key);
    (void)result;
}

}
