/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <uavcan/debug.hpp>
#include <uavcan/transport/transfer_sender.hpp>


namespace uavcan
{

void TransferSender::registerError()
{
    dispatcher_.getTransferPerfCounter().addError();
}

int TransferSender::send(const uint8_t* payload, int payload_len, MonotonicTime tx_deadline,
                         MonotonicTime blocking_deadline, TransferType transfer_type, NodeID dst_node_id,
                         TransferID tid)
{
    dispatcher_.getTransferPerfCounter().addTxTransfer();

    Frame frame(data_type_.getID(), transfer_type, dispatcher_.getNodeID(), dst_node_id, 0, tid);

    if (frame.getMaxPayloadLen() >= payload_len)           // Single Frame Transfer
    {
        const int res = frame.setPayload(payload, payload_len);
        if (res != payload_len)
        {
            assert(0);
            UAVCAN_TRACE("TransferSender", "Frame payload write failure, %i", res);
            registerError();
            return (res < 0) ? res : -1;
        }
        frame.makeLast();
        assert(frame.isLast() && frame.isFirst());
        return dispatcher_.send(frame, tx_deadline, blocking_deadline, qos_, flags_, iface_mask_);
    }
    else                                                   // Multi Frame Transfer
    {
        int offset = 0;
        {
            TransferCRC crc = crc_base_;
            crc.add(payload, payload_len);

            static const int BUFLEN = sizeof(CanFrame::data);
            uint8_t buf[BUFLEN];

            buf[0] = crc.get() & 0xFF;       // Transfer CRC, little endian
            buf[1] = (crc.get() >> 8) & 0xFF;
            std::copy(payload, payload + BUFLEN - 2, buf + 2);

            const int write_res = frame.setPayload(buf, BUFLEN);
            if (write_res < 2)
            {
                UAVCAN_TRACE("TransferSender", "Frame payload write failure, %i", write_res);
                registerError();
                return write_res;
            }
            offset = write_res - 2;
            assert(payload_len > offset);
        }

        int next_frame_index = 1;

        while (true)
        {
            const int send_res = dispatcher_.send(frame, tx_deadline, blocking_deadline, qos_, flags_, iface_mask_);
            if (send_res < 0)
            {
                registerError();
                return send_res;
            }

            if (frame.isLast())
            {
                return next_frame_index;  // Number of frames transmitted
            }
            frame.setIndex(next_frame_index++);

            const int write_res = frame.setPayload(payload + offset, payload_len - offset);
            if (write_res < 0)
            {
                UAVCAN_TRACE("TransferSender", "Frame payload write failure, %i", write_res);
                registerError();
                return write_res;
            }

            offset += write_res;
            assert(offset <= payload_len);
            if (offset >= payload_len)
            {
                frame.makeLast();
            }
        }
    }

    assert(0);
    return -1; // Return path analysis is apparently broken. There should be no warning, this 'return' is unreachable.
}

int TransferSender::send(const uint8_t* payload, int payload_len, MonotonicTime tx_deadline,
                         MonotonicTime blocking_deadline, TransferType transfer_type, NodeID dst_node_id)
{
    const OutgoingTransferRegistryKey otr_key(data_type_.getID(), transfer_type, dst_node_id);

    assert(!tx_deadline.isZero());
    const MonotonicTime otr_deadline = tx_deadline + max_transfer_interval_;

    TransferID* const tid = dispatcher_.getOutgoingTransferRegistry().accessOrCreate(otr_key, otr_deadline);
    if (tid == NULL)
    {
        UAVCAN_TRACE("TransferSender", "OTR access failure, dtd=%s tt=%i",
                     data_type_.toString().c_str(), int(transfer_type));
        return -1;
    }

    const TransferID this_tid = tid->get();
    tid->increment();

    return send(payload, payload_len, tx_deadline, blocking_deadline, transfer_type,
                dst_node_id, this_tid);
}

}
