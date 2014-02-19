/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <uavcan/internal/debug.hpp>
#include <uavcan/internal/transport/transfer_sender.hpp>


namespace uavcan
{

int TransferSender::send(const uint8_t* payload, int payload_len, uint64_t monotonic_tx_deadline,
                         uint64_t monotonic_blocking_deadline, TransferType transfer_type, NodeID dst_node_id,
                         TransferID tid)
{
    Frame frame(data_type_.id, transfer_type, dispatcher_.getSelfNodeID(), dst_node_id, 0, tid);

    if (frame.getMaxPayloadLen() >= payload_len)           // Single Frame Transfer
    {
        const int res = frame.setPayload(payload, payload_len);
        if (res != payload_len)
        {
            assert(0);
            UAVCAN_TRACE("TransferSender", "Frame payload write failure, %i", res);
            return (res < 0) ? res : -1;
        }
        frame.makeLast();
        assert(frame.isLast() && frame.isFirst());
        return dispatcher_.send(frame, monotonic_tx_deadline, monotonic_blocking_deadline, qos_);
    }
    else                                                   // Multi Frame Transfer
    {
        int offset = 0;
        {
            Crc16 crc = crc_base_;
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
                return write_res;
            }
            offset = write_res - 2;
            assert(payload_len > offset);
        }

        int next_frame_index = 1;

        while (true)
        {
            const int send_res = dispatcher_.send(frame, monotonic_tx_deadline, monotonic_blocking_deadline, qos_);
            if (send_res < 0)
                return send_res;

            if (frame.isLast())
                return next_frame_index;  // Number of frames transmitted

            frame.setIndex(next_frame_index++);

            const int write_res = frame.setPayload(payload + offset, payload_len - offset);
            if (write_res < 0)
            {
                UAVCAN_TRACE("TransferSender", "Frame payload write failure, %i", write_res);
                return write_res;
            }

            offset += write_res;
            assert(offset <= payload_len);
            if (offset >= payload_len)
                frame.makeLast();
        }
    }

    assert(0);
    return -1; // Return path analysis is apparently broken. There should be no warning, this 'return' is unreachable.
}

int TransferSender::send(const uint8_t* payload, int payload_len, uint64_t monotonic_tx_deadline,
                         uint64_t monotonic_blocking_deadline, TransferType transfer_type, NodeID dst_node_id)
{
    const OutgoingTransferRegistryKey otr_key(data_type_.id, transfer_type, dst_node_id);

    assert(monotonic_tx_deadline > 0);
    const uint64_t otr_deadline = monotonic_tx_deadline + max_transfer_interval_;

    TransferID* const tid = dispatcher_.getOutgoingTransferRegistry().accessOrCreate(otr_key, otr_deadline);
    if (tid == NULL)
    {
        UAVCAN_TRACE("TransferSender", "OTR access failure, dtid=%i tt=%i", int(data_type_.id), int(transfer_type));
        return -1;
    }

    const TransferID this_tid = tid->get();
    tid->increment();

    return send(payload, payload_len, monotonic_tx_deadline, monotonic_blocking_deadline, transfer_type,
                dst_node_id, this_tid);
}

}
