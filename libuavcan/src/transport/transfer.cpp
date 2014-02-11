/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <cstdio>
#include <sstream>
#include <uavcan/internal/transport/transfer.hpp>

namespace uavcan
{

int TransferID::forwardDistance(TransferID rhs) const
{
    int d = int(rhs.get()) - int(get());
    if (d < 0)
        d += 1 << BITLEN;

    assert(((get() + d) & MAX) == rhs.get());
    return d;
}


template <int OFFSET, int WIDTH>
inline static uint32_t bitunpack(uint32_t val)
{
    return (val >> OFFSET) & ((1UL << WIDTH) - 1);
}

bool Frame::parse(const CanFrame& can_frame)
{
    if ((can_frame.id & CanFrame::FLAG_RTR) || !(can_frame.id & CanFrame::FLAG_EFF))
        return false;

    if (can_frame.dlc > 8)
    {
        assert(0);
        return false;
    }

    const uint32_t id = can_frame.id & CanFrame::MASK_EXTID;

    transfer_id    = bitunpack<0, 4>(id);
    last_frame     = bitunpack<4, 1>(id);
    frame_index    = bitunpack<5, 5>(id);
    source_node_id = bitunpack<10, 7>(id);
    transfer_type  = TransferType(bitunpack<17, 2>(id));
    data_type_id   = bitunpack<19, 10>(id);

    payload_len = can_frame.dlc;
    std::copy(can_frame.data, can_frame.data + can_frame.dlc, payload);
    return true;
}

template <int OFFSET, int WIDTH>
inline static uint32_t bitpack(uint32_t field)
{
    return (field & ((1UL << WIDTH) - 1)) << OFFSET;
}

CanFrame Frame::compile() const
{
    CanFrame frame;

    frame.id = CanFrame::FLAG_EFF |
        bitpack<0, 4>(transfer_id.get()) |
        bitpack<4, 1>(last_frame) |
        bitpack<5, 5>(frame_index) |
        bitpack<10, 7>(source_node_id) |
        bitpack<17, 2>(transfer_type) |
        bitpack<19, 10>(data_type_id);

    assert(payload_len <= sizeof(payload));
    frame.dlc = payload_len;
    std::copy(payload, payload + payload_len, frame.data);
    return frame;
}

std::string Frame::toString() const
{
    /*
     * Frame ID fields, according to UAVCAN specs:
     *  - Data Type ID
     *  - Transfer Type
     *  - Source Node ID
     *  - Frame Index
     *  - Last Frame
     *  - Transfer ID
     */
    static const int BUFLEN = 100;
    char buf[BUFLEN];
    int ofs = std::snprintf(buf, BUFLEN, "dtid=%i tt=%i snid=%i idx=%i last=%i tid=%i payload=[",
                            int(data_type_id), int(transfer_type), int(source_node_id), int(frame_index),
                            int(last_frame), int(transfer_id.get()));

    for (int i = 0; i < payload_len; i++)
    {
        ofs += std::snprintf(buf + ofs, BUFLEN - ofs, "%02x", payload[i]);
        if ((i + 1) < payload_len)
            ofs += std::snprintf(buf + ofs, BUFLEN - ofs, " ");
    }
    ofs += std::snprintf(buf + ofs, BUFLEN - ofs, "]");
    return std::string(buf);
}

std::string RxFrame::toString() const
{
    std::ostringstream os;  // C++03 doesn't support long long, so we need ostream to print the timestamp
    os << Frame::toString() << " ts_m=" << ts_monotonic << " ts_utc=" << ts_utc << " iface=" << int(iface_index);
    return os.str();
}

}
