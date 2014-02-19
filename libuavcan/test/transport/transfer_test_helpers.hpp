/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <algorithm>
#include <queue>
#include <vector>
#include <gtest/gtest.h>
#include <uavcan/internal/transport/transfer_listener.hpp>

/**
 * UAVCAN transfer representation used in various tests.
 */
struct Transfer
{
    uint64_t ts_monotonic;
    uint64_t ts_utc;
    uavcan::TransferType transfer_type;
    uavcan::TransferID transfer_id;
    uavcan::NodeID src_node_id;
    uavcan::NodeID dst_node_id;
    uavcan::DataTypeDescriptor data_type;
    std::string payload;

    Transfer(const uavcan::IncomingTransfer& tr, const uavcan::DataTypeDescriptor& data_type)
    : ts_monotonic(tr.getMonotonicTimestamp())
    , ts_utc(tr.getUtcTimestamp())
    , transfer_type(tr.getTransferType())
    , transfer_id(tr.getTransferID())
    , src_node_id(tr.getSrcNodeID())
    , dst_node_id() // default is invalid
    , data_type(data_type)
    {
        unsigned int offset = 0;
        while (true)
        {
            uint8_t buf[256];
            int res = tr.read(offset, buf, sizeof(buf));
            if (res < 0)
            {
                std::cout << "IncomingTransferContainer: read failure " << res << std::endl;
                exit(1);
            }
            if (res == 0)
                break;
            payload += std::string(reinterpret_cast<const char*>(buf), res);
            offset += res;
        }
    }

    Transfer(uint64_t ts_monotonic, uint64_t ts_utc, uavcan::TransferType transfer_type,
             uavcan::TransferID transfer_id, uavcan::NodeID src_node_id, uavcan::NodeID dst_node_id,
             const std::string& payload, const uavcan::DataTypeDescriptor& data_type)
    : ts_monotonic(ts_monotonic)
    , ts_utc(ts_utc)
    , transfer_type(transfer_type)
    , transfer_id(transfer_id)
    , src_node_id(src_node_id)
    , dst_node_id(dst_node_id)
    , data_type(data_type)
    , payload(payload)
    { }

    bool operator==(const Transfer& rhs) const
    {
        return
            (ts_monotonic  == rhs.ts_monotonic) &&
            ((ts_utc && rhs.ts_utc) ? (ts_utc == rhs.ts_utc) : true) &&
            (transfer_type == rhs.transfer_type) &&
            (transfer_id   == rhs.transfer_id) &&
            (src_node_id   == rhs.src_node_id) &&
            ((dst_node_id.isValid() && rhs.dst_node_id.isValid()) ? (dst_node_id == rhs.dst_node_id) : true) &&
            (data_type     == rhs.data_type) &&
            (payload       == rhs.payload);
    }

    std::string toString() const
    {
        std::ostringstream os;
        os << "ts_m="     << ts_monotonic
            << " ts_utc=" << ts_utc
            << " tt="     << transfer_type
            << " tid="    << int(transfer_id.get())
            << " snid="   << int(src_node_id.get())
            << " dnid="   << int(dst_node_id.get())
            << " dtid="   << int(data_type.id)
            << "\n\t'" << payload << "'";
        return os.str();
    }
};

/**
 * This subscriber accepts any types of transfers - this makes testing easier.
 * In reality, uavcan::TransferListener should accept only specific transfer types
 * which are dispatched/filtered by uavcan::Dispatcher.
 */
template <unsigned int MAX_BUF_SIZE, unsigned int NUM_STATIC_BUFS, unsigned int NUM_STATIC_RECEIVERS>
class TestSubscriber : public uavcan::TransferListener<MAX_BUF_SIZE, NUM_STATIC_BUFS, NUM_STATIC_RECEIVERS>
{
    typedef uavcan::TransferListener<MAX_BUF_SIZE, NUM_STATIC_BUFS, NUM_STATIC_RECEIVERS> Base;

    std::queue<Transfer> transfers_;

public:
    TestSubscriber(const uavcan::DataTypeDescriptor& data_type, uavcan::IAllocator& allocator)
    : Base(data_type, allocator)
    { }

    void handleIncomingTransfer(uavcan::IncomingTransfer& transfer)
    {
        const Transfer rx(transfer, Base::getDataTypeDescriptor());
        transfers_.push(rx);
        std::cout << "Received transfer: " << rx.toString() << std::endl;
    }

    bool matchAndPop(const Transfer& reference)
    {
        if (transfers_.empty())
        {
            std::cout << "No received transfers" << std::endl;
            return false;
        }

        const Transfer tr = transfers_.front();
        transfers_.pop();

        const bool res = (tr == reference);
        if (!res)
        {
            std::cout << "TestSubscriber: Transfer mismatch:\n"
                << "Expected: " << reference.toString() << "\n"
                << "Received: " << tr.toString() << std::endl;
        }
        return res;
    }

    int getNumReceivedTransfers() const { return transfers_.size(); }
    bool isEmpty() const { return transfers_.empty(); }
};


namespace
{

std::vector<uavcan::RxFrame> serializeTransfer(const Transfer& transfer)
{
    bool need_crc = false;
    switch (transfer.transfer_type)
    {
    case uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST:
        need_crc = transfer.payload.length() > sizeof(uavcan::CanFrame::data);
        break;
    case uavcan::TRANSFER_TYPE_SERVICE_RESPONSE:
    case uavcan::TRANSFER_TYPE_SERVICE_REQUEST:
    case uavcan::TRANSFER_TYPE_MESSAGE_UNICAST:
        need_crc = transfer.payload.length() > (sizeof(uavcan::CanFrame::data) - 1);
        break;
    default:
        std::cerr << "X_X" << std::endl;
        std::exit(1);
    }

    std::vector<uint8_t> raw_payload;
    if (need_crc)
    {
        uavcan::Crc16 payload_crc(transfer.data_type.hash.value, uavcan::DataTypeHash::NUM_BYTES);
        payload_crc.add(reinterpret_cast<const uint8_t*>(transfer.payload.c_str()), transfer.payload.length());
        // Little endian
        raw_payload.push_back(payload_crc.get() & 0xFF);
        raw_payload.push_back((payload_crc.get() >> 8) & 0xFF);
    }
    raw_payload.insert(raw_payload.end(), transfer.payload.begin(), transfer.payload.end());

    std::vector<uavcan::RxFrame> output;
    unsigned int frame_index = 0;
    unsigned int offset = 0;
    uint64_t ts_monotonic = transfer.ts_monotonic;
    uint64_t ts_utc = transfer.ts_utc;

    while (true)
    {
        const int bytes_left = raw_payload.size() - offset;
        EXPECT_TRUE(bytes_left >= 0);

        uavcan::Frame frm(transfer.data_type.id, transfer.transfer_type, transfer.src_node_id, transfer.dst_node_id,
                          frame_index, transfer.transfer_id);
        const int spres = frm.setPayload(&*(raw_payload.begin() + offset), bytes_left);
        if (spres < 0)
        {
            std::cerr << ">_<" << std::endl;
            std::exit(1);
        }
        if (spres == bytes_left)
            frm.makeLast();

        offset += spres;
        EXPECT_GE(uavcan::Frame::INDEX_MAX, frame_index);
        frame_index++;

        const uavcan::RxFrame rxfrm(frm, ts_monotonic, ts_utc, 0);
        ts_monotonic += 1;
        ts_utc += 1;

        output.push_back(rxfrm);
        if (frm.isLast())
            break;
    }
    return output;
}

uavcan::DataTypeDescriptor makeDataType(uavcan::DataTypeKind kind, uint16_t id)
{
    uavcan::DataTypeDescriptor dtd(kind, id, uavcan::DataTypeHash());
    for (int i = 0; i < uavcan::DataTypeHash::NUM_BYTES; i += 2)
    {
        dtd.hash.value[i] = id & 0xFF;
        dtd.hash.value[i + 1] = id >> 8;
    }
    return dtd;
}

}


class IncomingTransferEmulatorBase
{
    uint64_t ts_;
    uavcan::TransferID tid_;
    uavcan::NodeID dst_node_id_;

public:
    IncomingTransferEmulatorBase(uavcan::NodeID dst_node_id)
    : ts_(0)
    , dst_node_id_(dst_node_id)
    { }

    virtual ~IncomingTransferEmulatorBase() { }

    Transfer makeTransfer(uavcan::TransferType transfer_type, uint8_t source_node_id, const std::string& payload,
                          const uavcan::DataTypeDescriptor& type,
                          uavcan::NodeID dst_node_id_override = uavcan::NodeID())
    {
        ts_ += 100;
        const uavcan::NodeID dst_node_id = (transfer_type == uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST)
                    ? uavcan::NodeID::BROADCAST
                    : (dst_node_id_override.isValid() ? dst_node_id_override : dst_node_id_);
        const Transfer tr(ts_, ts_ + 1000000000ul, transfer_type, tid_, source_node_id, dst_node_id, payload, type);
        tid_.increment();
        return tr;
    }

    virtual void sendOneFrame(const uavcan::RxFrame& frame) = 0;

    void send(const std::vector<std::vector<uavcan::RxFrame> >& sers)
    {
        unsigned int index = 0;
        while (true)
        {
            // Sending all transfers concurrently
            bool all_empty = true;
            for (std::vector<std::vector<uavcan::RxFrame> >::const_iterator it = sers.begin(); it != sers.end(); ++it)
            {
                if (it->size() <= index)
                    continue;
                all_empty = false;
                std::cout << "Incoming Transfer Emulator: Sending: " << it->at(index).toString() << std::endl;
                sendOneFrame(it->at(index));
            }
            index++;
            if (all_empty)
                break;
        }
    }

    void send(const Transfer* transfers, unsigned int num_transfers)
    {
        std::vector<std::vector<uavcan::RxFrame> > sers;
        while (num_transfers--)
            sers.push_back(serializeTransfer(*transfers++));
        send(sers);
    }

    template <int SIZE> void send(const Transfer (&transfers)[SIZE]) { send(transfers, SIZE); }
};
