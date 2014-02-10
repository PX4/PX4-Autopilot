/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <algorithm>
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <uavcan/internal/transport/transfer_receiver.hpp>


struct RxFrameGenerator
{
    static const uint8_t DEFAULT_NODE_ID = 42;

    uint16_t data_type_id;
    uavcan::TransferType ttype;
    uint8_t source_node_id;

    RxFrameGenerator(uint16_t data_type_id, uavcan::TransferType ttype, uint8_t source_node_id = DEFAULT_NODE_ID)
    : data_type_id(data_type_id)
    , ttype(ttype)
    , source_node_id(source_node_id)
    { }

    uavcan::RxFrame operator()(int iface_index, const std::string& data, uint8_t frame_index, bool last,
                               uint8_t transfer_id, uint64_t timestamp)
    {
        if (data.length() > uavcan::Frame::PAYLOAD_LEN_MAX)
        {
            std::cerr << "RxFrameGenerator(): Data is too long" << std::endl;
            std::exit(1);
        }

        uavcan::RxFrame frm;

        frm.iface_index = iface_index;
        frm.timestamp = timestamp;

        frm.data_type_id = data_type_id;
        frm.frame_index = frame_index;
        frm.last_frame = last;
        std::copy(data.begin(), data.end(), frm.payload);
        frm.payload_len = data.length();
        frm.source_node_id = source_node_id;
        frm.transfer_id = uavcan::TransferID(transfer_id);
        frm.transfer_type = ttype;

        return frm;
    }
};


template <unsigned int BUFSIZE>
struct Context
{
    uavcan::PoolManager<1> poolmgr;        // We don't need dynamic memory for this test
    uavcan::TransferReceiver receiver;     // Must be default constructible and copyable
    uavcan::TransferBufferManager<BUFSIZE, 1> bufmgr;

    Context()
    : bufmgr(&poolmgr)
    {
        assert(poolmgr.allocate(1) == NULL);
        receiver = uavcan::TransferReceiver(&bufmgr, RxFrameGenerator::DEFAULT_NODE_ID);
    }

    ~Context()
    {
        // We need to destroy the receiver before its buffer manager
        receiver = uavcan::TransferReceiver();
    }
};


static bool matchBufferContent(const uavcan::TransferBufferBase* tbb, const std::string& content)
{
    uint8_t data[1024];
    std::fill(data, data + sizeof(data), 0);
    if (content.length() > sizeof(data))
    {
        std::cerr << "matchBufferContent(): Content is too long" << std::endl;
        std::exit(1);
    }
    tbb->read(0, data, content.length());
    if (std::equal(content.begin(), content.end(), data))
        return true;
    std::cout << "Buffer content mismatch:"
        << "\n\tExpected: " << content
        << "\n\tActually: " << reinterpret_cast<const char*>(data)
        << std::endl;
    return false;
}


#define CHECK_NOT_COMPLETE(x) ASSERT_EQ(uavcan::TransferReceiver::RESULT_NOT_COMPLETE, (x))
#define CHECK_COMPLETE(x)     ASSERT_EQ(uavcan::TransferReceiver::RESULT_COMPLETE, (x))
#define CHECK_SINGLE_FRAME(x) ASSERT_EQ(uavcan::TransferReceiver::RESULT_SINGLE_FRAME, (x))

TEST(TransferReceiver, Basic)
{
    using uavcan::TransferReceiver;
    Context<32> context;
    RxFrameGenerator gen(789, uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST);
    uavcan::TransferReceiver& rcv = context.receiver;
    uavcan::ITransferBufferManager& bufmgr = context.bufmgr;

    /*
     * Empty
     */
    ASSERT_EQ(TransferReceiver::DEFAULT_TRANSFER_INTERVAL, rcv.getInterval());
    ASSERT_EQ(0, rcv.getLastTransferTimestamp());

    /*
     * Single frame transfer with zero ts, must be ignored
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "Foo", 0, true, 0, 0)));
    ASSERT_EQ(TransferReceiver::DEFAULT_TRANSFER_INTERVAL, rcv.getInterval());
    ASSERT_EQ(0, rcv.getLastTransferTimestamp());

    /*
     * Valid compound transfer
     * Args: iface_index, data, frame_index, last, transfer_id, timestamp
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "12345678", 0, false, 0, 100)));
    CHECK_COMPLETE(rcv.addFrame(gen(0, "foo", 1, true, 0, 200)));

    ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.source_node_id), "12345678foo"));
    ASSERT_EQ(TransferReceiver::DEFAULT_TRANSFER_INTERVAL, rcv.getInterval());           // Not initialized yet
    ASSERT_EQ(100, rcv.getLastTransferTimestamp());

    /*
     * Compound transfer mixed with invalid frames; buffer was not released explicitly
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "qwe",      0, false, 0, 300)));   // Previous TID, rejected
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "rty",      0, false, 0, 300)));   // Previous TID, wrong iface
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "12345678", 0, false, 1, 1000)));
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "qwertyui", 0, false, 1, 1100)));  // Old FI
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "abcdefgh", 1, false, 1, 1200)));
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "45678910", 1, false, 2, 1300)));  // Next TID, but FI > 0
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "",         2, true,  1, 1300)));  // Wrong iface
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "",         31,true,  1, 1300)));  // Unexpected FI
    CHECK_COMPLETE(    rcv.addFrame(gen(0, "",         2, true,  1, 1300)));

    ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.source_node_id), "12345678abcdefgh"));
    ASSERT_GT(TransferReceiver::DEFAULT_TRANSFER_INTERVAL, rcv.getInterval());
    ASSERT_LT(TransferReceiver::MIN_TRANSFER_INTERVAL, rcv.getInterval());
    ASSERT_EQ(1000, rcv.getLastTransferTimestamp());
    ASSERT_FALSE(rcv.isTimedOut(1000));
    ASSERT_FALSE(rcv.isTimedOut(5000));
    ASSERT_TRUE(rcv.isTimedOut(60000000));

    /*
     * Single-frame transfers
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "qwe",      0, true, 1, 2000)));   // Previous TID
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "qwe",      0, true, 2, 2100)));   // Wrong iface
    CHECK_SINGLE_FRAME(rcv.addFrame(gen(0, "qwe",      0, true, 2, 2200)));

    ASSERT_FALSE(bufmgr.access(RxFrameGenerator::DEFAULT_NODE_ID));          // Buffer must be removed
    ASSERT_GT(TransferReceiver::DEFAULT_TRANSFER_INTERVAL, rcv.getInterval());
    ASSERT_EQ(2200, rcv.getLastTransferTimestamp());

    CHECK_SINGLE_FRAME(rcv.addFrame(gen(0, "",         0, true, 3, 2500)));
    ASSERT_EQ(2500, rcv.getLastTransferTimestamp());

    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "",         0, true, 0, 3000)));   // Old TID
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "",         0, true, 15,3100)));   // Old TID
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "",         0, true, 3, 3200)));   // Old TID
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "",         0, true, 0, 3300)));   // Old TID, wrong iface
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "",         0, true, 15,3400)));   // Old TID, wrong iface
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "",         0, true, 3, 3500)));   // Old TID, wrong iface
    CHECK_SINGLE_FRAME(rcv.addFrame(gen(0, "",         0, true, 8, 3600)));
    ASSERT_EQ(3600, rcv.getLastTransferTimestamp());

    /*
     * Timeouts
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "qwe",      0, true, 9, 100000))); // Wrong iface - ignored
    CHECK_SINGLE_FRAME(rcv.addFrame(gen(1, "qwe",      0, true, 9, 600000))); // Accepted due to iface timeout
    ASSERT_EQ(600000, rcv.getLastTransferTimestamp());

    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "qwe",      0, true, 10, 600100)));// Ignored - old iface 0
    CHECK_SINGLE_FRAME(rcv.addFrame(gen(1, "qwe",      0, true, 10, 600100)));
    ASSERT_EQ(600100, rcv.getLastTransferTimestamp());

    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "qwe",      0, true, 10, 600100)));// Old TID
    CHECK_SINGLE_FRAME(rcv.addFrame(gen(0, "qwe",      0, true, 10, 100000000)));
    ASSERT_EQ(100000000, rcv.getLastTransferTimestamp());

    CHECK_SINGLE_FRAME(rcv.addFrame(gen(0, "qwe",      0, true, 11, 100000100)));
    ASSERT_EQ(100000100, rcv.getLastTransferTimestamp());

    ASSERT_TRUE(rcv.isTimedOut(900000000));
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "12345678", 0, false, 11, 900000000)));// Global timeout
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "12345678", 0, false, 11, 900000100)));// Wrong iface
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "qwe",      1, true,  11, 900000200)));// Wrong iface
    CHECK_COMPLETE(    rcv.addFrame(gen(1, "qwe",      1, true,  11, 900000200)));
    ASSERT_EQ(900000000, rcv.getLastTransferTimestamp());
    ASSERT_FALSE(rcv.isTimedOut(1000));
    ASSERT_FALSE(rcv.isTimedOut(900000200));
    ASSERT_TRUE(rcv.isTimedOut(1000 * 1000000));
    ASSERT_LT(TransferReceiver::DEFAULT_TRANSFER_INTERVAL, rcv.getInterval());
    ASSERT_LE(TransferReceiver::MIN_TRANSFER_INTERVAL, rcv.getInterval());
    ASSERT_GE(TransferReceiver::MAX_TRANSFER_INTERVAL, rcv.getInterval());
    ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.source_node_id), "12345678qwe"));

    /*
     * Buffer cleanup
     */
    ASSERT_TRUE(bufmgr.access(gen.source_node_id));
    context.receiver = TransferReceiver();
    ASSERT_FALSE(bufmgr.access(gen.source_node_id));
}


TEST(TransferReceiver, OutOfBufferSpace_32bytes)
{
    Context<32> context;
    RxFrameGenerator gen(789, uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST);
    uavcan::TransferReceiver& rcv = context.receiver;
    uavcan::ITransferBufferManager& bufmgr = context.bufmgr;

    /*
     * Simple transfer, maximum buffer length
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "12345678", 0, false, 10, 100000000))); // 8
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "12345678", 1, false, 10, 100000100))); // 16
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "12345678", 2, false, 10, 100000200))); // 24
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "12345678", 3, false, 10, 100000300))); // 32
    CHECK_COMPLETE(    rcv.addFrame(gen(1, "",         4, true,  10, 100000400))); // 32

    ASSERT_EQ(100000000, rcv.getLastTransferTimestamp());
    ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.source_node_id), "12345678123456781234567812345678"));

    /*
     * Transfer longer than available buffer space
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "12345678", 0, false, 11, 100001000))); // 8
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "12345678", 1, false, 11, 100001100))); // 16
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "12345678", 2, false, 11, 100001200))); // 24
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "12345678", 3, false, 11, 100001200))); // 32
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "12345678", 4, true,  11, 100001300))); // 40 // EOT, ignored - lost sync

    ASSERT_EQ(100000000, rcv.getLastTransferTimestamp());
    ASSERT_FALSE(bufmgr.access(gen.source_node_id));          // Buffer should be removed
}


TEST(TransferReceiver, UnterminatedTransfer)
{
    Context<256> context;
    RxFrameGenerator gen(789, uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST);
    uavcan::TransferReceiver& rcv = context.receiver;
    uavcan::ITransferBufferManager& bufmgr = context.bufmgr;

    std::string content;
    for (int i = 0; i <= uavcan::Frame::FRAME_INDEX_MAX; i++)
    {
        CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "12345678", i, false, 0, 1000 + i))); // Last one will be dropped
        content += "12345678";
    }
    CHECK_COMPLETE(rcv.addFrame(gen(1, "12345678", uavcan::Frame::FRAME_INDEX_MAX, true, 0, 1100)));
    ASSERT_EQ(1000, rcv.getLastTransferTimestamp());
    ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.source_node_id), content));
}


TEST(TransferReceiver, OutOfOrderFrames)
{
    Context<32> context;
    RxFrameGenerator gen(789, uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST);
    uavcan::TransferReceiver& rcv = context.receiver;
    uavcan::ITransferBufferManager& bufmgr = context.bufmgr;

    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "12345678", 0, false, 10, 100000000)));
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "--------", 3, false, 10, 100000100)));  // Out of order
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "--------", 2, true,  10, 100000200)));  // Out of order
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "qwertyui", 1, false, 10, 100000300)));
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "--------", 4, true,  10, 100000200)));  // Out of order
    CHECK_COMPLETE(    rcv.addFrame(gen(1, "abcd",     2, true,  10, 100000400)));

    ASSERT_EQ(100000000, rcv.getLastTransferTimestamp());
    ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.source_node_id), "12345678qwertyuiabcd"));
}


TEST(TransferReceiver, IntervalMeasurement)
{
    Context<32> context;
    RxFrameGenerator gen(789, uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST);
    uavcan::TransferReceiver& rcv = context.receiver;
    uavcan::ITransferBufferManager& bufmgr = context.bufmgr;

    static const int INTERVAL = 1000;
    uavcan::TransferID tid;
    uint64_t timestamp = 100000000;

    for (int i = 0; i < 1000; i++)
    {
        CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "12345678", 0, false, tid.get(), timestamp)));
        CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "qwertyui", 1, false, tid.get(), timestamp)));
        CHECK_COMPLETE(    rcv.addFrame(gen(1, "abcd",     2, true,  tid.get(), timestamp)));

        ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.source_node_id), "12345678qwertyuiabcd"));
        ASSERT_EQ(timestamp, rcv.getLastTransferTimestamp());

        timestamp += INTERVAL;
        tid.increment();
    }

    ASSERT_EQ(INTERVAL, rcv.getInterval());
}
