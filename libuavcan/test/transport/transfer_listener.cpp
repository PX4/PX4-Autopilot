/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <algorithm>
#include <gtest/gtest.h>
#include <uavcan/internal/transport/transfer_listener.hpp>


static uavcan::RxFrame makeFrame()
{
    uavcan::RxFrame frame;
    frame.data_type_id = 123;
    frame.last_frame = true;
    frame.source_node_id = 42;
    frame.transfer_id.increment();
    frame.ts_monotonic = 123;
    frame.ts_utc = 456;
    frame.payload_len = uavcan::RxFrame::PAYLOAD_LEN_MAX;
    for (int i = 0; i < uavcan::RxFrame::PAYLOAD_LEN_MAX; i++)
        frame.payload[i] = i;
    return frame;
}


static bool match(const uavcan::IncomingTransfer& it, const uavcan::RxFrame& frame,
                  const uint8_t* payload, unsigned int payload_len)
{
    // Fields extracted from the frame struct
    EXPECT_EQ(it.getMonotonicTimestamp(), frame.ts_monotonic);
    EXPECT_EQ(it.getUtcTimestamp(), frame.ts_utc);
    EXPECT_EQ(it.getSourceNodeID(), frame.source_node_id);
    EXPECT_EQ(it.getTransferID(), frame.transfer_id);
    EXPECT_EQ(it.getTransferType(), frame.transfer_type);

    // Payload comparison
    static const unsigned int BUFLEN = 1024;
    uint8_t buf_reference[BUFLEN], buf_actual[BUFLEN];

    if (payload_len > BUFLEN)
    {
        std::cout << "match(): Payload is too long" << std::endl;
        exit(1);
    }

    std::fill(buf_reference, buf_reference + BUFLEN, 0);
    std::fill(buf_actual, buf_actual + BUFLEN, 0);
    std::copy(payload, payload + payload_len, buf_reference);

    EXPECT_EQ(payload_len, it.read(0, buf_actual, payload_len * 3));
    EXPECT_EQ(0, it.read(payload_len, buf_actual, payload_len * 3));

    return std::equal(buf_reference, buf_reference + BUFLEN, buf_actual);
}


TEST(SingleFrameIncomingTransfer, Basic)
{
    using uavcan::RxFrame;
    using uavcan::SingleFrameIncomingTransfer;

    const RxFrame frame = makeFrame();
    SingleFrameIncomingTransfer it(frame);

    ASSERT_TRUE(match(it, frame, frame.payload, frame.payload_len));
}


TEST(MultiFrameIncomingTransfer, Basic)
{
    using uavcan::RxFrame;
    using uavcan::MultiFrameIncomingTransfer;

    uavcan::PoolManager<1> poolmgr;                 // We don't need dynamic memory
    uavcan::TransferBufferManager<256, 1> bufmgr(&poolmgr);

    const RxFrame frame = makeFrame();
    uavcan::TransferBufferManagerKey bufmgr_key(frame.source_node_id, frame.transfer_type);
    uavcan::TransferBufferAccessor tba(&bufmgr, bufmgr_key);

    MultiFrameIncomingTransfer it(frame.ts_monotonic, frame.ts_utc, frame, tba);

    /*
     * Empty read must fail
     */
    uint8_t data_byte = 0;
    ASSERT_GT(0, it.read(0, &data_byte, 1));  // Error - no such buffer

    /*
     * Filling the test data
     */
    const std::string data = "123Hello world";
    const uint8_t* const data_ptr = reinterpret_cast<const uint8_t*>(data.c_str());
    ASSERT_FALSE(bufmgr.access(bufmgr_key));
    ASSERT_TRUE(bufmgr.create(bufmgr_key));
    ASSERT_EQ(data.length(), bufmgr.access(bufmgr_key)->write(0, data_ptr, data.length()));

    /*
     * Check
     */
    ASSERT_TRUE(match(it, frame, data_ptr, data.length()));

    /*
     * Buffer release
     */
    ASSERT_TRUE(bufmgr.access(bufmgr_key));
    it.release();
    ASSERT_FALSE(bufmgr.access(bufmgr_key));
}

