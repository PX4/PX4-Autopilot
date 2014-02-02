/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <string>
#include <gtest/gtest.h>
#include <uavcan/internal/transport/transfer.hpp>
#include "../common.hpp"


TEST(Transfer, TransferID)
{
    using uavcan::TransferID;

    // Tests below are based on this assumption
    ASSERT_EQ(16, 1 << TransferID::BITLEN);

    /*
     * forwardDistance()
     */
    EXPECT_EQ(0, TransferID(0).forwardDistance(0));
    EXPECT_EQ(1, TransferID(0).forwardDistance(1));
    EXPECT_EQ(15, TransferID(0).forwardDistance(15));

    EXPECT_EQ(0, TransferID(7).forwardDistance(7));
    EXPECT_EQ(15, TransferID(7).forwardDistance(6));
    EXPECT_EQ(1, TransferID(7).forwardDistance(8));

    EXPECT_EQ(9, TransferID(10).forwardDistance(3));
    EXPECT_EQ(7, TransferID(3).forwardDistance(10));

    EXPECT_EQ(8, TransferID(6).forwardDistance(14));
    EXPECT_EQ(8, TransferID(14).forwardDistance(6));

    EXPECT_EQ(1, TransferID(14).forwardDistance(15));
    EXPECT_EQ(2, TransferID(14).forwardDistance(0));
    EXPECT_EQ(4, TransferID(14).forwardDistance(2));

    EXPECT_EQ(15, TransferID(15).forwardDistance(14));
    EXPECT_EQ(14, TransferID(0).forwardDistance(14));
    EXPECT_EQ(12, TransferID(2).forwardDistance(14));

    /*
     * Misc
     */
    EXPECT_TRUE(TransferID(2) == TransferID(2));
    EXPECT_FALSE(TransferID(2) != TransferID(2));
    EXPECT_FALSE(TransferID(2) == TransferID(8));
    EXPECT_TRUE(TransferID(2) != TransferID(8));

    TransferID tid;
    for (int i = 0; i < 999; i++)
    {
        ASSERT_EQ(i & ((1 << TransferID::BITLEN) - 1), tid.get());
        const TransferID copy = tid;
        tid.increment();
        ASSERT_EQ(1, copy.forwardDistance(tid));
        ASSERT_EQ(15, tid.forwardDistance(copy));
        ASSERT_EQ(0, tid.forwardDistance(tid));
    }
}

TEST(Transfer, FrameParseCompile)
{
    using uavcan::Frame;
    using uavcan::CanFrame;
    using uavcan::TransferID;
    using uavcan::TransferType;

    Frame frame;

    const uint32_t can_id =
        (2 << 0)   |      //    Transfer ID
        (1 << 4)   |      //    Last Frame
        (29 << 5)  |      //    Frame Index
        (42 << 10) |      //    Source Node ID
        (3 << 17)  |      //    Transfer Type
        (456 << 19);      //    Data Type ID

    const std::string payload_string = "hello";

    /*
     * Parse
     */
    // Invalid CAN frames
    ASSERT_FALSE(frame.parse(CanFrame(can_id | CanFrame::FLAG_RTR, (const uint8_t*)"", 0)));
    ASSERT_FALSE(frame.parse(makeCanFrame(can_id, payload_string, STD)));

    // Valid
    ASSERT_TRUE(frame.parse(makeCanFrame(can_id, payload_string, EXT)));

    EXPECT_EQ(TransferID(2), frame.transfer_id);
    EXPECT_TRUE(frame.last_frame);
    EXPECT_EQ(29, frame.frame_index);
    EXPECT_EQ(42, frame.source_node_id);
    EXPECT_EQ(TransferType(3), frame.transfer_type);
    EXPECT_EQ(456, frame.data_type_id);

    EXPECT_EQ(payload_string.length(), frame.payload_len);
    EXPECT_TRUE(std::equal(frame.payload, frame.payload + frame.payload_len, payload_string.begin()));

    // Default
    ASSERT_TRUE(frame.parse(CanFrame(CanFrame::FLAG_EFF, (const uint8_t*)"", 0)));
    ASSERT_EQ(Frame(), frame);

    /*
     * Compile
     */
    // Default
    frame = Frame();
    CanFrame can_frame = frame.compile();
    ASSERT_EQ(can_frame.id, CanFrame::FLAG_EFF);

    // Custom
    ASSERT_TRUE(frame.parse(makeCanFrame(can_id, payload_string, EXT)));

    can_frame = frame.compile();
    ASSERT_EQ(can_frame, makeCanFrame(can_id, payload_string, EXT));

    EXPECT_EQ(payload_string.length(), can_frame.dlc);
    EXPECT_TRUE(std::equal(can_frame.data, can_frame.data + can_frame.dlc, payload_string.begin()));

    /*
     * Comparison
     */
    ASSERT_FALSE(Frame() == frame);
    ASSERT_TRUE(Frame() != frame);
    frame = Frame();
    ASSERT_TRUE(Frame() == frame);
    ASSERT_FALSE(Frame() != frame);
}


TEST(Transfer, RxFrameParseCompile)
{
    using uavcan::Frame;
    using uavcan::RxFrame;
    using uavcan::CanFrame;
    using uavcan::CanRxFrame;

    CanRxFrame can_rx_frame;
    RxFrame rx_frame;

    // Failure
    ASSERT_FALSE(rx_frame.parse(can_rx_frame));

    // Default
    can_rx_frame.frame.id = CanFrame::FLAG_EFF;
    ASSERT_TRUE(rx_frame.parse(can_rx_frame));
    ASSERT_EQ(0, rx_frame.timestamp);
    ASSERT_EQ(0, rx_frame.iface_index);

    // Custom
    can_rx_frame.timestamp = 123;
    can_rx_frame.iface_index = 2;
    ASSERT_TRUE(rx_frame.parse(can_rx_frame));
    ASSERT_EQ(123, rx_frame.timestamp);
    ASSERT_EQ(2, rx_frame.iface_index);
}
