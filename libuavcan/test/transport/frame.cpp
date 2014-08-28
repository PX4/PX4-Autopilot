/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <string>
#include <gtest/gtest.h>
#include <uavcan/transport/transfer.hpp>
#include "../clock.hpp"
#include "can/can.hpp"


TEST(Frame, FrameParseCompile)
{
    using uavcan::Frame;
    using uavcan::CanFrame;
    using uavcan::TransferID;
    using uavcan::TransferType;

    Frame frame;

    const uint32_t can_id =
        (2 << 0)   |      //    Transfer ID
        (1 << 3)   |      //    Last Frame
        (29 << 4)  |      //    Frame Index
        (42 << 10) |      //    Source Node ID
        (uavcan::TransferTypeMessageBroadcast << 17)  |
        (456 << 19);      //    Data Type ID

    const std::string payload_string = "hello";

    /*
     * Parse
     */
    // Invalid CAN frames
    ASSERT_FALSE(frame.parse(CanFrame(can_id | CanFrame::FlagRTR, (const uint8_t*)"", 0)));
    ASSERT_FALSE(frame.parse(makeCanFrame(can_id, payload_string, STD)));

    // Valid
    ASSERT_TRUE(frame.parse(makeCanFrame(can_id, payload_string, EXT)));

    EXPECT_EQ(TransferID(2), frame.getTransferID());
    EXPECT_TRUE(frame.isLast());
    EXPECT_EQ(29, frame.getIndex());
    EXPECT_EQ(uavcan::NodeID(42), frame.getSrcNodeID());
    EXPECT_EQ(uavcan::TransferTypeMessageBroadcast, frame.getTransferType());
    EXPECT_EQ(456, frame.getDataTypeID().get());

    EXPECT_EQ(payload_string.length(), frame.getPayloadLen());
    EXPECT_TRUE(std::equal(frame.getPayloadPtr(), frame.getPayloadPtr() + frame.getPayloadLen(),
                           payload_string.begin()));

    /*
     * Compile
     */
    CanFrame can_frame;
    ASSERT_TRUE(frame.parse(makeCanFrame(can_id, payload_string, EXT)));

    ASSERT_TRUE(frame.compile(can_frame));
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


TEST(Frame, FrameParsing)
{
    using uavcan::Frame;
    using uavcan::CanFrame;
    using uavcan::NodeID;
    using uavcan::TransferID;

    CanFrame can;
    Frame frame;
    ASSERT_FALSE(frame.parse(can));

    for (unsigned i = 0; i < sizeof(CanFrame::data); i++)
    {
        can.data[i] = uint8_t(i | (i << 4));
    }

    // CAN ID field order: Transfer ID, Last Frame, Frame Index, Source Node ID, Transfer Type, Data Type ID

    /*
     * SFT broadcast
     */
    can.id = CanFrame::FlagEFF |
             (2 << 0) | (1 << 3) | (0 << 4) | (42 << 10) | (uavcan::TransferTypeMessageBroadcast << 17) | (456 << 19);

    ASSERT_TRUE(frame.parse(can));
    ASSERT_TRUE(frame.isFirst());
    ASSERT_TRUE(frame.isLast());
    ASSERT_EQ(0, frame.getIndex());
    ASSERT_EQ(NodeID(42), frame.getSrcNodeID());
    ASSERT_EQ(NodeID::Broadcast, frame.getDstNodeID());
    ASSERT_EQ(456, frame.getDataTypeID().get());
    ASSERT_EQ(TransferID(2), frame.getTransferID());
    ASSERT_EQ(uavcan::TransferTypeMessageBroadcast, frame.getTransferType());
    ASSERT_EQ(sizeof(CanFrame::data), frame.getMaxPayloadLen());

    /*
     * SFT addressed
     */
    can.id = CanFrame::FlagEFF |
             (2 << 0) | (1 << 3) | (0 << 4) | (42 << 10) | (uavcan::TransferTypeMessageUnicast << 17) | (456 << 19);

    ASSERT_FALSE(frame.parse(can));  // No payload - failure

    can.dlc = 1;
    can.data[0] = 0xFF;
    ASSERT_FALSE(frame.parse(can));   // Invalid first byte - failure

    can.data[0] = 127;
    ASSERT_TRUE(frame.parse(can));

    ASSERT_TRUE(frame.isFirst());
    ASSERT_TRUE(frame.isLast());
    ASSERT_EQ(0, frame.getIndex());
    ASSERT_EQ(NodeID(42), frame.getSrcNodeID());
    ASSERT_EQ(NodeID(127), frame.getDstNodeID());
    ASSERT_EQ(456, frame.getDataTypeID().get());
    ASSERT_EQ(TransferID(2), frame.getTransferID());
    ASSERT_EQ(uavcan::TransferTypeMessageUnicast, frame.getTransferType());
    ASSERT_EQ(sizeof(CanFrame::data) - 1, frame.getMaxPayloadLen());

    /*
     * MFT invalid - unterminated transfer
     */
    can.id = CanFrame::FlagEFF |
             (2 << 0) | (0 << 3) | (Frame::MaxIndex << 4) | (42 << 10) |
             (uavcan::TransferTypeMessageUnicast << 17) | (456 << 19);

    ASSERT_FALSE(frame.parse(can));

    /*
     * MFT invalid - invalid frame index
     */
    can.id = CanFrame::FlagEFF |
             (2 << 0) | (0 << 3) | (63 << 4) | (42 << 10) | (uavcan::TransferTypeMessageUnicast << 17) | (456 << 19);

    ASSERT_FALSE(frame.parse(can));

    /*
     * Malformed frames
     */
    can.id = CanFrame::FlagEFF |
             (2 << 0) | (1 << 3) | (0 << 4) | (42 << 10) | (uavcan::TransferTypeMessageUnicast << 17) | (456 << 19);
    can.dlc = 3;
    can.data[0] = 42;
    ASSERT_FALSE(frame.parse(can)); // Src == Dst Node ID
    can.data[0] = 41;
    ASSERT_TRUE(frame.parse(can));

    can.id = CanFrame::FlagEFF | // cppcheck-suppress duplicateExpression
             (2 << 0) | (1 << 3) | (0 << 4) | (0 << 10) | (uavcan::TransferTypeMessageUnicast << 17) | (456 << 19);
    ASSERT_FALSE(frame.parse(can)); // Broadcast Src Node ID
}


TEST(Frame, RxFrameParse)
{
    using uavcan::Frame;
    using uavcan::RxFrame;
    using uavcan::CanFrame;
    using uavcan::CanRxFrame;

    CanRxFrame can_rx_frame;
    RxFrame rx_frame;

    // Failure
    ASSERT_FALSE(rx_frame.parse(can_rx_frame));

    // Valid
    can_rx_frame.ts_mono = uavcan::MonotonicTime::fromUSec(1);  // Zero is not allowed
    can_rx_frame.id =
        CanFrame::FlagEFF |
        (2 << 0)   |      //    Transfer ID
        (1 << 3)   |      //    Last Frame
        (29 << 4)  |      //    Frame Index
        (42 << 10) |      //    Source Node ID
        (uavcan::TransferTypeMessageBroadcast << 17)  |
        (456 << 19);      //    Data Type ID

    ASSERT_TRUE(rx_frame.parse(can_rx_frame));
    ASSERT_EQ(1, rx_frame.getMonotonicTimestamp().toUSec());
    ASSERT_EQ(0, rx_frame.getIfaceIndex());

    can_rx_frame.ts_mono = tsMono(123);
    can_rx_frame.iface_index = 2;

    Frame frame(456, uavcan::TransferTypeMessageBroadcast, 1, uavcan::NodeID::Broadcast, 0, 0);
    ASSERT_TRUE(frame.compile(can_rx_frame));

    ASSERT_TRUE(rx_frame.parse(can_rx_frame));
    ASSERT_EQ(123, rx_frame.getMonotonicTimestamp().toUSec());
    ASSERT_EQ(2, rx_frame.getIfaceIndex());
    ASSERT_EQ(456, rx_frame.getDataTypeID().get());
    ASSERT_EQ(uavcan::TransferTypeMessageBroadcast, rx_frame.getTransferType());
}


TEST(Frame, FrameToString)
{
    using uavcan::Frame;
    using uavcan::RxFrame;

    // RX frame default
    RxFrame rx_frame;
    EXPECT_EQ("dtid=65535 tt=4 snid=255 dnid=255 idx=0 last=0 tid=0 payload=[] ts_m=0.000000 ts_utc=0.000000 iface=0",
              rx_frame.toString());

    // RX frame max len
    rx_frame = RxFrame(Frame(uavcan::DataTypeID::Max, uavcan::TransferTypeMessageUnicast,
                             uavcan::NodeID::Max, uavcan::NodeID::Max - 1, Frame::MaxIndex,
                             uavcan::TransferID::Max, true),
                       uavcan::MonotonicTime::getMax(), uavcan::UtcTime::getMax(), 3);

    uint8_t data[8];
    for (unsigned i = 0; i < sizeof(data); i++)
    {
        data[i] = uint8_t(i);
    }
    rx_frame.setPayload(data, sizeof(data));

    EXPECT_EQ("dtid=1023 tt=3 snid=127 dnid=126 idx=62 last=1 tid=7 payload=[00 01 02 03 04 05 06] "
              "ts_m=18446744073709.551615 ts_utc=18446744073709.551615 iface=3",
              rx_frame.toString());

    // Plain frame default
    Frame frame;
    EXPECT_EQ("dtid=65535 tt=4 snid=255 dnid=255 idx=0 last=0 tid=0 payload=[]", frame.toString());

    // Plain frame max len
    frame = rx_frame;
    EXPECT_EQ("dtid=1023 tt=3 snid=127 dnid=126 idx=62 last=1 tid=7 payload=[00 01 02 03 04 05 06]", frame.toString());
}
