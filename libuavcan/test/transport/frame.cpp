/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <string>
#include <gtest/gtest.h>
#include <uavcan/transport/transfer.hpp>
#include "../clock.hpp"
#include "can/can.hpp"


TEST(Frame, MessageParseCompile)
{
    using uavcan::Frame;
    using uavcan::CanFrame;
    using uavcan::TransferID;
    using uavcan::TransferType;

    Frame frame;

    /*
     * Priority (LOW, NORMAL, HIGH)
     * Message Type ID
     * Source Node ID
     * BroadcastNotUnicast
     * Frame Index
     * Last Frame
     * Transfer ID
     */
    const uint32_t can_id =
        (1 << 27) |     // Priority
        (2000 << 16) |  // Message Type ID
        (42 << 9) |     // Source Node ID
        (1 << 8) |      // BroadcastNotUnicast
        (11 << 4) |     // Frame Index
        (1 << 3) |      // Last Frame
        (2 << 0);       // Transfer ID

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
    EXPECT_EQ(11, frame.getIndex());
    EXPECT_EQ(uavcan::NodeID(42), frame.getSrcNodeID());
    EXPECT_TRUE(frame.getDstNodeID().isBroadcast());
    EXPECT_EQ(uavcan::TransferTypeMessageBroadcast, frame.getTransferType());
    EXPECT_EQ(2000, frame.getDataTypeID().get());
    EXPECT_EQ(uavcan::TransferPriorityNormal, frame.getPriority());

    EXPECT_EQ(payload_string.length(), frame.getPayloadLen());
    EXPECT_TRUE(std::equal(frame.getPayloadPtr(), frame.getPayloadPtr() + frame.getPayloadLen(),
                           payload_string.begin()));

    std::cout << frame.toString() << std::endl;

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


TEST(Frame, ServiceParseCompile)
{
    using uavcan::Frame;
    using uavcan::CanFrame;
    using uavcan::TransferID;
    using uavcan::TransferType;

    Frame frame;

    /*
     * Priority = SERVICE
     * RequestNotResponse
     * Service Type ID
     * Source Node ID
     * Frame Index
     * Last Frame
     * Transfer ID
     */
    const uint32_t can_id =
        (2 << 27) |     // Priority (Service)
        (1 << 26) |     // RequestNotResponse
        (500 << 17) |   // Service Type ID
        (42 << 10) |    // Source Node ID
        (60 << 4) |     // Frame Index
        (1 << 3) |      // Last Frame
        (5 << 0);       // Transfer ID

    const std::string payload_string = "\x42hello"; // dst = 0x42

    /*
     * Parse
     */
    // Invalid CAN frames
    ASSERT_FALSE(frame.parse(CanFrame(can_id | CanFrame::FlagRTR, (const uint8_t*)"", 0)));
    ASSERT_FALSE(frame.parse(makeCanFrame(can_id, payload_string, STD)));

    // Valid
    ASSERT_TRUE(frame.parse(makeCanFrame(can_id, payload_string, EXT)));

    EXPECT_EQ(TransferID(5), frame.getTransferID());
    EXPECT_TRUE(frame.isLast());
    EXPECT_EQ(60, frame.getIndex());
    EXPECT_EQ(uavcan::NodeID(42), frame.getSrcNodeID());
    EXPECT_EQ(uavcan::NodeID(0x42), frame.getDstNodeID());
    EXPECT_EQ(uavcan::TransferTypeServiceRequest, frame.getTransferType());
    EXPECT_EQ(500, frame.getDataTypeID().get());
    EXPECT_EQ(uavcan::TransferPriorityService, frame.getPriority());

    EXPECT_EQ(payload_string.length(), frame.getPayloadLen() + 1);
    EXPECT_TRUE(std::equal(frame.getPayloadPtr(), frame.getPayloadPtr() + frame.getPayloadLen(),
                           payload_string.begin() + 1));

    std::cout << frame.toString() << std::endl;

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


TEST(Frame, AnonymousParseCompile)
{
    using uavcan::Frame;
    using uavcan::CanFrame;
    using uavcan::TransferID;
    using uavcan::TransferType;

    Frame frame;

    /*
     * Priority (LOW, NORMAL, HIGH)
     * Message Type ID
     * Source Node ID
     * BroadcastNotUnicast
     * Frame Index
     * Last Frame
     * Transfer ID
     */
    const uint32_t can_id =
        (0 << 27) |     // Priority (high)
        (2000 << 16) |  // Message Type ID
        (0 << 9) |      // Source Node ID
        (1 << 8) |      // BroadcastNotUnicast
        (11 << 4) |     // Frame Index (will be ignored)
        (1 << 3) |      // Last Frame  (will be ignored)
        (2 << 0);       // Transfer ID (will be ignored)

    const std::string payload_string = "\x01\x02\x03\x04";
    const uint8_t payload_sum        =     1 + 2 + 3 + 4;

    /*
     * Parse
     */
    ASSERT_TRUE(frame.parse(makeCanFrame(can_id, payload_string, EXT)));

    EXPECT_EQ(TransferID(0), frame.getTransferID());
    EXPECT_TRUE(frame.isLast());
    EXPECT_EQ(0, frame.getIndex());
    EXPECT_TRUE(frame.getSrcNodeID().isBroadcast());
    EXPECT_TRUE(frame.getDstNodeID().isBroadcast());
    EXPECT_EQ(uavcan::TransferTypeMessageBroadcast, frame.getTransferType());
    EXPECT_EQ(2000, frame.getDataTypeID().get());
    EXPECT_EQ(uavcan::TransferPriorityHigh, frame.getPriority());

    EXPECT_EQ(payload_string.length(), frame.getPayloadLen());
    EXPECT_TRUE(std::equal(frame.getPayloadPtr(), frame.getPayloadPtr() + frame.getPayloadLen(),
                           payload_string.begin()));

    std::cout << frame.toString() << std::endl;

    /*
     * Compile
     */
    CanFrame can_frame;
    ASSERT_TRUE(frame.parse(makeCanFrame(can_id, payload_string, EXT)));

    ASSERT_TRUE(frame.compile(can_frame));
    ASSERT_EQ(can_id & 0xFFFFFF00 & uavcan::CanFrame::MaskExtID,
              can_frame.id & 0xFFFFFF00 & uavcan::CanFrame::MaskExtID);

    EXPECT_EQ(payload_string.length(), can_frame.dlc);
    EXPECT_TRUE(std::equal(can_frame.data, can_frame.data + can_frame.dlc, payload_string.begin()));

    ASSERT_EQ(payload_sum, can_frame.id & 0xFF);

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

    /*
     * Message CAN ID fields and offsets:
     *   27 Priority (LOW=3, NORMAL=1, HIGH=0)
     *   16 Message Type ID
     *   9  Source Node ID
     *   8  BroadcastNotUnicast
     *   4  Frame Index
     *   3  Last Frame
     *   0  Transfer ID
     *
     * Service CAN ID fields and offsets:
     *   27 Priority (SERVICE=2)
     *   26 RequestNotResponse
     *   17 Service Type ID
     *   10 Source Node ID
     *   4  Frame Index
     *   3  Last Frame
     *   0  Transfer ID
     */

    /*
     * SFT message broadcast
     */
    can.id = CanFrame::FlagEFF |
             (2 << 0) |
             (1 << 3) |
             (0 << 4) |
             (1 << 8) |
             (42 << 9) |
             (456 << 16) |
             (1 << 27);

    ASSERT_TRUE(frame.parse(can));
    ASSERT_TRUE(frame.isFirst());
    ASSERT_TRUE(frame.isLast());
    ASSERT_EQ(uavcan::TransferPriorityNormal, frame.getPriority());
    ASSERT_EQ(0, frame.getIndex());
    ASSERT_EQ(NodeID(42), frame.getSrcNodeID());
    ASSERT_EQ(NodeID::Broadcast, frame.getDstNodeID());
    ASSERT_EQ(456, frame.getDataTypeID().get());
    ASSERT_EQ(TransferID(2), frame.getTransferID());
    ASSERT_EQ(uavcan::TransferTypeMessageBroadcast, frame.getTransferType());
    ASSERT_EQ(sizeof(CanFrame::data), frame.getMaxPayloadLen());

    /*
     * SFT message unicast
     */
    can.id = CanFrame::FlagEFF |
             (2 << 0) |
             (1 << 3) |         // Last Frame
             (0 << 4) |         // Frame Index
             (0 << 8) |
             (42 << 9) |
             (456 << 16) |
             (0 << 27);

    ASSERT_FALSE(frame.parse(can));  // No payload - failure

    can.dlc = 1;
    can.data[0] = 0xFF;
    ASSERT_FALSE(frame.parse(can));   // Invalid first byte - failure

    can.data[0] = 127;
    ASSERT_TRUE(frame.parse(can));

    ASSERT_TRUE(frame.isFirst());
    ASSERT_TRUE(frame.isLast());
    ASSERT_EQ(uavcan::TransferPriorityHigh, frame.getPriority());
    ASSERT_EQ(0, frame.getIndex());
    ASSERT_EQ(NodeID(42), frame.getSrcNodeID());
    ASSERT_EQ(NodeID(127), frame.getDstNodeID());
    ASSERT_EQ(456, frame.getDataTypeID().get());
    ASSERT_EQ(TransferID(2), frame.getTransferID());
    ASSERT_EQ(uavcan::TransferTypeMessageUnicast, frame.getTransferType());
    ASSERT_EQ(sizeof(CanFrame::data) - 1, frame.getMaxPayloadLen());

    /*
     * MFT message unicast
     * Invalid - unterminated transfer
     */
    can.id = CanFrame::FlagEFF |
             (2 << 0) |
             (0 << 3) |
             (15 << 4) |
             (0 << 8) |
             (42 << 9) |
             (456 << 16) |
             (0 << 27);

    ASSERT_FALSE(frame.parse(can));

    /*
     * MFT service request
     * Invalid frame index
     */
    can.id = CanFrame::FlagEFF |
        (2 << 27) |     // Priority (Service)
        (1 << 26) |     // RequestNotResponse
        (500 << 17) |   // Service Type ID
        (42 << 10) |    // Source Node ID
        (63 << 4) |     // Frame Index
        (1 << 3) |      // Last Frame
        (5 << 0);       // Transfer ID

    ASSERT_FALSE(frame.parse(can));

    /*
     * Malformed frames
     */
    can.id = CanFrame::FlagEFF |
             (2 << 0) |
             (1 << 3) |
             (15 << 4) |
             (0 << 8) |
             (42 << 9) |
             (456 << 16) |
             (3 << 27);
    can.dlc = 3;
    can.data[0] = 42;
    ASSERT_FALSE(frame.parse(can)); // Src == Dst Node ID
    can.data[0] = 41;
    ASSERT_TRUE(frame.parse(can));
    ASSERT_EQ(uavcan::TransferPriorityLow, frame.getPriority());

    /*
     * Broadcast SNID exceptions
     * Note that last 3 fields are ignored
     */
    can.id = CanFrame::FlagEFF |
             (2 << 27) |     // Priority
             (2000 << 16) |  // Message Type ID
             (0 << 9) |      // Source Node ID
             (1 << 8);       // BroadcastNotUnicast
    ASSERT_FALSE(frame.parse(can));  // Invalid priority

    can.id = CanFrame::FlagEFF |
             (1 << 27) |     // Priority
             (2000 << 16) |  // Message Type ID
             (0 << 9) |      // Source Node ID
             (1 << 8);       // BroadcastNotUnicast
    ASSERT_TRUE(frame.parse(can));
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
    can_rx_frame.id = CanFrame::FlagEFF |
             (2 << 0) |
             (1 << 3) |
             (0 << 4) |
             (1 << 8) |
             (42 << 9) |
             (456 << 16) |
             (1 << 27);

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
    EXPECT_EQ("prio=4 dtid=65535 tt=4 snid=255 dnid=255 idx=0 last=0 tid=0 payload=[] ts_m=0.000000 ts_utc=0.000000 iface=0",
              rx_frame.toString());

    // RX frame max len
    rx_frame = RxFrame(Frame(uavcan::DataTypeID::MaxPossibleDataTypeIDValue, uavcan::TransferTypeMessageUnicast,
                             uavcan::NodeID::Max, uavcan::NodeID::Max - 1,
                             Frame::getMaxIndexForTransferType(uavcan::TransferTypeMessageUnicast),
                             uavcan::TransferID::Max, true),
                       uavcan::MonotonicTime::getMax(), uavcan::UtcTime::getMax(), 3);

    uint8_t data[8];
    for (unsigned i = 0; i < sizeof(data); i++)
    {
        data[i] = uint8_t(i);
    }
    rx_frame.setPayload(data, sizeof(data));

    EXPECT_EQ("prio=1 dtid=2047 tt=3 snid=127 dnid=126 idx=15 last=1 tid=7 payload=[00 01 02 03 04 05 06] "
              "ts_m=18446744073709.551615 ts_utc=18446744073709.551615 iface=3",
              rx_frame.toString());

    // Plain frame default
    Frame frame;
    EXPECT_EQ("prio=4 dtid=65535 tt=4 snid=255 dnid=255 idx=0 last=0 tid=0 payload=[]", frame.toString());

    // Plain frame max len
    frame = rx_frame;
    EXPECT_EQ("prio=1 dtid=2047 tt=3 snid=127 dnid=126 idx=15 last=1 tid=7 payload=[00 01 02 03 04 05 06]",
              frame.toString());
}
