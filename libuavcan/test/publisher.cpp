/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/publisher.hpp>
#include <uavcan/mavlink/Message.hpp>
#include "clock.hpp"
#include "internal/transport/can/can.hpp"


TEST(Publisher, Basic)
{
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * 8, uavcan::MemPoolBlockSize> pool;
    uavcan::PoolManager<1> poolmgr;
    poolmgr.addPool(&pool);

    SystemClockMock clock_mock(100);
    CanDriverMock can_driver(2, clock_mock);

    uavcan::OutgoingTransferRegistry<8> out_trans_reg(poolmgr);

    uavcan::Scheduler sch(can_driver, poolmgr, clock_mock, out_trans_reg, uavcan::NodeID(1));

    uavcan::MarshalBufferProvider<> buffer_provider;

    uavcan::Publisher<uavcan::mavlink::Message> publisher(sch, buffer_provider);

    std::cout <<
        "sizeof(uavcan::Publisher<uavcan::mavlink::Message>): " <<
        sizeof(uavcan::Publisher<uavcan::mavlink::Message>) << std::endl;

    // Manual type registration - we can't rely on the GDTR state
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::mavlink::Message> _registrator;

    /*
     * Message layout:
     * uint8 seq
     * uint8 sysid
     * uint8 compid
     * uint8 msgid
     * uint8[<256] payload
     */
    uavcan::mavlink::Message msg;
    msg.seq = 0x42;
    msg.sysid = 0x72;
    msg.compid = 0x08;
    msg.msgid = 0xa5;
    msg.payload = "Msg";

    const uint8_t expected_transfer_payload[] = {0x42, 0x72, 0x08, 0xa5, 'M', 's', 'g'};
    const uint64_t tx_timeout_usec = publisher.getDefaultTxTimeout().toUSec();

    /*
     * Broadcast
     */
    {
        ASSERT_LT(0, publisher.broadcast(msg));

        // uint_fast16_t data_type_id, TransferType transfer_type, NodeID src_node_id, NodeID dst_node_id,
        // uint_fast8_t frame_index, TransferID transfer_id, bool last_frame = false
        uavcan::Frame expected_frame(uavcan::mavlink::Message::DefaultDataTypeID, uavcan::TransferTypeMessageBroadcast,
                                     sch.getDispatcher().getSelfNodeID(), uavcan::NodeID::Broadcast, 0, 0, true);
        expected_frame.setPayload(expected_transfer_payload, 7);

        uavcan::CanFrame expected_can_frame;
        ASSERT_TRUE(expected_frame.compile(expected_can_frame));

        ASSERT_TRUE(can_driver.ifaces[0].matchAndPopTx(expected_can_frame, tx_timeout_usec + 100));
        ASSERT_TRUE(can_driver.ifaces[1].matchAndPopTx(expected_can_frame, tx_timeout_usec + 100));
        ASSERT_TRUE(can_driver.ifaces[0].tx.empty());
        ASSERT_TRUE(can_driver.ifaces[1].tx.empty());

        // Second shot - checking the transfer ID
        ASSERT_LT(0, publisher.broadcast(msg));

        expected_frame = uavcan::Frame(uavcan::mavlink::Message::DefaultDataTypeID, uavcan::TransferTypeMessageBroadcast,
                                       sch.getDispatcher().getSelfNodeID(), uavcan::NodeID::Broadcast, 0, 1, true);
        expected_frame.setPayload(expected_transfer_payload, 7);
        ASSERT_TRUE(expected_frame.compile(expected_can_frame));

        ASSERT_TRUE(can_driver.ifaces[0].matchAndPopTx(expected_can_frame, tx_timeout_usec + 100));
        ASSERT_TRUE(can_driver.ifaces[1].matchAndPopTx(expected_can_frame, tx_timeout_usec + 100));
        ASSERT_TRUE(can_driver.ifaces[0].tx.empty());
        ASSERT_TRUE(can_driver.ifaces[1].tx.empty());
    }

    clock_mock.advance(1000);

    /*
     * Unicast
     */
    {
        ASSERT_LT(0, publisher.unicast(msg, 0x44));

        // uint_fast16_t data_type_id, TransferType transfer_type, NodeID src_node_id, NodeID dst_node_id,
        // uint_fast8_t frame_index, TransferID transfer_id, bool last_frame = false
        uavcan::Frame expected_frame(uavcan::mavlink::Message::DefaultDataTypeID, uavcan::TransferTypeMessageUnicast,
                                     sch.getDispatcher().getSelfNodeID(), uavcan::NodeID(0x44), 0, 0, true);
        expected_frame.setPayload(expected_transfer_payload, 7);

        uavcan::CanFrame expected_can_frame;
        ASSERT_TRUE(expected_frame.compile(expected_can_frame));

        ASSERT_TRUE(can_driver.ifaces[0].matchAndPopTx(expected_can_frame, tx_timeout_usec + 100 + 1000));
        ASSERT_TRUE(can_driver.ifaces[1].matchAndPopTx(expected_can_frame, tx_timeout_usec + 100 + 1000));
        ASSERT_TRUE(can_driver.ifaces[0].tx.empty());
        ASSERT_TRUE(can_driver.ifaces[1].tx.empty());
    }

    /*
     * Misc
     */
    ASSERT_TRUE(uavcan::GlobalDataTypeRegistry::instance().isFrozen());
}
