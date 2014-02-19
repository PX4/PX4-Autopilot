/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <algorithm>
#include <gtest/gtest.h>
#include "transfer_test_helpers.hpp"
#include "can/iface_mock.hpp"
#include <uavcan/internal/transport/transfer_sender.hpp>


static int sendOne(uavcan::TransferSender& sender, const std::string& data,
                   uint64_t monotonic_tx_deadline, uint64_t monotonic_blocking_deadline,
                   uavcan::TransferType transfer_type, uavcan::NodeID dst_node_id)
{
    return sender.send(reinterpret_cast<const uint8_t*>(data.c_str()), data.length(), monotonic_tx_deadline,
                       monotonic_blocking_deadline, transfer_type, dst_node_id);
}

static int sendOne(uavcan::TransferSender& sender, const std::string& data,
                   uint64_t monotonic_tx_deadline, uint64_t monotonic_blocking_deadline,
                   uavcan::TransferType transfer_type, uavcan::NodeID dst_node_id, uavcan::TransferID tid)
{
    return sender.send(reinterpret_cast<const uint8_t*>(data.c_str()), data.length(), monotonic_tx_deadline,
                       monotonic_blocking_deadline, transfer_type, dst_node_id, tid);
}


TEST(TransferSender, Basic)
{
    uavcan::PoolManager<1> poolmgr;

    SystemClockMock clockmock(100);
    CanDriverMock driver(2, clockmock);

    uavcan::OutgoingTransferRegistry<8> out_trans_reg(poolmgr);

    static const uavcan::NodeID TX_NODE_ID(64);
    static const uavcan::NodeID RX_NODE_ID(65);
    uavcan::Dispatcher dispatcher_tx(driver, poolmgr, clockmock, out_trans_reg, TX_NODE_ID);
    uavcan::Dispatcher dispatcher_rx(driver, poolmgr, clockmock, out_trans_reg, RX_NODE_ID);

    /*
     * Test environment
     */
    static const uavcan::DataTypeDescriptor TYPES[2] =
    {
        makeDataType(uavcan::DATA_TYPE_KIND_MESSAGE, 1),
        makeDataType(uavcan::DATA_TYPE_KIND_SERVICE, 1)
    };

    uavcan::TransferSender senders[2] =
    {
        uavcan::TransferSender(dispatcher_tx, TYPES[0], uavcan::CanTxQueue::VOLATILE),
        uavcan::TransferSender(dispatcher_tx, TYPES[1], uavcan::CanTxQueue::PERSISTENT)
    };

    static const std::string DATA[4] =
    {
        "Don't panic.",

        "The ships hung in the sky in much the same way that bricks don't.",

        "Would it save you a lot of time if I just gave up and went mad now?",

        "If there's anything more important than my ego around, I want it caught and shot now."
    };

    /*
     * Transmission
     */
    static const uint64_t TX_DEADLINE = 1000000;

    sendOne(senders[0], DATA[0], TX_DEADLINE, 0, uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 0);
    sendOne(senders[0], DATA[1], TX_DEADLINE, 0, uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   RX_NODE_ID);
    sendOne(senders[0], "123",   TX_DEADLINE, 0, uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 0);
    sendOne(senders[0], "456",   TX_DEADLINE, 0, uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   RX_NODE_ID);

    sendOne(senders[1], DATA[2], TX_DEADLINE, 0, uavcan::TRANSFER_TYPE_SERVICE_REQUEST,  RX_NODE_ID);
    sendOne(senders[1], DATA[3], TX_DEADLINE, 0, uavcan::TRANSFER_TYPE_SERVICE_RESPONSE, RX_NODE_ID, 1);
    sendOne(senders[1], "",      TX_DEADLINE, 0, uavcan::TRANSFER_TYPE_SERVICE_REQUEST,  RX_NODE_ID);
    sendOne(senders[1], "",      TX_DEADLINE, 0, uavcan::TRANSFER_TYPE_SERVICE_RESPONSE, RX_NODE_ID, 2);

    static const Transfer TRANSFERS[8] =
    {
        Transfer(TX_DEADLINE, 0, uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 0, TX_NODE_ID, 0,          DATA[0], TYPES[0]),
        Transfer(TX_DEADLINE, 0, uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   0, TX_NODE_ID, RX_NODE_ID, DATA[1], TYPES[0]),
        Transfer(TX_DEADLINE, 0, uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 1, TX_NODE_ID, 0,          "123",   TYPES[0]),
        Transfer(TX_DEADLINE, 0, uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   1, TX_NODE_ID, RX_NODE_ID, "456",   TYPES[0]),

        Transfer(TX_DEADLINE, 0, uavcan::TRANSFER_TYPE_SERVICE_REQUEST,   0, TX_NODE_ID, RX_NODE_ID, DATA[2], TYPES[1]),
        Transfer(TX_DEADLINE, 0, uavcan::TRANSFER_TYPE_SERVICE_RESPONSE,  1, TX_NODE_ID, RX_NODE_ID, DATA[3], TYPES[1]),
        Transfer(TX_DEADLINE, 0, uavcan::TRANSFER_TYPE_SERVICE_REQUEST,   1, TX_NODE_ID, RX_NODE_ID, "",      TYPES[1]),
        Transfer(TX_DEADLINE, 0, uavcan::TRANSFER_TYPE_SERVICE_RESPONSE,  2, TX_NODE_ID, RX_NODE_ID, "",      TYPES[1])
    };

    /*
     * Receiving on the other side.
     */
    for (int i = 0; i < driver.getNumIfaces(); i++)   // Moving the frames from TX to RX side
    {
        CanIfaceMock& iface = driver.ifaces.at(i);
        std::cout << "Num frames: " << iface.tx.size() << std::endl;
        while (!iface.tx.empty())
        {
            CanIfaceMock::FrameWithTime ft = iface.tx.front();
            iface.tx.pop();
            iface.rx.push(ft);
        }
    }

    TestSubscriber<512, 2, 2> sub_msg(TYPES[0], poolmgr);
    TestSubscriber<512, 2, 2> sub_srv_req(TYPES[1], poolmgr);
    TestSubscriber<512, 2, 2> sub_srv_resp(TYPES[1], poolmgr);

    dispatcher_rx.registerMessageListener(&sub_msg);
    dispatcher_rx.registerServiceRequestListener(&sub_srv_req);
    dispatcher_rx.registerServiceResponseListener(&sub_srv_resp);

    while (true)
    {
        const int res = dispatcher_rx.spin(0);
        ASSERT_LE(0, res);
        clockmock.advance(100);
        if (res == 0)
            break;
    }

    /*
     * Validation
     */
    ASSERT_TRUE(sub_msg.matchAndPop(TRANSFERS[0]));
    ASSERT_TRUE(sub_msg.matchAndPop(TRANSFERS[1]));
    ASSERT_TRUE(sub_msg.matchAndPop(TRANSFERS[2]));
    ASSERT_TRUE(sub_msg.matchAndPop(TRANSFERS[3]));

    ASSERT_TRUE(sub_srv_req.matchAndPop(TRANSFERS[4]));
    ASSERT_TRUE(sub_srv_req.matchAndPop(TRANSFERS[6]));

    ASSERT_TRUE(sub_srv_resp.matchAndPop(TRANSFERS[5]));
    ASSERT_TRUE(sub_srv_resp.matchAndPop(TRANSFERS[7]));
}
