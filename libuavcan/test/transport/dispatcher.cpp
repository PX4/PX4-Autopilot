/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <memory>
#include <gtest/gtest.h>
#include "transfer_test_helpers.hpp"
#include "can/can.hpp"
#include <uavcan/transport/dispatcher.hpp>


class DispatcherTransferEmulator : public IncomingTransferEmulatorBase
{
    CanDriverMock& target_;

public:
    DispatcherTransferEmulator(CanDriverMock& target, uavcan::NodeID dst_node_id = 127)
    : IncomingTransferEmulatorBase(dst_node_id)
    , target_(target)
    { }

    void sendOneFrame(const uavcan::RxFrame& frame)
    {
        CanIfaceMock* const iface = static_cast<CanIfaceMock*>(target_.getIface(frame.getIfaceIndex()));
        EXPECT_TRUE(iface);
        if (iface)
            iface->pushRx(frame);
    }
};


static const uavcan::NodeID SELF_NODE_ID(64);


TEST(Dispatcher, Reception)
{
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * 8, uavcan::MemPoolBlockSize> pool;
    uavcan::PoolManager<1> poolmgr;
    poolmgr.addPool(&pool);

    SystemClockMock clockmock(100);
    CanDriverMock driver(2, clockmock);

    uavcan::OutgoingTransferRegistry<8> out_trans_reg(poolmgr);

    uavcan::Dispatcher dispatcher(driver, poolmgr, clockmock, out_trans_reg, SELF_NODE_ID);

    DispatcherTransferEmulator emulator(driver, SELF_NODE_ID);

    /*
     * Test environment
     */
    static const uavcan::DataTypeDescriptor TYPES[4] =
    {
        makeDataType(uavcan::DataTypeKindMessage, 1),
        makeDataType(uavcan::DataTypeKindMessage, 2),
        makeDataType(uavcan::DataTypeKindService, 1),
        makeDataType(uavcan::DataTypeKindService, 1)
    };

    typedef TestSubscriber<512, 2, 2> Subscriber;
    typedef std::auto_ptr<Subscriber> SubscriberPtr;
    static const int NUM_SUBSCRIBERS = 6;
    SubscriberPtr subscribers[NUM_SUBSCRIBERS] =
    {
        SubscriberPtr(new Subscriber(TYPES[0], poolmgr)),  // msg
        SubscriberPtr(new Subscriber(TYPES[0], poolmgr)),  // msg // Two similar, yes
        SubscriberPtr(new Subscriber(TYPES[1], poolmgr)),  // msg
        SubscriberPtr(new Subscriber(TYPES[2], poolmgr)),  // srv
        SubscriberPtr(new Subscriber(TYPES[3], poolmgr)),  // srv
        SubscriberPtr(new Subscriber(TYPES[3], poolmgr))   // srv // Repeat again
    };

    static const std::string DATA[6] =
    {
        "Yes, man is mortal, but that would be only half the trouble. "
        "The worst of it is that he's sometimes unexpectedly mortal - there's the trick!",

        "In fact, I'm beginning to fear that this confusion will go on for a long time. "
        "And all because he writes down what I said incorrectly.",

        "I had the pleasure of meeting that young man at the Patriarch's Ponds. "
        "He almost drove me mad myself, proving to me that I don't exist.",

        "He was a dreamer, a thinker, a speculative philosopher... or, as his wife would have it, an idiot.",

        "The only way to get ideas for stories is to drink way too much coffee and buy a desk that doesn't "
        "collapse when you beat your head against it",

        ""
    };

    const Transfer transfers[9] =
    {
        emulator.makeTransfer(uavcan::TransferTypeMessageBroadcast, 10, DATA[0], TYPES[0]),
        emulator.makeTransfer(uavcan::TransferTypeMessageUnicast,   11, DATA[1], TYPES[1]),
        emulator.makeTransfer(uavcan::TransferTypeServiceRequest,   12, DATA[2], TYPES[2]),
        emulator.makeTransfer(uavcan::TransferTypeServiceResponse,  13, DATA[3], TYPES[3]),
        emulator.makeTransfer(uavcan::TransferTypeMessageUnicast,   14, DATA[4], TYPES[0]),
        emulator.makeTransfer(uavcan::TransferTypeMessageBroadcast, 15, DATA[5], TYPES[1]),
        // Wrongly addressed:
        emulator.makeTransfer(uavcan::TransferTypeServiceResponse,  10, DATA[0], TYPES[3], 100),
        emulator.makeTransfer(uavcan::TransferTypeServiceRequest,   11, DATA[1], TYPES[2], 101),
        emulator.makeTransfer(uavcan::TransferTypeMessageUnicast,   12, DATA[2], TYPES[1], 102)
    };

    /*
     * Sending the transfers
     */
    ASSERT_TRUE(dispatcher.registerMessageListener(subscribers[0].get()));
    ASSERT_TRUE(dispatcher.registerMessageListener(subscribers[1].get()));
    ASSERT_TRUE(dispatcher.registerMessageListener(subscribers[2].get()));
    ASSERT_TRUE(dispatcher.registerServiceRequestListener(subscribers[3].get()));
    ASSERT_TRUE(dispatcher.registerServiceResponseListener(subscribers[4].get()));
    ASSERT_TRUE(dispatcher.registerServiceResponseListener(subscribers[5].get()));

    // Multiple service request listeners are not allowed
    ASSERT_FALSE(dispatcher.registerServiceRequestListener(subscribers[3].get()));

    // Item count validation
    ASSERT_EQ(3, dispatcher.getNumMessageListeners());
    ASSERT_EQ(1, dispatcher.getNumServiceRequestListeners());
    ASSERT_EQ(2, dispatcher.getNumServiceResponseListeners());

    for (int i = 0; i < NUM_SUBSCRIBERS; i++)
        ASSERT_TRUE(subscribers[i]->isEmpty());

    emulator.send(transfers);
    emulator.send(transfers);  // Just for fun, they will be ignored anyway.

    while (true)
    {
        const int res = dispatcher.spin(tsMono(0));
        ASSERT_LE(0, res);
        clockmock.advance(100);
        if (res == 0)
            break;
    }

    /*
     * Matching.
     * Expected reception order per subsciber:
     * 0: 0, 4
     * 1: 0, 4
     * 2: 5, 1
     * 3: 2
     * 4: 3
     * 5: 3
     */
    ASSERT_TRUE(subscribers[0]->matchAndPop(transfers[0]));
    ASSERT_TRUE(subscribers[0]->matchAndPop(transfers[4]));

    ASSERT_TRUE(subscribers[1]->matchAndPop(transfers[0]));
    ASSERT_TRUE(subscribers[1]->matchAndPop(transfers[4]));

    ASSERT_TRUE(subscribers[2]->matchAndPop(transfers[5]));
    ASSERT_TRUE(subscribers[2]->matchAndPop(transfers[1]));

    ASSERT_TRUE(subscribers[3]->matchAndPop(transfers[2]));

    ASSERT_TRUE(subscribers[4]->matchAndPop(transfers[3]));

    ASSERT_TRUE(subscribers[5]->matchAndPop(transfers[3]));

    for (int i = 0; i < NUM_SUBSCRIBERS; i++)
        ASSERT_TRUE(subscribers[i]->isEmpty());

    /*
     * Unregistering all transfers
     */
    dispatcher.unregisterMessageListener(subscribers[0].get());
    dispatcher.unregisterMessageListener(subscribers[1].get());
    dispatcher.unregisterMessageListener(subscribers[2].get());
    dispatcher.unregisterServiceRequestListener(subscribers[3].get());
    dispatcher.unregisterServiceResponseListener(subscribers[4].get());
    dispatcher.unregisterServiceResponseListener(subscribers[5].get());

    ASSERT_EQ(0, dispatcher.getNumMessageListeners());
    ASSERT_EQ(0, dispatcher.getNumServiceRequestListeners());
    ASSERT_EQ(0, dispatcher.getNumServiceResponseListeners());
}


TEST(Dispatcher, Transmission)
{
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * 8, uavcan::MemPoolBlockSize> pool;
    uavcan::PoolManager<1> poolmgr;
    poolmgr.addPool(&pool);

    SystemClockMock clockmock(100);
    CanDriverMock driver(2, clockmock);

    uavcan::OutgoingTransferRegistry<8> out_trans_reg(poolmgr);

    uavcan::Dispatcher dispatcher(driver, poolmgr, clockmock, out_trans_reg, SELF_NODE_ID);

    /*
     * Transmission
     */
    static const uavcan::MonotonicTime TX_DEADLINE = tsMono(123456);

    // uint_fast16_t data_type_id, TransferType transfer_type, NodeID src_node_id, NodeID dst_node_id,
    // uint_fast8_t frame_index, TransferID transfer_id, bool last_frame = false
    uavcan::Frame frame(123, uavcan::TransferTypeMessageUnicast, SELF_NODE_ID, 2, 0, 0, true);
    frame.setPayload(reinterpret_cast<const uint8_t*>("123"), 3);

    ASSERT_EQ(2, dispatcher.send(frame, TX_DEADLINE, tsMono(0), uavcan::CanTxQueue::Volatile));

    /*
     * Validation
     */
    uavcan::CanFrame expected_can_frame;
    ASSERT_TRUE(frame.compile(expected_can_frame));

    ASSERT_TRUE(driver.ifaces.at(0).matchAndPopTx(expected_can_frame, TX_DEADLINE));
    ASSERT_TRUE(driver.ifaces.at(1).matchAndPopTx(expected_can_frame, TX_DEADLINE));

    ASSERT_TRUE(driver.ifaces.at(0).tx.empty());
    ASSERT_TRUE(driver.ifaces.at(1).tx.empty());
}


TEST(Dispatcher, Spin)
{
    uavcan::PoolManager<1> poolmgr;

    SystemClockMock clockmock(100);
    CanDriverMock driver(2, clockmock);

    uavcan::OutgoingTransferRegistry<8> out_trans_reg(poolmgr);

    uavcan::Dispatcher dispatcher(driver, poolmgr, clockmock, out_trans_reg, SELF_NODE_ID);

    clockmock.monotonic_auto_advance = 100;

    ASSERT_EQ(100, clockmock.monotonic);
    ASSERT_EQ(0, dispatcher.spin(tsMono(1000)));
    ASSERT_LE(1000, clockmock.monotonic);
    ASSERT_EQ(0, dispatcher.spin(tsMono(0)));
    ASSERT_LE(1000, clockmock.monotonic);
    ASSERT_EQ(0, dispatcher.spin(tsMono(1100)));
    ASSERT_LE(1100, clockmock.monotonic);
}
