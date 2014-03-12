/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/server.hpp>
#include <uavcan/util/method_binder.hpp>
#include <root_ns_a/StringService.hpp>
#include "clock.hpp"
#include "internal/transport/can/can.hpp"


struct ServerImpl
{
    const char* string_response;

    ServerImpl(const char* string_response) : string_response(string_response) { }

    void handleRequest(const uavcan::ReceivedDataStructure<typename root_ns_a::StringService::Request>& request,
                       typename root_ns_a::StringService::Response& response)
    {
        std::cout << request << std::endl;
        response.string_response = request.string_request;
        response.string_response += " --> ";
        response.string_response += string_response;
        std::cout << response << std::endl;
    }

    typedef uavcan::MethodBinder<ServerImpl*,
        void (ServerImpl::*)(const uavcan::ReceivedDataStructure<typename root_ns_a::StringService::Request>&,
                             typename root_ns_a::StringService::Response&)> Binder;

    Binder bind() { return Binder(this, &ServerImpl::handleRequest); }
};


TEST(Server, Basic)
{
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * 8, uavcan::MemPoolBlockSize> pool;
    uavcan::PoolManager<1> poolmgr;
    poolmgr.addPool(&pool);

    // Manual type registration - we can't rely on the GDTR state
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<root_ns_a::StringService> _registrator;

    SystemClockDriver clock_driver;
    CanDriverMock can_driver(1, clock_driver);

    uavcan::MarshalBufferProvider<> buffer_provider;
    uavcan::OutgoingTransferRegistry<8> out_trans_reg(poolmgr);

    uavcan::Scheduler sch(can_driver, poolmgr, clock_driver, out_trans_reg, uavcan::NodeID(1));

    ServerImpl impl("456");

    {
        uavcan::Server<root_ns_a::StringService, ServerImpl::Binder> server(sch, poolmgr, buffer_provider);

        ASSERT_EQ(0, sch.getDispatcher().getNumServiceRequestListeners());
        server.start(impl.bind());
        ASSERT_EQ(1, sch.getDispatcher().getNumServiceRequestListeners());

        /*
         * Request frames
         */
        for (int i = 0; i < 2; i++)
        {
            // uint_fast16_t data_type_id, TransferType transfer_type, NodeID src_node_id, NodeID dst_node_id,
            // uint_fast8_t frame_index, TransferID transfer_id, bool last_frame
            uavcan::Frame frame(root_ns_a::StringService::DefaultDataTypeID, uavcan::TransferTypeServiceRequest,
                                uavcan::NodeID(i + 0x10), 1, 0, i, true);

            const uint8_t req[] = {'r', 'e', 'q', uint8_t(i + '0')};
            frame.setPayload(req, sizeof(req));

            uavcan::RxFrame rx_frame(frame, clock_driver.getMonotonic(), clock_driver.getUtc(), 0);
            can_driver.ifaces[0].pushRx(rx_frame);
        }

        sch.spin(clock_driver.getMonotonic() + uavcan::MonotonicDuration::fromUSec(10000));

        /*
         * Responses (MFT)
         */
        ASSERT_EQ(4, can_driver.ifaces[0].tx.size());
        for (int i = 0; i < 2; i++)
        {
            char payloads[2][8];
            std::snprintf(payloads[0], 8, "req%i ", i);
            std::snprintf(payloads[1], 8, "--> 456");

            // First frame
            uavcan::Frame fr;
            ASSERT_TRUE(fr.parse(can_driver.ifaces[0].popTxFrame()));
            std::cout << fr.toString() << std::endl;
            ASSERT_STREQ(payloads[0], reinterpret_cast<const char*>(fr.getPayloadPtr() + 2)); // Skipping CRC

            ASSERT_EQ(i, fr.getTransferID().get());
            ASSERT_EQ(uavcan::TransferTypeServiceResponse, fr.getTransferType());
            ASSERT_EQ(i + 0x10, fr.getDstNodeID().get());

            // Second frame
            ASSERT_TRUE(fr.parse(can_driver.ifaces[0].popTxFrame()));
            std::cout << fr.toString() << std::endl;
            ASSERT_STREQ(payloads[1], reinterpret_cast<const char*>(fr.getPayloadPtr()));

            ASSERT_EQ(i, fr.getTransferID().get());
            ASSERT_EQ(uavcan::TransferTypeServiceResponse, fr.getTransferType());
            ASSERT_EQ(i + 0x10, fr.getDstNodeID().get());
        }

        ASSERT_EQ(0, server.getRequestFailureCount());
        ASSERT_EQ(0, server.getResponseFailureCount());

        ASSERT_EQ(1, sch.getDispatcher().getNumServiceRequestListeners());
    }
    ASSERT_EQ(0, sch.getDispatcher().getNumServiceRequestListeners());
}
