/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/node/service_client.hpp>
#include <uavcan/node/service_server.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/protocol/ComputeAggregateTypeSignature.hpp>
#include <uavcan/protocol/RestartNode.hpp>
#include <uavcan/protocol/GetDataTypeInfo.hpp>
#include <root_ns_a/StringService.hpp>
#include <queue>
#include "test_node.hpp"


template <typename DataType>
struct ServiceCallResultHandler
{
    typedef typename uavcan::ServiceCallResult<DataType>::Status StatusType;
    StatusType last_status;
    uavcan::NodeID last_server_node_id;
    typename DataType::Response last_response;

    void handleResponse(const uavcan::ServiceCallResult<DataType>& result)
    {
        std::cout << result << std::endl;
        last_status = result.status;
        last_response = result.response;
        last_server_node_id = result.server_node_id;
    }

    bool match(StatusType status, uavcan::NodeID server_node_id, const typename DataType::Response& response) const
    {
        return
            status == last_status &&
            server_node_id == last_server_node_id &&
            response == last_response;
    }

    typedef uavcan::MethodBinder<ServiceCallResultHandler*,
                                 void (ServiceCallResultHandler::*)(const uavcan::ServiceCallResult<DataType>&)> Binder;

    Binder bind() { return Binder(this, &ServiceCallResultHandler::handleResponse); }
};


static void stringServiceServerCallback(const uavcan::ReceivedDataStructure<root_ns_a::StringService::Request>& req,
                                        root_ns_a::StringService::Response& rsp)
{
    rsp.string_response = "Request string: ";
    rsp.string_response += req.string_request;
}


TEST(ServiceClient, Basic)
{
    InterlinkedTestNodesWithSysClock nodes;

    // Type registration
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<root_ns_a::StringService> _registrator;

    // Server
    uavcan::ServiceServer<root_ns_a::StringService> server(nodes.a);
    ASSERT_EQ(0, server.start(stringServiceServerCallback));

    {
        // Caller
        typedef uavcan::ServiceCallResult<root_ns_a::StringService> ResultType;
        typedef uavcan::ServiceClient<root_ns_a::StringService,
                                      typename ServiceCallResultHandler<root_ns_a::StringService>::Binder > ClientType;
        ServiceCallResultHandler<root_ns_a::StringService> handler;

        ClientType client1(nodes.b);
        ClientType client2(nodes.b);
        ClientType client3(nodes.b);

        client1.setCallback(handler.bind());
        client2.setCallback(client1.getCallback());
        client3.setCallback(client1.getCallback());
        client3.setRequestTimeout(uavcan::MonotonicDuration::fromMSec(100));

        ASSERT_EQ(1, nodes.a.getDispatcher().getNumServiceRequestListeners());
        ASSERT_EQ(0, nodes.b.getDispatcher().getNumServiceResponseListeners()); // NOT listening!

        root_ns_a::StringService::Request request;
        request.string_request = "Hello world";

        ASSERT_LT(0, client1.call(1, request)); // OK
        ASSERT_LT(0, client2.call(1, request)); // OK
        ASSERT_LT(0, client3.call(99, request)); // Will timeout!

        ASSERT_EQ(3, nodes.b.getDispatcher().getNumServiceResponseListeners()); // Listening now!

        ASSERT_TRUE(client1.isPending());
        ASSERT_TRUE(client2.isPending());
        ASSERT_TRUE(client3.isPending());

        nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(20));

        ASSERT_EQ(1, nodes.b.getDispatcher().getNumServiceResponseListeners()); // Third is still listening!

        ASSERT_FALSE(client1.isPending());
        ASSERT_FALSE(client2.isPending());
        ASSERT_TRUE(client3.isPending());

        // Validating
        root_ns_a::StringService::Response expected_response;
        expected_response.string_response = "Request string: Hello world";
        ASSERT_TRUE(handler.match(ResultType::Success, 1, expected_response));

        nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(200));

        ASSERT_FALSE(client1.isPending());
        ASSERT_FALSE(client2.isPending());
        ASSERT_FALSE(client3.isPending());

        ASSERT_EQ(0, nodes.b.getDispatcher().getNumServiceResponseListeners()); // Third has timed out :(

        // Validating
        ASSERT_TRUE(handler.match(ResultType::ErrorTimeout, 99, root_ns_a::StringService::Response()));

        // Stray request
        ASSERT_LT(0, client3.call(99, request)); // Will timeout!
        ASSERT_TRUE(client3.isPending());
        ASSERT_EQ(1, nodes.b.getDispatcher().getNumServiceResponseListeners());
    }

    // All destroyed - nobody listening
    ASSERT_EQ(0, nodes.b.getDispatcher().getNumServiceResponseListeners());
}


TEST(ServiceClient, Sizes)
{
    using namespace uavcan;

    std::cout << "ComputeAggregateTypeSignature server: " <<
        sizeof(ServiceServer<protocol::ComputeAggregateTypeSignature>) << std::endl;

    std::cout << "ComputeAggregateTypeSignature client: " <<
        sizeof(ServiceClient<protocol::ComputeAggregateTypeSignature>) << std::endl;

    std::cout << "ComputeAggregateTypeSignature request data struct: " <<
        sizeof(protocol::ComputeAggregateTypeSignature::Request) << std::endl;

    std::cout << "GetDataTypeInfo server: " <<
        sizeof(ServiceServer<protocol::GetDataTypeInfo>) << std::endl;

    std::cout << "RestartNode server: " <<
        sizeof(ServiceServer<protocol::RestartNode>) << std::endl;

    std::cout << "GetDataTypeInfo client: " <<
        sizeof(ServiceClient<protocol::GetDataTypeInfo>) << std::endl;
}
