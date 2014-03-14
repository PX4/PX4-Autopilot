/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/service_client.hpp>
#include <uavcan/service_server.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/protocol/ComputeAggregateTypeSignature.hpp>
#include <uavcan/protocol/GetDataTypeInfo.hpp>
#include <root_ns_a/StringService.hpp>
#include <queue>
#include "clock.hpp"
#include "internal/transport/can/can.hpp"


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


struct MakeshiftNode : uavcan::Noncopyable
{
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * 8, uavcan::MemPoolBlockSize> pool;
    uavcan::PoolManager<1> poolmgr;
    SystemClockDriver clock_driver;
    uavcan::MarshalBufferProvider<> buffer_provider;
    uavcan::OutgoingTransferRegistry<8> otr;
    uavcan::Scheduler scheduler;

    MakeshiftNode(uavcan::ICanDriver& can_driver, uavcan::NodeID self_node_id)
    : otr(poolmgr)
    , scheduler(can_driver, poolmgr, clock_driver, otr, self_node_id)
    {
        poolmgr.addPool(&pool);
    }

    void spin(uavcan::MonotonicDuration duration)
    {
        scheduler.spin(clock_driver.getMonotonic() + duration);
    }
};


struct PairableCanDriver : public uavcan::ICanDriver, public uavcan::ICanIface
{
    uavcan::ISystemClock& clock;
    PairableCanDriver* other;
    std::queue<uavcan::CanFrame> read_queue;

    PairableCanDriver(uavcan::ISystemClock& clock)
    : clock(clock)
    , other(NULL)
    { }

    void linkTogether(PairableCanDriver* with)
    {
        this->other = with;
        with->other = this;
    }

    uavcan::ICanIface* getIface(int iface_index)
    {
        if (iface_index == 0)
            return this;
        return NULL;
    }

    int getNumIfaces() const { return 1; }

    int select(int& inout_write_iface_mask, int& inout_read_iface_mask, uavcan::MonotonicTime blocking_deadline)
    {
        assert(other);
        if (inout_read_iface_mask == 1)
            inout_read_iface_mask = read_queue.size() ? 1 : 0;

        if (inout_read_iface_mask || inout_write_iface_mask)
            return 1;

        while (clock.getMonotonic() < blocking_deadline)
            usleep(1000);

        return 0;
    }

    int send(const uavcan::CanFrame& frame, uavcan::MonotonicTime)
    {
        assert(other);
        other->read_queue.push(frame);
        return 1;
    }

    int receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic, uavcan::UtcTime& out_ts_utc)
    {
        assert(other);
        assert(read_queue.size());
        out_frame = read_queue.front();
        read_queue.pop();
        out_ts_monotonic = clock.getMonotonic();
        out_ts_utc = clock.getUtc();
        return 1;
    }

    int configureFilters(const uavcan::CanFilterConfig*, int) { return -1; }
    int getNumFilters() const { return 0; }
    uint64_t getNumErrors() const { return 0; }
};


TEST(ServiceClient, Basic)
{
    SystemClockDriver clock;

    PairableCanDriver can_a(clock), can_b(clock);
    can_a.linkTogether(&can_b);

    MakeshiftNode node_a(can_a, 1), node_b(can_b, 2);

    // Type registration
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<root_ns_a::StringService> _registrator;

    // Server
    uavcan::ServiceServer<root_ns_a::StringService> server(node_a.scheduler, node_a.poolmgr, node_a.buffer_provider);
    ASSERT_EQ(1, server.start(stringServiceServerCallback));

    {
        // Caller
        typedef uavcan::ServiceCallResult<root_ns_a::StringService> ResultType;
        typedef uavcan::ServiceClient<root_ns_a::StringService,
            typename ServiceCallResultHandler<root_ns_a::StringService>::Binder > ClientType;
        ServiceCallResultHandler<root_ns_a::StringService> handler;

        ClientType client1(node_b.scheduler, node_b.poolmgr, node_b.buffer_provider);
        ClientType client2(node_b.scheduler, node_b.poolmgr, node_b.buffer_provider);
        ClientType client3(node_b.scheduler, node_b.poolmgr, node_b.buffer_provider);

        client1.setCallback(handler.bind());
        client2.setCallback(client1.getCallback());
        client3.setCallback(client1.getCallback());
        client3.setRequestTimeout(uavcan::MonotonicDuration::fromMSec(100));

        ASSERT_EQ(1, node_a.scheduler.getDispatcher().getNumServiceRequestListeners());
        ASSERT_EQ(0, node_b.scheduler.getDispatcher().getNumServiceResponseListeners()); // NOT listening!

        root_ns_a::StringService::Request request;
        request.string_request = "Hello world";

        ASSERT_LT(0, client1.call(1, request)); // OK
        ASSERT_LT(0, client2.call(1, request)); // OK
        ASSERT_LT(0, client3.call(99, request)); // Will timeout!

        ASSERT_EQ(3, node_b.scheduler.getDispatcher().getNumServiceResponseListeners()); // Listening now!

        ASSERT_TRUE(client1.isPending());
        ASSERT_TRUE(client2.isPending());
        ASSERT_TRUE(client3.isPending());

        node_a.spin(uavcan::MonotonicDuration::fromMSec(10));
        node_b.spin(uavcan::MonotonicDuration::fromMSec(10));

        ASSERT_EQ(1, node_b.scheduler.getDispatcher().getNumServiceResponseListeners()); // Third is still listening!

        ASSERT_FALSE(client1.isPending());
        ASSERT_FALSE(client2.isPending());
        ASSERT_TRUE(client3.isPending());

        // Validating
        root_ns_a::StringService::Response expected_response;
        expected_response.string_response = "Request string: Hello world";
        ASSERT_TRUE(handler.match(ResultType::Success, 1, expected_response));

        node_a.spin(uavcan::MonotonicDuration::fromMSec(100));
        node_b.spin(uavcan::MonotonicDuration::fromMSec(100));

        ASSERT_FALSE(client1.isPending());
        ASSERT_FALSE(client2.isPending());
        ASSERT_FALSE(client3.isPending());

        ASSERT_EQ(0, node_b.scheduler.getDispatcher().getNumServiceResponseListeners()); // Third has timed out :(

        // Validating
        ASSERT_TRUE(handler.match(ResultType::ErrorTimeout, 99, root_ns_a::StringService::Response()));

        // Stray request
        ASSERT_LT(0, client3.call(99, request)); // Will timeout!
        ASSERT_TRUE(client3.isPending());
        ASSERT_EQ(1, node_b.scheduler.getDispatcher().getNumServiceResponseListeners());
    }

    // All destroyed - nobody listening
    ASSERT_EQ(0, node_b.scheduler.getDispatcher().getNumServiceResponseListeners());
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

    std::cout << "GetDataTypeInfo client: " <<
        sizeof(ServiceClient<protocol::GetDataTypeInfo>) << std::endl;
}
