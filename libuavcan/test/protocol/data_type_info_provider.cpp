/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/node/publisher.hpp>
#include <uavcan/protocol/data_type_info_provider.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include "helpers.hpp"

using uavcan::protocol::GetDataTypeInfo;
using uavcan::protocol::ComputeAggregateTypeSignature;
using uavcan::protocol::NodeStatus;
using uavcan::protocol::DataTypeKind;
using uavcan::ServiceCallResult;
using uavcan::DataTypeInfoProvider;
using uavcan::GlobalDataTypeRegistry;
using uavcan::DefaultDataTypeRegistrator;
using uavcan::MonotonicDuration;

template <typename DataType>
static bool validateDataTypeInfoResponse(const std::auto_ptr<ServiceCallResult<GetDataTypeInfo> >& resp, unsigned mask)
{
    if (!resp.get())
    {
        return false;
    }
    if (!resp->isSuccessful())
    {
        return false;
    }
    if (resp->response.name != DataType::getDataTypeFullName())
    {
        return false;
    }
    if (DataType::getDataTypeSignature().get() != resp->response.signature)
    {
        return false;
    }
    if (resp->response.mask != mask)
    {
        return false;
    }
    return true;
}


TEST(DataTypeInfoProvider, Basic)
{
    InterlinkedTestNodesWithSysClock nodes;

    DataTypeInfoProvider dtip(nodes.a);

    GlobalDataTypeRegistry::instance().reset();
    DefaultDataTypeRegistrator<GetDataTypeInfo> _reg1;
    DefaultDataTypeRegistrator<ComputeAggregateTypeSignature> _reg2;
    DefaultDataTypeRegistrator<NodeStatus> _reg3;

    ASSERT_LE(0, dtip.start());

    ServiceClientWithCollector<GetDataTypeInfo> gdti_cln(nodes.b);
    ServiceClientWithCollector<ComputeAggregateTypeSignature> cats_cln(nodes.b);

    /*
     * GetDataTypeInfo request for GetDataTypeInfo
     */
    GetDataTypeInfo::Request gdti_request;
    gdti_request.id = GetDataTypeInfo::DefaultDataTypeID;
    gdti_request.kind.value = DataTypeKind::SERVICE;
    ASSERT_LE(0, gdti_cln.call(1, gdti_request));
    nodes.spinBoth(MonotonicDuration::fromMSec(10));

    ASSERT_TRUE(validateDataTypeInfoResponse<GetDataTypeInfo>(gdti_cln.collector.result,
                                                              GetDataTypeInfo::Response::MASK_KNOWN |
                                                              GetDataTypeInfo::Response::MASK_SERVING));
    ASSERT_EQ(1, gdti_cln.collector.result->server_node_id.get());

    /*
     * GetDataTypeInfo request for NodeStatus - not used yet
     */
    gdti_request.id = NodeStatus::DefaultDataTypeID;
    gdti_request.kind.value = DataTypeKind::MESSAGE;
    ASSERT_LE(0, gdti_cln.call(1, gdti_request));
    nodes.spinBoth(MonotonicDuration::fromMSec(10));

    ASSERT_TRUE(validateDataTypeInfoResponse<NodeStatus>(gdti_cln.collector.result,
                                                         GetDataTypeInfo::Response::MASK_KNOWN));

    /*
     * Starting publisher and subscriber for NodeStatus, requesting info again
     */
    uavcan::Publisher<NodeStatus> ns_pub(nodes.a);
    SubscriberWithCollector<NodeStatus> ns_sub(nodes.a);

    ASSERT_LE(0, ns_pub.broadcast(NodeStatus()));
    ASSERT_LE(0, ns_sub.start());

    // Request again
    ASSERT_LE(0, gdti_cln.call(1, gdti_request));
    nodes.spinBoth(MonotonicDuration::fromMSec(10));

    ASSERT_TRUE(validateDataTypeInfoResponse<NodeStatus>(gdti_cln.collector.result,
                                                         GetDataTypeInfo::Response::MASK_KNOWN |
                                                         GetDataTypeInfo::Response::MASK_PUBLISHING |
                                                         GetDataTypeInfo::Response::MASK_SUBSCRIBED));

    /*
     * Requesting a non-existent type
     */
    gdti_request.id = ComputeAggregateTypeSignature::DefaultDataTypeID;
    gdti_request.kind.value = 0xFF;               // INVALID VALUE
    ASSERT_LE(0, gdti_cln.call(1, gdti_request));
    nodes.spinBoth(MonotonicDuration::fromMSec(10));

    ASSERT_TRUE(gdti_cln.collector.result.get());
    ASSERT_TRUE(gdti_cln.collector.result->isSuccessful());
    ASSERT_EQ(1, gdti_cln.collector.result->server_node_id.get());
    ASSERT_TRUE(gdti_cln.collector.result->response == GetDataTypeInfo::Response());  // Empty response

    /*
     * ComputeAggregateTypeSignature test
     */
    ComputeAggregateTypeSignature::Request cats_request;
    cats_request.kind.value = DataTypeKind::MESSAGE;
    cats_request.known_ids.set();                   // Assuming we have all 1023 types
    ASSERT_LE(0, cats_cln.call(1, cats_request));
    nodes.spinBoth(MonotonicDuration::fromMSec(10));

    ASSERT_TRUE(cats_cln.collector.result.get());
    ASSERT_TRUE(cats_cln.collector.result->isSuccessful());
    ASSERT_EQ(1, cats_cln.collector.result->server_node_id.get());
    ASSERT_EQ(NodeStatus::getDataTypeSignature().get(), cats_cln.collector.result->response.aggregate_signature);
    ASSERT_TRUE(cats_cln.collector.result->response.mutually_known_ids[NodeStatus::DefaultDataTypeID]);
    cats_cln.collector.result->response.mutually_known_ids[NodeStatus::DefaultDataTypeID] = false;
    ASSERT_FALSE(cats_cln.collector.result->response.mutually_known_ids.any());

    /*
     * ComputeAggregateTypeSignature test for a non-existent type
     */
    cats_request.kind.value = 0xFF;                 // INVALID
    cats_request.known_ids.set();                   // Assuming we have all 1023 types
    ASSERT_LE(0, cats_cln.call(1, cats_request));
    nodes.spinBoth(MonotonicDuration::fromMSec(10));

    ASSERT_TRUE(cats_cln.collector.result.get());
    ASSERT_TRUE(cats_cln.collector.result->isSuccessful());
    ASSERT_EQ(0, cats_cln.collector.result->response.aggregate_signature);
    ASSERT_FALSE(cats_cln.collector.result->response.mutually_known_ids.any());
}
