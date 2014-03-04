/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <root_ns_a/EmptyService.hpp>
#include <root_ns_a/EmptyMessage.hpp>
#include <root_ns_a/NestedMessage.hpp>
#include <root_ns_a/ReportBackSoldier.hpp>
#include <root_ns_b/ServiceWithEmptyRequest.hpp>
#include <root_ns_b/ServiceWithEmptyResponse.hpp>


TEST(Dsdl, EmptyServices)
{
    uavcan::StaticTransferBuffer<100> buf;
    uavcan::BitStream bs_wr(buf);
    uavcan::ScalarCodec sc_wr(bs_wr);

    root_ns_b::ServiceWithEmptyRequest::Request req;
    ASSERT_EQ(1, root_ns_b::ServiceWithEmptyRequest::Request::encode(req, sc_wr));
    ASSERT_EQ("", bs_wr.toString());

    root_ns_b::ServiceWithEmptyRequest::Response resp;
    ASSERT_EQ(1, root_ns_b::ServiceWithEmptyRequest::Response::encode(resp, sc_wr));
    ASSERT_EQ("", bs_wr.toString());

    resp.covariance.push_back(-2);
    resp.covariance.push_back(65504);
    root_ns_b::ServiceWithEmptyRequest::Response::encode(resp, sc_wr);
    ASSERT_EQ("00000000 11000000 11111111 01111011", bs_wr.toString());

    resp.covariance.push_back(42);
    resp.covariance[0] = 999;

    uavcan::BitStream bs_rd(buf);
    uavcan::ScalarCodec sc_rd(bs_rd);
    ASSERT_EQ(1, root_ns_b::ServiceWithEmptyRequest::Response::decode(resp, sc_rd));

    ASSERT_EQ(2, resp.covariance.size());
    ASSERT_EQ(-2, resp.covariance[0]);
    ASSERT_EQ(65504, resp.covariance[1]);
}

TEST(Dsdl, Signature)
{
    ASSERT_EQ(0xe74617107a34aa9c, root_ns_a::EmptyService::getDataTypeSignature().get());
    ASSERT_EQ("root_ns_a.EmptyService", root_ns_a::EmptyService::getDataTypeFullName());
    ASSERT_EQ(uavcan::DataTypeKindService, root_ns_a::EmptyService::DataTypeKind);

    ASSERT_EQ(0x41a2582ee72be419, root_ns_a::NestedMessage::getDataTypeSignature().get());  // Computed manually
    ASSERT_EQ("root_ns_a.NestedMessage", root_ns_a::NestedMessage::getDataTypeFullName());
    ASSERT_EQ(uavcan::DataTypeKindMessage, root_ns_a::NestedMessage::DataTypeKind);
}
