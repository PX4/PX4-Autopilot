/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/node/service_server.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/protocol/ComputeAggregateTypeSignature.hpp>
#include <uavcan/protocol/GetDataTypeInfo.hpp>

namespace uavcan
{
/**
 * This class implements the standard services for data type introspection.
 * Once started it does not require any attention from the application.
 * The user does not need to deal with it directly - it's started by the node class.
 */
class UAVCAN_EXPORT DataTypeInfoProvider : Noncopyable
{
    typedef MethodBinder<DataTypeInfoProvider*,
                         void (DataTypeInfoProvider::*)(const protocol::ComputeAggregateTypeSignature::Request&,
                                                        protocol::ComputeAggregateTypeSignature::Response&)>
        ComputeAggregateTypeSignatureCallback;

    typedef MethodBinder<DataTypeInfoProvider*,
                         void (DataTypeInfoProvider::*)(const protocol::GetDataTypeInfo::Request&,
                                                        protocol::GetDataTypeInfo::Response&)> GetDataTypeInfoCallback;

    ServiceServer<protocol::ComputeAggregateTypeSignature, ComputeAggregateTypeSignatureCallback> cats_srv_;
    ServiceServer<protocol::GetDataTypeInfo, GetDataTypeInfoCallback> gdti_srv_;

    INode& getNode() { return cats_srv_.getNode(); }

    static bool isValidDataTypeKind(DataTypeKind kind);

    void handleComputeAggregateTypeSignatureRequest(const protocol::ComputeAggregateTypeSignature::Request& request,
                                                    protocol::ComputeAggregateTypeSignature::Response& response);

    void handleGetDataTypeInfoRequest(const protocol::GetDataTypeInfo::Request& request,
                                      protocol::GetDataTypeInfo::Response& response);

public:
    explicit DataTypeInfoProvider(INode& node)
        : cats_srv_(node)
        , gdti_srv_(node)
    { }

    int start();
};

}
