/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/protocol/data_type_info_provider.hpp>
#include <uavcan/debug.hpp>

namespace uavcan
{

bool DataTypeInfoProvider::isValidDataTypeKind(DataTypeKind kind)
{
    return (kind == DataTypeKindMessage) || (kind == DataTypeKindService);
}

void DataTypeInfoProvider::handleComputeAggregateTypeSignatureRequest(
    const protocol::ComputeAggregateTypeSignature::Request& request,
    protocol::ComputeAggregateTypeSignature::Response& response)
{
    const DataTypeKind kind = DataTypeKind(request.kind.value);
    if (!isValidDataTypeKind(kind))
    {
        UAVCAN_TRACE("DataTypeInfoProvider",
                     "ComputeAggregateTypeSignature request with invalid DataTypeKind %i", kind);
        return;
    }

    UAVCAN_TRACE("DataTypeInfoProvider", "ComputeAggregateTypeSignature request for dtk=%i", int(request.kind.value));

    response.mutually_known_ids = request.known_ids;
    response.aggregate_signature =
        GlobalDataTypeRegistry::instance().computeAggregateSignature(kind, response.mutually_known_ids).get();
}

void DataTypeInfoProvider::handleGetDataTypeInfoRequest(const protocol::GetDataTypeInfo::Request& request,
                                                        protocol::GetDataTypeInfo::Response& response)
{
    const DataTypeKind kind = DataTypeKind(request.kind.value);
    if (!isValidDataTypeKind(kind))
    {
        UAVCAN_TRACE("DataTypeInfoProvider", "GetDataTypeInfo request with invalid DataTypeKind %i", kind);
        return;
    }

    const DataTypeDescriptor* const desc = GlobalDataTypeRegistry::instance().find(kind, request.id);
    if (!desc)
    {
        UAVCAN_TRACE("DataTypeInfoProvider", "Cannot process GetDataTypeInfo for nonexistent type dtid=%i dtk=%i",
                     int(request.id), int(request.kind.value));
        return;
    }

    UAVCAN_TRACE("DataTypeInfoProvider", "GetDataTypeInfo request for %s", desc->toString().c_str());

    response.signature = desc->getSignature().get();
    response.name = desc->getFullName();
    response.mask = protocol::GetDataTypeInfo::Response::MASK_KNOWN;

    const Dispatcher& dispatcher = getNode().getDispatcher();

    if (request.kind.value == protocol::DataTypeKind::SERVICE)
    {
        if (dispatcher.hasServer(request.id))
        {
            response.mask |= protocol::GetDataTypeInfo::Response::MASK_SERVING;
        }
    }
    else if (request.kind.value == protocol::DataTypeKind::MESSAGE)
    {
        if (dispatcher.hasSubscriber(request.id))
        {
            response.mask |= protocol::GetDataTypeInfo::Response::MASK_SUBSCRIBED;
        }
        if (dispatcher.hasPublisher(request.id))
        {
            response.mask |= protocol::GetDataTypeInfo::Response::MASK_PUBLISHING;
        }
    }
    else
    {
        UAVCAN_ASSERT(0); // That means that GDTR somehow found a type of an unknown kind. The horror.
    }
}

int DataTypeInfoProvider::start()
{
    int res = 0;

    res = cats_srv_.start(
        ComputeAggregateTypeSignatureCallback(this, &DataTypeInfoProvider::handleComputeAggregateTypeSignatureRequest));
    if (res < 0)
    {
        goto fail;
    }

    res = gdti_srv_.start(GetDataTypeInfoCallback(this, &DataTypeInfoProvider::handleGetDataTypeInfoRequest));
    if (res < 0)
    {
        goto fail;
    }

    UAVCAN_ASSERT(res >= 0);
    return res;

fail:
    UAVCAN_ASSERT(res < 0);
    cats_srv_.stop();
    gdti_srv_.stop();
    return res;
}

}
