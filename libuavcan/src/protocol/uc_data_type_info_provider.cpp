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
    /*
     * Asking the Global Data Type Registry for the matching type descriptor, either by name or by ID
     */
    const DataTypeDescriptor* desc = NULL;

    if (request.name.empty())
    {
        response.id   = request.id;   // Pre-setting the fields so they have meaningful values even in
        response.kind = request.kind; // ...case of failure.

        if (!isValidDataTypeKind(DataTypeKind(request.kind.value)))
        {
            UAVCAN_TRACE("DataTypeInfoProvider", "GetDataTypeInfo request with invalid DataTypeKind %i",
                         static_cast<int>(request.kind.value));
            return;
        }

        desc = GlobalDataTypeRegistry::instance().find(DataTypeKind(request.kind.value), request.id);
    }
    else
    {
        response.name = request.name;

        desc = GlobalDataTypeRegistry::instance().find(request.name.c_str());
    }

    if (desc == NULL)
    {
        UAVCAN_TRACE("DataTypeInfoProvider",
                     "Cannot process GetDataTypeInfo for nonexistent type: dtid=%i dtk=%i name='%s'",
                     static_cast<int>(request.id), static_cast<int>(request.kind.value), request.name.c_str());
        return;
    }

    UAVCAN_TRACE("DataTypeInfoProvider", "GetDataTypeInfo request for %s", desc->toString().c_str());

    /*
     * Filling the response struct
     */
    response.signature  = desc->getSignature().get();
    response.id         = desc->getID().get();
    response.kind.value = desc->getKind();
    response.mask       = protocol::GetDataTypeInfo::Response::MASK_KNOWN;
    response.name       = desc->getFullName();

    const Dispatcher& dispatcher = getNode().getDispatcher();

    if (desc->getKind() == DataTypeKindService)
    {
        if (dispatcher.hasServer(desc->getID().get()))
        {
            response.mask |= protocol::GetDataTypeInfo::Response::MASK_SERVING;
        }
    }
    else if (desc->getKind() == DataTypeKindMessage)
    {
        if (dispatcher.hasSubscriber(desc->getID().get()))
        {
            response.mask |= protocol::GetDataTypeInfo::Response::MASK_SUBSCRIBED;
        }
        if (dispatcher.hasPublisher(desc->getID().get()))
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
