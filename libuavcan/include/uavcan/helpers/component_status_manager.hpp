/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/impl_constants.hpp>
#include <uavcan/stdint.hpp>
#include <uavcan/protocol/NodeStatus.hpp>

namespace uavcan
{

template <unsigned NumComponents_>
class ComponentStatusManager
{
public:
    typedef typename StorageType<protocol::NodeStatus::FieldTypes::status_code>::Type StatusCode;
    static const unsigned NumComponents = NumComponents_;

private:
    StatusCode status_array[NumComponents];

public:
    ComponentStatusManager(StatusCode default_status = protocol::NodeStatus::STATUS_OK)
    {
        for (unsigned i = 0; i < NumComponents; i++)
        {
            status_array[i] = default_status;
        }
    }

    template <typename ComponentIndexType>
    void setComponentStatus(ComponentIndexType component_index, StatusCode status_code)
    {
        const unsigned compidx = static_cast<unsigned>(component_index);  // This cast allows to use typesafe enums
        if (compidx < NumComponents)
        {
            status_array[compidx] = status_code;
        }
    }

    StatusCode getWorstStatusCode() const
    {
        StatusCode result = 0;
        for (unsigned i = 0; i < NumComponents; i++)
        {
            result = max(result, status_array[i]);
        }
        return result;
    }
};

template <unsigned NumComponents_>
const unsigned ComponentStatusManager<NumComponents_>::NumComponents;

}
