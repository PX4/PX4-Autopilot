/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/build_config.hpp>
#include <uavcan/stdint.hpp>
#include <uavcan/protocol/NodeStatus.hpp>

namespace uavcan
{
/**
 * This helper class stores status codes of multiple components, and provides a method to quickly retrieve the
 * worst status code.
 * It allows to easily assign the node status code with the worst component status code.
 * Refer to the standard message type uavcan.protocol.NodeStatus for available status codes.
 */
template <unsigned NumComponents_>
class UAVCAN_EXPORT ComponentStatusManager
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

    /**
     * Assign the component status by index. Normally, an index would be defined by some enum constant.
     * @param component_index   Normally an enum constant
     * @param status_code       Status code from uavcan.protocol.NodeStatus
     */
    template <typename ComponentIndexType>
    void setComponentStatus(ComponentIndexType component_index, StatusCode status_code)
    {
        const unsigned compidx = static_cast<unsigned>(component_index);  // This cast allows to use typesafe enums
        if (compidx < NumComponents)
        {
            status_array[compidx] = status_code;
        }
    }

    /**
     * Returns worst status code, i.e. highest value.
     */
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
