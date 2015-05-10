/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <iostream>
#include <string>
#include <uavcan/protocol/dynamic_node_id_server/event.hpp>


class EventTracer : public uavcan::dynamic_node_id_server::IEventTracer
{
    const std::string id_;

    virtual void onEvent(uavcan::dynamic_node_id_server::TraceCode code, uavcan::int64_t argument)
    {
        std::cout << "EVENT [" << id_ << "]\t" << code << "\t" << getEventName(code) << "\t" << argument << std::endl;
    }

public:
    EventTracer() { }

    EventTracer(const std::string& id) : id_(id) { }
};
