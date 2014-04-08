/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/node/service_server.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/protocol/GetTransportStats.hpp>

namespace uavcan
{

class UAVCAN_EXPORT TransportStatsProvider : Noncopyable
{
    typedef MethodBinder<const TransportStatsProvider*,
                         void (TransportStatsProvider::*)(const protocol::GetTransportStats::Request&,
                                                          protocol::GetTransportStats::Response&) const>
            GetTransportStatsCallback;

    ServiceServer<protocol::GetTransportStats, GetTransportStatsCallback> srv_;

    void handleGetTransportStats(const protocol::GetTransportStats::Request&,
                                 protocol::GetTransportStats::Response& resp) const;

public:
    explicit TransportStatsProvider(INode& node)
        : srv_(node)
    { }

    int start();
};

}
