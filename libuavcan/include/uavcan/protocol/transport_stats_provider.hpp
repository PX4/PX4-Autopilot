/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/node/service_server.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/protocol/GetTransportStats.hpp>

namespace uavcan
{
/**
 * This class provides statistics about the transport layer performance on the local node.
 * The user's application does not deal with this class directly because it's instantiated by the node class.
 */
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

    /**
     * Once started, this class requires no further attention.
     * Returns negative error code.
     */
    int start();
};

}
