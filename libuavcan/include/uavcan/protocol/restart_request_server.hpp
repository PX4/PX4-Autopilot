/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/node/service_server.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/protocol/RestartNode.hpp>

namespace uavcan
{

class UAVCAN_EXPORT IRestartRequestHandler
{
public:
    virtual ~IRestartRequestHandler() { }
    virtual bool handleRestartRequest(NodeID request_source) = 0;
};


class UAVCAN_EXPORT RestartRequestServer : Noncopyable
{
    typedef MethodBinder<const RestartRequestServer*,
                         void (RestartRequestServer::*)(const ReceivedDataStructure<protocol::RestartNode::Request>&,
                                                        protocol::RestartNode::Response&) const> RestartNodeCallback;

    ServiceServer<protocol::RestartNode, RestartNodeCallback> srv_;
    IRestartRequestHandler* handler_;

    void handleRestartNode(const ReceivedDataStructure<protocol::RestartNode::Request>& request,
                           protocol::RestartNode::Response& response) const;

public:
    explicit RestartRequestServer(INode& node)
        : srv_(node)
        , handler_(NULL)
    { }

    IRestartRequestHandler* getHandler() const { return handler_; }
    void setHandler(IRestartRequestHandler* handler) { handler_ = handler; }

    int start();
};

}
