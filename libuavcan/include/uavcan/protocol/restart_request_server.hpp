/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/node/service_server.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/protocol/RestartNode.hpp>

namespace uavcan
{
/**
 * Implement this interface in the application to support the standard node restart service.
 */
class UAVCAN_EXPORT IRestartRequestHandler
{
public:
    virtual ~IRestartRequestHandler() { }

    /**
     * This method shall do either:
     *  - restart the local node immediately;
     *  - initiate the restart procedure to complete it asynchronously;
     *  - reject the restart request and return false.
     *
     * If the restart requets was accepted, this method shall either return true or don't return at all.
     */
    virtual bool handleRestartRequest(NodeID request_source) = 0;
};

/**
 * Convenience class for supporting the standard node restart service.
 * Highly recommended to use.
 */
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

    /**
     * Restart request handler configuration.
     * All restart requests will be explicitly rejected if there's no handler installed.
     */
    IRestartRequestHandler* getHandler() const { return handler_; }
    void setHandler(IRestartRequestHandler* handler) { handler_ = handler; }

    /**
     * Starts the server.
     * Returns negative error code.
     */
    int start();
};

}
