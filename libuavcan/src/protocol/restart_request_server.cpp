/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/protocol/restart_request_server.hpp>
#include <uavcan/debug.hpp>

namespace uavcan
{

void RestartRequestServer::handleRestartNode(const ReceivedDataStructure<protocol::RestartNode::Request>& request,
                                             protocol::RestartNode::Response& response) const
{
    UAVCAN_TRACE("RestartRequestServer", "Request from snid=%i", int(request.getSrcNodeID().get()));
    response.ok = false;
    if (request.magic_number == protocol::RestartNode::Request::MAGIC_NUMBER)
    {
        if (handler_)
        {
            response.ok = handler_->handleRestartRequest(request.getSrcNodeID());
        }
        UAVCAN_TRACE("RestartRequestServer", "%s", (response.ok ? "Accepted" : "Rejected"));
    }
    else
    {
        UAVCAN_TRACE("RestartRequestServer", "Invalid magic number 0x%llx",
                     static_cast<unsigned long long>(request.magic_number));
    }
}

int RestartRequestServer::start()
{
    return srv_.start(RestartNodeCallback(this, &RestartRequestServer::handleRestartNode));
}

}
