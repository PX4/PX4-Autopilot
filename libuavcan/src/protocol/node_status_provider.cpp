/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <uavcan/debug.hpp>
#include <uavcan/protocol/node_status_provider.hpp>

namespace uavcan
{

bool NodeStatusProvider::isNodeInfoInitialized() const
{
    // Hardware version is not required
    return (node_info_.software_version != protocol::SoftwareVersion()) &&
           (node_info_.uavcan_version != protocol::SoftwareVersion()) &&
           (!node_info_.name.empty());
}

int NodeStatusProvider::publish()
{
    const MonotonicDuration uptime = getNode().getMonotonicTime() - creation_timestamp_;
    assert(uptime.isPositive());
    node_info_.status.uptime_sec = uptime.toMSec() / 1000;

    assert(node_info_.status.status_code <= protocol::NodeStatus::FieldTypes::status_code::max());

    return node_status_pub_.broadcast(node_info_.status);
}

void NodeStatusProvider::publishWithErrorHandling()
{
    const int res = publish();
    if (res < 0)
    {
        getNode().registerInternalFailure("NodeStatus publication failed");
    }
}

void NodeStatusProvider::handleTimerEvent(const TimerEvent&)
{
    UAVCAN_TRACE("NodeStatusProvider", "Publishing node status by timer");
    publishWithErrorHandling();
}

void NodeStatusProvider::handleGlobalDiscoveryRequest(const protocol::GlobalDiscoveryRequest&)
{
    UAVCAN_TRACE("NodeStatusProvider", "Got GlobalDiscoveryRequest");
    publishWithErrorHandling();
}

void NodeStatusProvider::handleGetNodeInfoRequest(const protocol::GetNodeInfo::Request&,
                                                  protocol::GetNodeInfo::Response& rsp)
{
    UAVCAN_TRACE("NodeStatusProvider", "Got GetNodeInfo request");
    assert(isNodeInfoInitialized());
    rsp = node_info_;
}

int NodeStatusProvider::startAndPublish()
{
    int res = -1;

    if (!isNodeInfoInitialized())
    {
        UAVCAN_TRACE("NodeStatusProvider", "Node info was not initialized");
        return -1;
    }

    res = publish(); // Initial broadcast
    if (res < 0)
    {
        goto fail;
    }

    res = gdr_sub_.start(GlobalDiscoveryRequestCallback(this, &NodeStatusProvider::handleGlobalDiscoveryRequest));
    if (res < 0)
    {
        goto fail;
    }

    res = gni_srv_.start(GetNodeInfoCallback(this, &NodeStatusProvider::handleGetNodeInfoRequest));
    if (res < 0)
    {
        goto fail;
    }

    Timer::startPeriodic(MonotonicDuration::fromMSec(protocol::NodeStatus::PUBLICATION_PERIOD_MS));

    return res;

fail:
    assert(res < 0);
    gdr_sub_.stop();
    gni_srv_.stop();
    Timer::stop();
    return res;
}

void NodeStatusProvider::setStatusCode(uint8_t code)
{
    node_info_.status.status_code = code;
}

void NodeStatusProvider::setName(const char* name)
{
    node_info_.name = name;
}

void NodeStatusProvider::setSoftwareVersion(const protocol::SoftwareVersion& version)
{
    node_info_.software_version = version;
}

void NodeStatusProvider::setHardwareVersion(const protocol::HardwareVersion& version)
{
    node_info_.hardware_version = version;
}

}
