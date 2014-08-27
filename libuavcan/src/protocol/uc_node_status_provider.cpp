/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/protocol/node_status_provider.hpp>
#include <uavcan/debug.hpp>
#include <cassert>

namespace uavcan
{

bool NodeStatusProvider::isNodeInfoInitialized() const
{
    // Hardware/Software versions are not required
    return !node_info_.name.empty();
}

int NodeStatusProvider::publish()
{
    const MonotonicDuration uptime = getNode().getMonotonicTime() - creation_timestamp_;
    UAVCAN_ASSERT(uptime.isPositive());
    node_info_.status.uptime_sec = uint32_t(uptime.toMSec() / 1000);

    UAVCAN_ASSERT(node_info_.status.status_code <= protocol::NodeStatus::FieldTypes::status_code::max());

    return node_status_pub_.broadcast(node_info_.status);
}

void NodeStatusProvider::publishWithErrorHandling()
{
    if (getNode().isPassiveMode())
    {
        UAVCAN_TRACE("NodeStatusProvider", "NodeStatus pub skipped - passive mode");
    }
    else
    {
        const int res = publish();
        if (res < 0)
        {
            getNode().registerInternalFailure("NodeStatus pub failed");
        }
    }
}

void NodeStatusProvider::handleTimerEvent(const TimerEvent&)
{
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
    UAVCAN_ASSERT(isNodeInfoInitialized());
    rsp = node_info_;
}

int NodeStatusProvider::startAndPublish()
{
    if (!isNodeInfoInitialized())
    {
        UAVCAN_TRACE("NodeStatusProvider", "Node info was not initialized");
        return -ErrNotInited;
    }

    int res = -1;

    if (!getNode().isPassiveMode())
    {
        res = publish();
        if (res < 0)  // Initial broadcast
        {
            goto fail;
        }
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

    setStatusPublishingPeriod(MonotonicDuration::fromMSec(protocol::NodeStatus::MAX_PUBLICATION_PERIOD_MS));

    return res;

fail:
    UAVCAN_ASSERT(res < 0);
    gdr_sub_.stop();
    gni_srv_.stop();
    TimerBase::stop();
    return res;
}

void NodeStatusProvider::setStatusPublishingPeriod(uavcan::MonotonicDuration period)
{
    const MonotonicDuration maximum = MonotonicDuration::fromMSec(protocol::NodeStatus::MAX_PUBLICATION_PERIOD_MS);
    const MonotonicDuration minimum = MonotonicDuration::fromMSec(protocol::NodeStatus::MIN_PUBLICATION_PERIOD_MS);

    period = min(period, maximum);
    period = max(period, minimum);
    TimerBase::startPeriodic(period);

    const MonotonicDuration tx_timeout = period - MonotonicDuration::fromUSec(period.toUSec() / 20);
    node_status_pub_.setTxTimeout(tx_timeout);

    UAVCAN_TRACE("NodeStatusProvider", "Status pub period: %s, TX timeout: %s",
                 period.toString().c_str(), node_status_pub_.getTxTimeout().toString().c_str());
}

uavcan::MonotonicDuration NodeStatusProvider::getStatusPublishingPeriod() const
{
    return TimerBase::getPeriod();
}

void NodeStatusProvider::setStatusCode(uint8_t code)
{
    node_info_.status.status_code = code;
}

void NodeStatusProvider::setName(const char* name)
{
    if ((name != NULL) && (*name != '\0') && (node_info_.name.empty()))
    {
        node_info_.name = name;  // The string contents will be copied, not just pointer.
    }
}

void NodeStatusProvider::setSoftwareVersion(const protocol::SoftwareVersion& version)
{
    if (node_info_.software_version == protocol::SoftwareVersion())
    {
        node_info_.software_version = version;
    }
}

void NodeStatusProvider::setHardwareVersion(const protocol::HardwareVersion& version)
{
    if (node_info_.hardware_version == protocol::HardwareVersion())
    {
        node_info_.hardware_version = version;
    }
}

}
