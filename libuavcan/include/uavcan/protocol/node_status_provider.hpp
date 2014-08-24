/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/node/publisher.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/service_server.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>
#include <uavcan/protocol/GlobalDiscoveryRequest.hpp>

namespace uavcan
{
/**
 * Provides the status and basic information about this node to other network participants.
 * Usually the application does not need to deal with this class directly - it's instantiated by the node class.
 */
class UAVCAN_EXPORT NodeStatusProvider : private TimerBase
{
    typedef MethodBinder<NodeStatusProvider*, void (NodeStatusProvider::*)(const protocol::GlobalDiscoveryRequest&)>
        GlobalDiscoveryRequestCallback;
    typedef MethodBinder<NodeStatusProvider*,
                         void (NodeStatusProvider::*)(const protocol::GetNodeInfo::Request&,
                                                      protocol::GetNodeInfo::Response&)> GetNodeInfoCallback;

    const MonotonicTime creation_timestamp_;

    Publisher<protocol::NodeStatus> node_status_pub_;
    Subscriber<protocol::GlobalDiscoveryRequest, GlobalDiscoveryRequestCallback> gdr_sub_;
    ServiceServer<protocol::GetNodeInfo, GetNodeInfoCallback> gni_srv_;

    protocol::GetNodeInfo::Response node_info_;

    INode& getNode() { return node_status_pub_.getNode(); }

    bool isNodeInfoInitialized() const;

    int publish();
    void publishWithErrorHandling();

    virtual void handleTimerEvent(const TimerEvent&);
    void handleGlobalDiscoveryRequest(const protocol::GlobalDiscoveryRequest&);
    void handleGetNodeInfoRequest(const protocol::GetNodeInfo::Request&, protocol::GetNodeInfo::Response& rsp);

public:
    explicit NodeStatusProvider(INode& node)
        : TimerBase(node)
        , creation_timestamp_(node.getMonotonicTime())
        , node_status_pub_(node)
        , gdr_sub_(node)
        , gni_srv_(node)
    {
        UAVCAN_ASSERT(!creation_timestamp_.isZero());

        node_info_.status.status_code = protocol::NodeStatus::STATUS_INITIALIZING;
    }

    /**
     * Starts the provider and immediately broadcasts uavcan.protocol.NodeStatus.
     * Returns negative error code.
     */
    int startAndPublish();

    /**
     * Publish the message uavcan.protocol.NodeStatus right now, out of schedule.
     * Returns negative error code.
     */
    int forcePublish() { return publish(); }

    /**
     * Allows to override default publishing rate for uavcan.protocol.NodeStatus.
     * Refer to the DSDL definition of uavcan.protocol.NodeStatus to see what is the default rate.
     * Doesn't fail; if the value is outside of acceptable range, a closest valid value will be used instead.
     */
    void setStatusPublishingPeriod(uavcan::MonotonicDuration period);
    uavcan::MonotonicDuration getStatusPublishingPeriod() const;

    /**
     * Local node status code control.
     */
    uint8_t getStatusCode() const { return node_info_.status.status_code; }
    void setStatusCode(uint8_t code);
    void setStatusOk()           { setStatusCode(protocol::NodeStatus::STATUS_OK); }
    void setStatusInitializing() { setStatusCode(protocol::NodeStatus::STATUS_INITIALIZING); }
    void setStatusWarning()      { setStatusCode(protocol::NodeStatus::STATUS_WARNING); }
    void setStatusCritical()     { setStatusCode(protocol::NodeStatus::STATUS_CRITICAL); }
    void setStatusOffline()      { setStatusCode(protocol::NodeStatus::STATUS_OFFLINE); }

    /**
     * Local node name control.
     * Can be set only once before the provider is started.
     * The provider will refuse to start if the node name is not set.
     */
    const typename protocol::GetNodeInfo::Response::FieldTypes::name& getName() const { return node_info_.name; }
    void setName(const char* name);

    /**
     * Node version information.
     * Can be set only once before the provider is started.
     */
    const protocol::SoftwareVersion& getSoftwareVersion() const { return node_info_.software_version; }
    const protocol::HardwareVersion& getHardwareVersion() const { return node_info_.hardware_version; }
    void setSoftwareVersion(const protocol::SoftwareVersion& version);
    void setHardwareVersion(const protocol::HardwareVersion& version);
};

}
