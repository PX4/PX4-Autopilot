/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/node/publisher.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/service_server.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/impl_constants.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>
#include <uavcan/protocol/GlobalDiscoveryRequest.hpp>

namespace uavcan
{

class NodeStatusProvider : private Timer
{
    typedef MethodBinder<NodeStatusProvider*, void(NodeStatusProvider::*)(const protocol::GlobalDiscoveryRequest&)>
        GlobalDiscoveryRequestCallback;
    typedef MethodBinder<NodeStatusProvider*,
        void(NodeStatusProvider::*)(const protocol::GetNodeInfo::Request&,
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

    void handleTimerEvent(const TimerEvent&);
    void handleGlobalDiscoveryRequest(const protocol::GlobalDiscoveryRequest&);
    void handleGetNodeInfoRequest(const protocol::GetNodeInfo::Request&, protocol::GetNodeInfo::Response& rsp);

public:
    NodeStatusProvider(INode& node)
    : Timer(node)
    , creation_timestamp_(node.getMonotonicTime())
    , node_status_pub_(node)
    , gdr_sub_(node)
    , gni_srv_(node)
    {
        assert(!creation_timestamp_.isZero());

        node_info_.uavcan_version.major = UAVCAN_VERSION_MAJOR;
        node_info_.uavcan_version.minor = UAVCAN_VERSION_MINOR;
        node_info_.uavcan_version.build = UAVCAN_VERSION_BUILD;

        node_info_.status.status_code = protocol::NodeStatus::STATUS_INITIALIZING;
    }

    int startAndPublish();

    int forcePublish() { return publish(); }

    uint8_t getStatusCode() const { return node_info_.status.status_code; }
    void setStatusCode(uint8_t code);
    void setStatusOk()           { setStatusCode(protocol::NodeStatus::STATUS_OK); }
    void setStatusInitializing() { setStatusCode(protocol::NodeStatus::STATUS_INITIALIZING); }
    void setStatusWarning()      { setStatusCode(protocol::NodeStatus::STATUS_WARNING); }
    void setStatusCritical()     { setStatusCode(protocol::NodeStatus::STATUS_CRITICAL); }
    void setStatusOffline()      { setStatusCode(protocol::NodeStatus::STATUS_OFFLINE); }

    const typename protocol::GetNodeInfo::Response::FieldTypes::name& getName() const { return node_info_.name; }
    void setName(const char* name);

    const protocol::SoftwareVersion& getSoftwareVersion() const { return node_info_.software_version; }
    const protocol::HardwareVersion& getHardwareVersion() const { return node_info_.hardware_version; }
    void setSoftwareVersion(const protocol::SoftwareVersion& version);
    void setHardwareVersion(const protocol::HardwareVersion& version);
};

}
