/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <bitset>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/service_client.hpp>
#include <uavcan/protocol/ComputeAggregateTypeSignature.hpp>
#include <uavcan/protocol/NodeStatus.hpp>

namespace uavcan
{

struct UAVCAN_EXPORT NetworkCompatibilityCheckResult
{
    NodeID conflicting_node;
    bool isOk() const { return !conflicting_node.isValid(); }
};

/**
 * This class does not issue GlobalDiscoveryRequest, assuming that it was done already by the caller.
 * Instantiated object can execute() only once. Objects of this class are intended for stack allocation.
 */
class UAVCAN_EXPORT NetworkCompatibilityChecker : Noncopyable
{
    typedef std::bitset<NodeID::Max + 1> NodeIDMask;
    typedef MethodBinder<NetworkCompatibilityChecker*,
                         void (NetworkCompatibilityChecker::*)(const ReceivedDataStructure<protocol::NodeStatus>&)>
            NodeStatusCallback;
    typedef MethodBinder<NetworkCompatibilityChecker*,
                         void (NetworkCompatibilityChecker::*)(ServiceCallResult<protocol::ComputeAggregateTypeSignature>&)>
            CATSResponseCallback;

    Subscriber<protocol::NodeStatus, NodeStatusCallback> ns_sub_;
    ServiceClient<protocol::ComputeAggregateTypeSignature, CATSResponseCallback> cats_cln_;
    NodeIDMask nid_mask_present_;
    NodeIDMask nid_mask_checked_;
    NetworkCompatibilityCheckResult result_;
    DataTypeKind checking_dtkind_;
    bool last_cats_request_ok_;

    INode& getNode() { return ns_sub_.getNode(); }
    const INode& getNode() const { return ns_sub_.getNode(); }

    MonotonicDuration getNetworkDiscoveryDelay() const;

    NodeID findNextUncheckedNode();

    int waitForCATSResponse();

    void handleNodeStatus(const ReceivedDataStructure<protocol::NodeStatus>& msg);
    void handleCATSResponse(ServiceCallResult<protocol::ComputeAggregateTypeSignature>& resp);

    int checkOneNodeOneDataTypeKind(NodeID nid, DataTypeKind kind);
    int checkOneNode(NodeID nid);
    int checkNodes();

public:
    explicit NetworkCompatibilityChecker(INode& node)
        : ns_sub_(node)
        , cats_cln_(node)
        , checking_dtkind_(DataTypeKindService)
        , last_cats_request_ok_(false)
    { }

    int execute();

    const NetworkCompatibilityCheckResult& getResult() const { return result_; }

    static int publishGlobalDiscoveryRequest(INode& node);
};

}
