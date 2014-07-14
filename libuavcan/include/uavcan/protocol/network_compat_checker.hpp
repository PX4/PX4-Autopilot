/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/util/bitset.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/service_client.hpp>
#include <uavcan/protocol/ComputeAggregateTypeSignature.hpp>
#include <uavcan/protocol/NodeStatus.hpp>

namespace uavcan
{

struct UAVCAN_EXPORT NetworkCompatibilityCheckResult
{
    NodeID conflicting_node;   ///< First detected conflicting node
    uint8_t num_failed_nodes;  ///< Number of nodes that did not respond to service requests (probably not supporting)

    NetworkCompatibilityCheckResult()
        : num_failed_nodes(0)
    { }

    /**
     * Quick check if the network compatibility check did not discover any problems.
     */
    bool isOk() const { return !conflicting_node.isValid(); }
};

/**
 * Performs Network Compatibility Check. Please read the specs.
 *
 * This class does not issue GlobalDiscoveryRequest, assuming that it was done already by the caller.
 * Instantiated object can @ref execute() only once.
 * Objects of this class are intended for stack allocation.
 */
class UAVCAN_EXPORT NetworkCompatibilityChecker : Noncopyable
{
    typedef BitSet<NodeID::Max + 1> NodeIDMask;
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

    /**
     * Run the check.
     * Beware: this method may block for a several seconds!
     *
     * If this method has been executed successfully, use @ref getResult() to get the actual check result.
     *
     * This method can be executed only once on a given instance of this class (shall be
     * destroyed and reconstructed from scratch).
     *
     * Returns negative error code.
     */
    int execute();

    /**
     * This method can return a meaningful result only if @ref execute() has been executed successfully once.
     */
    const NetworkCompatibilityCheckResult& getResult() const { return result_; }

    /**
     * Convenience method. Should be called immediately before @ref execute().
     * Returns negative error code.
     */
    static int publishGlobalDiscoveryRequest(INode& node);
};

}
