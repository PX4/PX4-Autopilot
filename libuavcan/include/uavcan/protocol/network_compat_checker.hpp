/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_NETWORK_COMPAT_CHECKER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_NETWORK_COMPAT_CHECKER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/util/bitset.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/publisher.hpp>
#include <uavcan/node/service_client.hpp>
#include <uavcan/protocol/ComputeAggregateTypeSignature.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/GlobalDiscoveryRequest.hpp>
#include <cassert>

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

    MonotonicDuration getNetworkDiscoveryDelay() const
    {
        // Base duration is constant - NodeStatus max publication period
        MonotonicDuration dur = MonotonicDuration::fromMSec(protocol::NodeStatus::MAX_PUBLICATION_PERIOD_MS);
        // Additional duration depends on the node priority - gets larger with higher Node ID
        dur += MonotonicDuration::fromMSec(getNode().getNodeID().get() * 10);
        return dur;
    }

    NodeID findNextUncheckedNode()
    {
        for (uint8_t i = 1; i <= NodeID::Max; i++)
        {
            if (nid_mask_present_.test(i) && !nid_mask_checked_.test(i))
            {
                nid_mask_checked_[i] = true;
                return NodeID(i);
            }
        }
        return NodeID();
    }

    int waitForCATSResponse()
    {
        while (cats_cln_.hasPendingCalls())
        {
            const int res = getNode().spin(MonotonicDuration::fromMSec(10));
            if (res < 0 || !result_.isOk())
            {
                return res;
            }
        }
        return 0;
    }

    void handleNodeStatus(const ReceivedDataStructure<protocol::NodeStatus>& msg)
    {
        if (!nid_mask_present_.test(msg.getSrcNodeID().get()))
        {
            UAVCAN_TRACE("NodeInitializer", "New node nid=%i", int(msg.getSrcNodeID().get()));
            nid_mask_present_[msg.getSrcNodeID().get()] = true;
        }

        if (msg.getSrcNodeID() == getNode().getNodeID())
        {
            UAVCAN_TRACE("NodeInitializer", "Node ID collision; nid=%i", int(msg.getSrcNodeID().get()));
            result_.conflicting_node = msg.getSrcNodeID();
        }
    }

    void handleCATSResponse(ServiceCallResult<protocol::ComputeAggregateTypeSignature>& resp)
    {
        last_cats_request_ok_ = resp.isSuccessful();
        if (last_cats_request_ok_)
        {
            const DataTypeSignature sign = GlobalDataTypeRegistry::instance().
                computeAggregateSignature(checking_dtkind_, resp.getResponse().mutually_known_ids);

            UAVCAN_TRACE("NodeInitializer", "CATS response from nid=%i; local=%llu remote=%llu",
                         int(resp.getCallID().server_node_id.get()), static_cast<unsigned long long>(sign.get()),
                         static_cast<unsigned long long>(resp.getResponse().aggregate_signature));

            if (sign.get() != resp.getResponse().aggregate_signature)
            {
                result_.conflicting_node = resp.getCallID().server_node_id;
            }
        }
    }

    int checkOneNodeOneDataTypeKind(NodeID nid, DataTypeKind kind)
    {
        StaticAssert<DataTypeKindMessage == int(protocol::DataTypeKind::MESSAGE)>::check();
        StaticAssert<DataTypeKindService == int(protocol::DataTypeKind::SERVICE)>::check();

        UAVCAN_ASSERT(nid.isUnicast());
        UAVCAN_ASSERT(!cats_cln_.hasPendingCalls());

        checking_dtkind_ = kind;
        protocol::ComputeAggregateTypeSignature::Request request;
        request.kind.value = kind;
        GlobalDataTypeRegistry::instance().getDataTypeIDMask(kind, request.known_ids);

        int res = cats_cln_.call(nid, request);
        if (res < 0)
        {
            return res;
        }
        res = waitForCATSResponse();
        if (res < 0)
        {
            return res;
        }
        if (!last_cats_request_ok_)
        {
            return -ErrFailure;
        }
        return 0;
    }

    int checkOneNode(NodeID nid)
    {
        if (nid == getNode().getNodeID())
        {
            result_.conflicting_node = nid;  // NodeID collision
            return 0;
        }

        const int res = checkOneNodeOneDataTypeKind(nid, DataTypeKindMessage);
        if (res < 0 || !result_.isOk())
        {
            return res;
        }
        return checkOneNodeOneDataTypeKind(nid, DataTypeKindService);
    }

    int checkNodes()
    {
        (void)nid_mask_checked_.reset();
        result_ = NetworkCompatibilityCheckResult();

        while (result_.isOk())
        {
            const NodeID nid = findNextUncheckedNode();
            if (nid.isValid())
            {
                UAVCAN_TRACE("NodeInitializer", "Checking nid=%i", int(nid.get()));
                const int res = checkOneNode(nid);
                if (res < 0)
                {
                    result_.num_failed_nodes++;
                }
                UAVCAN_TRACE("NodeInitializer", "Checked nid=%i result=%i", int(nid.get()), res);
            }
            else { break; }
        }
        return 0;
    }

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
    int execute()
    {
        int res = 0;

        if (!getNode().getNodeID().isUnicast())
        {
            result_.conflicting_node = getNode().getNodeID();
            goto exit;
        }

        res = ns_sub_.start(NodeStatusCallback(this, &NetworkCompatibilityChecker::handleNodeStatus));
        if (res < 0)
        {
            goto exit;
        }

        cats_cln_.setCallback(CATSResponseCallback(this, &NetworkCompatibilityChecker::handleCATSResponse));
        res = cats_cln_.init();
        if (res < 0)
        {
            goto exit;
        }

        res = getNode().spin(getNetworkDiscoveryDelay());
        if (res < 0)
        {
            goto exit;
        }

        res = checkNodes();

    exit:
        ns_sub_.stop();
        cats_cln_.cancelAllCalls();
        return res;
    }

    /**
     * This method can return a meaningful result only if @ref execute() has been executed successfully once.
     */
    const NetworkCompatibilityCheckResult& getResult() const { return result_; }

    /**
     * Convenience method. Should be called immediately before @ref execute().
     * Returns negative error code.
     */
    static int publishGlobalDiscoveryRequest(INode& node)
    {
        Publisher<protocol::GlobalDiscoveryRequest> pub(node);
        return pub.broadcast(protocol::GlobalDiscoveryRequest());
    }
};

}

#endif // UAVCAN_PROTOCOL_NETWORK_COMPAT_CHECKER_HPP_INCLUDED
