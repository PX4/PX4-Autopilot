/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/build_config.hpp>
#if !UAVCAN_TINY

#include <uavcan/protocol/network_compat_checker.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/node/publisher.hpp>
#include <uavcan/protocol/GlobalDiscoveryRequest.hpp>
#include <cassert>

namespace uavcan
{

MonotonicDuration NetworkCompatibilityChecker::getNetworkDiscoveryDelay() const
{
    // Base duration is constant - NodeStatus max publication period
    MonotonicDuration dur = MonotonicDuration::fromMSec(protocol::NodeStatus::MAX_PUBLICATION_PERIOD_MS);
    // Additional duration depends on the node priority - gets larger with higher Node ID
    dur += MonotonicDuration::fromMSec(getNode().getNodeID().get() * 10);
    return dur;
}

NodeID NetworkCompatibilityChecker::findNextUncheckedNode()
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

int NetworkCompatibilityChecker::waitForCATSResponse()
{
    while (cats_cln_.isPending())
    {
        const int res = getNode().spin(MonotonicDuration::fromMSec(10));
        if (res < 0 || !result_.isOk())
        {
            return res;
        }
    }
    return 0;
}

void NetworkCompatibilityChecker::handleNodeStatus(const ReceivedDataStructure<protocol::NodeStatus>& msg)
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

void NetworkCompatibilityChecker::handleCATSResponse(ServiceCallResult<protocol::ComputeAggregateTypeSignature>& resp)
{
    last_cats_request_ok_ = resp.isSuccessful();
    if (last_cats_request_ok_)
    {
        const DataTypeSignature sign = GlobalDataTypeRegistry::instance().
            computeAggregateSignature(checking_dtkind_, resp.response.mutually_known_ids);

        UAVCAN_TRACE("NodeInitializer", "CATS response from nid=%i; local=%llu remote=%llu",
                     int(resp.server_node_id.get()), static_cast<unsigned long long>(sign.get()),
                     static_cast<unsigned long long>(resp.response.aggregate_signature));

        if (sign.get() != resp.response.aggregate_signature)
        {
            result_.conflicting_node = resp.server_node_id;
        }
    }
}

int NetworkCompatibilityChecker::checkOneNodeOneDataTypeKind(NodeID nid, DataTypeKind kind)
{
    StaticAssert<DataTypeKindMessage == int(protocol::DataTypeKind::MESSAGE)>::check();
    StaticAssert<DataTypeKindService == int(protocol::DataTypeKind::SERVICE)>::check();

    UAVCAN_ASSERT(nid.isUnicast());
    UAVCAN_ASSERT(!cats_cln_.isPending());

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

int NetworkCompatibilityChecker::checkOneNode(NodeID nid)
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

int NetworkCompatibilityChecker::checkNodes()
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

int NetworkCompatibilityChecker::execute()
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
    cats_cln_.cancel();
    return res;
}

int NetworkCompatibilityChecker::publishGlobalDiscoveryRequest(INode& node)
{
    Publisher<protocol::GlobalDiscoveryRequest> pub(node);
    return pub.broadcast(protocol::GlobalDiscoveryRequest());
}

}

#endif
