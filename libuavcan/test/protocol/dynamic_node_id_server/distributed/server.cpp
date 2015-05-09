/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#if __GNUC__
// We need auto_ptr for compatibility reasons
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#include <gtest/gtest.h>
#include <memory>
#include <uavcan/protocol/dynamic_node_id_server/distributed/server.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include "event_tracer.hpp"
#include "../../helpers.hpp"
#include "../memory_storage_backend.hpp"

using uavcan::dynamic_node_id_server::UniqueID;


class CommitHandler : public uavcan::dynamic_node_id_server::distributed::ILeaderLogCommitHandler
{
    const std::string id_;

    virtual void onEntryCommitted(const uavcan::protocol::dynamic_node_id::server::Entry& entry)
    {
        std::cout << "ENTRY COMMITTED [" << id_ << "]\n" << entry << std::endl;
    }

    virtual void onLeaderChange(bool local_node_is_leader)
    {
        std::cout << "I AM LEADER: " << (local_node_is_leader ? "YES" : "NOT ANYMORE") << std::endl;
    }

public:
    CommitHandler(const std::string& id) : id_(id) { }
};


TEST(DynamicNodeIDServer, RaftCoreBasic)
{
    using namespace uavcan::dynamic_node_id_server::distributed;
    using namespace uavcan::protocol::dynamic_node_id::server;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<Discovery> _reg1;
    uavcan::DefaultDataTypeRegistrator<AppendEntries> _reg2;
    uavcan::DefaultDataTypeRegistrator<RequestVote> _reg3;

    EventTracer tracer_a("a");
    EventTracer tracer_b("b");
    MemoryStorageBackend storage_a;
    MemoryStorageBackend storage_b;
    CommitHandler commit_handler_a("a");
    CommitHandler commit_handler_b("b");

    InterlinkedTestNodesWithSysClock nodes;

    std::auto_ptr<RaftCore> raft_a(new RaftCore(nodes.a, storage_a, tracer_a, commit_handler_a));
    std::auto_ptr<RaftCore> raft_b(new RaftCore(nodes.b, storage_b, tracer_b, commit_handler_b));

    /*
     * Initialization
     */
    ASSERT_LE(0, raft_a->init(2));
    ASSERT_LE(0, raft_b->init(2));

    /*
     * Running and trying not to fall
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(5000));

    // The one with lower node ID must become a leader
    ASSERT_TRUE(raft_a->isLeader());
    ASSERT_FALSE(raft_b->isLeader());

    ASSERT_EQ(0, raft_a->getCommitIndex());
    ASSERT_EQ(0, raft_b->getCommitIndex());

    /*
     * Adding some stuff
     */
    Entry::FieldTypes::unique_id unique_id;
    uavcan::fill_n(unique_id.begin(), 16, uint8_t(0xAA));

    ASSERT_LE(0, raft_a->appendLog(unique_id, uavcan::NodeID(1)));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(2000));

    ASSERT_EQ(1, raft_a->getCommitIndex());
    ASSERT_EQ(1, raft_b->getCommitIndex());

    /*
     * Terminating the leader - the Follower will continue to sleep
     */
    raft_a.reset();

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(2000));

    /*
     * Reinitializing the leader - current Follower will become the new Leader
     */
    storage_a.reset();

    raft_a.reset(new RaftCore(nodes.a, storage_a, tracer_a, commit_handler_a));
    ASSERT_LE(0, raft_a->init(2));
    ASSERT_EQ(0, raft_a->getCommitIndex());

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(5000));

    ASSERT_FALSE(raft_a->isLeader());
    ASSERT_TRUE(raft_b->isLeader());

    ASSERT_EQ(1, raft_a->getCommitIndex());
    ASSERT_EQ(1, raft_b->getCommitIndex());
}


TEST(DynamicNodeIDServer, Main)
{
    using namespace uavcan::dynamic_node_id_server;
    using namespace uavcan::protocol::dynamic_node_id;
    using namespace uavcan::protocol::dynamic_node_id::server;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<Discovery> _reg1;
    uavcan::DefaultDataTypeRegistrator<AppendEntries> _reg2;
    uavcan::DefaultDataTypeRegistrator<RequestVote> _reg3;
    uavcan::DefaultDataTypeRegistrator<Allocation> _reg4;

    EventTracer tracer;
    MemoryStorageBackend storage;

    // Node A is Allocator, Node B is Allocatee
    InterlinkedTestNodesWithSysClock nodes(uavcan::NodeID(10), uavcan::NodeID::Broadcast);

    /*
     * Server
     */
    distributed::Server server(nodes.a, storage, tracer);

    ASSERT_LE(0, server.init(1));

    /*
     * Client
     */
    uavcan::DynamicNodeIDClient client(nodes.b);
    uavcan::protocol::HardwareVersion hwver;
    for (uavcan::uint8_t i = 0; i < hwver.unique_id.size(); i++)
    {
        hwver.unique_id[i] = i;
    }
    const uavcan::NodeID PreferredNodeID = 42;
    ASSERT_LE(0, client.start(hwver, PreferredNodeID));

    /*
     * Fire
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(4000));

    ASSERT_TRUE(client.isAllocationComplete());
    ASSERT_EQ(PreferredNodeID, client.getAllocatedNodeID());
}


TEST(DynamicNodeIDServer, ObjectSizes)
{
    using namespace uavcan::dynamic_node_id_server;

    std::cout << "distributed::Log:             " << sizeof(distributed::Log) << std::endl;
    std::cout << "distributed::PersistentState: " << sizeof(distributed::PersistentState) << std::endl;
    std::cout << "distributed::ClusterManager:  " << sizeof(distributed::ClusterManager) << std::endl;
    std::cout << "distributed::RaftCore:        " << sizeof(distributed::RaftCore) << std::endl;
    std::cout << "distributed::Server:          " << sizeof(distributed::Server) << std::endl;
    std::cout << "AllocationRequestManager:     " << sizeof(AllocationRequestManager) << std::endl;
}
