/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/dynamic_node_id_server/centralized.hpp>
#include <uavcan/protocol/dynamic_node_id_server/centralized/storage.hpp>
#include <uavcan/protocol/dynamic_node_id_server/storage_marshaller.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include "../../helpers.hpp"
#include "../event_tracer.hpp"
#include "../../helpers.hpp"
#include "../memory_storage_backend.hpp"

using uavcan::dynamic_node_id_server::UniqueID;


TEST(dynamic_node_id_server_centralized_Server, Basic)
{
    using namespace uavcan::dynamic_node_id_server;
    using namespace uavcan::protocol::dynamic_node_id;
    using namespace uavcan::protocol::dynamic_node_id::server;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<Allocation> _reg1;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GetNodeInfo> _reg2;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::NodeStatus> _reg3;

    EventTracer tracer;
    MemoryStorageBackend storage;

    // Node A is Allocator, Node B is Allocatee
    InterlinkedTestNodesWithSysClock nodes(uavcan::NodeID(10), uavcan::NodeID::Broadcast);

    UniqueID own_unique_id;
    own_unique_id[0] = 0xAA;
    own_unique_id[3] = 0xCC;
    own_unique_id[7] = 0xEE;
    own_unique_id[9] = 0xBB;

    /*
     * Server
     */
    uavcan::dynamic_node_id_server::CentralizedServer server(nodes.a, storage, tracer);

    ASSERT_LE(0, server.init(own_unique_id));

    ASSERT_EQ(1, server.getNumAllocations());   // Server's own node ID

    /*
     * Client
     */
    uavcan::DynamicNodeIDClient client(nodes.b);
    uavcan::protocol::HardwareVersion::FieldTypes::unique_id unique_id;
    for (uavcan::uint8_t i = 0; i < unique_id.size(); i++)
    {
        unique_id[i] = i;
    }
    const uavcan::NodeID PreferredNodeID = 42;
    ASSERT_LE(0, client.start(unique_id, PreferredNodeID));

    /*
     * Fire
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(15000));

    ASSERT_TRUE(client.isAllocationComplete());
    ASSERT_EQ(PreferredNodeID, client.getAllocatedNodeID());

    ASSERT_EQ(2, server.getNumAllocations());   // Server's own node ID + client
}


TEST(dynamic_node_id_server_centralized_Server, ReuseOldestWhenExhausted)
{
    using namespace uavcan::dynamic_node_id_server;
    using namespace uavcan::protocol::dynamic_node_id;
    using namespace uavcan::protocol::dynamic_node_id::server;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<Allocation> _reg1;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GetNodeInfo> _reg2;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::NodeStatus> _reg3;

    EventTracer tracer;
    MemoryStorageBackend storage;

    // Pre-fill storage with allocations for every unicast node ID so the allocator is exhausted.
    {
        uavcan::dynamic_node_id_server::centralized::Storage stor(storage);
        ASSERT_LE(0, stor.init());

        UniqueID own_unique_id;
        own_unique_id[0] = 0xAA;
        own_unique_id[3] = 0xCC;
        own_unique_id[7] = 0xEE;
        own_unique_id[9] = 0xBB;

        for (uint8_t nid = 1; nid <= uavcan::NodeID::Max; nid++)
        {
            UniqueID uid;
            uid[0] = nid;
            if (nid == 10)
            {
                uid = own_unique_id;
            }
            ASSERT_LE(0, stor.add(uavcan::NodeID(nid), uid));
        }

        ASSERT_EQ(uavcan::NodeID::Max, stor.getSize());
    }

    // Node A is Allocator, Node B is Allocatee
    InterlinkedTestNodesWithSysClock nodes(uavcan::NodeID(10), uavcan::NodeID::Broadcast);

    UniqueID own_unique_id;
    own_unique_id[0] = 0xAA;
    own_unique_id[3] = 0xCC;
    own_unique_id[7] = 0xEE;
    own_unique_id[9] = 0xBB;

    uavcan::dynamic_node_id_server::CentralizedServer server(nodes.a, storage, tracer);
    ASSERT_LE(0, server.init(own_unique_id));

    // Client unique ID
    uavcan::DynamicNodeIDClient client(nodes.b);
    uavcan::protocol::HardwareVersion::FieldTypes::unique_id client_unique_id;
    for (uavcan::uint8_t i = 0; i < client_unique_id.size(); i++)
    {
        client_unique_id[i] = static_cast<uavcan::uint8_t>(0xF0U + i);
    }

    // Preferred node ID is occupied; allocator is exhausted so it must reuse the oldest allocated one.
    const uavcan::NodeID PreferredNodeID = 42;
    ASSERT_LE(0, client.start(client_unique_id, PreferredNodeID));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(15000));

    ASSERT_TRUE(client.isAllocationComplete());

    // Oldest allocation (skipping allocator's own Node ID=10) is Node ID 1.
    ASSERT_EQ(uavcan::NodeID(1), client.getAllocatedNodeID());

    // Verify that the old UniqueID->NodeID mapping for the evicted node ID has been removed.
    {
        UniqueID old_uid;
        old_uid[0] = 1;
        const uavcan::dynamic_node_id_server::IStorageBackend::String old_key =
            uavcan::dynamic_node_id_server::StorageMarshaller::convertUniqueIDToHex(old_uid);
        ASSERT_TRUE(storage.get(old_key).empty());
    }

    // Verify that the new client UniqueID now maps to Node ID 1.
    {
        UniqueID new_uid;
        for (uint8_t i = 0; i < UniqueID::MaxSize; i++)
        {
            new_uid[i] = static_cast<uint8_t>(0xF0U + i);
        }
        const uavcan::dynamic_node_id_server::IStorageBackend::String new_key =
            uavcan::dynamic_node_id_server::StorageMarshaller::convertUniqueIDToHex(new_uid);
        ASSERT_EQ("1", storage.get(new_key));
    }
}


TEST(dynamic_node_id_server_centralized, ObjectSizes)
{
    using namespace uavcan::dynamic_node_id_server;
    std::cout << "centralized::Storage: " << sizeof(centralized::Storage) << std::endl;
    std::cout << "centralized::Server:  " << sizeof(centralized::Server) << std::endl;
    std::cout << "NodeDiscoverer:       " << sizeof(NodeDiscoverer) << std::endl;
}
