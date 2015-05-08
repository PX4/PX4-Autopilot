/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <map>
#include <uavcan/protocol/dynamic_node_id_allocation_server.hpp>
#include "helpers.hpp"

class StorageBackend : public uavcan::IDynamicNodeIDStorageBackend
{
    typedef std::map<String, String> Container;
    Container container_;

    bool fail_;

public:
    StorageBackend()
        : fail_(false)
    { }

    virtual String get(const String& key) const
    {
        const Container::const_iterator it = container_.find(key);
        if (it == container_.end())
        {
            return String();
        }
        return it->second;
    }

    virtual void set(const String& key, const String& value)
    {
        if (!fail_)
        {
            container_[key] = value;
        }
    }

    void failOnSetCalls(bool really) { fail_ = really; }

    void reset() { container_.clear(); }

    unsigned getNumKeys() const { return unsigned(container_.size()); }

    void print() const
    {
        for (Container::const_iterator it = container_.begin(); it != container_.end(); ++it)
        {
            std::cout << it->first.c_str() << "\t" << it->second.c_str() << std::endl;
        }
    }
};


class EventTracer : public uavcan::IDynamicNodeIDAllocationServerEventTracer
{
    virtual void onEvent(uavcan::uint16_t event_code, uavcan::int64_t event_argument)
    {
        std::cout << "Event\t" << event_code << "\t" << event_argument << std::endl;
    }
};


static const unsigned NumEntriesInStorageWithEmptyLog = 4;  // last index + 3 items per log entry


TEST(DynamicNodeIDAllocationServer, MarshallingStorageDecorator)
{
    StorageBackend st;

    uavcan::dynamic_node_id_server_impl::MarshallingStorageDecorator marshaler(st);

    uavcan::IDynamicNodeIDStorageBackend::String key;

    /*
     * uint32
     */
    uint32_t u32 = 0;

    key = "foo";
    u32 = 0;
    ASSERT_LE(0, marshaler.setAndGetBack(key, u32));
    ASSERT_EQ(0, u32);

    key = "bar";
    u32 = 0xFFFFFFFF;
    ASSERT_LE(0, marshaler.setAndGetBack(key, u32));
    ASSERT_EQ(0xFFFFFFFF, u32);
    ASSERT_LE(0, marshaler.get(key, u32));
    ASSERT_EQ(0xFFFFFFFF, u32);

    key = "foo";
    ASSERT_LE(0, marshaler.get(key, u32));
    ASSERT_EQ(0, u32);

    key = "the_cake_is_a_lie";
    ASSERT_GT(0, marshaler.get(key, u32));
    ASSERT_EQ(0, u32);

    /*
     * uint8[16]
     */
    uavcan::protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id array;

    key = "a";
    // Set zero
    ASSERT_LE(0, marshaler.setAndGetBack(key, array));
    for (uint8_t i = 0; i < 16; i++)
    {
        ASSERT_EQ(0, array[i]);
    }

    // Make sure this will not be interpreted as uint32
    ASSERT_GT(0, marshaler.get(key, u32));
    ASSERT_EQ(0, u32);

    // Set pattern
    for (uint8_t i = 0; i < 16; i++)
    {
        array[i] = uint8_t(i + 1);
    }
    ASSERT_LE(0, marshaler.setAndGetBack(key, array));
    for (uint8_t i = 0; i < 16; i++)
    {
        ASSERT_EQ(i + 1, array[i]);
    }

    // Set another pattern
    for (uint8_t i = 0; i < 16; i++)
    {
        array[i] = uint8_t(i | (i << 4));
    }
    ASSERT_LE(0, marshaler.setAndGetBack(key, array));
    for (uint8_t i = 0; i < 16; i++)
    {
        ASSERT_EQ(uint8_t(i | (i << 4)), array[i]);
    }

    // Make sure uint32 cannot be interpreted as an array
    key = "foo";
    ASSERT_GT(0, marshaler.get(key, array));

    // Nonexistent key
    key = "the_cake_is_a_lie";
    ASSERT_GT(0, marshaler.get(key, array));
}


TEST(DynamicNodeIDAllocationServer, LogInitialization)
{
    EventTracer tracer;
    // No log data in the storage - initializing empty log
    {
        StorageBackend storage;
        uavcan::dynamic_node_id_server_impl::Log log(storage, tracer);

        ASSERT_EQ(0, storage.getNumKeys());
        ASSERT_LE(0, log.init());
        ASSERT_EQ(NumEntriesInStorageWithEmptyLog, storage.getNumKeys());
        ASSERT_EQ(0, log.getLastIndex());
        ASSERT_EQ(0, log.getEntryAtIndex(0)->term);
        ASSERT_EQ(0, log.getEntryAtIndex(0)->node_id);
        ASSERT_EQ(uavcan::protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id(),
                  log.getEntryAtIndex(0)->unique_id);
    }
    // Nonempty storage, one item
    {
        StorageBackend storage;
        uavcan::dynamic_node_id_server_impl::Log log(storage, tracer);

        storage.set("log_last_index", "0");
        ASSERT_LE(-uavcan::ErrFailure, log.init());     // Expected one entry, none found
        ASSERT_EQ(1, storage.getNumKeys());

        storage.set("log0_term",      "0");
        storage.set("log0_unique_id", "00000000000000000000000000000000");
        storage.set("log0_node_id",   "0");
        ASSERT_LE(0, log.init());                       // OK now
        ASSERT_EQ(NumEntriesInStorageWithEmptyLog, storage.getNumKeys());
        ASSERT_EQ(0, log.getLastIndex());
        ASSERT_EQ(0, log.getEntryAtIndex(0)->term);
    }
    // Nonempty storage, broken data
    {
        StorageBackend storage;
        uavcan::dynamic_node_id_server_impl::Log log(storage, tracer);

        storage.set("log_last_index", "foobar");
        ASSERT_LE(-uavcan::ErrFailure, log.init());     // Bad value

        storage.set("log_last_index", "128");
        ASSERT_LE(-uavcan::ErrFailure, log.init());     // Bad value

        storage.set("log_last_index", "0");
        ASSERT_LE(-uavcan::ErrFailure, log.init());     // No log items
        ASSERT_EQ(1, storage.getNumKeys());

        storage.set("log0_term",      "0");
        storage.set("log0_unique_id", "00000000000000000000000000000000");
        storage.set("log0_node_id",   "128");           // Bad value (127 max)
        ASSERT_LE(-uavcan::ErrFailure, log.init());     // Failed
        ASSERT_EQ(0, log.getLastIndex());
        ASSERT_EQ(0, log.getEntryAtIndex(0)->term);
        ASSERT_EQ(4, storage.getNumKeys());
    }
    // Nonempty storage, many items
    {
        StorageBackend storage;
        uavcan::dynamic_node_id_server_impl::Log log(storage, tracer);

        storage.set("log_last_index", "1");  // 2 items - 0, 1
        storage.set("log0_term",      "0");
        storage.set("log0_unique_id", "00000000000000000000000000000000");
        storage.set("log0_node_id",   "0");
        storage.set("log1_term",      "1");
        storage.set("log1_unique_id", "0123456789abcdef0123456789abcdef");
        storage.set("log1_node_id",   "127");

        ASSERT_LE(0, log.init());                       // OK now
        ASSERT_EQ(7, storage.getNumKeys());
        ASSERT_EQ(1, log.getLastIndex());

        ASSERT_EQ(0, log.getEntryAtIndex(0)->term);
        ASSERT_EQ(0, log.getEntryAtIndex(0)->node_id);
        ASSERT_EQ(uavcan::protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id(),
                  log.getEntryAtIndex(0)->unique_id);

        ASSERT_EQ(1, log.getEntryAtIndex(1)->term);
        ASSERT_EQ(127, log.getEntryAtIndex(1)->node_id);
        uavcan::protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id uid;
        uid[0] = 0x01;
        uid[1] = 0x23;
        uid[2] = 0x45;
        uid[3] = 0x67;
        uid[4] = 0x89;
        uid[5] = 0xab;
        uid[6] = 0xcd;
        uid[7] = 0xef;
        uavcan::copy(uid.begin(), uid.begin() + 8, uid.begin() + 8);
        ASSERT_EQ(uid, log.getEntryAtIndex(1)->unique_id);
    }
}


TEST(DynamicNodeIDAllocationServer, LogAppend)
{
    EventTracer tracer;
    StorageBackend storage;
    uavcan::dynamic_node_id_server_impl::Log log(storage, tracer);

    ASSERT_EQ(0, storage.getNumKeys());
    ASSERT_LE(0, log.init());
    storage.print();
    ASSERT_EQ(NumEntriesInStorageWithEmptyLog, storage.getNumKeys());

    /*
     * Entry at the index 0 always exists, and it's always zero-initialized.
     */
    ASSERT_EQ("0",                                storage.get("log_last_index"));
    ASSERT_EQ("0",                                storage.get("log0_term"));
    ASSERT_EQ("00000000000000000000000000000000", storage.get("log0_unique_id"));
    ASSERT_EQ("0",                                storage.get("log0_node_id"));

    /*
     * Adding one entry to the log, making sure it appears in the storage
     */
    uavcan::protocol::dynamic_node_id::server::Entry entry;
    entry.term = 1;
    entry.node_id = 1;
    entry.unique_id[0] = 1;
    ASSERT_LE(0, log.append(entry));

    ASSERT_EQ("1",                                storage.get("log_last_index"));
    ASSERT_EQ("1",                                storage.get("log1_term"));
    ASSERT_EQ("01000000000000000000000000000000", storage.get("log1_unique_id"));
    ASSERT_EQ("1",                                storage.get("log1_node_id"));

    ASSERT_EQ(1, log.getLastIndex());
    ASSERT_TRUE(entry == *log.getEntryAtIndex(1));

    /*
     * Adding another entry while storage is failing
     */
    storage.failOnSetCalls(true);

    ASSERT_EQ(7, storage.getNumKeys());

    entry.term = 2;
    entry.node_id = 2;
    entry.unique_id[0] = 2;
    ASSERT_GT(0, log.append(entry));

    ASSERT_EQ(7, storage.getNumKeys());  // No new entries, we failed

    ASSERT_EQ(1, log.getLastIndex());

    /*
     * Making sure append() fails when the log is full
     */
    storage.failOnSetCalls(false);

    while (log.getLastIndex() < (log.Capacity - 1))
    {
        ASSERT_LE(0, log.append(entry));
        ASSERT_TRUE(entry == *log.getEntryAtIndex(log.getLastIndex()));

        entry.term += 1;
        entry.node_id = uint8_t(entry.node_id + 1U);
        entry.unique_id[0] = uint8_t(entry.unique_id[0] + 1U);
    }

    ASSERT_GT(0, log.append(entry));  // Failing because full

    storage.print();
}


TEST(DynamicNodeIDAllocationServer, LogRemove)
{
    EventTracer tracer;
    StorageBackend storage;
    uavcan::dynamic_node_id_server_impl::Log log(storage, tracer);

    /*
     * Filling the log fully
     */
    uavcan::protocol::dynamic_node_id::server::Entry entry;
    entry.term = 1;
    entry.node_id = 1;
    entry.unique_id[0] = 1;

    while (log.getLastIndex() < (log.Capacity - 1))
    {
        ASSERT_LE(0, log.append(entry));
        ASSERT_TRUE(entry == *log.getEntryAtIndex(log.getLastIndex()));

        entry.term += 1;
        entry.node_id = uint8_t(entry.node_id + 1U);
        entry.unique_id[0] = uint8_t(entry.unique_id[0] + 1U);
    }

    /*
     * Removal will fail as the storage is failing to update
     */
    storage.failOnSetCalls(true);

    ASSERT_EQ(log.Capacity - 1, log.getLastIndex());
    ASSERT_GT(0, log.removeEntriesWhereIndexGreaterOrEqual(60));  // Failing
    ASSERT_EQ(log.Capacity - 1, log.getLastIndex());

    /*
     * Now removal must work
     */
    storage.failOnSetCalls(false);

    ASSERT_EQ(log.Capacity - 1, log.getLastIndex());

    ASSERT_LE(0, log.removeEntriesWhereIndexGreaterOrEqual(60));
    ASSERT_EQ(59, log.getLastIndex());
    ASSERT_EQ("59", storage.get("log_last_index"));

    ASSERT_LE(0, log.removeEntriesWhereIndexGreater(30));
    ASSERT_EQ(30, log.getLastIndex());
    ASSERT_EQ("30", storage.get("log_last_index"));

    ASSERT_LE(0, log.removeEntriesWhereIndexGreaterOrEqual(1));
    ASSERT_EQ(0, log.getLastIndex());
    ASSERT_EQ("0", storage.get("log_last_index"));

    storage.print();
}


TEST(DynamicNodeIDAllocationServer, PersistentStorageInitialization)
{
    EventTracer tracer;
    /*
     * First initialization
     */
    {
        StorageBackend storage;
        uavcan::dynamic_node_id_server_impl::PersistentState pers(storage, tracer);

        ASSERT_EQ(0, storage.getNumKeys());
        ASSERT_LE(0, pers.init());

        ASSERT_LE(3, storage.getNumKeys());
        ASSERT_EQ("0", storage.get("log_last_index"));
        ASSERT_EQ("0", storage.get("current_term"));
        ASSERT_EQ("0", storage.get("voted_for"));
    }
    /*
     * Partial recovery - only empty log is recovered
     */
    {
        StorageBackend storage;

        {
            // This log is used to initialize the storage
            uavcan::dynamic_node_id_server_impl::Log log(storage, tracer);
            ASSERT_LE(0, log.init());
        }
        ASSERT_LE(1, storage.getNumKeys());

        uavcan::dynamic_node_id_server_impl::PersistentState pers(storage, tracer);

        ASSERT_LE(0, pers.init());

        ASSERT_LE(3, storage.getNumKeys());
        ASSERT_EQ("0", storage.get("log_last_index"));
        ASSERT_EQ("0", storage.get("current_term"));
        ASSERT_EQ("0", storage.get("voted_for"));
    }
    /*
     * Partial recovery - log and current term are recovered
     */
    {
        StorageBackend storage;

        {
            // This log is used to initialize the storage
            uavcan::dynamic_node_id_server_impl::Log log(storage, tracer);
            ASSERT_LE(0, log.init());
        }
        ASSERT_LE(1, storage.getNumKeys());

        storage.set("current_term", "1");

        uavcan::dynamic_node_id_server_impl::PersistentState pers(storage, tracer);

        ASSERT_GT(0, pers.init());              // Fails because current term is not zero

        storage.set("current_term", "0");

        ASSERT_LE(0, pers.init());              // OK now

        ASSERT_LE(3, storage.getNumKeys());
        ASSERT_EQ("0", storage.get("log_last_index"));
        ASSERT_EQ("0", storage.get("current_term"));
        ASSERT_EQ("0", storage.get("voted_for"));
    }
    /*
     * Full recovery
     */
    {
        StorageBackend storage;

        {
            // This log is used to initialize the storage
            uavcan::dynamic_node_id_server_impl::Log log(storage, tracer);
            ASSERT_LE(0, log.init());

            uavcan::protocol::dynamic_node_id::server::Entry entry;
            entry.term = 1;
            entry.node_id = 1;
            entry.unique_id[0] = 1;
            ASSERT_LE(0, log.append(entry));
        }
        ASSERT_LE(4, storage.getNumKeys());

        uavcan::dynamic_node_id_server_impl::PersistentState pers(storage, tracer);

        ASSERT_GT(0, pers.init());              // Fails because log is not empty

        storage.set("current_term", "0");
        storage.set("voted_for", "0");
        ASSERT_GT(0, pers.init());              // Fails because of bad currentTerm

        storage.set("current_term", "1");       // OK
        storage.set("voted_for", "128");        // Invalid value
        ASSERT_GT(0, pers.init());              // Fails because of bad votedFor

        storage.set("voted_for", "0");          // OK now
        ASSERT_LE(0, pers.init());

        ASSERT_LE(3, storage.getNumKeys());
        ASSERT_EQ("1", storage.get("log_last_index"));
        ASSERT_EQ("1", storage.get("current_term"));
        ASSERT_EQ("0", storage.get("voted_for"));
    }
}


TEST(DynamicNodeIDAllocationServer, PersistentStorage)
{
    EventTracer tracer;
    StorageBackend storage;
    uavcan::dynamic_node_id_server_impl::PersistentState pers(storage, tracer);

    /*
     * Initializing
     */
    ASSERT_LE(0, pers.init());

    ASSERT_EQ("0", storage.get("log_last_index"));
    ASSERT_EQ("0", storage.get("current_term"));
    ASSERT_EQ("0", storage.get("voted_for"));

    /*
     * Inserting some log entries
     */
    uavcan::protocol::dynamic_node_id::server::Entry entry;
    entry.term = 1;
    entry.node_id = 1;
    entry.unique_id[0] = 1;
    ASSERT_LE(0, pers.getLog().append(entry));

    ASSERT_EQ("1", storage.get("log_last_index"));
    ASSERT_EQ("0", storage.get("current_term"));
    ASSERT_EQ("0", storage.get("voted_for"));

    /*
     * Changing current term
     */
    ASSERT_EQ(0, pers.getCurrentTerm());
    ASSERT_LE(0, pers.setCurrentTerm(2));
    ASSERT_EQ(2, pers.getCurrentTerm());

    ASSERT_EQ("1", storage.get("log_last_index"));
    ASSERT_EQ("2", storage.get("current_term"));
    ASSERT_EQ("0", storage.get("voted_for"));

    /*
     * Changing votedFor
     */
    ASSERT_FALSE(pers.isVotedForSet());
    ASSERT_EQ(0, pers.getVotedFor().get());
    ASSERT_LE(0, pers.setVotedFor(0));
    ASSERT_EQ(0, pers.getVotedFor().get());
    ASSERT_LE(0, pers.setVotedFor(45));
    ASSERT_EQ(45, pers.getVotedFor().get());
    ASSERT_TRUE(pers.isVotedForSet());

    ASSERT_EQ("1", storage.get("log_last_index"));
    ASSERT_EQ("2", storage.get("current_term"));
    ASSERT_EQ("45", storage.get("voted_for"));

    ASSERT_LE(0, pers.resetVotedFor());
    ASSERT_EQ(0, pers.getVotedFor().get());
    ASSERT_FALSE(pers.isVotedForSet());
    ASSERT_EQ("0", storage.get("voted_for"));

    ASSERT_LE(0, pers.setVotedFor(45));
    ASSERT_TRUE(pers.isVotedForSet());
    ASSERT_EQ("45", storage.get("voted_for"));

    /*
     * Handling errors
     */
    storage.failOnSetCalls(true);

    ASSERT_EQ(2, pers.getCurrentTerm());
    ASSERT_GT(0, pers.setCurrentTerm(7893));
    ASSERT_EQ(2, pers.getCurrentTerm());

    ASSERT_EQ(45, pers.getVotedFor().get());
    ASSERT_GT(0, pers.setVotedFor(78));
    ASSERT_EQ(45, pers.getVotedFor().get());

    ASSERT_EQ("1", storage.get("log_last_index"));
    ASSERT_EQ("2", storage.get("current_term"));
    ASSERT_EQ("45", storage.get("voted_for"));

    /*
     * Final checks
     */
    ASSERT_GT(10, storage.getNumKeys());  // Making sure there's some sane number of keys in the storage
}


TEST(DynamicNodeIDAllocationServer, ClusterManagerInitialization)
{
    const unsigned MaxClusterSize =
        uavcan::protocol::dynamic_node_id::server::Discovery::FieldTypes::known_nodes::MaxSize;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::dynamic_node_id::server::Discovery> _reg1;

    EventTracer tracer;

    /*
     * Simple initialization
     */
    {
        StorageBackend storage;
        uavcan::dynamic_node_id_server_impl::Log log(storage, tracer);
        InterlinkedTestNodesWithSysClock nodes;

        uavcan::dynamic_node_id_server_impl::ClusterManager mgr(nodes.a, storage, log, tracer);

        // Too big
        ASSERT_GT(0, mgr.init(MaxClusterSize + 1));
        ASSERT_EQ(0, storage.getNumKeys());

        // OK
        ASSERT_LE(0, mgr.init(5));
        ASSERT_EQ(1, storage.getNumKeys());
        ASSERT_EQ("5", storage.get("cluster_size"));

        // Testing other states
        ASSERT_EQ(0, mgr.getNumKnownServers());
        ASSERT_EQ(5, mgr.getClusterSize());
        ASSERT_EQ(3, mgr.getQuorumSize());
        ASSERT_FALSE(mgr.getRemoteServerNodeIDAtIndex(0).isValid());
    }
    /*
     * Recovery from the storage
     */
    {
        StorageBackend storage;
        uavcan::dynamic_node_id_server_impl::Log log(storage, tracer);
        InterlinkedTestNodesWithSysClock nodes;

        uavcan::dynamic_node_id_server_impl::ClusterManager mgr(nodes.a, storage, log, tracer);

        // Not configured
        ASSERT_GT(0, mgr.init());
        ASSERT_EQ(0, storage.getNumKeys());

        // OK
        storage.set("cluster_size", "5");
        ASSERT_LE(0, mgr.init());
        ASSERT_EQ(1, storage.getNumKeys());
    }
}


TEST(DynamicNodeIDAllocationServer, ClusterManagerOneServer)
{
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::dynamic_node_id::server::Discovery> _reg1;

    EventTracer tracer;
    StorageBackend storage;
    uavcan::dynamic_node_id_server_impl::Log log(storage, tracer);
    InterlinkedTestNodesWithSysClock nodes;

    uavcan::dynamic_node_id_server_impl::ClusterManager mgr(nodes.a, storage, log, tracer);

    /*
     * Pub and sub
     */
    SubscriberWithCollector<uavcan::protocol::dynamic_node_id::server::Discovery> sub(nodes.b);
    uavcan::Publisher<uavcan::protocol::dynamic_node_id::server::Discovery> pub(nodes.b);

    ASSERT_LE(0, sub.start());
    ASSERT_LE(0, pub.init());

    /*
     * Starting
     */
    ASSERT_LE(0, mgr.init(1));

    ASSERT_EQ(0, mgr.getNumKnownServers());
    ASSERT_TRUE(mgr.isClusterDiscovered());

    ASSERT_EQ(0, nodes.a.internal_failure_count);

    /*
     * Broadcasting discovery with wrong cluster size, it will be reported as internal failure
     */
    uavcan::protocol::dynamic_node_id::server::Discovery msg;
    msg.configured_cluster_size = 2;
    msg.known_nodes.push_back(2U);
    ASSERT_LE(0, pub.broadcast(msg));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));

    ASSERT_EQ(1, nodes.a.internal_failure_count);

    /*
     * Discovery rate limiting test
     */
    ASSERT_FALSE(sub.collector.msg.get());

    msg = uavcan::protocol::dynamic_node_id::server::Discovery();
    msg.configured_cluster_size = 1;              // Correct value
    ASSERT_LE(0, pub.broadcast(msg));             // List of known nodes is empty, intentionally

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(100));
    ASSERT_FALSE(sub.collector.msg.get());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1000));
    ASSERT_TRUE(sub.collector.msg.get());
    ASSERT_EQ(1, sub.collector.msg->configured_cluster_size);
    ASSERT_EQ(1, sub.collector.msg->known_nodes.size());
    ASSERT_EQ(1, sub.collector.msg->known_nodes[0]);
    sub.collector.msg.reset();

    // Rinse repeat
    ASSERT_LE(0, pub.broadcast(msg));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(100));
    ASSERT_FALSE(sub.collector.msg.get());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1000));
    ASSERT_TRUE(sub.collector.msg.get());
    ASSERT_EQ(1, sub.collector.msg->configured_cluster_size);
    ASSERT_EQ(1, sub.collector.msg->known_nodes.size());
    ASSERT_EQ(1, sub.collector.msg->known_nodes[0]);
    sub.collector.msg.reset();
}


TEST(DynamicNodeIDAllocationServer, ClusterManagerThreeServers)
{
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::dynamic_node_id::server::Discovery> _reg1;

    EventTracer tracer;
    StorageBackend storage;
    uavcan::dynamic_node_id_server_impl::Log log(storage, tracer);
    InterlinkedTestNodesWithSysClock nodes;

    uavcan::dynamic_node_id_server_impl::ClusterManager mgr(nodes.a, storage, log, tracer);

    /*
     * Pub and sub
     */
    SubscriberWithCollector<uavcan::protocol::dynamic_node_id::server::Discovery> sub(nodes.b);
    uavcan::Publisher<uavcan::protocol::dynamic_node_id::server::Discovery> pub(nodes.b);

    ASSERT_LE(0, sub.start());
    ASSERT_LE(0, pub.init());

    /*
     * Starting
     */
    ASSERT_LE(0, mgr.init(3));

    ASSERT_EQ(0, mgr.getNumKnownServers());
    ASSERT_FALSE(mgr.isClusterDiscovered());

    /*
     * Discovery publishing rate check
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(100));
    ASSERT_FALSE(sub.collector.msg.get());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1000));
    ASSERT_TRUE(sub.collector.msg.get());
    ASSERT_EQ(3, sub.collector.msg->configured_cluster_size);
    ASSERT_EQ(1, sub.collector.msg->known_nodes.size());
    ASSERT_EQ(1, sub.collector.msg->known_nodes[0]);
    sub.collector.msg.reset();

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(100));
    ASSERT_FALSE(sub.collector.msg.get());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1000));
    ASSERT_TRUE(sub.collector.msg.get());
    ASSERT_EQ(3, sub.collector.msg->configured_cluster_size);
    ASSERT_EQ(1, sub.collector.msg->known_nodes.size());
    ASSERT_EQ(1, sub.collector.msg->known_nodes[0]);
    sub.collector.msg.reset();

    /*
     * Discovering other nodes
     */
    uavcan::protocol::dynamic_node_id::server::Discovery msg;
    msg.configured_cluster_size = 3;
    msg.known_nodes.push_back(2U);
    ASSERT_LE(0, pub.broadcast(msg));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1050));
    ASSERT_TRUE(sub.collector.msg.get());
    ASSERT_EQ(3, sub.collector.msg->configured_cluster_size);
    ASSERT_EQ(2, sub.collector.msg->known_nodes.size());
    ASSERT_EQ(1, sub.collector.msg->known_nodes[0]);
    ASSERT_EQ(2, sub.collector.msg->known_nodes[1]);
    sub.collector.msg.reset();

    ASSERT_FALSE(mgr.isClusterDiscovered());

    // This will complete the discovery
    msg.known_nodes.push_back(127U);
    ASSERT_LE(0, pub.broadcast(msg));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1050));
    ASSERT_TRUE(sub.collector.msg.get());
    ASSERT_EQ(3, sub.collector.msg->configured_cluster_size);
    ASSERT_EQ(3, sub.collector.msg->known_nodes.size());
    ASSERT_EQ(1, sub.collector.msg->known_nodes[0]);
    ASSERT_EQ(2, sub.collector.msg->known_nodes[1]);
    ASSERT_EQ(127, sub.collector.msg->known_nodes[2]);
    sub.collector.msg.reset();

    // Making sure discovery is now terminated
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1500));
    ASSERT_FALSE(sub.collector.msg.get());

    /*
     * Checking Raft states
     */
    ASSERT_EQ(uavcan::NodeID(2),   mgr.getRemoteServerNodeIDAtIndex(0));
    ASSERT_EQ(uavcan::NodeID(127), mgr.getRemoteServerNodeIDAtIndex(1));
    ASSERT_EQ(uavcan::NodeID(),    mgr.getRemoteServerNodeIDAtIndex(2));

    ASSERT_EQ(0, mgr.getServerMatchIndex(2));
    ASSERT_EQ(0, mgr.getServerMatchIndex(127));

    ASSERT_EQ(log.getLastIndex() + 1, mgr.getServerNextIndex(2));
    ASSERT_EQ(log.getLastIndex() + 1, mgr.getServerNextIndex(127));

    mgr.setServerMatchIndex(2, 10);
    ASSERT_EQ(10, mgr.getServerMatchIndex(2));

    mgr.incrementServerNextIndexBy(2, 5);
    ASSERT_EQ(log.getLastIndex() + 1 + 5, mgr.getServerNextIndex(2));
    mgr.decrementServerNextIndex(2);
    ASSERT_EQ(log.getLastIndex() + 1 + 5 - 1, mgr.getServerNextIndex(2));

    mgr.resetAllServerIndices();

    ASSERT_EQ(0, mgr.getServerMatchIndex(2));
    ASSERT_EQ(0, mgr.getServerMatchIndex(127));

    ASSERT_EQ(log.getLastIndex() + 1, mgr.getServerNextIndex(2));
    ASSERT_EQ(log.getLastIndex() + 1, mgr.getServerNextIndex(127));
}


TEST(DynamicNodeIDAllocationServer, ObjectSizes)
{
    std::cout << "Log:             " << sizeof(uavcan::dynamic_node_id_server_impl::Log) << std::endl;
    std::cout << "PersistentState: " << sizeof(uavcan::dynamic_node_id_server_impl::PersistentState) << std::endl;
    std::cout << "ClusterManager:  " << sizeof(uavcan::dynamic_node_id_server_impl::ClusterManager) << std::endl;
    std::cout << "RaftCore:        " << sizeof(uavcan::dynamic_node_id_server_impl::RaftCore) << std::endl;
}
