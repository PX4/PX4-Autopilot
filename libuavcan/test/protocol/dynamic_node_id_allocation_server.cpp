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
    // No log data in the storage - initializing empty log
    {
        StorageBackend storage;
        uavcan::dynamic_node_id_server_impl::Log log(storage);

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
        uavcan::dynamic_node_id_server_impl::Log log(storage);

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
        uavcan::dynamic_node_id_server_impl::Log log(storage);

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
        uavcan::dynamic_node_id_server_impl::Log log(storage);

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
    StorageBackend storage;
    uavcan::dynamic_node_id_server_impl::Log log(storage);

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
    StorageBackend storage;
    uavcan::dynamic_node_id_server_impl::Log log(storage);

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

    ASSERT_LE(0, log.removeEntriesWhereIndexGreaterOrEqual(1));
    ASSERT_EQ(0, log.getLastIndex());

    ASSERT_EQ("0", storage.get("log_last_index"));

    storage.print();
}


TEST(DynamicNodeIDAllocationServer, PersistentStorageInitialization)
{
    /*
     * First initialization
     */
    {
        StorageBackend storage;
        uavcan::dynamic_node_id_server_impl::PersistentState pers(storage);

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
            uavcan::dynamic_node_id_server_impl::Log log(storage);
            ASSERT_LE(0, log.init());
        }
        ASSERT_LE(1, storage.getNumKeys());

        uavcan::dynamic_node_id_server_impl::PersistentState pers(storage);

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
            uavcan::dynamic_node_id_server_impl::Log log(storage);
            ASSERT_LE(0, log.init());
        }
        ASSERT_LE(1, storage.getNumKeys());

        storage.set("current_term", "1");

        uavcan::dynamic_node_id_server_impl::PersistentState pers(storage);

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
            uavcan::dynamic_node_id_server_impl::Log log(storage);
            ASSERT_LE(0, log.init());

            uavcan::protocol::dynamic_node_id::server::Entry entry;
            entry.term = 1;
            entry.node_id = 1;
            entry.unique_id[0] = 1;
            ASSERT_LE(0, log.append(entry));
        }
        ASSERT_LE(4, storage.getNumKeys());

        uavcan::dynamic_node_id_server_impl::PersistentState pers(storage);

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
    StorageBackend storage;
    uavcan::dynamic_node_id_server_impl::PersistentState pers(storage);

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
    ASSERT_EQ(0, pers.getVotedFor().get());
    ASSERT_LE(0, pers.setVotedFor(0));
    ASSERT_EQ(0, pers.getVotedFor().get());
    ASSERT_LE(0, pers.setVotedFor(45));
    ASSERT_EQ(45, pers.getVotedFor().get());

    ASSERT_EQ("1", storage.get("log_last_index"));
    ASSERT_EQ("2", storage.get("current_term"));
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
