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
        container_[key] = value;
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
    const unsigned NumEntriesInStorageWithEmptyLog = 4;  // last index + 3 items per log entry

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
