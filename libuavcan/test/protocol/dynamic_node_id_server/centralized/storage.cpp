/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/dynamic_node_id_server/centralized/storage.hpp>
#include "../../helpers.hpp"
#include "../memory_storage_backend.hpp"


TEST(dynamic_node_id_server_centralized_Storage, Initialization)
{
    using namespace uavcan::dynamic_node_id_server::centralized;

    // No data in the storage - initializing empty
    {
        MemoryStorageBackend storage;
        Storage stor(storage);

        ASSERT_EQ(0, storage.getNumKeys());
        ASSERT_LE(0, stor.init());

        ASSERT_EQ(1, storage.getNumKeys());
        ASSERT_EQ(0, stor.getSize());

        ASSERT_FALSE(stor.findByNodeID(1));
        ASSERT_FALSE(stor.findByNodeID(0));
    }
    // Nonempty storage, one item
    {
        MemoryStorageBackend storage;
        Storage stor(storage);

        storage.set("size", "1");
        ASSERT_EQ(-uavcan::ErrFailure, stor.init());     // Expected one entry, none found
        ASSERT_EQ(1, storage.getNumKeys());

        storage.set("0_unique_id", "00000000000000000000000000000000");
        storage.set("0_node_id",   "0");
        ASSERT_EQ(-uavcan::ErrFailure, stor.init());     // Invalid entry - zero Node ID

        storage.set("0_node_id",   "1");
        ASSERT_LE(0, stor.init());                       // OK now

        ASSERT_EQ(3, storage.getNumKeys());
        ASSERT_EQ(1, stor.getSize());

        ASSERT_TRUE(stor.findByNodeID(1));
        ASSERT_FALSE(stor.findByNodeID(0));
    }
    // Nonempty storage, broken data
    {
        MemoryStorageBackend storage;
        Storage stor(storage);

        storage.set("size", "foobar");
        ASSERT_EQ(-uavcan::ErrFailure, stor.init());     // Bad value

        storage.set("size", "128");
        ASSERT_EQ(-uavcan::ErrFailure, stor.init());     // Bad value

        storage.set("size", "1");
        ASSERT_EQ(-uavcan::ErrFailure, stor.init());     // No items
        ASSERT_EQ(1, storage.getNumKeys());

        storage.set("0_unique_id", "00000000000000000000000000000000");
        storage.set("0_node_id",   "128");               // Bad value (127 max)
        ASSERT_EQ(-uavcan::ErrFailure, stor.init());     // Failed

        storage.set("0_node_id",   "127");
        ASSERT_LE(0, stor.init());                       // OK now
        ASSERT_EQ(1, stor.getSize());

        ASSERT_TRUE(stor.findByNodeID(127));
        ASSERT_FALSE(stor.findByNodeID(0));

        ASSERT_EQ(3, storage.getNumKeys());
    }
    // Nonempty storage, many items
    {
        MemoryStorageBackend storage;
        Storage stor(storage);

        storage.set("size",        "2");
        storage.set("0_unique_id", "00000000000000000000000000000000");
        storage.set("0_node_id",   "1");
        storage.set("1_unique_id", "0123456789abcdef0123456789abcdef");
        storage.set("1_node_id",   "127");

        ASSERT_LE(0, stor.init());                       // OK now
        ASSERT_EQ(5, storage.getNumKeys());
        ASSERT_EQ(2, stor.getSize());

        ASSERT_TRUE(stor.findByNodeID(1));
        ASSERT_TRUE(stor.findByNodeID(127));
        ASSERT_FALSE(stor.findByNodeID(0));

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

        ASSERT_TRUE(stor.findByUniqueID(uid));
        ASSERT_EQ(127, stor.findByUniqueID(uid)->node_id.get());
        ASSERT_EQ(uid, stor.findByNodeID(127)->unique_id);
    }
}


TEST(dynamic_node_id_server_centralized_Storage, Basic)
{
    using namespace uavcan::dynamic_node_id_server::centralized;

    MemoryStorageBackend storage;
    Storage stor(storage);

    ASSERT_EQ(0, storage.getNumKeys());
    ASSERT_LE(0, stor.init());
    storage.print();
    ASSERT_EQ(1, storage.getNumKeys());

    /*
     * Adding one entry to the log, making sure it appears in the storage
     */
    Storage::Entry entry;
    entry.node_id = 1;
    entry.unique_id[0] = 1;
    ASSERT_LE(0, stor.add(entry.node_id, entry.unique_id));

    ASSERT_EQ("1",                                storage.get("size"));
    ASSERT_EQ("01000000000000000000000000000000", storage.get("0_unique_id"));
    ASSERT_EQ("1",                                storage.get("0_node_id"));

    ASSERT_EQ(3, storage.getNumKeys());
    ASSERT_EQ(1, stor.getSize());

    /*
     * Adding another entry while storage is failing
     */
    storage.failOnSetCalls(true);

    ASSERT_EQ(3, storage.getNumKeys());

    entry.node_id = 2;
    entry.unique_id[0] = 2;
    ASSERT_GT(0, stor.add(entry.node_id, entry.unique_id));

    ASSERT_EQ(3, storage.getNumKeys());  // No new entries, we failed

    ASSERT_EQ(1, stor.getSize());

    /*
     * Making sure add() fails when the log is full
     */
    storage.failOnSetCalls(false);

    while (stor.getSize() < stor.Capacity)
    {
        ASSERT_LE(0, stor.add(entry.node_id, entry.unique_id));

        entry.node_id = uint8_t(uavcan::min(entry.node_id.get() + 1U, 127U));
        entry.unique_id[0] = uint8_t(entry.unique_id[0] + 1U);
    }

    ASSERT_GT(0, stor.add(123, entry.unique_id));  // Failing because full

    storage.print();
}
