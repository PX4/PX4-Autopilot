/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/internal/transport/transfer.hpp>
#include <uavcan/internal/transport/transfer_id_registry.hpp>


struct OddNodeIDPredicate
{
    bool operator()(const uavcan::TransferIDRegistry::Key& key, const uavcan::TransferIDRegistry::Entry& entry) const
    {
        return key.node_id & 1;
    }
};


TEST(TransferIDRegistry, Basic)
{
    using uavcan::Frame;
    using uavcan::TransferID;
    using uavcan::TransferType;
    using uavcan::TransferIDRegistry;
    typedef TransferIDRegistry::Key Key;
    typedef TransferIDRegistry::Entry Entry;

    static const int POOL_BLOCKS = 8;
    uavcan::PoolAllocator<uavcan::MEM_POOL_BLOCK_SIZE * POOL_BLOCKS, uavcan::MEM_POOL_BLOCK_SIZE> pool;
    uavcan::PoolManager<2> poolmgr;
    poolmgr.addPool(&pool);

    TransferIDRegistry reg(&poolmgr);

    ASSERT_EQ(NULL, reg.access(Key(0, uavcan::MESSAGE_BROADCAST, 0)));

    static const int NUM_ITEMS = 100;    // Just to make sure it will be enough
    Key keys[NUM_ITEMS];
    Entry entries[NUM_ITEMS];
    Entry immutable_entries[NUM_ITEMS];

    // Initializing the test data
    for (int i = 0; i < NUM_ITEMS; i++)
    {
        keys[i].data_type_id = i * (Frame::DATA_TYPE_ID_MAX / NUM_ITEMS);
        keys[i].node_id = i * (uavcan::NODE_ID_MAX / NUM_ITEMS);
        keys[i].transfer_type = TransferType(i % uavcan::NUM_TRANSFER_TYPES);

        entries[i].timestamp = i * 10000000;
        entries[i].transfer_id = TransferID(i % TransferID::MAX);
        immutable_entries[i] = entries[i];
    }

    // Filling the registry
    bool filled = false;
    int num_registered = 0;
    for (int i = 0; i < NUM_ITEMS; i++)
    {
        const bool res = reg.create(keys[i], entries[i]);
        if (!res)
        {
            ASSERT_EQ(0, pool.getNumFreeBlocks());
            const int num_entries_per_block = (i + 1) / POOL_BLOCKS;
            ASSERT_LE(2, num_entries_per_block);                      // Ensuring minimal number of entries per block
            filled = true;
            break;
        }
        num_registered++;
    }
    ASSERT_TRUE(filled);                        // No free buffer space left by now
    ASSERT_LT(POOL_BLOCKS, num_registered);     // Being paranoid

    // Checking each value
    for (int i = 0; i < num_registered; i++)
    {
        const Entry* const entry = reg.access(keys[i]);
        ASSERT_TRUE(entry);
        ASSERT_EQ(*entry, immutable_entries[i]);
    }

    // Removing half of the values, making sure some of the memory blocks were released
    const int num_blocks_to_remove = num_registered / 2;
    for (int i = 0; i < num_blocks_to_remove; i++)
    {
        reg.remove(keys[i]);
    }
    for (int i = 0; i < num_blocks_to_remove; i++)
    {
        ASSERT_FALSE(reg.access(keys[i]));
    }
    ASSERT_LT(1, pool.getNumFreeBlocks());      // At least one should be freed

    // Adding another entries, making sure they all fit the memory
    const int new_limit = num_registered + num_blocks_to_remove;
    for (int i = num_registered; i < new_limit; i++)
    {
        ASSERT_TRUE(reg.create(keys[i], entries[i]));
    }
    ASSERT_EQ(0, pool.getNumFreeBlocks());

    // Making sure the old entries didn't creep into the registry
    for (int i = 0; i < new_limit; i++)
    {
        const Entry* const entry = reg.access(keys[i]);
        if (i < num_blocks_to_remove)
        {
            ASSERT_FALSE(entry);
        }
        else
        {
            ASSERT_TRUE(entry);
            ASSERT_EQ(*entry, immutable_entries[i]);
        }
    }

    // Removing something, making sure it was removed indeed
    reg.removeWhere(OddNodeIDPredicate());

    for (int i = 0; i < new_limit; i++)
    {
        const Entry* const entry = reg.access(keys[i]);
        if (i < num_blocks_to_remove)
        {
            ASSERT_FALSE(entry);
        }
        else if (keys[i].node_id & 1)
        {
            ASSERT_FALSE(entry);
        }
        else
        {
            ASSERT_TRUE(entry);
            ASSERT_EQ(*entry, immutable_entries[i]);
        }
    }
}
