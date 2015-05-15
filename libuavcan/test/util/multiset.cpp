/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <string>
#include <cstdio>
#include <memory>
#include <gtest/gtest.h>
#include <uavcan/util/multiset.hpp>


static std::string toString(long x)
{
    char buf[80];
    std::snprintf(buf, sizeof(buf), "%li", x);
    return std::string(buf);
}

static bool oddValuePredicate(const std::string& value)
{
    EXPECT_FALSE(value.empty());
    const int num = atoi(value.c_str());
    return num & 1;
}

struct FindPredicate
{
    const std::string target;
    FindPredicate(const std::string& target) : target(target) { }
    bool operator()(const std::string& value) const { return value == target; }
};


TEST(Multiset, Basic)
{
    using uavcan::Multiset;

    static const int POOL_BLOCKS = 3;
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * POOL_BLOCKS, uavcan::MemPoolBlockSize> pool;
    uavcan::PoolManager<2> poolmgr;
    poolmgr.addPool(&pool);

    typedef Multiset<std::string, 2> MultisetType;
    std::auto_ptr<MultisetType> mset(new MultisetType(poolmgr));

    // Empty
    mset->removeFirst("foo");
    ASSERT_EQ(0, pool.getNumUsedBlocks());
    ASSERT_FALSE(mset->getByIndex(0));
    ASSERT_FALSE(mset->getByIndex(1));
    ASSERT_FALSE(mset->getByIndex(10000));

    // Static addion
    ASSERT_EQ("1", *mset->add("1"));
    ASSERT_EQ("2", *mset->add("2"));
    ASSERT_EQ(0, pool.getNumUsedBlocks());
    ASSERT_EQ(2, mset->getNumStaticItems());
    ASSERT_EQ(0, mset->getNumDynamicItems());

    // Ordering
    ASSERT_TRUE(*mset->getByIndex(0) == "1");
    ASSERT_TRUE(*mset->getByIndex(1) == "2");

    // Dynamic addition
    ASSERT_EQ("3", *mset->add("3"));
    ASSERT_EQ("3", *mset->getByIndex(2));
    ASSERT_EQ(1, pool.getNumUsedBlocks());

    ASSERT_EQ("4", *mset->add("4"));
    ASSERT_LE(1, pool.getNumUsedBlocks());      // One or more
    ASSERT_EQ(2, mset->getNumStaticItems());
    ASSERT_EQ(2, mset->getNumDynamicItems());

    // Making sure everything is here
    ASSERT_EQ("1", *mset->getByIndex(0));
    ASSERT_EQ("2", *mset->getByIndex(1));
    // 2 and 3 are not tested because their placement depends on number of items per dynamic block
    ASSERT_FALSE(mset->getByIndex(100));
    ASSERT_FALSE(mset->getByIndex(4));

    const std::string data_at_pos2 = *mset->getByIndex(2);
    const std::string data_at_pos3 = *mset->getByIndex(3);

    // Finding some items
    ASSERT_EQ("1", *mset->find(FindPredicate("1")));
    ASSERT_EQ("2", *mset->find(FindPredicate("2")));
    ASSERT_EQ("3", *mset->find(FindPredicate("3")));
    ASSERT_EQ("4", *mset->find(FindPredicate("4")));
    ASSERT_FALSE(mset->find(FindPredicate("nonexistent")));

    // Removing one static; ordering will be preserved
    mset->removeFirst("1");
    mset->removeFirst("foo");                           // There's no such thing anyway
    ASSERT_LE(1, pool.getNumUsedBlocks());
    ASSERT_EQ(1, mset->getNumStaticItems());
    ASSERT_EQ(2, mset->getNumDynamicItems());           // This container does not move items

    // Ordering has not changed
    ASSERT_EQ("2", *mset->getByIndex(0));       // Entry "1" was here
    ASSERT_EQ(data_at_pos2, *mset->getByIndex(1));
    ASSERT_EQ(data_at_pos3, *mset->getByIndex(2));

    // Removing another static
    mset->removeFirst("2");
    ASSERT_EQ(0, mset->getNumStaticItems());
    ASSERT_EQ(2, mset->getNumDynamicItems());
    ASSERT_LE(1, pool.getNumUsedBlocks());

    // Adding some new items
    unsigned max_value_integer = 0;
    for (int i = 0; i < 100; i++)
    {
        const std::string value = toString(i);
        std::string* res = mset->add(value);  // Will NOT override above
        if (res == NULL)
        {
            ASSERT_LT(2, i);
            break;
        }
        else
        {
            ASSERT_EQ(value, *res);
        }
        max_value_integer = unsigned(i);
    }
    std::cout << "Max value: " << max_value_integer << std::endl;

    // Making sure there is true OOM
    ASSERT_EQ(0, pool.getNumFreeBlocks());
    ASSERT_FALSE(mset->add("nonexistent"));

    // Removing odd values - nearly half of them
    mset->removeAllMatching(oddValuePredicate);

    // Making sure there's no odd values left
    for (unsigned kv_int = 0; kv_int <= max_value_integer; kv_int++)
    {
        const std::string* val = mset->find(FindPredicate(toString(kv_int)));
        if (val)
        {
            ASSERT_FALSE(kv_int & 1);
        }
        else
        {
            ASSERT_TRUE(kv_int & 1);
        }
    }

    // Making sure the memory will be released
    mset.reset();
    ASSERT_EQ(0, pool.getNumUsedBlocks());
}


TEST(Multiset, NoStatic)
{
    using uavcan::Multiset;

    uavcan::PoolAllocator<1024, 128> pool;      // Large enough to keep everything
    uavcan::PoolManager<2> poolmgr;
    poolmgr.addPool(&pool);

    typedef Multiset<std::string> MultisetType;
    std::auto_ptr<MultisetType> mset(new MultisetType(poolmgr));

    ASSERT_LE(2, MultisetType::NumItemsPerDynamicChunk);

    // Empty
    mset->removeFirst("foo");
    ASSERT_EQ(0, pool.getNumUsedBlocks());
    ASSERT_FALSE(mset->getByIndex(0));

    // Insertion
    ASSERT_EQ("a", *mset->add("a"));
    ASSERT_EQ("b", *mset->add("b"));
    ASSERT_EQ(1, pool.getNumUsedBlocks());
    ASSERT_EQ(0, mset->getNumStaticItems());
    ASSERT_EQ(2, mset->getNumDynamicItems());

    // Ordering
    ASSERT_EQ("a", *mset->getByIndex(0));
    ASSERT_EQ("b", *mset->getByIndex(1));
    ASSERT_FALSE(mset->getByIndex(3));
    ASSERT_FALSE(mset->getByIndex(1000));
}


TEST(Multiset, PrimitiveKey)
{
    using uavcan::Multiset;

    uavcan::PoolAllocator<1024, 128> pool;      // Large enough to keep everything
    uavcan::PoolManager<2> poolmgr;
    poolmgr.addPool(&pool);

    typedef Multiset<int, 2> MultisetType;
    std::auto_ptr<MultisetType> mset(new MultisetType(poolmgr));

    ASSERT_LE(2, MultisetType::NumItemsPerDynamicChunk);

    // Empty
    mset->removeFirst(8);
    ASSERT_EQ(0, pool.getNumUsedBlocks());
    ASSERT_EQ(0, mset->getSize());
    ASSERT_FALSE(mset->getByIndex(0));

    // Insertion
    ASSERT_EQ(1, *mset->add(1));
    ASSERT_EQ(1, mset->getSize());
    ASSERT_EQ(2, *mset->add(2));
    ASSERT_EQ(2, mset->getSize());
    ASSERT_EQ(3, *mset->add(3));
    ASSERT_EQ(4, *mset->add(4));
    ASSERT_EQ(4, mset->getSize());

    // Ordering
    ASSERT_EQ(1, *mset->getByIndex(0));
    ASSERT_EQ(2, *mset->getByIndex(1));
    ASSERT_EQ(3, *mset->getByIndex(2));
    ASSERT_EQ(4, *mset->getByIndex(3));
    ASSERT_FALSE(mset->getByIndex(5));
    ASSERT_FALSE(mset->getByIndex(1000));
}
