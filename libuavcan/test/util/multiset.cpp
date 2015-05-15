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
    mset->remove("foo");
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

    // Dynamic addion
    ASSERT_EQ("3", *mset->add("3"));
    ASSERT_EQ(1, pool.getNumUsedBlocks());

    ASSERT_EQ("4", *mset->add("4"));
    ASSERT_EQ(1, pool.getNumUsedBlocks());       // Assuming that at least 2 items fit one block
    ASSERT_EQ(2, mset->getNumStaticItems());
    ASSERT_EQ(2, mset->getNumDynamicItems());

    // Making sure everything is here
    ASSERT_EQ("1", *mset->getByIndex(0));
    ASSERT_EQ("2", *mset->getByIndex(1));
    ASSERT_EQ("3", *mset->getByIndex(2));
    ASSERT_EQ("4", *mset->getByIndex(3));
    ASSERT_FALSE(mset->getByIndex(100));
    ASSERT_FALSE(mset->getByIndex(4));

    // Finding some items
    ASSERT_EQ("1", *mset->findFirst(FindPredicate("1")));
    ASSERT_EQ("2", *mset->findFirst(FindPredicate("2")));
    ASSERT_EQ("3", *mset->findFirst(FindPredicate("3")));
    ASSERT_EQ("4", *mset->findFirst(FindPredicate("4")));
    ASSERT_FALSE(mset->findFirst(FindPredicate("nonexistent")));

    // Removing one static
    mset->remove("1");                             // One of dynamics now migrates to the static storage
    mset->remove("foo");                           // There's no such thing anyway
    ASSERT_EQ(1, pool.getNumUsedBlocks());
    ASSERT_EQ(2, mset->getNumStaticItems());
    ASSERT_EQ(1, mset->getNumDynamicItems());

    // Ordering has not changed - first dynamic entry has moved to the first static slot
    ASSERT_EQ("3", *mset->getByIndex(0));
    ASSERT_EQ("2", *mset->getByIndex(1));
    ASSERT_EQ("4", *mset->getByIndex(2));

    // Removing another static
    mset->remove("2");
    ASSERT_EQ(2, mset->getNumStaticItems());
    ASSERT_EQ(0, mset->getNumDynamicItems());
    ASSERT_EQ(0, pool.getNumUsedBlocks());       // No dynamic entries left

    // Adding some new dynamics
    unsigned max_value_integer = 0;
    for (int i = 0; i < 100; i++)
    {
        const std::string value = toString(i);
        std::string* res = mset->add(value);  // Will override some from the above
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
    ASSERT_LT(4, max_value_integer);

    // Making sure there is true OOM
    ASSERT_EQ(0, pool.getNumFreeBlocks());
    ASSERT_FALSE(mset->add("nonexistent"));

    // Removing odd values - nearly half of them
    ASSERT_EQ(2, mset->getNumStaticItems());
    const unsigned num_dynamics_old = mset->getNumDynamicItems();
    mset->removeWhere(oddValuePredicate);
    ASSERT_EQ(2, mset->getNumStaticItems());
    const unsigned num_dynamics_new = mset->getNumDynamicItems();
    std::cout << "Num of dynamic pairs reduced from " << num_dynamics_old << " to " << num_dynamics_new << std::endl;
    ASSERT_LT(num_dynamics_new, num_dynamics_old);

    // Making sure there's no odd values left
    for (unsigned kv_int = 0; kv_int <= max_value_integer; kv_int++)
    {
        const std::string* val = mset->findFirst(FindPredicate(toString(kv_int)));
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

    static const int POOL_BLOCKS = 3;
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * POOL_BLOCKS, uavcan::MemPoolBlockSize> pool;
    uavcan::PoolManager<2> poolmgr;
    poolmgr.addPool(&pool);

    typedef Multiset<std::string> MultisetType;
    std::auto_ptr<MultisetType> mset(new MultisetType(poolmgr));

    // Empty
    mset->remove("foo");
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

    static const int POOL_BLOCKS = 3;
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * POOL_BLOCKS, uavcan::MemPoolBlockSize> pool;
    uavcan::PoolManager<2> poolmgr;
    poolmgr.addPool(&pool);

    typedef Multiset<short, 2> MultisetType;
    std::auto_ptr<MultisetType> mset(new MultisetType(poolmgr));

    // Empty
    mset->remove(8);
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
