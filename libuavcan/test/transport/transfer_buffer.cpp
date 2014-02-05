/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <algorithm>
#include <gtest/gtest.h>
#include <uavcan/internal/transport/transfer_buffer.hpp>

static const std::string TEST_DATA =
    "It was like this: I asked myself one day this question - what if Napoleon, for instance, had happened to be in my "
    "place, and if he had not had Toulon nor Egypt nor the passage of Mont Blanc to begin his career with, but "
    "instead of all those picturesque and monumental things, there had simply been some ridiculous old hag, a "
    "pawnbroker, who had to be murdered too to get money from her trunk (for his career, you understand). "
    "Well, would he have brought himself to that if there had been no other means?";

template <typename T>
static bool allEqual(const T a)
{
    int n = sizeof(a) / sizeof(a[0]);
    while (--n > 0 && a[n] == a[0]) { }
    return n == 0;
}

template <typename T>
static void fill(const T a, int value)
{
    for (unsigned int i = 0; i < sizeof(a) / sizeof(a[0]); i++)
        a[i] = value;
}

static bool matchAgainstTestData(const uavcan::TransferBufferBase& tbb, unsigned int offset, int len = -1)
{
    uint8_t local_buffer[1024];
    fill(local_buffer, 0);
    assert((len < 0) || (sizeof(local_buffer) >= static_cast<unsigned int>(len)));

    if (len < 0)
    {
        const int res = tbb.read(offset, local_buffer, sizeof(local_buffer));
        if (res < 0)
        {
            std::cout << "matchAgainstTestData(): res " << res << std::endl;
            return false;
        }
        len = res;
    }
    else
    {
        const int res = tbb.read(offset, local_buffer, len);
        if (res != len)
        {
            std::cout << "matchAgainstTestData(): res " << res << " expected " << len << std::endl;
            return false;
        }
    }
    const bool equals = std::equal(local_buffer, local_buffer + len, TEST_DATA.begin() + offset);
    if (!equals)
    {
        std::cout
            << "local_buffer:\n\t" << local_buffer
            << std::endl;
        std::cout
            << "test_data:\n\t" << std::string(TEST_DATA.begin() + offset, TEST_DATA.begin() + offset + len)
            << std::endl;
    }
    return equals;
}

TEST(TransferBuffer, TestDataValidation)
{
    ASSERT_LE(4, TEST_DATA.length() / uavcan::MEM_POOL_BLOCK_SIZE);
    uint8_t local_buffer[50];
    std::copy(TEST_DATA.begin(), TEST_DATA.begin() + sizeof(local_buffer), local_buffer);
    ASSERT_FALSE(allEqual(local_buffer));
}

static const int TEST_BUFFER_SIZE = 200;

TEST(StaticTransferBuffer, Basic)
{
    using uavcan::StaticTransferBuffer;
    StaticTransferBuffer<TEST_BUFFER_SIZE> buf;

    uint8_t local_buffer[TEST_BUFFER_SIZE * 2];
    const uint8_t* const test_data_ptr = reinterpret_cast<const uint8_t*>(TEST_DATA.c_str());

    // Empty reads
    fill(local_buffer, 0xA5);
    ASSERT_EQ(0, buf.read(0, local_buffer, 999));
    ASSERT_EQ(0, buf.read(0, local_buffer, 0));
    ASSERT_EQ(0, buf.read(999, local_buffer, 0));
    ASSERT_TRUE(allEqual(local_buffer));

    // Bulk write
    ASSERT_EQ(TEST_BUFFER_SIZE, buf.write(0, test_data_ptr, TEST_DATA.length()));
    ASSERT_TRUE(matchAgainstTestData(buf, 0));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE / 2));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE / 2, TEST_BUFFER_SIZE / 4));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE / 4, TEST_BUFFER_SIZE / 2));
    ASSERT_TRUE(matchAgainstTestData(buf, 0, TEST_BUFFER_SIZE / 4));

    // Reset
    fill(local_buffer, 0xA5);
    buf.reset();
    ASSERT_EQ(0, buf.read(0, local_buffer, 0));
    ASSERT_EQ(0, buf.read(0, local_buffer, 999));
    ASSERT_TRUE(allEqual(local_buffer));

    // Random write
    ASSERT_EQ(21, buf.write(12, test_data_ptr + 12, 21));
    ASSERT_TRUE(matchAgainstTestData(buf, 12, 21));

    ASSERT_EQ(12, buf.write(0, test_data_ptr, 12));
    ASSERT_TRUE(matchAgainstTestData(buf, 0));

    ASSERT_EQ(0, buf.write(21, test_data_ptr + 21, 0));
    ASSERT_EQ(TEST_BUFFER_SIZE - 21, buf.write(21, test_data_ptr + 21, 999));
    ASSERT_TRUE(matchAgainstTestData(buf, 21, TEST_BUFFER_SIZE - 21));
    ASSERT_TRUE(matchAgainstTestData(buf, 0));
}


TEST(DynamicTransferBuffer, Basic)
{
    using uavcan::DynamicTransferBuffer;

    static const int POOL_BLOCKS = 8;
    uavcan::PoolAllocator<uavcan::MEM_POOL_BLOCK_SIZE * POOL_BLOCKS, uavcan::MEM_POOL_BLOCK_SIZE> pool;
    uavcan::PoolManager<2> poolmgr;
    poolmgr.addPool(&pool);

    DynamicTransferBuffer buf(&poolmgr);

    uint8_t local_buffer[TEST_BUFFER_SIZE * 2];
    const uint8_t* const test_data_ptr = reinterpret_cast<const uint8_t*>(TEST_DATA.c_str());

    // Empty reads
    fill(local_buffer, 0xA5);
    ASSERT_EQ(0, buf.read(0, local_buffer, 999));
    ASSERT_EQ(0, buf.read(0, local_buffer, 0));
    ASSERT_EQ(0, buf.read(999, local_buffer, 0));
    ASSERT_TRUE(allEqual(local_buffer));

    // Bulk write
    const int max_size = buf.write(0, test_data_ptr, TEST_DATA.length());
    std::cout << "Dynamic buffer contains " << max_size << " bytes" << std::endl;
    ASSERT_LE(max_size, TEST_DATA.length());
    ASSERT_GT(max_size, 0);

    ASSERT_EQ(0, pool.getNumFreeBlocks());      // Making sure all memory was used up

    ASSERT_TRUE(matchAgainstTestData(buf, 0));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE / 2));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE / 2, TEST_BUFFER_SIZE / 4));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE / 4, TEST_BUFFER_SIZE / 2));
    ASSERT_TRUE(matchAgainstTestData(buf, 0, TEST_BUFFER_SIZE / 4));

    // Reset
    fill(local_buffer, 0xA5);
    buf.reset();
    ASSERT_EQ(0, buf.read(0, local_buffer, 0));
    ASSERT_EQ(0, buf.read(0, local_buffer, 999));
    ASSERT_TRUE(allEqual(local_buffer));
    ASSERT_EQ(0, pool.getNumUsedBlocks());

    // Random write
    ASSERT_EQ(21, buf.write(12, test_data_ptr + 12, 21));
    ASSERT_TRUE(matchAgainstTestData(buf, 12, 21));

    ASSERT_EQ(60, buf.write(TEST_BUFFER_SIZE - 60, test_data_ptr + TEST_BUFFER_SIZE - 60, 60));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE - 60));

    // Now we have two empty regions: empty-data-empty-data

    ASSERT_EQ(0, buf.write(0, test_data_ptr, 0));
    ASSERT_EQ(TEST_BUFFER_SIZE - 21, buf.write(21, test_data_ptr + 21, TEST_BUFFER_SIZE - 21));
    ASSERT_TRUE(matchAgainstTestData(buf, 21, TEST_BUFFER_SIZE - 21));

    // Now: empty-data-data-data

    ASSERT_EQ(21, buf.write(0, test_data_ptr, 21));
    ASSERT_TRUE(matchAgainstTestData(buf, 0));

    // Reset
    ASSERT_LT(0, pool.getNumUsedBlocks());
    buf.reset();
    ASSERT_EQ(0, pool.getNumUsedBlocks());
}
