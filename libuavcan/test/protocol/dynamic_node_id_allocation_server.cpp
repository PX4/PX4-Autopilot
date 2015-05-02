/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <map>
#include <uavcan/protocol/dynamic_node_id_allocation_server.hpp>
#include "helpers.hpp"

class StorageBackend : public uavcan::IDynamicNodeIDStorageBackend
{
public:
    typedef std::map<String, String> Container;
    Container container_;

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

    void printContainer() const
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
    ASSERT_TRUE(marshaler.setAndGetBack(key, u32));
    ASSERT_EQ(0, u32);

    key = "bar";
    u32 = 0xFFFFFFFF;
    ASSERT_TRUE(marshaler.setAndGetBack(key, u32));
    ASSERT_EQ(0xFFFFFFFF, u32);
    ASSERT_TRUE(marshaler.get(key, u32));
    ASSERT_EQ(0xFFFFFFFF, u32);

    key = "foo";
    ASSERT_TRUE(marshaler.get(key, u32));
    ASSERT_EQ(0, u32);

    key = "the_cake_is_a_lie";
    ASSERT_FALSE(marshaler.get(key, u32));
    ASSERT_EQ(0, u32);

    /*
     * uint8[16]
     */
    uavcan::protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id array;

    key = "a";
    // Set zero
    ASSERT_TRUE(marshaler.setAndGetBack(key, array));
    for (uint8_t i = 0; i < 16; i++)
    {
        ASSERT_EQ(0, array[i]);
    }

    // Make sure this will not be interpreted as uint32
    ASSERT_FALSE(marshaler.get(key, u32));
    ASSERT_EQ(0, u32);

    // Set pattern
    for (uint8_t i = 0; i < 16; i++)
    {
        array[i] = uint8_t(i + 1);
    }
    ASSERT_TRUE(marshaler.setAndGetBack(key, array));
    for (uint8_t i = 0; i < 16; i++)
    {
        ASSERT_EQ(i + 1, array[i]);
    }

    // Set another pattern
    for (uint8_t i = 0; i < 16; i++)
    {
        array[i] = uint8_t(i | (i << 4));
    }
    ASSERT_TRUE(marshaler.setAndGetBack(key, array));
    for (uint8_t i = 0; i < 16; i++)
    {
        ASSERT_EQ(uint8_t(i | (i << 4)), array[i]);
    }

    // Make sure uint32 cannot be interpreted as an array
    key = "foo";
    ASSERT_FALSE(marshaler.get(key, array));

    // Nonexistent key
    key = "the_cake_is_a_lie";
    ASSERT_FALSE(marshaler.get(key, array));
}
