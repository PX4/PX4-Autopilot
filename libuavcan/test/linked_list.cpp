/*
 * Copyright (C) 2013 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/linked_list.hpp>

struct ListItem : uavcan::LinkedListNode<ListItem>
{
    int value;
    ListItem() : value(0) { }
};

TEST(LinkedList, Basic)
{
    uavcan::LinkedListRoot<ListItem> root;

    /*
     * Insert/remove
     */
    EXPECT_EQ(0, root.length());

    ListItem item1;
    root.insert(&item1);
    root.insert(&item1);         // Insert twice - second will be ignored
    EXPECT_EQ(1, root.length());

    EXPECT_TRUE(root.remove(&item1));
    EXPECT_FALSE(root.remove(&item1));
    EXPECT_EQ(0, root.length());

    ListItem items[3];
    root.insert(&item1);
    root.insert(items + 0);
    root.insert(items + 1);
    root.insert(items + 2);
    EXPECT_EQ(4, root.length());

    /*
     * Order persistence
     */
    items[0].value = 10;
    items[1].value = 11;
    items[2].value = 12;
    const int expected_values[] = {12, 11, 10, 0};
    ListItem* node = root.get();
    for (int i = 0; i < 4; i++)
    {
        EXPECT_EQ(expected_values[i], node->value);
        node = node->getNextListNode();
    }

    EXPECT_TRUE(root.remove(items + 0));
    EXPECT_TRUE(root.remove(items + 2));
    EXPECT_FALSE(root.remove(items + 2));
    EXPECT_EQ(2, root.length());

    const int expected_values2[] = {11, 0};
    node = root.get();
    for (int i = 0; i < 2; i++)
    {
        EXPECT_EQ(expected_values2[i], node->value);
        node = node->getNextListNode();
    }

    root.insert(items + 2);
    EXPECT_EQ(3, root.length());
    EXPECT_EQ(12, root.get()->value);

    /*
     * Emptying
     */
    EXPECT_TRUE(root.remove(&item1));
    EXPECT_FALSE(root.remove(items + 0));
    EXPECT_TRUE(root.remove(items + 1));
    EXPECT_TRUE(root.remove(items + 2));
    EXPECT_EQ(0, root.length());
}

struct Summator
{
    long sum;
    Summator() : sum(0) { }
    void operator()(const ListItem* item)
    {
        sum += item->value;
    }
};

TEST(LinkedList, Predicate)
{
    uavcan::LinkedListRoot<ListItem> root;
    ListItem items[5];
    for (int i = 0 ; i < 5; i++)
    {
        EXPECT_FALSE(root.remove(items + i));  // Just to make sure that there's no such item
        root.insert(items + i);
        items[i].value = i;
    }
    EXPECT_EQ(5, root.length());

    Summator sum;
    root.map(sum);
    EXPECT_EQ(10, sum.sum);
}
