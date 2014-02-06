/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdlib>
#include <uavcan/internal/transport/transfer_id_registry.hpp>

namespace uavcan
{

/*
 * TransferIDRegistry::List
 * TODO: faster search
 */
TransferIDRegistry::StorageEntry* TransferIDRegistry::List::find(const Key& key)
{
    StorageEntryGroup* p = list_.get();
    while (p)
    {
        for (int i = 0; i < StorageEntryGroup::NUM_ENTRIES; i++)
        {
            if (p->entries[i].node_id == key.node_id && p->entries[i].data_type_id == key.data_type_id)
                return p->entries + i;
        }
        p = p->getNextListNode();
    }
    return NULL;
}

bool TransferIDRegistry::List::create(const StorageEntry& entry, IAllocator* allocator)
{
    assert(allocator);
    StorageEntryGroup* p = list_.get();
    while (p)
    {
        for (int i = 0; i < StorageEntryGroup::NUM_ENTRIES; i++)
        {
            if (p->entries[i].isEmpty())
            {
                p->entries[i] = entry;
                return true;
            }
        }
        p = p->getNextListNode();
    }

    void* praw = allocator->allocate(sizeof(StorageEntryGroup));
    if (praw == NULL)
        return false;

    StorageEntryGroup* seg = new (praw) StorageEntryGroup();
    assert(seg);

    seg->entries[0] = entry;
    list_.insert(seg);
    return true;
}

void TransferIDRegistry::List::remove(const Key& key, IAllocator* allocator)
{
    assert(allocator);
    StorageEntryGroup* p = list_.get();
    while (p)
    {
        for (int i = 0; i < StorageEntryGroup::NUM_ENTRIES; i++)
        {
            if (p->entries[i].node_id == key.node_id && p->entries[i].data_type_id == key.data_type_id)
                p->entries[i] = StorageEntry();
        }
        p = p->getNextListNode();
    }
    compact(allocator);
}

void TransferIDRegistry::List::compact(IAllocator* allocator)
{
    // TODO: defragment
    assert(allocator);
    StorageEntryGroup* p = list_.get();
    while (p)
    {
        StorageEntryGroup* const next = p->getNextListNode();
        bool remove_this = true;
        for (int i = 0; i < StorageEntryGroup::NUM_ENTRIES; i++)
        {
            if (!p->entries[i].isEmpty())
                remove_this = false;
        }
        if (remove_this)
        {
            list_.remove(p);
            p->~StorageEntryGroup();
            allocator->deallocate(p);
        }
        p = next;
    }
}

/*
 * TransferIDRegistry
 */
TransferIDRegistry::Entry* TransferIDRegistry::access(const Key& key)
{
    if (key.node_id > NODE_ID_MAX || key.data_type_kind >= NUM_DATA_TYPE_KINDS)
    {
        assert(0);
        return NULL;
    }
    return static_cast<Entry*>(lists_by_data_type_kind_[key.data_type_kind].find(key));
}

bool TransferIDRegistry::create(const Key& key, const Entry& entry)
{
    if (key.node_id > NODE_ID_MAX || key.data_type_kind >= NUM_DATA_TYPE_KINDS)
    {
        assert(0);
        return false;
    }
    return lists_by_data_type_kind_[key.data_type_kind].create(
        StorageEntry(key.node_id, key.data_type_id, entry), allocator_);
}

void TransferIDRegistry::remove(const Key& key)
{
    if (key.node_id > NODE_ID_MAX || key.data_type_kind >= NUM_DATA_TYPE_KINDS)
    {
        assert(0);
        return;
    }
    lists_by_data_type_kind_[key.data_type_kind].remove(key, allocator_);
}

}
