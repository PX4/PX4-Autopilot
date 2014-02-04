/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <stdint.h>
#include <uavcan/internal/transport/transfer.hpp>
#include <uavcan/internal/linked_list.hpp>
#include <uavcan/internal/impl_constants.hpp>
#include <uavcan/internal/dynamic_memory.hpp>

namespace uavcan
{

class TransferIDRegistry
{
public:
    struct Key
    {
        TransferType transfer_type;
        uint16_t data_type_id;
        uint8_t node_id;

        Key()
        : transfer_type(TransferType(0))
        , data_type_id(0)
        , node_id(NODE_ID_INVALID)
        { }

        Key(uint8_t node_id, TransferType transfer_type, uint16_t data_type_id)
        : transfer_type(transfer_type)
        , data_type_id(data_type_id)
        , node_id(node_id)
        { }
    };

#pragma pack(push, 1)
    struct Entry
    {
        uint64_t timestamp;
        TransferID transfer_id;

        Entry()
        : timestamp(0)
        { }

        Entry(TransferID transfer_id, uint64_t timestamp)
        : timestamp(timestamp)
        , transfer_id(transfer_id)
        { }

        bool operator==(const Entry& rhs) const
        {
            return (timestamp == rhs.timestamp) && (transfer_id == rhs.transfer_id);
        }
    };

private:
    struct StorageEntry : public Entry
    {
        uint16_t data_type_id;
        uint8_t node_id;

        StorageEntry()
        : data_type_id(0)
        , node_id(NODE_ID_INVALID)
        { }

        StorageEntry(uint8_t node_id, uint16_t data_type_id, const Entry& entry)
        : Entry(entry)
        , data_type_id(data_type_id)
        , node_id(node_id)
        { }

        bool isEmpty() const { return node_id == NODE_ID_INVALID; }
    };

    struct StorageEntryGroup : LinkedListNode<StorageEntryGroup>
    {
        enum
        {
            NUM_ENTRIES = (MEM_POOL_BLOCK_SIZE - sizeof(LinkedListNode<StorageEntryGroup>)) / sizeof(StorageEntry)
        };
        StorageEntry entries[NUM_ENTRIES];

        StorageEntryGroup()
        {
            IsDynamicallyAllocatable<StorageEntryGroup>::check();
            StaticAssert<NUM_ENTRIES >= 2>::check();
        }
    };
#pragma pack(pop)

    class List
    {
        LinkedListRoot<StorageEntryGroup> list_;

    public:
        StorageEntryGroup* getHead() const { return list_.get(); }
        StorageEntry* find(const Key& key);
        bool create(const StorageEntry& entry, IAllocator* allocator);
        void remove(const Key& key, IAllocator* allocator);
        void compact(IAllocator* allocator);
    };

    List lists_by_transfer_type_[NUM_TRANSFER_TYPES];
    IAllocator* const allocator_;

public:
    TransferIDRegistry(IAllocator* allocator)
    : allocator_(allocator)
    {
        assert(allocator);
    }

    Entry* access(const Key& key);

    bool create(const Key& key, const Entry& entry);
    void remove(const Key& key);

    /**
     * Removes entries where predicate returns true.
     * Predicate prototype:
     *  bool (const TransferIDRegistry::Key& key, const TransferIDRegistry::Entry& entry)
     */
    template <typename Predicate>
    void removeWhere(Predicate predicate)
    {
        for (int transfer_type = 0; transfer_type < NUM_TRANSFER_TYPES; transfer_type++)
        {
            StorageEntryGroup* p = lists_by_transfer_type_[transfer_type].getHead();
            while (p)
            {
                for (int i = 0; i < StorageEntryGroup::NUM_ENTRIES; i++)
                {
                    const StorageEntry* const entry = p->entries + i;
                    if (!entry->isEmpty())
                    {
                        const Key key(entry->node_id, TransferType(transfer_type), entry->data_type_id);
                        const bool result = predicate(key, static_cast<const Entry&>(*entry));
                        if (result)
                            p->entries[i] = StorageEntry();
                    }
                }
                p = p->getNextListNode();
            }
            lists_by_transfer_type_[transfer_type].compact(allocator_);
        }
    }
};

}
