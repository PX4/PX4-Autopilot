/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <stdint.h>
#include <uavcan/internal/linked_list.hpp>
#include <uavcan/internal/impl_constants.hpp>
#include <uavcan/internal/dynamic_memory.hpp>
#include <uavcan/internal/util.hpp>

namespace uavcan
{
/**
 * Slow but memory efficient KV container.
 * Type requirements:
 *  Both key and value must be copyable and default constructible.
 *  Key must implement a comparison operator.
 *  Key's default constructor must initialize the object into invalid state.
 *  Size of Key + Value + padding must not exceed MEM_POOL_BLOCK_SIZE.
 */
template <typename Key, typename Value, unsigned int NUM_STATIC_ENTRIES>
class Map : Noncopyable
{
#pragma pack(push, 1)
    struct KVPair
    {
        Key key;
        Value value;
        KVPair() { }
        KVPair(const Key& key, const Value& value) : key(key), value(value) { }
        bool match(const Key& rhs) const { return rhs == key; }
    };

    struct KVGroup : LinkedListNode<KVGroup>
    {
        enum { NUM_KV = (MEM_POOL_BLOCK_SIZE - sizeof(LinkedListNode<KVGroup>)) / sizeof(KVPair) };
        KVPair kvs[NUM_KV];

        KVGroup()
        {
            StaticAssert<(NUM_KV > 0)>::check();
            IsDynamicallyAllocatable<KVGroup>::check();
        }

        KVPair* find(const Key& key)
        {
            for (int i = 0; i < NUM_KV; i++)
                if (kvs[i].match(key))
                    return kvs + i;
            return NULL;
        }
    };
#pragma pack(pop)

    LinkedListRoot<KVGroup> list_;
    IAllocator* const allocator_;
    KVPair static_[NUM_STATIC_ENTRIES];

    KVPair* find(const Key& key)
    {
        for (unsigned int i = 0; i < NUM_STATIC_ENTRIES; i++)
            if (static_[i].match(key))
                return static_ + i;

        KVGroup* p = list_.get();
        while (p)
        {
            KVPair* const kv = p->find(key);
            if (kv)
                return kv;
            p = p->getNextListNode();
        }
        return NULL;
    }

    void optimizeStorage()
    {
        while (true)
        {
            // Looking for first EMPTY static entry
            KVPair* stat = NULL;
            for (unsigned int i = 0; i < NUM_STATIC_ENTRIES; i++)
            {
                if (static_[i].match(Key()))
                {
                    stat = static_ + i;
                    break;
                }
            }
            if (stat == NULL)
                break;

            // Looking for the first NON-EMPTY dynamic entry, erasing immediately
            KVGroup* p = list_.get();
            KVPair dyn;
            while (p)
            {
                bool stop = false;
                for (int i = 0; i < KVGroup::NUM_KV; i++)
                {
                    if (!p->kvs[i].match(Key())) // Non empty
                    {
                        dyn = p->kvs[i];         // Copy by value
                        p->kvs[i] = KVPair();    // Erase immediately
                        stop = true;
                        break;
                    }
                }
                if (stop)
                    break;
                p = p->getNextListNode();
            }
            if (dyn.match(Key()))
                break;

            // Migrating
            *stat = dyn;
        }
    }

    void compact()
    {
        KVGroup* p = list_.get();
        while (p)
        {
            KVGroup* const next = p->getNextListNode();
            bool remove_this = true;
            for (int i = 0; i < KVGroup::NUM_KV; i++)
            {
                if (!p->kvs[i].match(Key()))
                {
                    remove_this = false;
                    break;
                }
            }
            if (remove_this)
            {
                list_.remove(p);
                p->~KVGroup();
                allocator_->deallocate(p);
            }
            p = next;
        }
    }

    struct YesPredicate
    {
        bool operator()(const Key& k, const Value& v) const { (void)k; (void)v; return true; }
    };

    // This container is not copyable
    Map(const Map&);
    bool operator=(const Map&);

public:
    Map(IAllocator* allocator)
    : allocator_(allocator)
    {
        assert(allocator);
        assert(Key() == Key());
    }

    ~Map() { removeAll(); }

    Value* access(const Key& key)
    {
        assert(!(key == Key()));
        KVPair* const kv = find(key);
        return kv ? &kv->value : NULL;
    }

    /// If entry with the same key already exists, it will be replaced
    bool insert(const Key& key, const Value& value)
    {
        assert(!(key == Key()));
        remove(key);

        KVPair* const kv = find(Key());
        if (kv)
        {
            *kv = KVPair(key, value);
            return true;
        }

        void* const praw = allocator_->allocate(sizeof(KVGroup));
        if (praw == NULL)
            return false;

        KVGroup* const kvg = new (praw) KVGroup();
        assert(kvg);
        kvg->kvs[0] = KVPair(key, value);
        list_.insert(kvg);
        return true;
    }

    void remove(const Key& key)
    {
        assert(!(key == Key()));
        KVPair* const kv = find(key);
        if (kv)
        {
            *kv = KVPair();
            optimizeStorage();
            compact();
        }
    }

    /**
     * Remove entries where predicate returns true.
     * Predicate prototype:
     *  bool (const Key& key, const Value& value)
     */
    template <typename Predicate>
    void removeWhere(Predicate predicate)
    {
        unsigned int num_removed = 0;

        for (unsigned int i = 0; i < NUM_STATIC_ENTRIES; i++)
        {
            if (!static_[i].match(Key()))
            {
                if (predicate(static_[i].key, static_[i].value))
                {
                    num_removed++;
                    static_[i] = KVPair();
                }
            }
        }

        KVGroup* p = list_.get();
        while (p)
        {
            for (int i = 0; i < KVGroup::NUM_KV; i++)
            {
                const KVPair* const kv = p->kvs + i;
                if (!kv->match(Key()))
                {
                    if (predicate(kv->key, kv->value))
                    {
                        num_removed++;
                        p->kvs[i] = KVPair();
                    }
                }
            }
            p = p->getNextListNode();
        }

        if (num_removed > 0)
        {
            optimizeStorage();
            compact();
        }
    }

    void removeAll()
    {
        removeWhere(YesPredicate());
    }

    /// For testing
    unsigned int getNumStaticPairs() const
    {
        unsigned int num = 0;
        for (unsigned int i = 0; i < NUM_STATIC_ENTRIES; i++)
            if (!static_[i].match(Key()))
                num++;
        return num;
    }

    /// For testing
    unsigned int getNumDynamicPairs() const
    {
        unsigned int num = 0;
        KVGroup* p = list_.get();
        while (p)
        {
            for (int i = 0; i < KVGroup::NUM_KV; i++)
            {
                const KVPair* const kv = p->kvs + i;
                if (!kv->match(Key()))
                    num++;
            }
            p = p->getNextListNode();
        }
        return num;
    }
};

}
