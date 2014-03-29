/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <cstdlib>
#include <uavcan/linked_list.hpp>
#include <uavcan/impl_constants.hpp>
#include <uavcan/dynamic_memory.hpp>
#include <uavcan/util/compile_time.hpp>

namespace uavcan
{
/**
 * Slow but memory efficient KV container.
 * Type requirements:
 *  Both key and value must be copyable, assignable and default constructible.
 *  Key must implement a comparison operator.
 *  Key's default constructor must initialize the object into invalid state.
 *  Size of Key + Value + padding must not exceed MemPoolBlockSize.
 */
template <typename Key, typename Value, unsigned NumStaticEntries>
class Map : Noncopyable
{
    UAVCAN_PACKED_BEGIN
    struct KVPair
    {
        Key key;
        Value value;
        KVPair() { }
        KVPair(const Key& key, const Value& value)
            : key(key)
            , value(value)
        { }
        bool match(const Key& rhs) const { return rhs == key; }
    };

    struct KVGroup : LinkedListNode<KVGroup>
    {
        enum { NumKV = (MemPoolBlockSize - sizeof(LinkedListNode<KVGroup>)) / sizeof(KVPair) };
        KVPair kvs[NumKV];

        KVGroup()
        {
            StaticAssert<(NumKV > 0)>::check();
            IsDynamicallyAllocatable<KVGroup>::check();
        }

        static KVGroup* instantiate(IAllocator& allocator)
        {
            void* const praw = allocator.allocate(sizeof(KVGroup));
            if (praw == NULL)
            {
                return NULL;
            }
            return new (praw) KVGroup();
        }

        static void destroy(KVGroup*& obj, IAllocator& allocator)
        {
            if (obj != NULL)
            {
                obj->~KVGroup();
                allocator.deallocate(obj);
                obj = NULL;
            }
        }

        KVPair* find(const Key& key)
        {
            for (int i = 0; i < NumKV; i++)
            {
                if (kvs[i].match(key))
                {
                    return kvs + i;
                }
            }
            return NULL;
        }
    };
    UAVCAN_PACKED_END

    LinkedListRoot<KVGroup> list_;
    IAllocator& allocator_;
    KVPair static_[NumStaticEntries];

    KVPair* find(const Key& key)
    {
        for (unsigned i = 0; i < NumStaticEntries; i++)
        {
            if (static_[i].match(key))
            {
                return static_ + i;
            }
        }

        KVGroup* p = list_.get();
        while (p)
        {
            KVPair* const kv = p->find(key);
            if (kv)
            {
                return kv;
            }
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
            for (unsigned i = 0; i < NumStaticEntries; i++)
            {
                if (static_[i].match(Key()))
                {
                    stat = static_ + i;
                    break;
                }
            }
            if (stat == NULL)
            {
                break;
            }

            // Looking for the first NON-EMPTY dynamic entry, erasing immediately
            KVGroup* p = list_.get();
            KVPair dyn;
            while (p)
            {
                bool stop = false;
                for (int i = 0; i < KVGroup::NumKV; i++)
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
                {
                    break;
                }
                p = p->getNextListNode();
            }
            if (dyn.match(Key()))
            {
                break;
            }

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
            for (int i = 0; i < KVGroup::NumKV; i++)
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
                KVGroup::destroy(p, allocator_);
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
    Map(IAllocator& allocator)
        : allocator_(allocator)
    {
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
    Value* insert(const Key& key, const Value& value)
    {
        assert(!(key == Key()));
        remove(key);

        KVPair* const kv = find(Key());
        if (kv)
        {
            *kv = KVPair(key, value);
            return &kv->value;
        }

        KVGroup* const kvg = KVGroup::instantiate(allocator_);
        if (kvg == NULL)
        {
            return NULL;
        }
        list_.insert(kvg);
        kvg->kvs[0] = KVPair(key, value);
        return &kvg->kvs[0].value;
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
        unsigned num_removed = 0;

        for (unsigned i = 0; i < NumStaticEntries; i++)
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
            for (int i = 0; i < KVGroup::NumKV; i++)
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

    template <typename Predicate>
    const Key* findFirstKey(Predicate predicate) const
    {
        for (unsigned i = 0; i < NumStaticEntries; i++)
        {
            if (!static_[i].match(Key()))
            {
                if (predicate(static_[i].key, static_[i].value))
                {
                    return &static_[i].key;
                }
            }
        }

        KVGroup* p = list_.get();
        while (p)
        {
            for (int i = 0; i < KVGroup::NumKV; i++)
            {
                const KVPair* const kv = p->kvs + i;
                if (!kv->match(Key()))
                {
                    if (predicate(kv->key, kv->value))
                    {
                        return &p->kvs[i].key;
                    }
                }
            }
            p = p->getNextListNode();
        }
        return NULL;
    }

    void removeAll()
    {
        removeWhere(YesPredicate());
    }

    bool isEmpty() const { return (getNumStaticPairs() == 0) && (getNumDynamicPairs() == 0); }

    /// For testing
    unsigned getNumStaticPairs() const
    {
        unsigned num = 0;
        for (unsigned i = 0; i < NumStaticEntries; i++)
        {
            if (!static_[i].match(Key()))
            {
                num++;
            }
        }
        return num;
    }

    /// For testing
    unsigned getNumDynamicPairs() const
    {
        unsigned num = 0;
        KVGroup* p = list_.get();
        while (p)
        {
            for (int i = 0; i < KVGroup::NumKV; i++)
            {
                const KVPair* const kv = p->kvs + i;
                if (!kv->match(Key()))
                {
                    num++;
                }
            }
            p = p->getNextListNode();
        }
        return num;
    }
};

}
