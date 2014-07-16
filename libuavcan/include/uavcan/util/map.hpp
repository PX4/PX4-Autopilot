/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <cstdlib>
#include <uavcan/util/linked_list.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/dynamic_memory.hpp>
#include <uavcan/util/templates.hpp>

namespace uavcan
{
/**
 * Slow but memory efficient KV container.
 *
 * KV pairs can be allocated in a static buffer or in the node's memory pool if the static buffer is exhausted.
 * When a KV pair is deleted from the static buffer, one pair from the memory pool will be moved in the free
 * slot of the static buffer, so the use of the memory pool is minimized.
 *
 * Please be aware that this container does not perform any speed optimizations to minimize memory footprint,
 * so the complexity of most operations is O(N).
 *
 * Type requirements:
 *  Both key and value must be copyable, assignable and default constructible.
 *  Key must implement a comparison operator.
 *  Key's default constructor must initialize the object into invalid state.
 *  Size of Key + Value + padding must not exceed MemPoolBlockSize.
 */
template <typename Key, typename Value>
class UAVCAN_EXPORT MapBase : Noncopyable
{
    template <typename, typename, unsigned> friend class Map;

    UAVCAN_PACKED_BEGIN
    struct KVPair
    {
        Key key;
        Value value;
        KVPair() { }
        KVPair(const Key& arg_key, const Value& arg_value)
            : key(arg_key)
            , value(arg_value)
        { }
        bool match(const Key& rhs) const { return rhs == key; }
    };

    struct KVGroup : LinkedListNode<KVGroup>
    {
        enum { NumKV = (MemPoolBlockSize - sizeof(LinkedListNode<KVGroup>)) / sizeof(KVPair) };
        KVPair kvs[NumKV];

        KVGroup()
        {
            StaticAssert<(static_cast<unsigned>(NumKV) > 0)>::check();
            IsDynamicallyAllocatable<KVGroup>::check();
        }

        static KVGroup* instantiate(IPoolAllocator& allocator)
        {
            void* const praw = allocator.allocate(sizeof(KVGroup));
            if (praw == NULL)
            {
                return NULL;
            }
            return new (praw) KVGroup();
        }

        static void destroy(KVGroup*& obj, IPoolAllocator& allocator)
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
            for (unsigned i = 0; i < static_cast<unsigned>(NumKV); i++)
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
    IPoolAllocator& allocator_;
#if !UAVCAN_TINY
    KVPair* const static_;
    const unsigned num_static_entries_;
#endif

    KVPair* find(const Key& key);

#if !UAVCAN_TINY
    void optimizeStorage();
#endif
    void compact();

    struct YesPredicate
    {
        bool operator()(const Key& k, const Value& v) const { (void)k; (void)v; return true; }
    };

protected:
#if UAVCAN_TINY
    MapBase(IPoolAllocator& allocator)
        : allocator_(allocator)
    {
        UAVCAN_ASSERT(Key() == Key());
    }
#else
    MapBase(KVPair* static_buf, unsigned num_static_entries, IPoolAllocator& allocator)
        : allocator_(allocator)
        , static_(static_buf)
        , num_static_entries_(num_static_entries)
    {
        UAVCAN_ASSERT(Key() == Key());
    }
#endif

    /// Derived class destructor must call removeAll();
    ~MapBase() { }

public:
    /**
     * Returns null pointer if there's no such entry.
     */
    Value* access(const Key& key);

    /**
     * If entry with the same key already exists, it will be replaced
     */
    Value* insert(const Key& key, const Value& value);

    /**
     * Does nothing if there's no such entry.
     */
    void remove(const Key& key);

    /**
     * Removes entries where the predicate returns true.
     * Predicate prototype:
     *  bool (const Key& key, const Value& value)
     */
    template <typename Predicate>
    void removeWhere(Predicate predicate);

    /**
     * Returns first entry where the predicate returns true.
     * Predicate prototype:
     *  bool (const Key& key, const Value& value)
     */
    template <typename Predicate>
    const Key* findFirstKey(Predicate predicate) const;

    void removeAll();

    bool isEmpty() const;

    /// For testing
    unsigned getNumStaticPairs() const;

    /// For testing
    unsigned getNumDynamicPairs() const;
};


template <typename Key, typename Value, unsigned NumStaticEntries = 0>
class UAVCAN_EXPORT Map : public MapBase<Key, Value>
{
    typename MapBase<Key, Value>::KVPair static_[NumStaticEntries];

public:

#if !UAVCAN_TINY

    // This instantiation will not be valid in UAVCAN_TINY mode
    explicit Map(IPoolAllocator& allocator)
        : MapBase<Key, Value>(static_, NumStaticEntries, allocator)
    { }

    ~Map() { this->removeAll(); }

#endif // !UAVCAN_TINY
};


template <typename Key, typename Value>
class UAVCAN_EXPORT Map<Key, Value, 0> : public MapBase<Key, Value>
{
public:
    explicit Map(IPoolAllocator& allocator)
#if UAVCAN_TINY
        : MapBase<Key, Value>(allocator)
#else
        : MapBase<Key, Value>(NULL, 0, allocator)
#endif
    { }

    ~Map() { this->removeAll(); }
};

// ----------------------------------------------------------------------------

/*
 * MapBase<>
 */
template <typename Key, typename Value>
typename MapBase<Key, Value>::KVPair* MapBase<Key, Value>::find(const Key& key)
{
#if !UAVCAN_TINY
    for (unsigned i = 0; i < num_static_entries_; i++)
    {
        if (static_[i].match(key))
        {
            return static_ + i;
        }
    }
#endif

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

#if !UAVCAN_TINY

template <typename Key, typename Value>
void MapBase<Key, Value>::optimizeStorage()
{
    while (true)
    {
        // Looking for first EMPTY static entry
        KVPair* stat = NULL;
        for (unsigned i = 0; i < num_static_entries_; i++)
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

#endif // !UAVCAN_TINY

template <typename Key, typename Value>
void MapBase<Key, Value>::compact()
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

template <typename Key, typename Value>
Value* MapBase<Key, Value>::access(const Key& key)
{
    UAVCAN_ASSERT(!(key == Key()));
    KVPair* const kv = find(key);
    return kv ? &kv->value : NULL;
}

template <typename Key, typename Value>
Value* MapBase<Key, Value>::insert(const Key& key, const Value& value)
{
    UAVCAN_ASSERT(!(key == Key()));
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

template <typename Key, typename Value>
void MapBase<Key, Value>::remove(const Key& key)
{
    UAVCAN_ASSERT(!(key == Key()));
    KVPair* const kv = find(key);
    if (kv)
    {
        *kv = KVPair();
#if !UAVCAN_TINY
        optimizeStorage();
#endif
        compact();
    }
}

template <typename Key, typename Value>
template <typename Predicate>
void MapBase<Key, Value>::removeWhere(Predicate predicate)
{
    unsigned num_removed = 0;

#if !UAVCAN_TINY
    for (unsigned i = 0; i < num_static_entries_; i++)
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
#endif

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
#if !UAVCAN_TINY
        optimizeStorage();
#endif
        compact();
    }
}

template <typename Key, typename Value>
template <typename Predicate>
const Key* MapBase<Key, Value>::findFirstKey(Predicate predicate) const
{
#if !UAVCAN_TINY
    for (unsigned i = 0; i < num_static_entries_; i++)
    {
        if (!static_[i].match(Key()))
        {
            if (predicate(static_[i].key, static_[i].value))
            {
                return &static_[i].key;
            }
        }
    }
#endif

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

template <typename Key, typename Value>
void MapBase<Key, Value>::removeAll()
{
    removeWhere(YesPredicate());
}

template <typename Key, typename Value>
bool MapBase<Key, Value>::isEmpty() const
{
    return (getNumStaticPairs() == 0) && (getNumDynamicPairs() == 0);
}

template <typename Key, typename Value>
unsigned MapBase<Key, Value>::getNumStaticPairs() const
{
    unsigned num = 0;
#if !UAVCAN_TINY
    for (unsigned i = 0; i < num_static_entries_; i++)
    {
        if (!static_[i].match(Key()))
        {
            num++;
        }
    }
#endif
    return num;
}

template <typename Key, typename Value>
unsigned MapBase<Key, Value>::getNumDynamicPairs() const
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

}
