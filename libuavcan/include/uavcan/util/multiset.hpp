/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_UTIL_MULTISET_HPP_INCLUDED
#define UAVCAN_UTIL_MULTISET_HPP_INCLUDED

#include <cassert>
#include <cstdlib>
#include <uavcan/util/linked_list.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/dynamic_memory.hpp>
#include <uavcan/util/templates.hpp>

namespace uavcan
{
/**
 * Slow but memory efficient unordered set.
 *
 * Items can be allocated in a static buffer or in the node's memory pool if the static buffer is exhausted.
 * When an item is deleted from the static buffer, one pair from the memory pool will be moved in the free
 * slot of the static buffer, so the use of the memory pool is minimized.
 *
 * Please be aware that this container does not perform any speed optimizations to minimize memory footprint,
 * so the complexity of most operations is O(N).
 *
 * Type requirements:
 *  T must be copyable, assignable and default constructible.
 *  T must implement a comparison operator.
 *  T's default constructor must initialize the object into invalid state.
 *  Size of T must not exceed MemPoolBlockSize.
 */
template <typename T>
class UAVCAN_EXPORT MultisetBase : Noncopyable
{
    template <typename, unsigned> friend class Multiset;

protected:
    /*
     * Purpose of this type is to enforce default initialization of T
     */
    struct Item
    {
        T value;
        Item() : value() { }
        Item(const T& v) : value(v) { }
        bool operator==(const Item& rhs) const { return rhs.value == value; }
        bool operator!=(const Item& rhs) const { return !operator==(rhs); }
        operator T() const { return value; }
    };

    struct Chunk : LinkedListNode<Chunk>
    {
        enum { NumItems = (MemPoolBlockSize - sizeof(LinkedListNode<Chunk>)) / sizeof(Item) };
        Item items[NumItems];

        Chunk()
        {
            StaticAssert<(static_cast<unsigned>(NumItems) > 0)>::check();
            IsDynamicallyAllocatable<Chunk>::check();
            UAVCAN_ASSERT(items[0].value == T());
        }

        static Chunk* instantiate(IPoolAllocator& allocator)
        {
            void* const praw = allocator.allocate(sizeof(Chunk));
            if (praw == NULL)
            {
                return NULL;
            }
            return new (praw) Chunk();
        }

        static void destroy(Chunk*& obj, IPoolAllocator& allocator)
        {
            if (obj != NULL)
            {
                obj->~Chunk();
                allocator.deallocate(obj);
                obj = NULL;
            }
        }

        Item* find(const Item& item)
        {
            for (unsigned i = 0; i < static_cast<unsigned>(NumItems); i++)
            {
                if (items[i] == item)
                {
                    return items + i;
                }
            }
            return NULL;
        }
    };

private:
    LinkedListRoot<Chunk> list_;
    IPoolAllocator& allocator_;
#if !UAVCAN_TINY
    Item* const static_;
    const unsigned num_static_entries_;
#endif

    Item* find(const Item& item);

#if !UAVCAN_TINY
    void optimizeStorage();
#endif
    void compact();

    struct YesPredicate
    {
        bool operator()(const T&) const { return true; }
    };

protected:
#if UAVCAN_TINY
    MultisetBase(IPoolAllocator& allocator)
        : allocator_(allocator)
    {
        UAVCAN_ASSERT(Item() == Item());
    }
#else
    MultisetBase(Item* static_buf, unsigned num_static_entries, IPoolAllocator& allocator)
        : allocator_(allocator)
        , static_(static_buf)
        , num_static_entries_(num_static_entries)
    {
        UAVCAN_ASSERT(Item() == Item());
    }
#endif

    /// Derived class destructor must call removeAll();
    ~MultisetBase()
    {
        UAVCAN_ASSERT(getSize() == 0);
    }

public:
    /**
     * Adds one item and returns a pointer to it.
     * If add fails due to lack of memory, NULL will be returned.
     */
    T* add(const T& item);

    /**
     * Does nothing if there's no such item.
     * Only the first matching item will be removed.
     */
    void remove(const T& item);

    /**
     * Removes entries where the predicate returns true.
     * Predicate prototype:
     *  bool (T& item)
     */
    template <typename Predicate>
    void removeWhere(Predicate predicate);

    /**
     * Returns first entry where the predicate returns true.
     * Predicate prototype:
     *  bool (const T& item)
     */
    template <typename Predicate>
    const T* findFirst(Predicate predicate) const;

    /**
     * Removes all items; all pool memory will be released.
     */
    void removeAll();

    /**
     * Returns an item located at the specified position from the beginning.
     * Note that any insertion or deletion may greatly disturb internal ordering, so use with care.
     * If index is greater than or equal the number of items, null pointer will be returned.
     */
    T* getByIndex(unsigned index);
    const T* getByIndex(unsigned index) const;

    bool isEmpty() const;

    /**
     * Counts number of items stored.
     * Best case complexity is O(N).
     */
    unsigned getSize() const;

    /**
     * For testing, do not use directly.
     */
    unsigned getNumStaticItems() const;
    unsigned getNumDynamicItems() const;
};


template <typename T, unsigned NumStaticEntries = 0>
class UAVCAN_EXPORT Multiset : public MultisetBase<T>
{
    typename MultisetBase<T>::Item static_[NumStaticEntries];

public:

#if !UAVCAN_TINY

    // This instantiation will not be valid in UAVCAN_TINY mode
    explicit Multiset(IPoolAllocator& allocator)
        : MultisetBase<T>(static_, NumStaticEntries, allocator)
    {
        UAVCAN_ASSERT(static_[0].value == T());
    }

    ~Multiset() { this->removeAll(); }

#endif // !UAVCAN_TINY
};


template <typename T>
class UAVCAN_EXPORT Multiset<T, 0> : public MultisetBase<T>
{
public:
    explicit Multiset(IPoolAllocator& allocator)
#if UAVCAN_TINY
        : MultisetBase<T>(allocator)
#else
        : MultisetBase<T>(NULL, 0, allocator)
#endif
    { }

    ~Multiset() { this->removeAll(); }
};

// ----------------------------------------------------------------------------

/*
 * MultisetBase<>
 */
template <typename T>
typename MultisetBase<T>::Item* MultisetBase<T>::find(const Item& item)
{
#if !UAVCAN_TINY
    for (unsigned i = 0; i < num_static_entries_; i++)
    {
        if (static_[i] == item)
        {
            return static_ + i;
        }
    }
#endif

    Chunk* p = list_.get();
    while (p)
    {
        Item* const dyn = p->find(item);
        if (dyn != NULL)
        {
            return dyn;
        }
        p = p->getNextListNode();
    }
    return NULL;
}

#if !UAVCAN_TINY

template <typename T>
void MultisetBase<T>::optimizeStorage()
{
    while (true)
    {
        // Looking for first EMPTY static entry
        Item* stat = NULL;
        for (unsigned i = 0; i < num_static_entries_; i++)
        {
            if (static_[i] == Item())
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
        Chunk* p = list_.get();
        Item dyn;
        UAVCAN_ASSERT(dyn == Item());
        while (p)
        {
            bool stop = false;
            for (int i = 0; i < Chunk::NumItems; i++)
            {
                if (p->items[i] != Item())   // Non empty
                {
                    dyn = p->items[i];       // Copy by value
                    p->items[i] = Item();    // Erase immediately
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
        if (dyn == Item())
        {
            break;
        }

        // Migrating
        *stat = dyn;
    }
}

#endif // !UAVCAN_TINY

template <typename T>
void MultisetBase<T>::compact()
{
    Chunk* p = list_.get();
    while (p)
    {
        Chunk* const next = p->getNextListNode();
        bool remove_this = true;
        for (int i = 0; i < Chunk::NumItems; i++)
        {
            if (p->items[i] != Item())
            {
                remove_this = false;
                break;
            }
        }
        if (remove_this)
        {
            list_.remove(p);
            Chunk::destroy(p, allocator_);
        }
        p = next;
    }
}

template <typename T>
T* MultisetBase<T>::add(const T& value)
{
    UAVCAN_ASSERT(!(value == T()));
    remove(value);

    Item* const item = find(Item());
    if (item)
    {
        *item = Item(value);
        return &item->value;
    }

    Chunk* const itemg = Chunk::instantiate(allocator_);
    if (itemg == NULL)
    {
        return NULL;
    }
    list_.insert(itemg);
    itemg->items[0] = Item(value);
    return &itemg->items[0].value;
}

template <typename T>
void MultisetBase<T>::remove(const T& value)
{
    UAVCAN_ASSERT(!(value == T()));
    Item* const item = find(Item(value));
    if (item != NULL)
    {
        *item = Item();
#if !UAVCAN_TINY
        optimizeStorage();
#endif
        compact();
    }
}

template <typename T>
template <typename Predicate>
void MultisetBase<T>::removeWhere(Predicate predicate)
{
    unsigned num_removed = 0;

#if !UAVCAN_TINY
    for (unsigned i = 0; i < num_static_entries_; i++)
    {
        if (static_[i] != Item())
        {
            if (predicate(static_[i].value))
            {
                num_removed++;
                static_[i] = Item();
            }
        }
    }
#endif

    Chunk* p = list_.get();
    while (p)
    {
        for (int i = 0; i < Chunk::NumItems; i++)
        {
            const Item* const item = p->items + i;
            if ((*item) != Item())
            {
                if (predicate(item->value))
                {
                    num_removed++;
                    p->items[i] = Item();
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

template <typename T>
template <typename Predicate>
const T* MultisetBase<T>::findFirst(Predicate predicate) const
{
#if !UAVCAN_TINY
    for (unsigned i = 0; i < num_static_entries_; i++)
    {
        if (static_[i] != Item())
        {
            if (predicate(static_[i].value))
            {
                return &static_[i].value;
            }
        }
    }
#endif

    Chunk* p = list_.get();
    while (p)
    {
        for (int i = 0; i < Chunk::NumItems; i++)
        {
            const Item* const item = p->items + i;
            if ((*item) != Item())
            {
                if (predicate(item->value))
                {
                    return &p->items[i].value;
                }
            }
        }
        p = p->getNextListNode();
    }
    return NULL;
}

template <typename T>
void MultisetBase<T>::removeAll()
{
    removeWhere(YesPredicate());
}

template <typename T>
T* MultisetBase<T>::getByIndex(unsigned index)
{
#if !UAVCAN_TINY
    // Checking the static storage
    for (unsigned i = 0; i < num_static_entries_; i++)
    {
        if (static_[i] != Item())
        {
            if (index == 0)
            {
                return &static_[i].value;
            }
            index--;
        }
    }
#endif

    // Slowly crawling through the dynamic storage
    Chunk* p = list_.get();
    while (p)
    {
        for (int i = 0; i < Chunk::NumItems; i++)
        {
            Item* const item = p->items + i;
            if ((*item) != Item())
            {
                if (index == 0)
                {
                    return &item->value;
                }
                index--;
            }
        }
        p = p->getNextListNode();
    }

    return NULL;
}

template <typename T>
const T* MultisetBase<T>::getByIndex(unsigned index) const
{
    return const_cast<MultisetBase<T>*>(this)->getByIndex(index);
}

template <typename T>
bool MultisetBase<T>::isEmpty() const
{
    return getSize() == 0;
}

template <typename T>
unsigned MultisetBase<T>::getSize() const
{
    return getNumStaticItems() + getNumDynamicItems();
}

template <typename T>
unsigned MultisetBase<T>::getNumStaticItems() const
{
    unsigned num = 0;
#if !UAVCAN_TINY
    for (unsigned i = 0; i < num_static_entries_; i++)
    {
        if (static_[i] != Item())
        {
            num++;
        }
    }
#endif
    return num;
}

template <typename T>
unsigned MultisetBase<T>::getNumDynamicItems() const
{
    unsigned num = 0;
    Chunk* p = list_.get();
    while (p)
    {
        for (int i = 0; i < Chunk::NumItems; i++)
        {
            const Item* const item = p->items + i;
            if ((*item) != Item())
            {
                num++;
            }
        }
        p = p->getNextListNode();
    }
    return num;
}

}

#endif // Include guard
