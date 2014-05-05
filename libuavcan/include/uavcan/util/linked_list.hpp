/*
 * Singly-linked list.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cstdlib>
#include <cassert>
#include <uavcan/impl_constants.hpp>

namespace uavcan
{
/**
 * Classes that are supposed to be linked-listed should derive this.
 */
template <typename T>
class UAVCAN_EXPORT LinkedListNode
{
    T* next_;

protected:
    LinkedListNode()
        : next_(NULL)
    { }

    ~LinkedListNode() { }

public:
    T* getNextListNode() const { return next_; }

    void setNextListNode(T* node)
    {
        next_ = node;
    }
};

/**
 * Linked list root.
 */
template <typename T>
class UAVCAN_EXPORT LinkedListRoot
{
    T* root_;

public:
    LinkedListRoot()
        : root_(NULL)
    { }

    T* get() const { return root_; }
    bool isEmpty() const { return get() == NULL; }

    unsigned getLength() const;

    void insert(T* node);

    /**
     * Inserts node A immediately before the node B where
     *  predicate(B) -> true.
     */
    template <typename Predicate>
    void insertBefore(T* node, Predicate predicate);

    void remove(const T* node);
};

// ----------------------------------------------------------------------------

/*
 * LinkedListRoot<>
 */
template <typename T>
unsigned LinkedListRoot<T>::getLength() const
{
    T* node = root_;
    unsigned cnt = 0;
    while (node)
    {
        cnt++;
        node = node->getNextListNode();
    }
    return cnt;
}

template <typename T>
void LinkedListRoot<T>::insert(T* node)
{
    assert(node);
    remove(node);  // Making sure there will be no loops
    node->setNextListNode(root_);
    root_ = node;
}

template <typename T>
template <typename Predicate>
void LinkedListRoot<T>::insertBefore(T* node, Predicate predicate)
{
    assert(node);
    remove(node);

    if (root_ == NULL || predicate(root_))
    {
        insert(node);
    }
    else
    {
        T* p = root_;
        while (p->getNextListNode())
        {
            if (predicate(p->getNextListNode()))
            {
                break;
            }
            p = p->getNextListNode();
        }
        node->setNextListNode(p->getNextListNode());
        p->setNextListNode(node);
    }
}

template <typename T>
void LinkedListRoot<T>::remove(const T* node)
{
    if (root_ == NULL || node == NULL)
    {
        return;
    }

    if (root_ == node)
    {
        root_ = root_->getNextListNode();
    }
    else
    {
        T* p = root_;
        while (p->getNextListNode())
        {
            if (p->getNextListNode() == node)
            {
                p->setNextListNode(p->getNextListNode()->getNextListNode());
                break;
            }
            p = p->getNextListNode();
        }
    }
}

}
