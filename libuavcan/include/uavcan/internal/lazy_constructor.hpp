/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cstdlib>
#include <cassert>
#include <stdexcept>

namespace uavcan
{

template <typename T>
class LazyConstructor
{
    unsigned char data_[sizeof(T)] __attribute__((aligned(16)));  // TODO: compiler-independent alignment
    T* ptr_;

    void failure() const
    {
#if UAVCAN_EXCEPTIONS
        throw std::logic_error(typeid(*this).name());
#else
        assert(0);
        std::abort();
#endif
    }

    void ensureConstructed() const
    {
        if (!ptr_)
            failure();
    }

    void ensureNotConstructed() const
    {
        if (ptr_)
            failure();
    }

public:
    LazyConstructor()
    : ptr_(NULL)
    {
        std::fill(data_, data_ + sizeof(T), 0);
    }

    LazyConstructor(const LazyConstructor<T>& rhs)
    : ptr_(NULL)
    {
        std::fill(data_, data_ + sizeof(T), 0);
        if (rhs)
            construct(*rhs);  // Invoke copy constructor
    }

    ~LazyConstructor() { destroy(); }

    LazyConstructor<T>& operator=(const LazyConstructor<T>& rhs)
    {
        destroy();
        if (rhs)
            construct(*rhs);  // Invoke copy constructor
        return *this;
    }

    bool isConstructed() const { return ptr_ != NULL; }

    operator T*() const { return ptr_; }

    const T* operator->() const { ensureConstructed(); return ptr_; }
    T*       operator->()       { ensureConstructed(); return ptr_; }

    const T& operator*() const { ensureConstructed(); return *ptr_; }
    T&       operator*()       { ensureConstructed(); return *ptr_; }

    void destroy()
    {
        if (ptr_)
            ptr_->~T();
        ptr_ = NULL;
    }

    void construct()
    {
        ensureNotConstructed();
        ptr_ = new (static_cast<void*>(data_)) T();
    }

// MAX_ARGS = 6
// TEMPLATE = '''
//     template <%s>
//     void construct(%s)
//     {
//         ensureNotConstructed();
//         ptr_ = new (static_cast<void*>(data_)) T(%s);
//     }'''
// for nargs in range(1, MAX_ARGS + 1):
//     nums = [(x + 1) for x in range(nargs)]
//     l1 = ['typename P%d' % x for x in nums]
//     l2 = ['P%d p%d' % (x, x) for x in nums]
//     l3 = ['p%d' % x for x in nums]
//     print(TEMPLATE % (', '.join(l1), ', '.join(l2), ', '.join(l3)))

    template <typename P1>
    void construct(P1 p1)
    {
        ensureNotConstructed();
        ptr_ = new (static_cast<void*>(data_)) T(p1);
    }

    template <typename P1, typename P2>
    void construct(P1 p1, P2 p2)
    {
        ensureNotConstructed();
        ptr_ = new (static_cast<void*>(data_)) T(p1, p2);
    }

    template <typename P1, typename P2, typename P3>
    void construct(P1 p1, P2 p2, P3 p3)
    {
        ensureNotConstructed();
        ptr_ = new (static_cast<void*>(data_)) T(p1, p2, p3);
    }

    template <typename P1, typename P2, typename P3, typename P4>
    void construct(P1 p1, P2 p2, P3 p3, P4 p4)
    {
        ensureNotConstructed();
        ptr_ = new (static_cast<void*>(data_)) T(p1, p2, p3, p4);
    }

    template <typename P1, typename P2, typename P3, typename P4, typename P5>
    void construct(P1 p1, P2 p2, P3 p3, P4 p4, P5 p5)
    {
        ensureNotConstructed();
        ptr_ = new (static_cast<void*>(data_)) T(p1, p2, p3, p4, p5);
    }

    template <typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
    void construct(P1 p1, P2 p2, P3 p3, P4 p4, P5 p5, P6 p6)
    {
        ensureNotConstructed();
        ptr_ = new (static_cast<void*>(data_)) T(p1, p2, p3, p4, p5, p6);
    }
};

}
