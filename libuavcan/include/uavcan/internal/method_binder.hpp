/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <cstdlib>
#include <stdexcept>
#include <typeinfo>
#include <uavcan/internal/impl_constants.hpp>

namespace uavcan
{

template <typename ObjectPtr, typename MemFunPtr>
class MethodBinder
{
    ObjectPtr obj_;
    MemFunPtr fun_;

public:
    MethodBinder(ObjectPtr o, MemFunPtr f)
    : obj_(o)
    , fun_(f)
    {
        if (!o || !f)
        {
#if UAVCAN_EXCEPTIONS
            throw std::runtime_error(typeid(*this).name());
#else
            assert(0);
            std::abort();
#endif
        }
    }

    void operator()() { (obj_->*fun_)(); }

    template <typename Par1>
    void operator()(Par1 p1) { (obj_->*fun_)(p1); }

    template <typename Par1, typename Par2>
    void operator()(Par1 p1, Par2 p2) { (obj_->*fun_)(p1, p2); }
};

}
