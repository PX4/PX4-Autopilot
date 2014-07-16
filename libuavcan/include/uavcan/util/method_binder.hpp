/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/error.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/util/templates.hpp>

namespace uavcan
{
/**
 * Use this to call member functions as callbacks in C++03 mode.
 *
 * In C++11 or newer you don't need it because you can use std::function<>/std::bind<> instead.
 */
template <typename ObjectPtr, typename MemFunPtr>
class UAVCAN_EXPORT MethodBinder
{
    ObjectPtr obj_;
    MemFunPtr fun_;

    void validateBeforeCall() const
    {
        if (!operator bool())
        {
            handleFatalError("Null binder");
        }
    }

public:
    MethodBinder()
        : obj_()
        , fun_()
    { }

    MethodBinder(ObjectPtr o, MemFunPtr f)
        : obj_(o)
        , fun_(f)
    { }

    /**
     * Returns true if the binder is initialized (doesn't contain null pointers).
     */
    operator bool() const
    {
        return try_implicit_cast<bool>(obj_, true) && try_implicit_cast<bool>(fun_, true);
    }

    /**
     * Will raise a fatal error if either method pointer or object pointer are null.
     */
    void operator()()
    {
        validateBeforeCall();
        (obj_->*fun_)();
    }

    /**
     * Will raise a fatal error if either method pointer or object pointer are null.
     */
    template <typename Par1>
    void operator()(Par1& p1)
    {
        validateBeforeCall();
        (obj_->*fun_)(p1);
    }

    /**
     * Will raise a fatal error if either method pointer or object pointer are null.
     */
    template <typename Par1, typename Par2>
    void operator()(Par1& p1, Par2& p2)
    {
        validateBeforeCall();
        (obj_->*fun_)(p1, p2);
    }
};

}
