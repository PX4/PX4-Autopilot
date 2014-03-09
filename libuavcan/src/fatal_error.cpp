/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <cstdlib>
#include <stdexcept>
#include <uavcan/internal/fatal_error.hpp>
#include <uavcan/internal/impl_constants.hpp>

namespace uavcan
{

void handleFatalError(const char* msg)
{
#if UAVCAN_EXCEPTIONS
    throw std::runtime_error(msg);
#else
    (void)msg;
    assert(0);
    std::abort();
#endif
}

}
