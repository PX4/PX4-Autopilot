/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cstdint>
#include <chip.h>

/*
 * Compiler version check
 */
#ifdef __GNUC__
# if (__GNUC__ * 10 + __GNUC_MINOR__) < 49
#  error "Use GCC 4.9 or newer"
# endif
#endif


namespace uavcan_lpc11c24
{

/**
 * Locks UAVCAN driver interrupts.
 * TODO: priority.
 */
struct CriticalSectionLocker
{
    CriticalSectionLocker()
    {
        __disable_irq();
    }
    ~CriticalSectionLocker()
    {
        __enable_irq();
    }
};

/**
 * Internal for the driver
 */
namespace clock
{

std::uint64_t getUtcUSecFromCanInterrupt();

}

}
