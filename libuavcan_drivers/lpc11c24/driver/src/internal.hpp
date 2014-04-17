/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <chip.h>

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

uint64_t getUtcUSecFromCanInterrupt();

}

}
