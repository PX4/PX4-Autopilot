/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

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
        __asm volatile ("cpsid  i");
    }
    ~CriticalSectionLocker()
    {
        __asm volatile ("cpsie  i");
    }
};

}
