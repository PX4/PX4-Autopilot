/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#define CH_FREQUENCY                    1000

#define CH_USE_HEAP                     TRUE
#define CH_USE_DYNAMIC                  FALSE

#if defined(DEBUG_BUILD) && DEBUG_BUILD
#   define CH_OPTIMIZE_SPEED               FALSE
#   define CH_DBG_SYSTEM_STATE_CHECK       TRUE
#   define CH_DBG_ENABLE_CHECKS            TRUE
#   define CH_DBG_ENABLE_ASSERTS           TRUE
#   define CH_DBG_ENABLE_STACK_CHECK       TRUE
#   define CH_DBG_FILL_THREADS             TRUE
#   define CH_DBG_THREADS_PROFILING        TRUE
#elif defined(RELEASE_BUILD) && RELEASE_BUILD
#   define CH_DBG_THREADS_PROFILING        FALSE
#else
#   error "Invalid configuration: Either DEBUG_BUILD or RELEASE_BUILD must be true"
#endif

#define PORT_IDLE_THREAD_STACK_SIZE    64
#define PORT_INT_REQUIRED_STACK        256

#include <zubax_chibios/sys/chconf_tail.h>
