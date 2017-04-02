/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan_stm32/build_config.hpp>

#if UAVCAN_STM32_CHIBIOS
# include <hal.h>
#elif UAVCAN_STM32_NUTTX
# include <nuttx/arch.h>
# include <arch/board/board.h>
# include <chip/stm32_tim.h>
# include <syslog.h>
#elif UAVCAN_STM32_BAREMETAL
#include <chip.h>	// See http://uavcan.org/Implementations/Libuavcan/Platforms/STM32/
#elif UAVCAN_STM32_FREERTOS
# include <chip.h>
# include <cmsis_os.h>
#else
# error "Unknown OS"
#endif

/**
 * Debug output
 */
#ifndef UAVCAN_STM32_LOG
// syslog() crashes the system in this context
// # if UAVCAN_STM32_NUTTX && CONFIG_ARCH_LOWPUTC
# if 0
#  define UAVCAN_STM32_LOG(fmt, ...)  syslog("uavcan_stm32: " fmt "\n", ##__VA_ARGS__)
# else
#  define UAVCAN_STM32_LOG(...)       ((void)0)
# endif
#endif

/**
 * IRQ handler macros
 */
#if UAVCAN_STM32_CHIBIOS
# define UAVCAN_STM32_IRQ_HANDLER(id)  CH_IRQ_HANDLER(id)
# define UAVCAN_STM32_IRQ_PROLOGUE()    CH_IRQ_PROLOGUE()
# define UAVCAN_STM32_IRQ_EPILOGUE()    CH_IRQ_EPILOGUE()
#elif UAVCAN_STM32_NUTTX
# define UAVCAN_STM32_IRQ_HANDLER(id)  int id(int irq, FAR void* context)
# define UAVCAN_STM32_IRQ_PROLOGUE()
# define UAVCAN_STM32_IRQ_EPILOGUE()    return 0;
#else
# define UAVCAN_STM32_IRQ_HANDLER(id)  void id(void)
# define UAVCAN_STM32_IRQ_PROLOGUE()
# define UAVCAN_STM32_IRQ_EPILOGUE()
#endif

#if UAVCAN_STM32_CHIBIOS
/**
 * Priority mask for timer and CAN interrupts.
 */
# ifndef UAVCAN_STM32_IRQ_PRIORITY_MASK
#  if (CH_KERNEL_MAJOR == 2)
#   define UAVCAN_STM32_IRQ_PRIORITY_MASK  CORTEX_PRIORITY_MASK(CORTEX_MAX_KERNEL_PRIORITY)
#  else // ChibiOS 3+
#   define UAVCAN_STM32_IRQ_PRIORITY_MASK  CORTEX_MAX_KERNEL_PRIORITY
#  endif
# endif
#endif

#if UAVCAN_STM32_BAREMETAL
/**
 * Priority mask for timer and CAN interrupts.
 */
# ifndef UAVCAN_STM32_IRQ_PRIORITY_MASK
#  define UAVCAN_STM32_IRQ_PRIORITY_MASK  0
# endif
#endif

#if UAVCAN_STM32_FREERTOS
/**
 * Priority mask for timer and CAN interrupts.
 */
# ifndef UAVCAN_STM32_IRQ_PRIORITY_MASK
#  define UAVCAN_STM32_IRQ_PRIORITY_MASK  configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
# endif
#endif

/**
 * Glue macros
 */
#define UAVCAN_STM32_GLUE2_(A, B)       A##B
#define UAVCAN_STM32_GLUE2(A, B)        UAVCAN_STM32_GLUE2_(A, B)

#define UAVCAN_STM32_GLUE3_(A, B, C)    A##B##C
#define UAVCAN_STM32_GLUE3(A, B, C)     UAVCAN_STM32_GLUE3_(A, B, C)

namespace uavcan_stm32
{
#if UAVCAN_STM32_CHIBIOS

struct CriticalSectionLocker
{
    CriticalSectionLocker() { chSysSuspend(); }
    ~CriticalSectionLocker() { chSysEnable(); }
};

#elif UAVCAN_STM32_NUTTX

struct CriticalSectionLocker
{
    const irqstate_t flags_;

    CriticalSectionLocker()
        : flags_(enter_critical_section())
    { }

    ~CriticalSectionLocker()
    {
        leave_critical_section(flags_);
    }
};

#elif UAVCAN_STM32_BAREMETAL

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

#elif UAVCAN_STM32_FREERTOS

struct CriticalSectionLocker
{

    CriticalSectionLocker()
    {
        taskENTER_CRITICAL();
    }

    ~CriticalSectionLocker()
    {
        taskEXIT_CRITICAL();
    }
};

#endif

namespace clock
{
uavcan::uint64_t getUtcUSecFromCanInterrupt();
}
}
