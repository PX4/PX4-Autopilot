/****************************************************************************
 * arch/z80/src/common/up_internal.h
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_Z80_SRC_COMMON_UP_INTERNAL_H
#define __ARCH_Z80_SRC_COMMON_UP_INTERNAL_H

/****************************************************************************
 * Conditional Compilation
 ****************************************************************************/

/* Bring-up debug configurations.  These are here (vs defconfig)
 * because these should only be controlled during low level
 * board bring-up and not part of normal platform configuration.
 */

#undef  CONFIG_SUPPRESS_INTERRUPTS    /* Do not enable interrupts */
#undef  CONFIG_SUPPRESS_TIMER_INTS    /* No timer */
#undef  CONFIG_SUPPRESS_SERIAL_INTS   /* Console will poll */
#undef  CONFIG_SUPPRESS_UART_CONFIG   /* Do not reconfig UART */
#undef  CONFIG_DUMP_ON_EXIT           /* Dump task state on exit */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include <arch/irq.h>
#include "chip/chip.h"
#include "chip/switch.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Determine which (if any) console driver to use.  If a console is enabled
 * and no other console device is specified, then a serial console is
 * assumed.
 */

#if CONFIG_NFILE_DESCRIPTORS == 0 || defined(CONFIG_DEV_LOWCONSOLE)
#  undef USE_SERIALDRIVER
#  ifdef CONFIG_HAVE_LOWSERIALINIT
#    define USE_LOWSERIALINIT 1
#  else
#    undef USE_LOWSERIALINIT
#  endif
#elif !defined(CONFIG_DEV_CONSOLE) || CONFIG_NFILE_DESCRIPTORS <= 0
#  undef  USE_SERIALDRIVER
#  undef  USE_LOWSERIALINIT
#  undef  CONFIG_DEV_LOWCONSOLE
#  undef  CONFIG_RAMLOG_CONSOLE
#else
#  undef  USE_LOWSERIALINIT
#  if defined(CONFIG_RAMLOG_CONSOLE)
#    undef  USE_SERIALDRIVER
#    undef  CONFIG_DEV_LOWCONSOLE
#  elif defined(CONFIG_DEV_LOWCONSOLE)
#    undef  USE_SERIALDRIVER
#  else
#    define USE_SERIALDRIVER 1
#  endif
#endif

/* If some other device is used as the console, then the serial driver may
 * still be needed.  Let's assume that if the upper half serial driver is
 * built, then the lower half will also be needed.  There is no need for
 * the early serial initialization in this case.
 */

#if !defined(USE_SERIALDRIVER) && defined(CONFIG_STANDARD_SERIAL)
#  define USE_SERIALDRIVER 1
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

/* Supplied by chip- or board-specific logic */

void up_irqinitialize(void);
int  up_timerisr(int irq, FAR chipreg_t *regs);

#ifdef USE_LOWSERIALINIT
void up_lowserialinit(void);
#endif

/* Defined in up_doirq.c */

FAR chipreg_t *up_doirq(uint8_t irq, FAR chipreg_t *regs);

/* Define in up_sigdeliver */

void up_sigdeliver(void);

/* Defined in CPU-specific logic (only for Z180) */

#if CONFIG_ADDRENV
int up_mmuinit(void);
#endif

/* Defined in up_allocateheap.c */

#if CONFIG_MM_REGIONS > 1
void up_addregion(void);
#endif

/* Defined in up_serial.c */

#ifdef USE_SERIALDRIVER
void up_serialinit(void);
#else
# define up_serialinit()
#endif

/* Defined in drivers/lowconsole.c */

#ifdef CONFIG_DEV_LOWCONSOLE
void lowconsole_init(void);
#else
# define lowconsole_init()
#endif

/* Defined in drivers/ramlog.c */

#ifdef CONFIG_RAMLOG_CONSOLE
extern void ramlog_consoleinit(void);
#else
# define ramlog_consoleinit()
#endif

/* Low level string output */

extern void up_puts(const char *str);

/* Defined in up_timerisr.c */

void up_timerinit(void);

/* Defined in board/up_leds.c */

#ifdef CONFIG_ARCH_LEDS
void up_ledinit(void);
void up_ledon(int led);
void up_ledoff(int led);
#else
# define up_ledinit()
# define up_ledon(led)
# define up_ledoff(led)
#endif

/* Architecture specific hook into the timer interrupt handler */

#ifdef CONFIG_ARCH_TIMERHOOK
void up_timerhook(void);
#endif

/* Defined in board/up_network.c */

#ifdef CONFIG_NET
int  up_netinitialize(void);
void up_netuninitialize(void);
# ifdef CONFIG_ARCH_MCFILTER
int up_multicastfilter(FAR struct uip_driver_s *dev, FAR uint8_t *mac, bool enable);
# else
#   define up_multicastfilter(dev, mac, enable)
# endif
#else
# define up_netinitialize()
# define up_netuninitialize()
# define up_multicastfilter(dev, mac, enable)
#endif

/* Return the current value of the stack pointer (used in stack dump logic) */

uint16_t up_getsp(void);

/* Dump stack and registers */

#ifdef CONFIG_ARCH_STACKDUMP
void up_stackdump(void);
# define REGISTER_DUMP() _REGISTER_DUMP()
#else
# define up_stackdump()
# define REGISTER_DUMP()
#endif

#ifdef __cplusplus
}
#endif
#endif

#endif  /* __ARCH_Z80_SRC_COMMON_UP_INTERNAL_H */
