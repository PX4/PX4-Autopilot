/****************************************************************************
 * common/up_internal.h
 *
 *   Copyright (C) 2008-2009, 2011-2013 Gregory Nutt. All rights reserved.
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

#ifndef __UP_INTERNAL_H
#define __UP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>
#include "chip/chip.h"

/****************************************************************************
 * Definitions
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
#undef  CONFIG_Z16_LOWPUTC            /* Support up_lowputc for debug */
#undef  CONFIG_Z16_LOWGETC            /* support up_lowgetc for debug */

/* Determine which (if any) console driver to use.  If a console is enabled
 * and no other console device is specified, then a serial console is
 * assumed.
 */

#if defined(CONFIG_Z16_LOWPUTC) || defined(CONFIG_Z16_LOWGETC) || \
    CONFIG_NFILE_DESCRIPTORS == 0 || defined(CONFIG_DEV_LOWCONSOLE)
#  define USE_LOWCONSOLE 1
#  define USE_LOWUARTINIT 1
#elif !defined(CONFIG_DEV_CONSOLE) || CONFIG_NFILE_DESCRIPTORS <= 0
#  undef  USE_SERIALDRIVER
#  undef  USE_EARLYSERIALINIT
#  undef  CONFIG_DEV_LOWCONSOLE
#  undef  CONFIG_RAMLOG_CONSOLE
#else
#  if defined(CONFIG_RAMLOG_CONSOLE)
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#    undef  CONFIG_DEV_LOWCONSOLE
#  elif defined(CONFIG_DEV_LOWCONSOLE)
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#  else
#    define USE_SERIALDRIVER 1
#    define USE_EARLYSERIALINIT 1
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

/* Determine which device to use as the system logging device */

#ifndef CONFIG_SYSLOG
#  undef CONFIG_SYSLOG_CHAR
#  undef CONFIG_RAMLOG_SYSLOG
#endif

/* Macros for portability */

#define IN_INTERRUPT             (current_regs != NULL)
#define SAVE_IRQCONTEXT(tcb)     up_copystate((tcb)->xcp.regs, (FAR chipreg_t*)current_regs)
#define SET_IRQCONTEXT(tcb)      do { current_regs = (tcb)->xcp.regs; } while (0)
#define SAVE_USERCONTEXT(tcb)    up_saveusercontext((tcb)->xcp.regs)
#define RESTORE_USERCONTEXT(tcb) up_restoreusercontext((tcb)->xcp.regs)
#define SIGNAL_RETURN(regs)      up_restoreusercontext(regs)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*up_vector_t)(void);
#endif

/****************************************************************************
 * Public Variables
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* This holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during
 * interrupt processing.
 */

extern volatile FAR chipreg_t *current_regs;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Defined in files with the same name as the function */

extern void up_copystate(FAR chipreg_t *dest, FAR chipreg_t *src);
extern FAR chipreg_t *up_doirq(int irq, FAR chipreg_t *regs);
extern void up_restoreusercontext(FAR chipreg_t *regs);
extern void up_irqinitialize(void);
extern int  up_saveusercontext(FAR chipreg_t *regs);
extern void up_sigdeliver(void);
extern int  up_timerisr(int irq, FAR chipreg_t *regs);

#if defined(CONFIG_Z16_LOWPUTC) || defined(CONFIG_Z16_LOWGETC)
extern void up_lowputc(char ch);
#else
# define up_lowputc(ch)
#endif

/* Defined in up_allocateheap.c */

#if CONFIG_MM_REGIONS > 1
void up_addregion(void);
#endif

/* Defined in up_serial.c */

#ifdef USE_SERIALDRIVER
extern void up_earlyserialinit(void);
extern void up_serialinit(void);
#endif

#ifdef USE_LOWCONSOLE
extern void lowconsole_init(void);
#endif

/* Defined in up_timerisr.c */

extern void up_timerinit(void);

/* Defined in up_irq.c */

extern void up_maskack_irq(int irq);

/* Defined in board/up_leds.c */

#ifdef CONFIG_ARCH_LEDS
extern void up_ledinit(void);
extern void up_ledon(int led);
extern void up_ledoff(int led);
#else
# define up_ledinit()
# define up_ledon(led)
# define up_ledoff(led)
#endif

/* Defined in board/up_network.c */

#ifdef CONFIG_NET
extern void up_netinitialize(void);
#else
# define up_netinitialize()
#endif

/* Return the current value of the stack pointer (used in stack dump logic) */

extern chipreg_t up_getsp(void);

/* Dump stack and registers */

#ifdef CONFIG_ARCH_STACKDUMP
extern void up_stackdump(void);
extern void up_registerdump(void);
#else
# define up_stackdump()
# define up_registerdump()
#endif

#endif /* __ASSEMBLY__ */

#endif  /* __UP_INTERNAL_H */
