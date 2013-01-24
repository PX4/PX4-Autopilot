/****************************************************************************
 * common/up_internal.h
 *
 *   Copyright (C) 2007-2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_COMMON_UP_INTERNAL_H
#define __ARCH_ARM_SRC_COMMON_UP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#  include <sys/types.h>
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bring-up debug configurations.  These are here (vs defconfig)
 * because these should only be controlled during low level
 * board bring-up and not part of normal platform configuration.
 */

#undef  CONFIG_SUPPRESS_INTERRUPTS    /* DEFINED: Do not enable interrupts */
#undef  CONFIG_SUPPRESS_TIMER_INTS    /* DEFINED: No timer */
#undef  CONFIG_SUPPRESS_SERIAL_INTS   /* DEFINED: Console will poll */
#undef  CONFIG_SUPPRESS_UART_CONFIG   /* DEFINED: Do not reconfig UART */
#undef  CONFIG_DUMP_ON_EXIT           /* DEFINED: Dump task state on exit */

/* Determine which (if any) console driver to use */

#if !defined(CONFIG_DEV_CONSOLE) || CONFIG_NFILE_DESCRIPTORS == 0
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

/* Determine which device to use as the system logging device */

#ifndef CONFIG_SYSLOG
#  undef CONFIG_SYSLOG_CHAR
#  undef CONFIG_RAMLOG_SYSLOG
#endif

/* Check if an interrupt stack size is configured */

#ifndef CONFIG_ARCH_INTERRUPTSTACK
# define CONFIG_ARCH_INTERRUPTSTACK 0
#endif

/* Macros to handle saving and restoring interrupt state.  In the current ARM
 * model, the state is always copied to and from the stack and TCB.  In the
 * Cortex-M3 model, the state is copied from the stack to the TCB, but only
 * a referenced is passed to get the state from the TCB.
 */

#if defined(CONFIG_ARCH_CORTEXM3) || defined(CONFIG_ARCH_CORTEXM4)
#  if defined(CONFIG_ARCH_FPU) && !defined(CONFIG_ARMV7M_CMNVECTOR)
#    define up_savestate(regs) \
       do { \
         up_copystate(regs, (uint32_t*)current_regs); \
         up_savefpu(regs); \
       } \
       while (0)
#  else
#    define up_savestate(regs)  up_copystate(regs, (uint32_t*)current_regs)
#  endif
#  define up_restorestate(regs) (current_regs = regs)
#else
#  define up_savestate(regs)    up_copystate(regs, (uint32_t*)current_regs)
#  define up_restorestate(regs) up_copystate((uint32_t*)current_regs, regs)
#endif

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

extern volatile uint32_t *current_regs;

/* This is the beginning of heap as provided from up_head.S.
 * This is the first address in DRAM after the loaded
 * program+bss+idle stack.  The end of the heap is
 * CONFIG_DRAM_END
 */

extern const uint32_t g_heapbase;

/* Address of the saved user stack pointer */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
#if defined(CONFIG_ARCH_CORTEXM3) || defined(CONFIG_ARCH_CORTEXM4)
extern uint32_t g_intstackbase;
#  else
extern uint32_t g_userstack;
#  endif
#endif

/* These 'addresses' of these values are setup by the linker script.  They are
 * not actual uint32_t storage locations! They are only used meaningfully in the
 * following way:
 *
 *  - The linker script defines, for example, the symbol_sdata.
 *  - The declareion extern uint32_t _sdata; makes C happy.  C will believe
 *    that the value _sdata is the address of a uint32_t variable _data (it is
 *    not!).
 *  - We can recoved the linker value then by simply taking the address of
 *    of _data.  like:  uint32_t *pdata = &_sdata;
 */

extern uint32_t _stext;           /* Start of .text */
extern uint32_t _etext;           /* End_1 of .text + .rodata */
extern const uint32_t _eronly;    /* End+1 of read only section (.text + .rodata) */
extern uint32_t _sdata;           /* Start of .data */
extern uint32_t _edata;           /* End+1 of .data */
extern uint32_t _sbss;            /* Start of .bss */
extern uint32_t _ebss;            /* End+1 of .bss */

/* Sometimes, functions must be executed from RAM.  In this case, the following
 * macro may be used (with GCC!) to specify a function that will execute from
 * RAM.  For example,
 *
 *   int __ramfunc__ foo (void);
 *   int __ramfunc__ foo (void) { return bar; }
 *
 * will create a function named foo that will execute from RAM.
 */
 
#ifdef CONFIG_ARCH_RAMFUNCS

#  define __ramfunc__ __attribute__ ((section(".ramfunc")))

/* Functions decleared in the .ramfunc section will be packaged together
 * by the linker script and stored in FLASH.  During boot-up, the start
 * logic must include logic to copy the RAM functions from their storage
 * location in FLASH to their correct destination in SRAM.  The following
 * following linker-defined values provide the information to copy the
 * functions from flash to RAM.
 */

extern const uint32_t _framfuncs; /* Copy source address in FLASH */
extern uint32_t _sramfuncs;       /* Copy destination start address in RAM */
extern uint32_t _eramfuncs;       /* Copy destination start address in RAM */

#endif /* CONFIG_ARCH_RAMFUNCS */
#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Low level initialization provided by board-level logic ******************/

extern void up_boot(void);

/* Context switching */

extern void up_copystate(uint32_t *dest, uint32_t *src);
extern void up_decodeirq(uint32_t *regs);
extern int  up_saveusercontext(uint32_t *saveregs);
extern void up_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
extern void up_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);

/* Signal handling **********************************************************/

extern void up_sigdeliver(void);

/* Power management *********************************************************/

#ifdef CONFIG_PM
extern void up_pminitialize(void);
#else
#  define up_pminitialize()
#endif

#if defined(CONFIG_ARCH_CORTEXM3) || defined(CONFIG_ARCH_CORTEXM4)
extern void up_systemreset(void) noreturn_function;
#endif

/* Interrupt handling *******************************************************/

extern void up_irqinitialize(void);
extern void up_maskack_irq(int irq);

#if defined(CONFIG_ARCH_CORTEXM3) || defined(CONFIG_ARCH_CORTEXM4)

extern uint32_t *up_doirq(int irq, uint32_t *regs);
extern int  up_svcall(int irq, FAR void *context);
extern int  up_hardfault(int irq, FAR void *context);
extern int  up_memfault(int irq, FAR void *context);

#else /* CONFIG_ARCH_CORTEXM3 || CONFIG_ARCH_CORTEXM4 */

extern void up_doirq(int irq, uint32_t *regs);
#ifdef CONFIG_PAGING
extern void up_pginitialize(void);
extern uint32_t *up_va2pte(uintptr_t vaddr);
extern void up_dataabort(uint32_t *regs, uint32_t far, uint32_t fsr);
#else /* CONFIG_PAGING */
# define up_pginitialize()
extern void up_dataabort(uint32_t *regs);
#endif /* CONFIG_PAGING */
extern void up_prefetchabort(uint32_t *regs);
extern void up_syscall(uint32_t *regs);
extern void up_undefinedinsn(uint32_t *regs);

#endif /* CONFIG_ARCH_CORTEXM3 || CONFIG_ARCH_CORTEXM4 */

extern void up_vectorundefinsn(void);
extern void up_vectorswi(void);
extern void up_vectorprefetch(void);
extern void up_vectordata(void);
extern void up_vectoraddrexcptn(void);
extern void up_vectorirq(void);
extern void up_vectorfiq(void);

/* Floating point unit ******************************************************/

#ifdef CONFIG_ARCH_FPU
extern void up_savefpu(uint32_t *regs);
extern void up_restorefpu(const uint32_t *regs);
#else
#  define up_savefpu(regs)
#  define up_restorefpu(regs)
#endif

/* System timer *************************************************************/

extern void up_timerinit(void);
extern int  up_timerisr(int irq, uint32_t *regs);

/* Low level serial output **************************************************/

extern void up_lowputc(char ch);
extern void up_puts(const char *str);
extern void up_lowputs(const char *str);

#if CONFIG_NFILE_DESCRIPTORS > 0
extern void up_earlyserialinit(void);
extern void up_serialinit(void);
#else
# define up_earlyserialinit()
# define up_serialinit()
#endif

/* Defined in drivers/lowconsole.c */

#ifdef CONFIG_DEV_LOWCONSOLE
extern void lowconsole_init(void);
#else
# define lowconsole_init()
#endif

/* DMA **********************************************************************/

#ifdef CONFIG_ARCH_DMA
extern void weak_function up_dmainitialize(void);
#endif

/* Memory management ********************************************************/

#if CONFIG_MM_REGIONS > 1
void up_addregion(void);
#else
# define up_addregion()
#endif

/* Watchdog timer ***********************************************************/

extern void up_wdtinit(void);

/* LED interfaces provided by board-level logic *****************************/

#ifdef CONFIG_ARCH_LEDS
extern void up_ledinit(void);
extern void up_ledon(int led);
extern void up_ledoff(int led);
#else
# define up_ledinit()
# define up_ledon(led)
# define up_ledoff(led)
#endif

/* Networking ***************************************************************/

/* Defined in board/up_network.c for board-specific ethernet implementations,
 * or chip/xyx_ethernet.c for chip-specific ethernet implementations, or
 * common/up_etherstub.c for a cornercase where the network is enabled yet
 * there is no ethernet driver to be initialized.
 */

#ifdef CONFIG_NET
extern void up_netinitialize(void);
#else
# define up_netinitialize()
#endif

/* USB **********************************************************************/

#ifdef CONFIG_USBDEV
extern void up_usbinitialize(void);
extern void up_usbuninitialize(void);
#else
# define up_usbinitialize()
# define up_usbuninitialize()
#endif

/* Random Number Generator (RNG) ********************************************/

#ifdef CONFIG_DEV_RANDOM
extern void up_rnginitialize(void);
#endif

/****************************************************************************
 * Name: up_check_stack
 *
 * Description:
 *   Determine (approximately) how much stack has been used be searching the
 *   stack memory for a high water mark.  That is, the deepest level of the
 *   stack that clobbered some recognizable marker in the stack memory.
 *
 * Input Parameters:
 *   None
 *
 * Returned value:
 *   The estimated amount of stack space used.
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_STACK)
extern size_t up_check_stack(void);
extern size_t up_check_tcbstack(FAR _TCB);
extern size_t up_check_tcbstack_remain(FAR _TCB);
#endif

#endif /* __ASSEMBLY__ */

#endif  /* __ARCH_ARM_SRC_COMMON_UP_INTERNAL_H */
