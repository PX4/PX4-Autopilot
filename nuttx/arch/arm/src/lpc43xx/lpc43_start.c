/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_start.c
 * arch/arm/src/chip/lpc43_start.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
/*
 * Power-Up Reset Overview
 * -----------------------
 *
 * The ARM core starts executing code on reset with the program counter set
 * to 0x0000:0000.  The LPC43xx contains a shadow pointer register that
 * allows areas of memory to be mapped to address 0x0000:0000. The default,
 * reset value of the shadow pointer is 0x1040:0000 so that on reset code in
 * the boot ROM is always executed first.
 *
 * The boot starts after reset is released.  The IRC is selected as CPU clock
 * and the Cortex-M4 starts the boot loader. By default the JTAG access to the
 * chip is disabled at reset.  The boot ROM determines the boot mode based on
 * the OTP BOOT_SRC value or reset state pins.  For flash-based parts, the part
 * boots from internal flash by default.  Otherwse, the boot ROM copies the
 * image to internal SRAM at location 0x1000:0000, sets the ARM's shadow
 * pointer to 0x1000:0000, and jumps to that location.
 *
 * However, using JTAG the executable image can be also loaded directly into
 * and executed from SRAM.
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/init.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "nvic.h"

#include "chip/lpc43_creg.h"

#include "lpc43_rgu.h"
#include "lpc43_cgu.h"
#include "lpc43_emc.h"
#include "lpc43_lowputc.h"

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

 /****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG
#  define showprogress(c) up_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_setbootrom
 *
 * Description:
 *   Set the shadow register to 0x1040:0000 and the VTOR to 0x0000:0000 so
 *   that any exceptions (particulary things like hard faults) that occur
 *   before we are initialized are caught by the BOOT ROM.
 *
 ****************************************************************************/

static inline void lpc43_setbootrom(void)
{
  /* Set the shadow register to the beginning of the boot ROM (Only bits 12-31) */

  putreg32(LPC43_ROM_BASE, LPC43_CREG_M4MEMMAP);

  /* Address zero now maps to the Boot ROM.  Make sure the the VTOR will
   * use the ROM vector table at that address.
   */

  putreg32(0, NVIC_VECTAB);
}

/****************************************************************************
 * Name: lpc43_enabuffering
 *
 * Description:
 *   If we are executing from external FLASH, then enable buffering.
 *
 ****************************************************************************/

#if defined(CONFIG_BOOT_CS0FLASH) || defined(CONFIG_BOOT_CS1FLASH) || \
    defined(CONFIG_BOOT_CS2FLASH) || defined(CONFIG_BOOT_CS3FLASH)
static inline void lpc43_enabuffering(void)
{
  uint32_t regval;

#ifdef CONFIG_BOOT_CS0FLASH
  regval = getreg32(LPC43_EMC_STATCONFIG0);
  regval |= EMC_STATCONFIG_BENA
  putreg32(regval, LPC43_EMC_STATCONFIG0);
#endif

#ifdef CONFIG_BOOT_CS1FLASH
  regval = getreg32(LPC43_EMC_STATCONFIG1);
  regval |= EMC_STATCONFIG_BENA
  putreg32(regval, LPC43_EMC_STATCONFIG1);
#endif

#ifdef CONFIG_BOOT_CS2FLASH
  regval = getreg32(LPC43_EMC_STATCONFIG2);
  regval |= EMC_STATCONFIG_BENA
  putreg32(regval, LPC43_EMC_STATCONFIG2);
#endif

#ifdef CONFIG_BOOT_CS3FLASH
  regval = getreg32(LPC43_EMC_STATCONFIG3);
  regval |= EMC_STATCONFIG_BENA
  putreg32(regval, LPC43_EMC_STATCONFIG3);
#endif
}
#else
#  define lpc43_enabuffering()
#endif

/****************************************************************************
 * Name: lpc43_fpuconfig
 *
 * Description:
 *   Configure the FPU.  Relative bit settings:
 *
 *     CPACR:  Enables access to CP10 and CP11
 *     CONTROL.FPCA: Determines whether the FP extension is active in the
 *       current context:
 *     FPCCR.ASPEN:  Enables automatic FP state preservation, then the
 *       processor sets this bit to 1 on successful completion of any FP
 *       instruction.
 *     FPCCR.LSPEN:  Enables lazy context save of FP state. When this is
 *       done, the processor reserves space on the stack for the FP state,
 *       but does not save that state information to the stack.
 *
 *  Software must not change the value of the ASPEN bit or LSPEN bit while either:
 *   - the CPACR permits access to CP10 and CP11, that give access to the FP
 *     extension, or
 *   - the CONTROL.FPCA bit is set to 1
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU
#ifdef CONFIG_ARMV7M_CMNVECTOR

static inline void lpc43_fpuconfig(void)
{
  uint32_t regval;

  /* Set CONTROL.FPCA so that we always get the extended context frame
   * with the volatile FP registers stacked above the basic context.
   */

  regval = getcontrol(); 
  regval |= (1 << 2);
  setcontrol(regval);

  /* Ensure that FPCCR.LSPEN is disabled, so that we don't have to contend
   * with the lazy FP context save behaviour.  Clear FPCCR.ASPEN since we
   * are going to turn on CONTROL.FPCA for all contexts.
   */

  regval = getreg32(NVIC_FPCCR);
  regval &= ~((1 << 31) | (1 << 30));
  putreg32(regval, NVIC_FPCCR);

  /* Enable full access to CP10 and CP11 */

  regval = getreg32(NVIC_CPACR);
  regval |= ((3 << (2*10)) | (3 << (2*11)));
  putreg32(regval, NVIC_CPACR);
}

#else

static inline void lpc43_fpuconfig(void)
{
  uint32_t regval;

  /* Clear CONTROL.FPCA so that we do not get the extended context frame
   * with the volatile FP registers stacked in the saved context.
   */

  regval = getcontrol(); 
  regval &= ~(1 << 2);
  setcontrol(regval);

  /* Ensure that FPCCR.LSPEN is disabled, so that we don't have to contend
   * with the lazy FP context save behaviour.  Clear FPCCR.ASPEN since we
   * are going to keep CONTROL.FPCA off for all contexts.
   */

  regval = getreg32(NVIC_FPCCR);
  regval &= ~((1 << 31) | (1 << 30));
  putreg32(regval, NVIC_FPCCR);

  /* Enable full access to CP10 and CP11 */

  regval = getreg32(NVIC_CPACR);
  regval |= ((3 << (2*10)) | (3 << (2*11)));
  putreg32(regval, NVIC_CPACR);
}

#endif

#else
#  define lpc43_fpuconfig()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _start
 *
 * Description:
 *   This is the reset entry point.
 *
 ****************************************************************************/

void __start(void)
{
  const uint32_t *src;
  uint32_t *dest;

  /* Reset as many of the LPC43 peripherals as possible. This is necessary
   * because the LPC43 does not provide any way of performing a full system
   * reset under debugger control.  So, if CONFIG_DEBUG is set (indicating
   * that a debugger is being used?), the the boot logic will call this 
   * function on all restarts.
   */

#ifdef CONFIG_DEBUG
  lpc43_softreset();
#endif

  /* Make sure that any exceptions (such as hard faults) that occur before
   * we are initialized are caught by the BOOT ROM.
   */

  lpc43_setbootrom();

  /* Configure the CGU clocking and the console uart so that we can get
   * debug output as soon as possible.
   */

  lpc43_clockconfig();
  lpc43_lowsetup();
  showprogress('A');

  /* If we are executing from external FLASH, then enable buffering */

  lpc43_enabuffering();

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = &_sbss; dest < &_ebss; )
    {
      *dest++ = 0;
    }
  showprogress('B');

  /* Move the intialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  for (src = &_eronly, dest = &_sdata; dest < &_edata; )
    {
      *dest++ = *src++;
    }
  showprogress('C');

  /* Initialize the FPU (if configured) */

  lpc43_fpuconfig();
  showprogress('D');

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  up_earlyserialinit();
#endif
  showprogress('E');

  /* Initialize onboard resources */

  lpc43_boardinitialize();
  showprogress('F');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');
  os_start();

  /* Shouldn't get here */

  for(;;);
}
