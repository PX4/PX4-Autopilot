/************************************************************************************
 * arch/x86/src/qemu/qemu_internal.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_X86_SRC_QEMU_QEMU_INTERNAL_H
#define __ARCH_X86_SRC_QEMU_QEMU_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "up_internal.h"
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: i486_clockconfig
 *
 * Description:
 *   Called to initialize the LPC17XX.  This does whatever setup is needed to put the
 *   MCU in a usable state.  This includes the initialization of clocking using the
 *   settings in board.h.
 *
 ************************************************************************************/

EXTERN void i486_clockconfig(void);

/************************************************************************************
 * Name: i486_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level initialization
 *   including setup of the console UART.  This UART done early so that the serial
 *   console is available for debugging very early in the boot sequence.
 *
 ************************************************************************************/

EXTERN void i486_lowsetup(void);

/************************************************************************************
 * Name: i486_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for GPIO pins.
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void i486_gpioirqinitialize(void);
#else
#  define i486_gpioirqinitialize()
#endif

/************************************************************************************
 * Name: i486_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

EXTERN int i486_configgpio(uint16_t cfgset);

/************************************************************************************
 * Name: i486_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

EXTERN void i486_gpiowrite(uint16_t pinset, bool value);

/************************************************************************************
 * Name: i486_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

EXTERN bool i486_gpioread(uint16_t pinset);

/************************************************************************************
 * Name: i486_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void i486_gpioirqenable(int irq);
#else
#  define i486_gpioirqenable(irq)
#endif

/************************************************************************************
 * Name: i486_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void i486_gpioirqdisable(int irq);
#else
#  define i486_gpioirqdisable(irq)
#endif

/************************************************************************************
 * Function:  i486_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO
EXTERN int i486_dumpgpio(uint16_t pinset, const char *msg);
#else
#  define i486_dumpgpio(p,m)
#endif

/************************************************************************************
 * Name:  i486_spi/ssp0/ssp1select, i486_spi/ssp0/ssp1status, and
 *        i486_spi/ssp0/ssp1cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi.h). All other methods 
 *   including up_spiinitialize()) are provided by common LPC17xx logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in i486_boardinitialize() to configure SPI/SSP chip select
 *      pins.
 *   2. Provide i486_spi/ssp0/ssp1select() and i486_spi/ssp0/ssp1status() functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      i486_spi/ssp0/ssp1cmddata() functions in your board-specific logic.  These
 *      functions will perform cmd/data selection operations using GPIOs in the way
 *      your board is configured.
 *   3. Add a call to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling 
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

struct spi_dev_s;
enum spi_dev_e;

#ifdef CONFIG_I486_SPI
EXTERN void  i486_spiselect(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t i486_spistatus(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
EXTERN int i486_spicmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
#endif

/****************************************************************************
 * Name: ssp_flush
 *
 * Description:
 *   Flush and discard any words left in the RX fifo.  This can be called
 *   from ssp0/1select after a device is deselected (if you worry about such
 *   things).
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

struct spi_dev_s;
#ifdef CONFIG_I486_SPI
EXTERN void spi_flush(FAR struct spi_dev_s *dev);
#endif
#if defined(CONFIG_I486_SSP0) || defined(CONFIG_I486_SSP1)
EXTERN void ssp_flush(FAR struct spi_dev_s *dev);
#endif

/****************************************************************************
 * Name: i486_dmainitialize
 *
 * Description:
 *   Initialize the GPDMA subsystem.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_I486_GPDMA
EXTERN void i486_dmainitilaize(void);
#endif

/****************************************************************************
 * Name: i486_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   One success, this function returns a non-NULL, void* DMA channel
 *   handle.  NULL is returned on any failure.  This function can fail only
 *   if no DMA channel is available.
 *
 ****************************************************************************/

#ifdef CONFIG_I486_GPDMA
EXTERN DMA_HANDLE i486_dmachannel(void);
#endif

/****************************************************************************
 * Name: i486_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until i486_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_I486_GPDMA
EXTERN void i486_dmafree(DMA_HANDLE handle);
#endif

/****************************************************************************
 * Name: i486_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

#ifdef CONFIG_I486_GPDMA
EXTERN int i486_dmarxsetup(DMA_HANDLE handle,
                            uint32_t control, uint32_t config,
                            uint32_t srcaddr, uint32_t destaddr,
                            size_t nbytes);
#endif

/****************************************************************************
 * Name: i486_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

#ifdef CONFIG_I486_GPDMA
EXTERN int i486_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);
#endif

/****************************************************************************
 * Name: i486_dmastop
 *
 * Description:
 *   Cancel the DMA.  After i486_dmastop() is called, the DMA channel is
 *   reset and i486_dmasetup() must be called before i486_dmastart() can be
 *   called again
 *
 ****************************************************************************/

#ifdef CONFIG_I486_GPDMA
EXTERN void i486_dmastop(DMA_HANDLE handle);
#endif

/****************************************************************************
 * Name: i486_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_I486_GPDMA
#ifdef CONFIG_DEBUG_DMA
EXTERN void i486_dmasample(DMA_HANDLE handle, struct i486_dmaregs_s *regs);
#else
#  define i486_dmasample(handle,regs)
#endif
#endif

/****************************************************************************
 * Name: i486_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_I486_GPDMA
#ifdef CONFIG_DEBUG_DMA
EXTERN void i486_dmadump(DMA_HANDLE handle, const struct i486_dmaregs_s *regs,
                          const char *msg);
#else
#  define i486_dmadump(handle,regs,msg)
#endif
#endif

/****************************************************************************
 * Name: vector_*
 *
 * Description:
 *   These are the various ISR/IRQ vector address exported from
 *   qemu_vectors.S.  These addresses need to have global scope so that they
 *   can be known to the interrupt intialization logic in qemu_irq.c.
 *
 ****************************************************************************/

EXTERN void vector_isr0(void);
EXTERN void vector_isr1(void);
EXTERN void vector_isr2(void);
EXTERN void vector_isr3(void);
EXTERN void vector_isr4(void);
EXTERN void vector_isr5(void);
EXTERN void vector_isr6(void);
EXTERN void vector_isr7(void);
EXTERN void vector_isr8(void);
EXTERN void vector_isr9(void);
EXTERN void vector_isr10(void);
EXTERN void vector_isr11(void);
EXTERN void vector_isr12(void);
EXTERN void vector_isr13(void);
EXTERN void vector_isr14(void);
EXTERN void vector_isr15(void);
EXTERN void vector_isr16(void);
EXTERN void vector_isr17(void);
EXTERN void vector_isr18(void);
EXTERN void vector_isr19(void);
EXTERN void vector_isr20(void);
EXTERN void vector_isr21(void);
EXTERN void vector_isr22(void);
EXTERN void vector_isr23(void);
EXTERN void vector_isr24(void);
EXTERN void vector_isr25(void);
EXTERN void vector_isr26(void);
EXTERN void vector_isr27(void);
EXTERN void vector_isr28(void);
EXTERN void vector_isr29(void);
EXTERN void vector_isr30(void);
EXTERN void vector_isr31(void);
EXTERN void vector_irq0(void);
EXTERN void vector_irq1(void);
EXTERN void vector_irq2(void);
EXTERN void vector_irq3(void);
EXTERN void vector_irq4(void);
EXTERN void vector_irq5(void);
EXTERN void vector_irq6(void);
EXTERN void vector_irq7(void);
EXTERN void vector_irq8(void);
EXTERN void vector_irq9(void);
EXTERN void vector_irq10(void);
EXTERN void vector_irq11(void);
EXTERN void vector_irq12(void);
EXTERN void vector_irq13(void);
EXTERN void vector_irq14(void);
EXTERN void vector_irq15(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_X86_SRC_QEMU_QEMU_INTERNAL_H */
