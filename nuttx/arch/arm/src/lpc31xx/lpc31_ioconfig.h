/************************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_ioconfig.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_IOCONFIG_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_IOCONFIG_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* IOCONFIG register base address offset into the APB0 domain ***********************************/

#define LPC31_IOCONFIG_VBASE                (LPC31_APB0_VADDR+LPC31_APB0_IOCONFIG_OFFSET)
#define LPC31_IOCONFIG_PBASE                (LPC31_APB0_PADDR+LPC31_APB0_IOCONFIG_OFFSET)

/* IOCONFIG function block offsets (with respect to the IOCONFIG register base address) *********/

#define LPC31_IOCONFIG_EBIMCI_OFFSET        0x000 /* First set of 32 multiplexed pads */
#define LPC31_IOCONFIG_EBII2STX0_OFFSET     0X040 /* Second set of 32 of multiplexed pads */
#define LPC31_IOCONFIG_CGU_OFFSET           0X080 /* Clock Generation Unit function block */
#define LPC31_IOCONFIG_I2SRX0_OFFSET        0x0c0 /* I2SRX function block 0 */
#define LPC31_IOCONFIG_I2SRX1_OFFSET        0x100 /* I2SRX function block 1 */
#define LPC31_IOCONFIG_I2STX1_OFFSET        0x140 /* I2STX function block 1 */
#define LPC31_IOCONFIG_EBI_OFFSET           0x180 /* External Bus Interface function block */
#define LPC31_IOCONFIG_GPIO_OFFSET          0x1c0 /* General purpose IO */
#define LPC31_IOCONFIG_I2C1_OFFSET          0x200 /* I2C function block */
#define LPC31_IOCONFIG_SPI_OFFSET           0x240 /* SPI function block */
#define LPC31_IOCONFIG_NAND_OFFSET          0x280 /* NANDFLASH function block */
#define LPC31_IOCONFIG_PWM_OFFSET           0x2c0 /* PWM function block */
#define LPC31_IOCONFIG_UART_OFFSET          0x300 /* UART function block */

/* IOCONFIG register offsets (with respect to any funcion block base address) *******************/

#define LPC31_IOCONFIG_PINS_OFFSET          0x000 /* WR:           RD: Input pin state */
                                                    /* 0x004-0x00c: Reserved */
#define LPC31_IOCONFIG_MODE0_OFFSET         0x010 /* WR:Load       RD: */
#define LPC31_IOCONFIG_MODE0SET_OFFSET      0x014 /* WR:Set Bits   RD:Read Mode 0 */
#define LPC31_IOCONFIG_MODE0RESET_OFFSET    0x018 /* WR:Reset Bits RD: */
                                                    /* 0x01c: Reserved */
#define LPC31_IOCONFIG_MODE1_OFFSET         0x020 /* WR:Load       RD: */
#define LPC31_IOCONFIG_MODE1SET_OFFSET      0x024 /* WR:Set Bits   RD:Read Mode 1 */
#define LPC31_IOCONFIG_MODE1RESET_OFFSET    0x028 /* WR:Reset Bits RD: */
                                                    /* 0x02c-0x3c: Reserved */

/* IOCONFIG function block (virtual) base addresses *********************************************/

#define LPC31_IOCONFIG_EBIMCI               (LPC31_IOCONFIG_VBASE+LPC31_IOCONFIG_EBIMCI_OFFSET)
#define LPC31_IOCONFIG_EBII2STX0            (LPC31_IOCONFIG_VBASE+LPC31_IOCONFIG_EBII2STX0_OFFSET)
#define LPC31_IOCONFIG_CGU                  (LPC31_IOCONFIG_VBASE+LPC31_IOCONFIG_CGU_OFFSET)
#define LPC31_IOCONFIG_I2SRX0               (LPC31_IOCONFIG_VBASE+LPC31_IOCONFIG_I2SRX0_OFFSET)
#define LPC31_IOCONFIG_I2SRX1               (LPC31_IOCONFIG_VBASE+LPC31_IOCONFIG_I2SRX1_OFFSET)
#define LPC31_IOCONFIG_I2STX1               (LPC31_IOCONFIG_VBASE+LPC31_IOCONFIG_I2STX1_OFFSET)
#define LPC31_IOCONFIG_EBI                  (LPC31_IOCONFIG_VBASE+LPC31_IOCONFIG_EBI_OFFSET)
#define LPC31_IOCONFIG_GPIO                 (LPC31_IOCONFIG_VBASE+LPC31_IOCONFIG_GPIO_OFFSET)
#define LPC31_IOCONFIG_I2C1                 (LPC31_IOCONFIG_VBASE+LPC31_IOCONFIG_I2C1_OFFSET)
#define LPC31_IOCONFIG_SPI                  (LPC31_IOCONFIG_VBASE+LPC31_IOCONFIG_SPI_OFFSET)
#define LPC31_IOCONFIG_NAND                 (LPC31_IOCONFIG_VBASE+LPC31_IOCONFIG_NAND_OFFSET)
#define LPC31_IOCONFIG_PWM                  (LPC31_IOCONFIG_VBASE+LPC31_IOCONFIG_PWM_OFFSET)
#define LPC31_IOCONFIG_UART                 (LPC31_IOCONFIG_VBASE+LPC31_IOCONFIG_UART_OFFSET)

/* IOCONFIG register (virtual) addresses ********************************************************/

#define LPC31_IOCONFIG_EBIMCI_PINS          (LPC31_IOCONFIG_EBIMCI+LPC31_IOCONFIG_PINS_OFFSET)
#define LPC31_IOCONFIG_EBIMCI_MODE0         (LPC31_IOCONFIG_EBIMCI+LPC31_IOCONFIG_MODE0_OFFSET)
#define LPC31_IOCONFIG_EBIMCI_MODE0SET      (LPC31_IOCONFIG_EBIMCI+LPC31_IOCONFIG_MODE0SET_OFFSET)
#define LPC31_IOCONFIG_EBIMCI_MODE0RESET    (LPC31_IOCONFIG_EBIMCI+LPC31_IOCONFIG_MODE0RESET_OFFSET)
#define LPC31_IOCONFIG_EBIMCI_MODE1         (LPC31_IOCONFIG_EBIMCI+LPC31_IOCONFIG_MODE1_OFFSET)
#define LPC31_IOCONFIG_EBIMCI_MODE1SET      (LPC31_IOCONFIG_EBIMCI+LPC31_IOCONFIG_MODE1SET_OFFSET)
#define LPC31_IOCONFIG_EBIMCI_MODE1RESET    (LPC31_IOCONFIG_EBIMCI+LPC31_IOCONFIG_MODE1RESET_OFFSET)

#define LPC31_IOCONFIG_EBII2STX0_PINS       (LPC31_IOCONFIG_EBII2STX0+LPC31_IOCONFIG_PINS_OFFSET)
#define LPC31_IOCONFIG_EBII2STX0_MODE0      (LPC31_IOCONFIG_EBII2STX0+LPC31_IOCONFIG_MODE0_OFFSET)
#define LPC31_IOCONFIG_EBII2STX0_MODE0SET   (LPC31_IOCONFIG_EBII2STX0+LPC31_IOCONFIG_MODE0SET_OFFSET)
#define LPC31_IOCONFIG_EBII2STX0_MODE0RESET (LPC31_IOCONFIG_EBII2STX0+LPC31_IOCONFIG_MODE0RESET_OFFSET)
#define LPC31_IOCONFIG_EBII2STX0_MODE1      (LPC31_IOCONFIG_EBII2STX0+LPC31_IOCONFIG_MODE1_OFFSET)
#define LPC31_IOCONFIG_EBII2STX0_MODE1SET   (LPC31_IOCONFIG_EBII2STX0+LPC31_IOCONFIG_MODE1SET_OFFSET)
#define LPC31_IOCONFIG_EBII2STX0_MODE1RESET (LPC31_IOCONFIG_EBII2STX0+LPC31_IOCONFIG_MODE1RESET_OFFSET)

#define LPC31_IOCONFIG_CGU_PINS             (LPC31_IOCONFIG_CGU+LPC31_IOCONFIG_PINS_OFFSET)
#define LPC31_IOCONFIG_CGU_MODE0            (LPC31_IOCONFIG_CGU+LPC31_IOCONFIG_MODE0_OFFSET)
#define LPC31_IOCONFIG_CGU_MODE0SET         (LPC31_IOCONFIG_CGU+LPC31_IOCONFIG_MODE0SET_OFFSET)
#define LPC31_IOCONFIG_CGU_MODE0RESET       (LPC31_IOCONFIG_CGU+LPC31_IOCONFIG_MODE0RESET_OFFSET)
#define LPC31_IOCONFIG_CGU_MODE1            (LPC31_IOCONFIG_CGU+LPC31_IOCONFIG_MODE1_OFFSET)
#define LPC31_IOCONFIG_CGU_MODE1SET         (LPC31_IOCONFIG_CGU+LPC31_IOCONFIG_MODE1SET_OFFSET)
#define LPC31_IOCONFIG_CGU_MODE1RESET       (LPC31_IOCONFIG_CGU+LPC31_IOCONFIG_MODE1RESET_OFFSET)

#define LPC31_IOCONFIG_I2SRX0_PINS          (LPC31_IOCONFIG_I2SRX0+LPC31_IOCONFIG_PINS_OFFSET)
#define LPC31_IOCONFIG_I2SRX0_MODE0         (LPC31_IOCONFIG_I2SRX0+LPC31_IOCONFIG_MODE0_OFFSET)
#define LPC31_IOCONFIG_I2SRX0_MODE0SET      (LPC31_IOCONFIG_I2SRX0+LPC31_IOCONFIG_MODE0SET_OFFSET)
#define LPC31_IOCONFIG_I2SRX0_MODE0RESET    (LPC31_IOCONFIG_I2SRX0+LPC31_IOCONFIG_MODE0RESET_OFFSET)
#define LPC31_IOCONFIG_I2SRX0_MODE1         (LPC31_IOCONFIG_I2SRX0+LPC31_IOCONFIG_MODE1_OFFSET)
#define LPC31_IOCONFIG_I2SRX0_MODE1SET      (LPC31_IOCONFIG_I2SRX0+LPC31_IOCONFIG_MODE1SET_OFFSET)
#define LPC31_IOCONFIG_I2SRX0_MODE1RESET    (LPC31_IOCONFIG_I2SRX0+LPC31_IOCONFIG_MODE1RESET_OFFSET)

#define LPC31_IOCONFIG_I2SRX1_PINS          (LPC31_IOCONFIG_I2SRX1+LPC31_IOCONFIG_PINS_OFFSET)
#define LPC31_IOCONFIG_I2SRX1_MODE0         (LPC31_IOCONFIG_I2SRX1+LPC31_IOCONFIG_MODE0_OFFSET)
#define LPC31_IOCONFIG_I2SRX1_MODE0SET      (LPC31_IOCONFIG_I2SRX1+LPC31_IOCONFIG_MODE0SET_OFFSET)
#define LPC31_IOCONFIG_I2SRX1_MODE0RESET    (LPC31_IOCONFIG_I2SRX1+LPC31_IOCONFIG_MODE0RESET_OFFSET)
#define LPC31_IOCONFIG_I2SRX1_MODE1         (LPC31_IOCONFIG_I2SRX1+LPC31_IOCONFIG_MODE1_OFFSET)
#define LPC31_IOCONFIG_I2SRX1_MODE1SET      (LPC31_IOCONFIG_I2SRX1+LPC31_IOCONFIG_MODE1SET_OFFSET)
#define LPC31_IOCONFIG_I2SRX1_MODE1RESET    (LPC31_IOCONFIG_I2SRX1+LPC31_IOCONFIG_MODE1RESET_OFFSET)

#define LPC31_IOCONFIG_I2STX1_PINS          (LPC31_IOCONFIG_I2STX1+LPC31_IOCONFIG_PINS_OFFSET)
#define LPC31_IOCONFIG_I2STX1_MODE0         (LPC31_IOCONFIG_I2STX1+LPC31_IOCONFIG_MODE0_OFFSET)
#define LPC31_IOCONFIG_I2STX1_MODE0SET      (LPC31_IOCONFIG_I2STX1+LPC31_IOCONFIG_MODE0SET_OFFSET)
#define LPC31_IOCONFIG_I2STX1_MODE0RESET    (LPC31_IOCONFIG_I2STX1+LPC31_IOCONFIG_MODE0RESET_OFFSET)
#define LPC31_IOCONFIG_I2STX1_MODE1         (LPC31_IOCONFIG_I2STX1+LPC31_IOCONFIG_MODE1_OFFSET)
#define LPC31_IOCONFIG_I2STX1_MODE1SET      (LPC31_IOCONFIG_I2STX1+LPC31_IOCONFIG_MODE1SET_OFFSET)
#define LPC31_IOCONFIG_I2STX1_MODE1RESET    (LPC31_IOCONFIG_I2STX1+LPC31_IOCONFIG_MODE1RESET_OFFSET)

#define LPC31_IOCONFIG_EBI_PINS             (LPC31_IOCONFIG_EBI+LPC31_IOCONFIG_PINS_OFFSET)
#define LPC31_IOCONFIG_EBI_MODE0            (LPC31_IOCONFIG_EBI+LPC31_IOCONFIG_MODE0_OFFSET)
#define LPC31_IOCONFIG_EBI_MODE0SET         (LPC31_IOCONFIG_EBI+LPC31_IOCONFIG_MODE0SET_OFFSET)
#define LPC31_IOCONFIG_EBI_MODE0RESET       (LPC31_IOCONFIG_EBI+LPC31_IOCONFIG_MODE0RESET_OFFSET)
#define LPC31_IOCONFIG_EBI_MODE1            (LPC31_IOCONFIG_EBI+LPC31_IOCONFIG_MODE1_OFFSET)
#define LPC31_IOCONFIG_EBI_MODE1SET         (LPC31_IOCONFIG_EBI+LPC31_IOCONFIG_MODE1SET_OFFSET)
#define LPC31_IOCONFIG_EBI_MODE1RESET       (LPC31_IOCONFIG_EBI+LPC31_IOCONFIG_MODE1RESET_OFFSET)

#define LPC31_IOCONFIG_GPIO_PINS            (LPC31_IOCONFIG_GPIO+LPC31_IOCONFIG_PINS_OFFSET)
#define LPC31_IOCONFIG_GPIO_MODE0           (LPC31_IOCONFIG_GPIO+LPC31_IOCONFIG_MODE0_OFFSET)
#define LPC31_IOCONFIG_GPIO_MODE0SET        (LPC31_IOCONFIG_GPIO+LPC31_IOCONFIG_MODE0SET_OFFSET)
#define LPC31_IOCONFIG_GPIO_MODE0RESET      (LPC31_IOCONFIG_GPIO+LPC31_IOCONFIG_MODE0RESET_OFFSET)
#define LPC31_IOCONFIG_GPIO_MODE1           (LPC31_IOCONFIG_GPIO+LPC31_IOCONFIG_MODE1_OFFSET)
#define LPC31_IOCONFIG_GPIO_MODE1SET        (LPC31_IOCONFIG_GPIO+LPC31_IOCONFIG_MODE1SET_OFFSET)
#define LPC31_IOCONFIG_GPIO_MODE1RESET      (LPC31_IOCONFIG_GPIO+LPC31_IOCONFIG_MODE1RESET_OFFSET)

#define LPC31_IOCONFIG_I2C1_PINS            (LPC31_IOCONFIG_I2C1+LPC31_IOCONFIG_PINS_OFFSET)
#define LPC31_IOCONFIG_I2C1_MODE0           (LPC31_IOCONFIG_I2C1+LPC31_IOCONFIG_MODE0_OFFSET)
#define LPC31_IOCONFIG_I2C1_MODE0SET        (LPC31_IOCONFIG_I2C1+LPC31_IOCONFIG_MODE0SET_OFFSET)
#define LPC31_IOCONFIG_I2C1_MODE0RESET      (LPC31_IOCONFIG_I2C1+LPC31_IOCONFIG_MODE0RESET_OFFSET)
#define LPC31_IOCONFIG_I2C1_MODE1           (LPC31_IOCONFIG_I2C1+LPC31_IOCONFIG_MODE1_OFFSET)
#define LPC31_IOCONFIG_I2C1_MODE1SET        (LPC31_IOCONFIG_I2C1+LPC31_IOCONFIG_MODE1SET_OFFSET)
#define LPC31_IOCONFIG_I2C1_MODE1RESET      (LPC31_IOCONFIG_I2C1+LPC31_IOCONFIG_MODE1RESET_OFFSET)

#define LPC31_IOCONFIG_SPI_PINS             (LPC31_IOCONFIG_SPI+LPC31_IOCONFIG_PINS_OFFSET)
#define LPC31_IOCONFIG_SPI_MODE0            (LPC31_IOCONFIG_SPI+LPC31_IOCONFIG_MODE0_OFFSET)
#define LPC31_IOCONFIG_SPI_MODE0SET         (LPC31_IOCONFIG_SPI+LPC31_IOCONFIG_MODE0SET_OFFSET)
#define LPC31_IOCONFIG_SPI_MODE0RESET       (LPC31_IOCONFIG_SPI+LPC31_IOCONFIG_MODE0RESET_OFFSET)
#define LPC31_IOCONFIG_SPI_MODE1            (LPC31_IOCONFIG_SPI+LPC31_IOCONFIG_MODE1_OFFSET)
#define LPC31_IOCONFIG_SPI_MODE1SET         (LPC31_IOCONFIG_SPI+LPC31_IOCONFIG_MODE1SET_OFFSET)
#define LPC31_IOCONFIG_SPI_MODE1RESET       (LPC31_IOCONFIG_SPI+LPC31_IOCONFIG_MODE1RESET_OFFSET)

#define LPC31_IOCONFIG_NAND_PINS            (LPC31_IOCONFIG_NAND+LPC31_IOCONFIG_PINS_OFFSET)
#define LPC31_IOCONFIG_NAND_MODE0           (LPC31_IOCONFIG_NAND+LPC31_IOCONFIG_MODE0_OFFSET)
#define LPC31_IOCONFIG_NAND_MODE0SET        (LPC31_IOCONFIG_NAND+LPC31_IOCONFIG_MODE0SET_OFFSET)
#define LPC31_IOCONFIG_NAND_MODE0RESET      (LPC31_IOCONFIG_NAND+LPC31_IOCONFIG_MODE0RESET_OFFSET)
#define LPC31_IOCONFIG_NAND_MODE1           (LPC31_IOCONFIG_NAND+LPC31_IOCONFIG_MODE1_OFFSET)
#define LPC31_IOCONFIG_NAND_MODE1SET        (LPC31_IOCONFIG_NAND+LPC31_IOCONFIG_MODE1SET_OFFSET)
#define LPC31_IOCONFIG_NAND_MODE1RESET      (LPC31_IOCONFIG_NAND+LPC31_IOCONFIG_MODE1RESET_OFFSET)

#define LPC31_IOCONFIG_PWM_PINS             (LPC31_IOCONFIG_PWM+LPC31_IOCONFIG_PINS_OFFSET)
#define LPC31_IOCONFIG_PWM_MODE0            (LPC31_IOCONFIG_PWM+LPC31_IOCONFIG_MODE0_OFFSET)
#define LPC31_IOCONFIG_PWM_MODE0SET         (LPC31_IOCONFIG_PWM+LPC31_IOCONFIG_MODE0SET_OFFSET)
#define LPC31_IOCONFIG_PWM_MODE0RESET       (LPC31_IOCONFIG_PWM+LPC31_IOCONFIG_MODE0RESET_OFFSET)
#define LPC31_IOCONFIG_PWM_MODE1            (LPC31_IOCONFIG_PWM+LPC31_IOCONFIG_MODE1_OFFSET)
#define LPC31_IOCONFIG_PWM_MODE1SET         (LPC31_IOCONFIG_PWM+LPC31_IOCONFIG_MODE1SET_OFFSET)
#define LPC31_IOCONFIG_PWM_MODE1RESET       (LPC31_IOCONFIG_PWM+LPC31_IOCONFIG_MODE1RESET_OFFSET)

#define LPC31_IOCONFIG_UART_PINS            (LPC31_IOCONFIG_UART+LPC31_IOCONFIG_PINS_OFFSET)
#define LPC31_IOCONFIG_UART_MODE0           (LPC31_IOCONFIG_UART+LPC31_IOCONFIG_MODE0_OFFSET)
#define LPC31_IOCONFIG_UART_MODE0SET        (LPC31_IOCONFIG_UART+LPC31_IOCONFIG_MODE0SET_OFFSET)
#define LPC31_IOCONFIG_UART_MODE0RESET      (LPC31_IOCONFIG_UART+LPC31_IOCONFIG_MODE0RESET_OFFSET)
#define LPC31_IOCONFIG_UART_MODE1           (LPC31_IOCONFIG_UART+LPC31_IOCONFIG_MODE1_OFFSET)
#define LPC31_IOCONFIG_UART_MODE1SET        (LPC31_IOCONFIG_UART+LPC31_IOCONFIG_MODE1SET_OFFSET)
#define LPC31_IOCONFIG_UART_MODE1RESET      (LPC31_IOCONFIG_UART+LPC31_IOCONFIG_MODE1RESET_OFFSET)

/* IOCONFIG register bit definitions ************************************************************/
/* EBI_MCI register bit definitions (all registers) */


#define IOCONFIG_EBIMCI_MGPIO9                (1 << 0)
#define IOCONFIG_EBIMCI_MGPIO6                (1 << 1)
#define IOCONFIG_EBIMCI_MLCDDB7               (1 << 2)
#define IOCONFIG_EBIMCI_MLCDDB4               (1 << 3)
#define IOCONFIG_EBIMCI_MLCDDB2               (1 << 4)
#define IOCONFIG_EBIMCI_MNANDRYBN0            (1 << 5)
#define IOCONFIG_EBIMCI_MI2STXCLK0            (1 << 6)
#define IOCONFIG_EBIMCI_MI2STXBCK0            (1 << 7)
#define IOCONFIG_EBIMCI_EBIA1CLE              (1 << 8)
#define IOCONFIG_EBIMCI_EBINCASBLOUT0         (1 << 9)
#define IOCONFIG_EBIMCI_MLCDDB0               (1 << 10)
#define IOCONFIG_EBIMCI_EBIDQM0NOE            (1 << 11)
#define IOCONFIG_EBIMCI_MLCDCSB               (1 << 12)
#define IOCONFIG_EBIMCI_MLCDDB1               (1 << 13)
#define IOCONFIG_EBIMCI_MLCDERD               (1 << 14)
#define IOCONFIG_EBIMCI_MLCDRS                (1 << 15)
#define IOCONFIG_EBIMCI_MLCDRWWR              (1 << 16)
#define IOCONFIG_EBIMCI_MLCDDB3               (1 << 17)
#define IOCONFIG_EBIMCI_MLCDDB5               (1 << 18)
#define IOCONFIG_EBIMCI_MLCDDB6               (1 << 19)
#define IOCONFIG_EBIMCI_MLCDDB8               (1 << 20)
#define IOCONFIG_EBIMCI_MLCDDB9               (1 << 21)
#define IOCONFIG_EBIMCI_MLCDDB10              (1 << 22)
#define IOCONFIG_EBIMCI_MLCDDB11              (1 << 23)
#define IOCONFIG_EBIMCI_MLCDDB12              (1 << 24)
#define IOCONFIG_EBIMCI_MLCDDB13              (1 << 25)
#define IOCONFIG_EBIMCI_MLCDDB14              (1 << 26)
#define IOCONFIG_EBIMCI_MLCDDB15              (1 << 27)
#define IOCONFIG_EBIMCI_MGPIO5                (1 << 28)
#define IOCONFIG_EBIMCI_MGPIO7                (1 << 29)
#define IOCONFIG_EBIMCI_MGPIO8                (1 << 30)
#define IOCONFIG_EBIMCI_MGPIO10               (1 << 31)

/* EBI_I2STX_0 register bit definitions (all registers) */

#define IOCONFIG_EBII2STX0_MNANDRYBN1         (1 << 0)
#define IOCONFIG_EBII2STX0_MNANDRYBN2         (1 << 1)
#define IOCONFIG_EBII2STX0_MNANDRYBN3         (1 << 2)
#define IOCONFIG_EBII2STX0_MUARTCTSN          (1 << 3)
#define IOCONFIG_EBII2STX0_MUARTRTSN          (1 << 4)
#define IOCONFIG_EBII2STX0_MI2STXDATA0        (1 << 5)
#define IOCONFIG_EBII2STX0_MI2STXWS0          (1 << 6)
#define IOCONFIG_EBII2STX0_EBINRASBLOUT1      (1 << 7)
#define IOCONFIG_EBII2STX0_EBIA0ALE           (1 << 8)
#define IOCONFIG_EBII2STX0_EBINWE             (1 << 9)

/* CGU register bit definitions (all registers) */

#define IOCONFIG_CGU_SYSCLKO                  (1 << 0)

/* I2SRX_0 register bit definitions (all registers) */

#define IOCONFIG_I2SRX0_BCK                   (1 << 0)
#define IOCONFIG_I2SRX0_DATA                  (1 << 1)
#define IOCONFIG_I2SRX0_WS                    (1 << 2)

/* I2SRX_1 register bit definitions (all registers) */

#define IOCONFIG_I2SRX1_DATA                  (1 << 0)
#define IOCONFIG_I2SRX1_BCK                   (1 << 1)
#define IOCONFIG_I2SRX1_WS                    (1 << 2)

/* I2STX_1 register bit definitions (all registers) */

#define IOCONFIG_I2STX1_DATA                  (1 << 0)
#define IOCONFIG_I2STX1_BCK                   (1 << 1)
#define IOCONFIG_I2STX1_WS                    (1 << 2)
#define IOCONFIG_I2STX1_256FSO                (1 << 3)

/* EBI register bit definitions (all registers) */

#define IOCONFIG_EBI_D9                       (1 << 0)
#define IOCONFIG_EBI_D10                      (1 << 1)
#define IOCONFIG_EBI_D11                      (1 << 2)
#define IOCONFIG_EBI_D12                      (1 << 3)
#define IOCONFIG_EBI_D13                      (1 << 4)
#define IOCONFIG_EBI_D14                      (1 << 5)
#define IOCONFIG_EBI_D4                       (1 << 6)
#define IOCONFIG_EBI_D0                       (1 << 7)
#define IOCONFIG_EBI_D1                       (1 << 8)
#define IOCONFIG_EBI_D2                       (1 << 9)
#define IOCONFIG_EBI_D3                       (1 << 10)
#define IOCONFIG_EBI_D5                       (1 << 11)
#define IOCONFIG_EBI_D6                       (1 << 12)
#define IOCONFIG_EBI_D7                       (1 << 13)
#define IOCONFIG_EBI_D8                       (1 << 14)
#define IOCONFIG_EBI_D15                      (1 << 15)

/* GPIO register bit definitions (all registers) */

#define IOCONFIG_GPIO_GPIO1                   (1 << 0)
#define IOCONFIG_GPIO_GPIO0                   (1 << 1)
#define IOCONFIG_GPIO_GPIO2                   (1 << 2)
#define IOCONFIG_GPIO_GPIO3                   (1 << 3)
#define IOCONFIG_GPIO_GPIO4                   (1 << 4)
#define IOCONFIG_GPIO_GPIO11                  (1 << 5)
#define IOCONFIG_GPIO_GPIO12                  (1 << 6)
#define IOCONFIG_GPIO_GPIO13                  (1 << 7)
#define IOCONFIG_GPIO_GPIO14                  (1 << 8)
#define IOCONFIG_GPIO_GPIO15                  (1 << 9)
#define IOCONFIG_GPIO_GPIO16                  (1 << 10)
#define IOCONFIG_GPIO_GPIO17                  (1 << 11)
#define IOCONFIG_GPIO_GPIO18                  (1 << 12)
#define IOCONFIG_GPIO_GPIO19                  (1 << 13)
#define IOCONFIG_GPIO_GPIO20                  (1 << 14)

/* I2C1 register bit definitions (all registers) */

#define IOCONFIG_I2C1_SDA1                    (1 << 0)
#define IOCONFIG_I2C1_SCL1                    (1 << 1)

/* SPI register bit definitions (all registers) */

#define IOCONFIG_SPI_MISO                     (1 << 0)
#define IOCONFIG_SPI_MOSI                     (1 << 1)
#define IOCONFIG_SPI_CSIN                     (1 << 2)
#define IOCONFIG_SPI_SCK                      (1 << 3)
#define IOCONFIG_SPI_CSOUT0                   (1 << 4)

/* NAND register bit definitions (all registers) */

#define IOCONFIG_NAND_NCS3                    (1 << 0)
#define IOCONFIG_NAND_NCS0                    (1 << 1)
#define IOCONFIG_NAND_NCS1                    (1 << 2)
#define IOCONFIG_NAND_NCS2                    (1 << 3)

/* PWM register bit definitions (all registers) */

#define IOCONFIG_PWM_DATA                     (1 << 0)

/* UART register bit definitions (all registers) */

#define IOCONFIG_UART_RXD                     (1 << 0)
#define IOCONFIG_UART_TXD                     (1 << 1)

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_IOCONFIG_H */
