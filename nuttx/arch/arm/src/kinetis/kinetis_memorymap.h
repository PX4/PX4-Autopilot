/************************************************************************************
 * arch/arm/src/kinetis/kinetis_memorymap.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_MEMORYMAP_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Memory Map ***********************************************************************/
/* K40 Family
 *
 * The memory map for the following parts is defined in Freescale document
 * K40P144M100SF2RM
 */

#if defined(CONFIG_ARCH_CHIP_MK40X128VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X128VMD100) || \
    defined(CONFIG_ARCH_CHIP_MK40X256VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X256VMD100) || \
    defined(CONFIG_ARCH_CHIP_MK40N512VLQ100) || defined(CONFIG_ARCH_CHIP_MK40N512VMD100)

# define KINETIS_FLASH_BASE     0x00000000 /* –0x0fffffff Program flash and read-
                                            *             only data (Includes exception
                                            *             vectors in first 1024 bytes) */
# if !defined(KINETIS_FLEXMEM_SIZE)
#  define KINETIS_FLEXNVM_BASE  0x10000000 /* –0x13ffffff FlexNVM */
#  define KINETIS_FLEXRAM_BASE  0x14000000 /* –0x17ffffff FlexRAM */
# endif
# define KINETIS_SRAML_BASE     0x18000000 /* –0x1fffffff SRAM_L: Lower SRAM
                                            *             (ICODE/DCODE) */
# define KINETIS_SRAMU_BASE     0x20000000 /* –0x200fffff SRAM_U: Upper SRAM bitband
                                            *             region */
                             /* 0x20100000  * –0x21ffffff Reserved */
# define KINETIS_SALIAS_BASE    0x22000000 /* –0x23ffffff Aliased to SRAM_U bitband */
                             /* 0x24000000  * –0x3fffffff Reserved */
# define KINETIS_BRIDGE0_BASE   0x40000000 /* –0x4007ffff Bitband region for peripheral
                                            *             bridge 0 (AIPS-Lite0) */
# define KINETIS_BRIDGE1_BASE   0x40080000 /* –0x400fffff Bitband region for peripheral
                                            *             bridge 1 (AIPS-Lite1) */
# define KINETIS_GPIOBB_BASE    0x400ff000 /* –0x400fffff Bitband region for general
                                            *             purpose input/output (GPIO) */
                             /* 0x40100000  * –0x41ffffff Reserved */
# define KINETIS_PALIAS_BASE    0x42000000 /* –0x43ffffff Aliased to peripheral bridge
                                            *             (AIPS-Lite) and general purpose
                                            *             input/output (GPIO) bitband */
                             /* 0x44000000  * –0x5fffffff Reserved */
# define KINETIS_FLEXBUS_WBBASE 0x60000000 /* –0x7fffffff FlexBus (External Memory -
                                            *             Write-back) */
# define KINETIS_FLEXBUS_WTBASE 0x80000000 /* –0x9fffffff FlexBus (External Memory -
                                            *             Write-through) */
# define KINETIS_FLEXBUS_NXBASE 0xa0000000 /* –0xdfffffff FlexBus (External Memory -
                                            *             Non-executable) */
# define KINETIS_PERIPH_BASE    0xe0000000 /* –0xe00fffff Private peripherals */
                             /* 0xe0100000  * –0xffffffff Reserved */

/* Peripheral Bridge 0 Memory Map ***************************************************/

# define KINETIS_AIPS0_BASE     0x40000000 /* Peripheral bridge 0 (AIPS-Lite 0) */
# define KINETIS_XBAR_BASE      0x40004000 /* Crossbar switch */
# define KINETIS_DMAC_BASE      0x40008000 /* DMA controller */
# define KINETIS_DMADESC_BASE   0x40009000 /* DMA controller transfer control descriptors */
# define KINETIS_FLEXBUSC_BASE  0x4000c000 /* FlexBus controller */
# define KINETIS_MPU_BASE       0x4000d000 /* MPU */
# define KINETIS_FMC_BASE       0x4001f000 /* Flash memory controller */
# define KINETIS_FTFL_BASE      0x40020000 /* Flash memory */
# define KINETIS_DMAMUX0_BASE   0x40021000 /* DMA channel mutiplexer 0 */
# define KINETIS_CAN0_BASE      0x40024000 /* FlexCAN 0 */
# define KINETIS_SPI0_BASE      0x4002c000 /* SPI 0 */
# define KINETIS_SPI1_BASE      0x4002d000 /* SPI 1 */
# define KINETIS_I2S0_BASE      0x4002f000 /* I2S 0 */
# define KINETIS_CRC_BASE       0x40032000 /* CRC */
# define KINETIS_USBDCD_BASE    0x40035000 /* USB DCD */
# define KINETIS_PDB0_BASE      0x40036000 /* Programmable delay block */
# define KINETIS_PIT_BASE       0x40037000 /* Periodic interrupt timers (PIT) */
# define KINETIS_FTM0_BASE      0x40038000 /* FlexTimer 0 */
# define KINETIS_FTM1_BASE      0x40039000 /* FlexTimer 1 */
# define KINETIS_ADC0_BASE      0x4003b000 /* Analog-to-digital converter (ADC) 0 */
# define KINETIS_RTC_BASE       0x4003d000 /* Real time clock */
# define KINETIS_VBATR_BASE     0x4003e000 /* VBAT register file */
# define KINETIS_LPTMR_BASE     0x40040000 /* Low power timer */
# define KINETIS_SYSR_BASE      0x40041000 /* System register file */
# define KINETIS_DRYICE_BASE    0x40042000 /* DryIce */
# define KINETIS_DRYICESS_BASE  0x40043000 /* DryIce secure storage */
# define KINETIS_TSI0_BASE      0x40045000 /* Touch sense interface */
# define KINETIS_SIMLP_BASE     0x40047000 /* SIM low-power logic */
# define KINETIS_SIM_BASE       0x40048000 /* System integration module (SIM) */
# define KINETIS_PORT_BASE(n)   (0x40049000 + ((n) << 12))
# define KINETIS_PORTA_BASE     0x40049000 /* Port A multiplexing control */
# define KINETIS_PORTB_BASE     0x4004a000 /* Port B multiplexing control */
# define KINETIS_PORTC_BASE     0x4004b000 /* Port C multiplexing control */
# define KINETIS_PORTD_BASE     0x4004c000 /* Port D multiplexing control */
# define KINETIS_PORTE_BASE     0x4004d000 /* Port E multiplexing control */
# define KINETIS_WDOG_BASE      0x40052000 /* Software watchdog */
# define KINETIS_EWM_BASE       0x40061000 /* External watchdog */
# define KINETIS_CMT_BASE       0x40062000 /* Carrier modulator timer (CMT) */
# define KINETIS_MCG_BASE       0x40064000 /* Multi-purpose Clock Generator (MCG) */
# define KINETIS_OSC_BASE       0x40065000 /* System oscillator (OSC) */
# define KINETIS_I2C0_BASE      0x40066000 /* I2C 0 */
# define KINETIS_I2C1_BASE      0x40067000 /* I2C 1 */
# define KINETIS_UART0_BASE     0x4006a000 /* UART0 */
# define KINETIS_UART1_BASE     0x4006b000 /* UART1 */
# define KINETIS_UART2_BASE     0x4006c000 /* UART2 */
# define KINETIS_UART3_BASE     0x4006d000 /* UART3 */
# define KINETIS_USB0_BASE      0x40072000 /* USB OTG FS/LS */
# define KINETIS_CMP_BASE       0x40073000 /* Analog comparator (CMP) / 6-bit digital-to-analog converter (DAC) */
# define KINETIS_VREF_BASE      0x40074000 /* Voltage reference (VREF) */
# define KINETIS_LLWU_BASE      0x4007c000 /* Low-leakage wakeup unit (LLWU) */
# define KINETIS_PMC_BASE       0x4007d000 /* Power management controller (PMC) */
# define KINETIS_SMC_BASE       0x4007e000 /* System Mode controller (SMC) */

/* Peripheral Bridge 1 Memory Map ***************************************************/

# define KINETIS_AIPS1_BASE     0x40080000 /* Peripheral bridge 1 (AIPS-Lite 1) */
# define KINETIS_CAN1_BASE      0x400a4000 /* FlexCAN 1 */
# define KINETIS_SPI2_BASE      0x400ac000 /* SPI 2 */
# define KINETIS_SDHC_BASE      0x400b1000 /* SDHC */
# define KINETIS_FTM2_BASE      0x400b8000 /* FlexTimer 2 */
# define KINETIS_ADC1_BASE      0x400bb000 /* Analog-to-digital converter (ADC) 1 */
# define KINETIS_SLCD_BASE      0x400be000 /* Segment LCD */
# define KINETIS_DAC0_BASE      0x400cc000 /* 12-bit digital-to-analog converter (DAC) 0 */
# define KINETIS_DAC1_BASE      0x400cd000 /* 12-bit digital-to-analog converter (DAC) 1 */
# define KINETIS_UART4_BASE     0x400ea000 /* UART4 */
# define KINETIS_UART5_BASE     0x400eb000 /* UART5 */
# define KINETIS_XBARSS_BASE    0x400ff000 /* Not an AIPS-Lite slot. The 32-bit general
                                            * purpose input/output module that shares the
                                            * crossbar switch slave port with the AIPS-Lite
                                            * is accessed at this address. */
# define KINETIS_GPIO_BASE(n)   (0x400ff000 + ((n) << 6))
# define KINETIS_GPIOA_BASE     0x400ff000 /* GPIO PORTA registers */
# define KINETIS_GPIOB_BASE     0x400ff040 /* GPIO PORTB registers */
# define KINETIS_GPIOC_BASE     0x400ff080 /* GPIO PORTC registers */
# define KINETIS_GPIOD_BASE     0x400ff0c0 /* GPIO PORTD registers */
# define KINETIS_GPIOE_BASE     0x400ff100 /* GPIO PORTE registers */

/* Private Peripheral Bus (PPB) Memory Map ******************************************/

# define KINETIS_ITM_BASE       0xe0000000 /* Instrumentation Trace Macrocell (ITM) */
# define KINETIS_DWT_BASE       0xe0001000 /* Data Watchpoint and Trace (DWT) */
# define KINETIS_FPB_BASE       0xe0002000 /* Flash Patch and Breakpoint (FPB) */
# define KINETIS_SCS_BASE       0xe000e000 /* System Control Space (SCS) (for NVIC) */
# define KINETIS_TPIU_BASE      0xe0040000 /* Trace Port Interface Unit (TPIU) */
# define KINETIS_ETM_BASE       0xe0041000 /* Embedded Trace Macrocell (ETM) */
# define KINETIS_ETB_BASE       0xe0042000 /* Embedded Trace Buffer (ETB) */
# define KINETIS_TFUN_BASE      0xe0043000 /* Embedded Trace Funnel */
# define KINETIS_MCM_BASE       0xe0080000 /* Miscellaneous Control Module (including ETB Almost Full) */
# define KINETIS_ROMTAB_BASE    0xe00ff000 /* ROM Table - allows auto-detection of debug components */

/* K60 Family 
 *
 * The memory map for the following parts is defined in Freescale document
 * K60P144M100SF2RM
 */

#elif defined(CONFIG_ARCH_CHIP_MK60N256VLQ100) || defined(CONFIG_ARCH_CHIP_MK60X256VLQ100) || \
      defined(CONFIG_ARCH_CHIP_MK60N512VLQ100) || defined(CONFIG_ARCH_CHIP_MK60N256VMD100) || \
      defined(CONFIG_ARCH_CHIP_MK60X256VMD100) || defined(CONFIG_ARCH_CHIP_MK60N512VMD100)

# define KINETIS_FLASH_BASE     0x00000000 /* –0x0fffffff Program flash and read-
                                            *             only data (Includes exception
                                            *             vectors in first 1024 bytes) */
# if !defined(KINETIS_FLEXMEM_SIZE)
#  define KINETIS_FLEXNVM_BASE  0x10000000 /* –0x13ffffff FlexNVM */
#  define KINETIS_FLEXRAM_BASE  0x14000000 /* –0x17ffffff FlexRAM */
# endif
# define KINETIS_SRAML_BASE     0x18000000 /* –0x1fffffff SRAM_L: Lower SRAM
                                            *             (ICODE/DCODE) */
# define KINETIS_SRAMU_BASE     0x20000000 /* –0x200fffff SRAM_U: Upper SRAM bitband
                                            *             region */
                             /* 0x20100000  * –0x21ffffff Reserved */
# define KINETIS_SALIAS_BASE    0x22000000 /* –0x23ffffff Aliased to SRAM_U bitband */
                             /* 0x24000000  * –0x3fffffff Reserved */
# define KINETIS_BRIDGE0_BASE   0x40000000 /* –0x4007ffff Bitband region for peripheral
                                            *             bridge 0 (AIPS-Lite0) */
# define KINETIS_BRIDGE1_BASE   0x40080000 /* –0x400fffff Bitband region for peripheral
                                            *             bridge 1 (AIPS-Lite1) */
# define KINETIS_GPIOBB_BASE    0x400ff000 /* –0x400fffff Bitband region for general
                                            *             purpose input/output (GPIO) */
                             /* 0x40100000  * –0x41ffffff Reserved */
# define KINETIS_PALIAS_BASE    0x42000000 /* –0x43ffffff Aliased to peripheral bridge
                                            *             (AIPS-Lite) and general purpose
                                            *             input/output (GPIO) bitband */
                             /* 0x44000000  * –0x5fffffff Reserved */
# define KINETIS_FLEXBUS_BASE   0x60000000 /* –0x7fffffff FlexBus */
# define KINETIS_PERIPH_BASE    0xe0000000 /* –0xe00fffff Private peripherals */
                             /* 0xe0100000  * –0xffffffff Reserved */

/* Peripheral Bridge 0 Memory Map ***************************************************/

# define KINETIS_AIPS0_BASE     0x40000000 /* Peripheral bridge 0 (AIPS-Lite 0) */
# define KINETIS_XBAR_BASE      0x40004000 /* Crossbar switch */
# define KINETIS_DMAC_BASE      0x40008000 /* DMA controller */
# define KINETIS_DMADESC_BASE   0x40009000 /* DMA controller transfer control descriptors */
# define KINETIS_FLEXBUSC_BASE  0x4000c000 /* FlexBus controller */
# define KINETIS_MPU_BASE       0x4000d000 /* MPU */
# define KINETIS_FMC_BASE       0x4001f000 /* Flash memory controller */
# define KINETIS_FTFL_BASE      0x40020000 /* Flash memory */
# define KINETIS_DMAMUX0_BASE   0x40021000 /* DMA channel mutiplexer 0 */
# define KINETIS_CAN0_BASE      0x40024000 /* FlexCAN 0 */
# define KINETIS_SPI0_BASE      0x4002c000 /* DSPI 0 */
# define KINETIS_SPI1_BASE      0x4002d000 /* DSPI 1 */
# define KINETIS_I2S0_BASE      0x4002f000 /* I2S 0 */
# define KINETIS_CRC_BASE       0x40032000 /* CRC */
# define KINETIS_USBDCD_BASE    0x40035000 /* USB DCD */
# define KINETIS_PDB0_BASE      0x40036000 /* Programmable delay block */
# define KINETIS_PIT_BASE       0x40037000 /* Periodic interrupt timers (PIT) */
# define KINETIS_FTM0_BASE      0x40038000 /* FlexTimer (FTM) 0 */
# define KINETIS_FTM1_BASE      0x40039000 /* FlexTimer (FTM) 1 */
# define KINETIS_ADC0_BASE      0x4003b000 /* Analog-to-digital converter (ADC) 0 */
# define KINETIS_RTC_BASE       0x4003d000 /* Real time clock */
# define KINETIS_VBATR_BASE     0x4003e000 /* VBAT register file */
# define KINETIS_LPTMR_BASE     0x40040000 /* Low power timer */
# define KINETIS_SYSR_BASE      0x40041000 /* System register file */
# define KINETIS_DRYICE_BASE    0x40042000 /* DryIce */
# define KINETIS_DRYICESS_BASE  0x40043000 /* DryIce secure storage */
# define KINETIS_TSI0_BASE      0x40045000 /* Touch sense interface */
# define KINETIS_SIMLP_BASE     0x40047000 /* SIM low-power logic */
# define KINETIS_SIM_BASE       0x40048000 /* System integration module (SIM) */
# define KINETIS_PORT_BASE(n)   (0x40049000 + ((n) << 12))
# define KINETIS_PORTA_BASE     0x40049000 /* Port A multiplexing control */
# define KINETIS_PORTB_BASE     0x4004a000 /* Port B multiplexing control */
# define KINETIS_PORTC_BASE     0x4004b000 /* Port C multiplexing control */
# define KINETIS_PORTD_BASE     0x4004c000 /* Port D multiplexing control */
# define KINETIS_PORTE_BASE     0x4004d000 /* Port E multiplexing control */
# define KINETIS_WDOG_BASE      0x40052000 /* Software watchdog */
# define KINETIS_EWM_BASE       0x40061000 /* External watchdog */
# define KINETIS_CMT_BASE       0x40062000 /* Carrier modulator timer (CMT) */
# define KINETIS_MCG_BASE       0x40064000 /* Multi-purpose Clock Generator (MCG) */
# define KINETIS_OSC_BASE       0x40065000 /* System oscillator (XOSC) */
# define KINETIS_I2C0_BASE      0x40066000 /* I2C 0 */
# define KINETIS_I2C1_BASE      0x40067000 /* I2C 1 */
# define KINETIS_UART0_BASE     0x4006a000 /* UART0 */
# define KINETIS_UART1_BASE     0x4006b000 /* UART1 */
# define KINETIS_UART2_BASE     0x4006c000 /* UART2 */
# define KINETIS_UART3_BASE     0x4006d000 /* UART3 */
# define KINETIS_USB0_BASE      0x40072000 /* USB OTG FS/LS */
# define KINETIS_CMP_BASE       0x40073000 /* Analog comparator (CMP) / 6-bit digital-to-analog converter (DAC) */
# define KINETIS_VREF_BASE      0x40074000 /* Voltage reference (VREF) */
# define KINETIS_LLWU_BASE      0x4007c000 /* Low-leakage wakeup unit (LLWU) */
# define KINETIS_PMC_BASE       0x4007d000 /* Power management controller (PMC) */
# define KINETIS_SMC_BASE       0x4007e000 /* System Mode controller (SMC) */

/* Peripheral Bridge 1 Memory Map ***************************************************/

# define KINETIS_AIPS1_BASE     0x40080000 /* Peripheral bridge 1 (AIPS-Lite 1) */
# define KINETIS_RNGB_BASE      0x400a0000 /* Random number generator (RNGB) */
# define KINETIS_CAN1_BASE      0x400a4000 /* FlexCAN 1 */
# define KINETIS_SPI2_BASE      0x400ac000 /* DSPI 2 */
# define KINETIS_SDHC_BASE      0x400b1000 /* SDHC */
# define KINETIS_FTM2_BASE      0x400b8000 /* FlexTimer 2 */
# define KINETIS_ADC1_BASE      0x400bb000 /* Analog-to-digital converter (ADC) 1 */
# define KINETIS_EMAC_BASE      0x400c0000 /* Ethernet MAC and IEEE 1588 timers */
# define KINETIS_DAC0_BASE      0x400cc000 /* 12-bit digital-to-analog converter (DAC) 0 */
# define KINETIS_DAC1_BASE      0x400cd000 /* 12-bit digital-to-analog converter (DAC) 1 */
# define KINETIS_UART4_BASE     0x400ea000 /* UART4 */
# define KINETIS_UART5_BASE     0x400eb000 /* UART5 */
# define KINETIS_XBARSS_BASE    0x400ff000 /* Not an AIPS-Lite slot. The 32-bit general
                                            * purpose input/output module that shares the
                                            * crossbar switch slave port with the AIPS-Lite
                                            * is accessed at this address. */
# define KINETIS_GPIO_BASE(n)   (0x400ff000 + ((n) << 6))
# define KINETIS_GPIOA_BASE     0x400ff000 /* GPIO PORTA registers */
# define KINETIS_GPIOB_BASE     0x400ff040 /* GPIO PORTB registers */
# define KINETIS_GPIOC_BASE     0x400ff080 /* GPIO PORTC registers */
# define KINETIS_GPIOD_BASE     0x400ff0c0 /* GPIO PORTD registers */
# define KINETIS_GPIOE_BASE     0x400ff100 /* GPIO PORTE registers */

/* Private Peripheral Bus (PPB) Memory Map ******************************************/

# define KINETIS_ITM_BASE       0xe0000000 /* Instrumentation Trace Macrocell (ITM) */
# define KINETIS_DWT_BASE       0xe0001000 /* Data Watchpoint and Trace (DWT) */
# define KINETIS_FPB_BASE       0xe0002000 /* Flash Patch and Breakpoint (FPB) */
# define KINETIS_SCS_BASE       0xe000e000 /* System Control Space (SCS) (for NVIC) */
# define KINETIS_TPIU_BASE      0xe0040000 /* Trace Port Interface Unit (TPIU) */
# define KINETIS_ETM_BASE       0xe0041000 /* Embedded Trace Macrocell (ETM) */
# define KINETIS_ETB_BASE       0xe0042000 /* Embedded Trace Buffer (ETB) */
# define KINETIS_TFUN_BASE      0xe0043000 /* Embedded Trace Funnel */
# define KINETIS_MCM_BASE       0xe0080000 /* Miscellaneous Control Module (including ETB Almost Full) */
# define KINETIS_MMCAU_BASE     0xe0081000 /* Memory Mapped Cryptographic Acceleration Unit (MMCAU) */
# define KINETIS_ROMTAB_BASE    0xe00ff000 /* ROM Table - allows auto-detection of debug components */

#else
  /* The memory map for other parts is defined in other documents and may or may not
   * be the same as above (the family members are all very similar)  This error just
   * means that you have to look at the document and determine for yourself if the
   * memory map is the same.
   */

#  error "No memory map for this Kinetis part"
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_MEMORYMAP_H */
