/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

/**
 * @file board_config.h
 *
 * PX4FLOWv2 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#endif

/* High-resolution timer
 */
#define HRT_TIMER               8       /* use timer1 for the HRT */
#define HRT_TIMER_CHANNEL       8       /* use capture/compare channel */

/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PA[00] PA0/TIM2_CH1/TIM5_CH1/USART2_CTS/UART4_TX         23       TIM5_CH1
 *  PA[01] PA1/TIM2_CH2/TIM5_CH2/USART2_RTS/UART4_RX         24       TIM5_CH2 NC.
 *  PA[02] PA2/TIM2_CH3/TIM5_CH3/TIM9_CH1/USART2_TX          25       TIM5_CH3_EXPOSURE
 *  PA[03] PA3/TIM2_CH4/TIM5_CH4/TIM9_CH2/USART2_RX          26       TIM5_CH4_STANDBY
 *  PA[04] PA4/SPI1_NSS/SPI3_NSS/DCMI_HSYNC/LCD_VSYNC        29       DCMI_HSYNC
 *  PA[05] PA5/TIM2_CH1/TIM2_ETR/TIM8_CH1N/SPI1_SCK          30       CAM_NRESET
 *  PA[06] PA6/TIM3_CH1/SPI1_MISO/TIM13_CH1/DCMI_PIXCLK      31       DCMI_PIXCK
 *  PA[07] PA7/TIM1,8_CH1N/TIM3_CH2/SPI1_MOSI/TIM14_CH1      32       SPI1_MOSI---
 *  PA[08] PA8/MCO1/TIM1_CH1/I2C3_SCL/USART1_CK/LCD_R6/      67       MPU_CS
 *  PA[09] PA9/TIM1_CH2/USART1_TX/DCMI_D0/OTG_FS_VBUS        68       VBUS
 *  PA[10] PA10/TIM1_CH3/USART1_RX/OTG_FS_ID/DCMI_D1         69       OTG_FS_ID
 *  PA[11] PA11/TIM1_CH4/USART1_CTS/CAN1_RX/LCD_R4/OTG_FS_DM 70       OTG_FS_N
 *  PA[12] PA12/TIM1_ETR/USART1_RTS/CAN1_TX/LCD_R5/OTG_FS_DP 71       OTG_FS_P
 *  PA[13] PA13/JTMS-SWDIO                                   72       JTMS-SWDIO
 *  PA[14] PA14/JTCK-SWCLK                                   76       JTCK-SWCLK
 *  PA[15] PA15/JTDI/TIM2_CH1/TIM2_ETR/SPI1_NSS              77       JTDI
 *
 */
/*
 *  PB[00] PB0/TIM1_CH2N/TIM3_CH3/TIM8_CH2N/OTG_HS_ULPI_D1   35       PRES_DRDY---
 *  PB[01] PB1/TIM1,8_CH3N/TIM3_CH4/LCD_R6/OTG_HS_ULPI_D2    36       MAG_DRDY---
 *  PB[02] PB2/BOOT1                                         37       PB2-BOOT1
 *  PB[03] PB3/JTDO/TRACESWO/TIM2_CH2/SPI1_SCK/SPI3_SCK      89       JTDO
 *  PB[04] PB4/NJTRST/TIM3_CH1/SPI1_MISO/SPI3_MISO           90       JTRST
 *  PB[05] PB5/TIM3_CH2/SPI1_MOSI/SPI3_MOSI/CAN2_RX/DCMI_D10 91       PCM1_D10 N.C
 *  PB[06] PB6/TIM4_CH1/I2C1_SCL/USART1_TX/CAN2_TX/DCMI_D5   92       PCMI_D5
 *  PB[07] PB7/TIM4_CH2/I2C1_SDA/USART1_RX/FMC_NL/DCMI_VSYNC 93       DCMI_VSYNC
 *  PB[08] PB8/TIM4_CH3/TIM10_CH1/I2C1_SCL/CAN1_RX/DCMI_D6   95       J2C1_SCL
 *  PB[09] PB9/TIM2_CH3/I2C2_SCL/SPI2_SCK/I2S2_CK/USART3_TX/ 96       J2C1_SDA
 *  PB[10] PB10/TIM2_CH3/I2C2_SCL/SPI2_SCK/I2S2_CK/USART3_TX 47       J2C2_SCL
 *  PB[11] PB11/TIM2_CH4/I2C2_SDA/USART3_RX/OTG_HS_ULPI_D4   48       J2C2_SDA
 *  PB[12] PB12/TIM1_BKIN/I2C2_SMBA/SPI2_NSS/CAN2_RX         51       SPI2_NSS
 *  PB[13] PB13/TIM1_CH1N/SPI2_SCK/USART3_CTS/CAN2_TX        52       SPI2_SCK
 *  PB[14] PB14/TIM1,8_CH2N/SPI2_MISO/USART3_RTS/TIM12_CH1   53       SPI2_MISO
 *  PB[15] PB15/TIM1_CH3N/TIM8_CH3N/SPI2_MOSI/TIM12_CH2      54       SPI2_MOSI


 *  PC[00] PC0/OTG_HS_ULPI_STP/FMC_SDNWE                     15       ADC123_IN10
 *  PC[01] PC1/ETH_MDC                                       16       ADC123_IN11
 *  PC[02] PC2/SPI2_MISO/I2S2ext_SD                          17       ADC123_IN12
 *  PC[03] PC3/SPI2_MOSI/I2S2_SD                             18       ADC123_IN13
 *  PC[04] PC4/ETH_MII_RXD0/ETH_RMII_RXD0                    33       ADC123_IN4
 *  PC[05] PC5/ETH_MII_RXD1/ETH_RMII_RXD1                    34       ADC123_IN5
 *  PC[06] PC6/TIM3_CH1/TIM8_CH1/USART6_TX/SDIO_D6/DCMI_D0   63       DCMI_O0
 *  PC[07] PC7/TIM3_CH2/TIM8_CH2/USART6_RX/SDIO_D7/DCMI_D1   64       DCMI_O1
 *  PC[08] PC8/TIM3_CH3/TIM8_CH3/USART6_CK/SDIO_D0/DCMI_D2   65       TIM8_CH3_MASTERCLOCK
 *  PC[09] PC9/MCO2/TIM3_CH3/TIM8_CH4/SDIO_D1/DCMI_D3        66       TIM8_CH4_LED_OUT
 *  PC[10] PC10/SPI3_SCK/USART3_TX/UART4_TX/SDIO_D2/DCMI_D8  78       DCMI_D8
 *  PC[11] PC11/SPI3_MISO/USART3_RX/UART4_RX/SDIO_D3DCMI_D4  78       UART4_RX
 *  PC[12] PC12/SPI3_MOSI/USART3_CK/UART5_TX/SDIO_CK/DCMI_D9 80       DCMI_09
 *  PC[13] PC13                                               7       ADDR0 NC
 *  PC[14] PC14                                               8       ADDR1 NC
 *  PC[15] PC15                                               9       ADDR2 NC

 *  PD[00] PD0/CAN1_RX/FMC_D2                                81       CAN1_RX
 *  PD[01] PD1/CAN1_TX/FMC_D3                                82       CAN1_TX
 *  PD[02] PD2/TIM3_ETR/UARTS_RX/SDIO_CMD/DCMI_D11           83       DCMI_D11_UARTS_RX NC
 *  PD[03] PD3/SPI2_SCK/USART2_CTS/FMC_CLK/DCMI_D5/LCD_G7    84       UART5_CTS
 *  PD[04] PD4/USART2_RTS/FMC_NOE                            85       USART2_RTS
 *  PD[05] PD5/USART2_TX/FMC_NWE                             86       USART2_TX
 *  PD[06] PD6/SPI3_MOSI/SAI1_SD_A/USART2_RX/DCMI_D10/LCD_B2 87       USART2_RX
 *  PD[07] PD7/USART2_CK/FMC_NE1/FMC_NCE2                    88       USART2_CK NC
 *  PD[08] PD8/USART3_TX/FMC_D13                             55       USART3_TX
 *  PD[09] PD9/USART3_RX/FMC_D14                             56       USART3_RX
 *  PD[10] PD10/USART3_CK/FMC_D15/LCD_B3                     57       USART3_CK
 *  PD[11] PD11/USART3_CTS/FMC_A16                           58       USART3_CTS
 *  PD[12] PD12/TIM4_CH1/USART3_RTS/FMC_A17                  59       USART3_RTS
 *  PD[13] PD13/TIM4_CH2/FMC_A18                             60       PWM
 *  PD[14] PD14/TIM4_CH3/FMC_D0                              61       POWER
 *  PD[15] PD15/TIM4_CH4/FMC_D1                              62       PWM_EN

 *  PE[00] PE0/TIM4_ETR/UART8_RX/FMC_NBL0/DCMI_D2            97       DCMI_D2
 *  PE[01] PE1/UART8_Tx/FMC_NBL1/DCMI_D3                     98       DCMI_D3
 *  PE[02] PE2/TRACECLK/SPI4_SCK/SAI1_MCLK_A/FMC_A23          1       LED_AMBER
 *  PE[03] PE3/TRACED0/SAI1_SD_B/FMC_A19                      2       LED_BLUE
 *  PE[04] PE4/TRACED1/SPI4_NSS/SAI1_FS_A/DCMI_D4/LCD_B0      3       DCMI_D4
 *  PE[05] PE5/TRACED2/TIM9_CH1/SPI4_MISO/DCMI_D6/LCD_G0      4       DCMI_D6
 *  PE[06] PE6/TRACED3/TIM9_CH2/SPI4_MOSI/DCMI_D7/LCD_G1      5       DCMI_D7
 *  PE[07] PE7/TIM1_ETR/UART7_Rx/FMC_D4                      38       LED RED
 *  PE[08] PE8/TIM1_CH1N/UART7_Tx/FMC_D5                     39       US_FN1
 *  PE[09] PE9/TIM1_CH1/FMC_D6                               40       RED_LED
 *  PE[10] PE10/TIM1_CH2N/FMC_D7                             41       US_EN2
 *  PE[11] PE11/TIM1_CH2/8PI4_NSS/FMC_D8/LCD_G3              42       GREEN_LED
 *  PE[12] PE12/TIM1_CH3N/SPI4_SCK/FMC_D9/LCD_B4             43       UDD_3U3
 *  PE[13] PE13/TIM1_CH3/SPI4_MISO/FMC_D10/LCD_DE            44       BLUE_LED
 *  PE[14] PE14/TIM1_CH4/SPI4_MOSI/FMC_D11/LCD_CLK           45       GND
 *  PE[15] PE15/TIM1_BKIN/FMC_D12/LCD_R7                     46       GND
*/




/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PB0
 */

/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PC0
 */

/* CAN ************************************************************************ *
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ------------------------
 *
 *  PD[00] PD0/TIM4_CH3/I2C1_SCL/CAN1_RX                     81       CAN1_RX
 *  PD[01] PD1/TIM4_CH4/I2C1_SDA/CAN1_TX                     82       CAN1_TX
 *
 */

/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 */

#define GPIO_WAIT_GETNODEINFO   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN13)


/* LEDs ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *  PE[09] PE9/TIM1_CH1/FMC_D6                               40       RED_LED
 *  PE[11] PE11/TIM1_CH2/8PI4_NSS/FMC_D8/LCD_G3              42       GREEN_LED
 *  PE[13] PE13/TIM1_CH3/SPI4_MISO/FMC_D10/LCD_DE            44       BLUE_LED
 *
 */

#define GPIO_LED1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
			 GPIO_PORTE | GPIO_PIN9 | GPIO_OUTPUT_SET)
#define GPIO_LED2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
			 GPIO_PORTE | GPIO_PIN11 | GPIO_OUTPUT_SET)
#define GPIO_LED3       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
			 GPIO_PORTE | GPIO_PIN13 | GPIO_OUTPUT_SET)

#define GPIO_TIM1_CH1OUT        GPIO_TIM1_CH1OUT_2      /* Red */
#define GPIO_TIM1_CH2OUT        GPIO_TIM1_CH2OUT_2      /* Green */
#define GPIO_TIM1_CH3OUT        GPIO_TIM1_CH3OUT_2      /* Blue */

#define MIS_WIRED_V20
/* todo:remove this */
#if defined(MIS_WIRED_V20)
#pragma message "Built with MIS_WIRED_V20 set Fix this when New HW comes out"
/* The first proto (PCB) had red and Blow swapped */
# define GPIO_LED_RED     GPIO_LED3
# define GPIO_LED_GREEN   GPIO_LED2
# define GPIO_LED_BLUE    GPIO_LED1
#else
/* This is the corrected wiring */
# define GPIO_LED_RED     GPIO_LED1
# define GPIO_LED_GREEN   GPIO_LED2
# define GPIO_LED_BLUE    GPIO_LED3
#endif

#define BOARD_NAME "PX4FLOW_V2"

__BEGIN_DECLS

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ************************************************************************************/

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || \
    defined(CONFIG_STM32_SPI3)
void weak_function board_spiinitialize(void);
#endif

/************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins.
 *
 ************************************************************************************/

void stm32_usbinitialize(void);

/************************************************************************************
 * Name: stm32_usb_set_pwr_callback()
 *
 * Description:
 *   Called to setup set a call back for USB power state changes.
 *
 * Inputs:
 *   pwr_changed_handler: An interrupt handler that will be called on VBUS power
 *   state changes.
 *
 ************************************************************************************/

void stm32_usb_set_pwr_callback(xcpt_t pwr_changed_handler);

/****************************************************************************
 * Name: stm32_led_initialize
 *
 * Description:
 *   This functions is called very early in initialization to perform board-
 *   specific initialization of LED-related resources.  This includes such
 *   things as, for example, configure GPIO pins to drive the LEDs and also
 *   putting the LEDs in their correct initial state.
 *
 *   NOTE: In most architectures, LED initialization() is called from
 *   board-specific initialization and should, therefore, have the name
 *   <arch>_led_intialize().  But there are a few architectures where the
 *   LED initialization function is still called from common chip
 *   architecture logic.  This interface is not, however, a common board
 *   interface in any event and the name board_autoled_initialization is
 *   deprecated.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_initialize(void);
#endif

/************************************************************************************
 * Name: stm32_can_initialize
 *
 * Description:
 *   Called at application startup time to initialize the CAN functionality.
 *
 ************************************************************************************/

#if defined(CONFIG_CAN) && (defined(CONFIG_STM32_CAN1) || defined(CONFIG_STM32_CAN2))
int board_can_initialize(void);
#endif

/************************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   Called at application startup time to initialize the Buttons functionality.
 *
 ************************************************************************************/

#if defined(CONFIG_ARCH_BUTTONS)
void board_button_initialize(void);
#endif

/****************************************************************************
 * Name: usbmsc_archinitialize
 *
 * Description:
 *   Called from the application system/usbmc or the boards_nsh if the
 *   application is not included.
 *   Perform architecture specific initialization.  This function must
 *   configure the block device to export via USB.  This function must be
 *   provided by architecture-specific logic in order to use this add-on.
 *
 ****************************************************************************/

#if !defined(CONFIG_NSH_BUILTIN_APPS) && !defined(CONFIG_SYSTEM_USBMSC)
int usbmsc_archinitialize(void);
#endif

/****************************************************************************
 * Name: composite_archinitialize
 *
 * Description:
 *   Called from the application system/composite or the boards_nsh if the
 *   application is not included.
 *   Perform architecture specific initialization.  This function must
 *   configure the block device to export via USB.  This function must be
 *   provided by architecture-specific logic in order to use this add-on.
 *
 ****************************************************************************/

#if !defined(CONFIG_NSH_BUILTIN_APPS) && !defined(CONFIG_SYSTEM_COMPOSITE)
extern int composite_archinitialize(void);
#endif

#include "../common/board_common.h"

#endif /* __ASSEMBLY__ */

__END_DECLS
