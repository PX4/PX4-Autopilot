/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

/*
FDCAN2 => no DMA, but dedicated message RAM
ETH => has its own DMA engine
I2C => has its own DMA engine

The rest are general DMA engines:

MDMA (D1): not supported by NuttX

DMA1 (D2): 8 streams, mapped to DMAMUX1 requests 0-7

DMA2 (D2): 8 streams, mapped to DMAMUX1 requests 8-15

BDMA (D3): 8 streams, mapped to DMAMUX2 requests 0-7

I2C1_RX => DMAMUX1:33 (EXT)
I2C1_TX => DMAMUX1:34 (EXT)
I2C4_RX => DMAMUX2:29 (BARO + MAG + EEPROM)
I2C4_TX => DMAMUX2:30 (BARO + MAG + EEPROM)

SPI3_RX => DMAMUX1:61 (IMU3 BMI088)
SPI3_TX => DMAMUX1:62 (IMU3 BMI088)
SPI4_RX => DMAMUX1:83 (FRAM)
SPI4_TX => DMAMUX1:84 (FRAM)

UART4_RX => DMAMUX1:63 (NSH DEBUG)
UART4_TX => DMAMUX1:64 (NSH DEBUG)
UART7_RX => DMAMUX1:79 (SBUS INPUT)
UART7_RX => DMAMUX1:79 (ESC RX)
UART8_RX => DMAMUX1:81 (GPS)
UART8_TX => DMAMUX1:82 (GPS)
USART3_RX => DMAMUX1:45 (EXTRAS)
USART3_TX => DMAMUX1:46 (EXTRAS)

USART1_RX => DMAMUX1:41 (MAVLINK)
USART1_TX => DMAMUX1:42 (MAVLINK)
USART2_RX => DMAMUX1:43 (NSH IMX)
USART2_TX => DMAMUX1:44 (NSH IMX)

(TIM1_CH1 => DMAMUX1:11)
(TIM1_CH2 => DMAMUX1:12)
(TIM1_CH3 => DMAMUX1:13)
(TIM1_CH4 => DMAMUX1:14)
TIM1_UP => DMAMUX1:15 (PWM)

(TIM3_CH1 => DMAMUX1:23)
(TIM3_CH2 => DMAMUX1:24)
(TIM3_CH3 => DMAMUX1:25)
(TIM3_CH4 => DMAMUX1:26)
TIM3_UP => DMAMUX1:27 (PWM)

TIM8_CH2 => DMAMUX1:48 (PPM INPUT)
(TIM8_UP => DMAMUX1:51)

*/



// DMAMUX1 Using at most 8 Channels on DMA1 --------   Assigned
//                                                     V

#define DMAMAP_USART1_RX  DMAMAP_DMA12_USART1RX_0   /* 1 TELEM1 (MAVLINK) */
#define DMAMAP_USART1_TX  DMAMAP_DMA12_USART1TX_0   /* 2 TELEM1 (MAVLINK) */

#define DMAMAP_UART8_RX   DMAMAP_DMA12_UART8RX_0    /* 3 GPS1 */
#define DMAMAP_UART8_TX   DMAMAP_DMA12_UART8TX_0    /* 4 GPS1 */

#define DMAMAP_USART2_TX  DMAMAP_DMA12_USART2TX_0   /* 5 NSH IMX TX */
#define DMAMAP_UART4_TX   DMAMAP_DMA12_UART4TX_0    /* 6 NSH DBG TX */

// Assigned in timer_config.cpp

// Timer 1                                          /* 7 TIM1UP */
// Timer 3                                          /* 8 TIM3UP */



// DMAMUX2 Using at most 8 Channels on DMA2 --------   Assigned
//                                                     V

#define DMAMAP_SPI3_RX    DMAMAP_DMA12_SPI3RX_1     /* 1 BMI088 */
#define DMAMAP_SPI3_TX    DMAMAP_DMA12_SPI3TX_1     /* 2 BMI088 */

#define DMAMAP_SPI4_RX    DMAMAP_DMA12_SPI4RX_1     /* 3 FRAM */
#define DMAMAP_SPI4_TX    DMAMAP_DMA12_SPI4TX_1     /* 4 FRAM */

#define DMAMAP_USART3_RX  DMAMAP_DMA12_USART3RX_1   /* 5 TELEM2 */
#define DMAMAP_USART3_TX  DMAMAP_DMA12_USART3TX_1   /* 6 TELEM2 */

#define DMAMAP_UART5_RX   DMAMAP_DMA12_UART5RX_1    /* 7 SBUS IN */
#define DMAMAP_UART7_RX   DMAMAP_DMA12_UART7RX_1    /* 8 ESC IN */
