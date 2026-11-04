/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

// DMAMUX1 Using at most 8 Channels on DMA1 --------   Assigned
//                                                     V

#define DMAMAP_SPI1_RX    DMAMAP_DMA12_SPI1RX_0     /* 1 DMA1:37 BMI088, BMM150, MS5611 */
#define DMAMAP_SPI1_TX    DMAMAP_DMA12_SPI1TX_0     /* 2 DMA1:38 BMI088, BMM150, MS5611 */

//#define DMAMAP_SPI2_RX    DMAMAP_DMA12_SPI2RX_0     /* 3 DMA1:39 Base */
//#define DMAMAP_SPI2_TX    DMAMAP_DMA12_SPI2TX_0     /* 4 DMA1:40 Base */

#define DMAMAP_SPI4_RX    DMAMAP_DMA12_SPI4RX_0     /* 5 DMA1:83 FRAM, ICM-42688-P */
#define DMAMAP_SPI4_TX    DMAMAP_DMA12_SPI4TX_0     /* 6 DMA1:84 FRAM, ICM-42688-P */

#define DMAMAP_USART2_RX  DMAMAP_DMA12_USART2RX_0   /* 7 DMA1:43 Telem1 */
#define DMAMAP_USART2_TX  DMAMAP_DMA12_USART2TX_0   /* 8 DMA1:44 Telem1 */

//#define DMAMAP_USART3_RX  DMAMAP_DMA12_USART3RX_0   /* 9 DMA1:45 Telem2 */
//#define DMAMAP_USART3_TX  DMAMAP_DMA12_USART3TX_0   /* 10 DMA1:46 Telem2 */

//#define DMAMAP_UART4_RX  DMAMAP_DMA12_UART4RX_0     /* 11 DMA1:63 GPS1 */
//#define DMAMAP_UART4_TX  DMAMAP_DMA12_UART4TX_0     /* 12 DMA1:64 GPS1 */


// DMAMUX2 Using at most 8 Channels on DMA2 --------   Assigned
//                                                     V

//#define DMAMAP_USART5_RX  DMAMAP_DMA12_UART5RX_1     /* 13 DMA2:65 Base */
//#define DMAMAP_USART5_TX  DMAMAP_DMA12_UART5TX_1     /* 14 DMA2:66 Base */

#define DMAMAP_USART6_RX  DMAMAP_DMA12_USART6RX_1   /* 15 DMA2:71 RC */
#define DMAMAP_USART6_TX  DMAMAP_DMA12_USART6TX_1   /* 16 DMA2:72 RC */

// #define DMAMAP_UART7_RX   DMAMAP_DMA12_UART7RX_1    /* 17 DMA2:79  GPS2 */
// #define DMAMAP_UART7_TX   DMAMAP_DMA12_UART7TX_1    /* 18 DMA2:80  GPS2 */

// #define DMAMAP_UART8_RX   DMAMAP_DMA12_UART8RX_1    /* 19 DMA2:81  DEBUG */
// #define DMAMAP_UART8_TX   DMAMAP_DMA12_UART8TX_1    /* 20 DMA2:82  DEBUG */
