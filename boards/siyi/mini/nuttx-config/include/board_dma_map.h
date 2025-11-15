/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#define DMAMAP_SPI1_RX    DMAMAP_DMA12_SPI1RX_0     /* 1 DMA1:37 BMI088 */
#define DMAMAP_SPI1_TX    DMAMAP_DMA12_SPI1TX_0     /* 2 DMA1:38 BMI088 */

#define DMAMAP_SPI2_RX    DMAMAP_DMA12_SPI2RX_0     /* 3 DMA1:61 FRAM */
#define DMAMAP_SPI2_TX    DMAMAP_DMA12_SPI2TX_0     /* 4 DMA1:62 FRAM */

#define DMAMAP_SPI3_RX    DMAMAP_DMA12_SPI3RX_0     /* 5 DMA1:61 ICM-42688-P */
#define DMAMAP_SPI3_TX    DMAMAP_DMA12_SPI3TX_0     /* 6 DMA1:62 ICM-42688-P */

#define DMAMAP_SPI4_RX    DMAMAP_DMA12_SPI4RX_0     /* 7 DMA1:61 BMP581 */
#define DMAMAP_SPI4_TX    DMAMAP_DMA12_SPI4TX_0     /* 8 DMA1:62 BMP581 */

// Assigned in timer_config.cpp

// DMAMUX2 Using at most 8 Channels on DMA2 --------   Assigned
//                                                     V

#define DMAMAP_USART1_RX  DMAMAP_DMA12_USART1RX_1   /* 1 DMA2:45 GPS1 */
#define DMAMAP_USART1_TX  DMAMAP_DMA12_USART1TX_1   /* 2 DMA2:46 GPS1 */

#define DMAMAP_USART2_RX  DMAMAP_DMA12_USART2RX_1    /* 3 DMA2:65 TELEM1 */
#define DMAMAP_USART2_TX  DMAMAP_DMA12_USART2TX_1    /* 4 DMA2:66 TELEM1 */

#define DMAMAP_UART4_RX   DMAMAP_DMA12_UART4RX_1    /* 5 DMA1:79  TELEM2 */
#define DMAMAP_UART4_TX   DMAMAP_DMA12_UART4TX_1    /* 6 DMA1:80  TELEM2 */

#define DMAMAP_USART6_RX   DMAMAP_DMA12_USART6RX_1    /* 5 DMA1:79  GPS2 */
#define DMAMAP_USART6_TX   DMAMAP_DMA12_USART6TX_1    /* 6 DMA1:80  GPS2 */

// DMAMUX2 Using at most 8 Channels on BDMA --------   Assigned
//                                                     V

