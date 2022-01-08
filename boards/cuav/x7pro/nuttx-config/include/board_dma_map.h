/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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


// DMAMUX1
//      DMAMAP_TIM1UP     DMAMAP_DMA12_TIM1UP_0   /* DMA1:15 */  // DSHOT 9,10,11,12
//      DMAMAP_TIM4UP     DMAMAP_DMA12_TIM4UP_0   /* DMA1:32 */  // DSHOT 5, 6, 7, 8
#define DMAMAP_SPI1_RX    DMAMAP_DMA12_SPI1RX_0   /* DMA1:37 */
#define DMAMAP_SPI1_TX    DMAMAP_DMA12_SPI1TX_0   /* DMA1:38 */
#define DMAMAP_SPI2_RX    DMAMAP_DMA12_SPI2RX_0   /* DMA1:39 */
#define DMAMAP_SPI2_TX    DMAMAP_DMA12_SPI2TX_0   /* DMA1:40 */
#define DMAMAP_USART1_RX  DMAMAP_DMA12_USART1RX_0 /* DMA1:41 */  // GPS1 (RX only)
//      DMAMAP_TIM5UP     DMAMAP_DMA12_TIM5UP_0   /* DMA1:59 */  // DSHOT 1, 2, 3, 4


// DMAMUX2
#define DMAMAP_USART2_RX  DMAMAP_DMA12_USART2RX_1 /* DMA2:43 */  // TELEM1 RX
#define DMAMAP_USART2_TX  DMAMAP_DMA12_USART2TX_1 /* DMA2:44 */  // TELEM1 TX
#define DMAMAP_USART6_RX  DMAMAP_DMA12_USART6RX_1 /* DMA2:71 */  // TELEM2 RX
#define DMAMAP_USART6_TX  DMAMAP_DMA12_USART6TX_1 /* DMA2:72 */  // TELEM2 TX
#define DMAMAP_UART7_RX   DMAMAP_DMA12_UART7RX_1  /* DMA2:79 */  // CONSOLE (RX only)
#define DMAMAP_UART8_RX   DMAMAP_DMA12_UART8RX_1  /* DMA2:81 */  // RC (RX only)
#define DMAMAP_SPI4_RX    DMAMAP_DMA12_SPI4RX_1   /* DMA2:83 */
#define DMAMAP_SPI4_TX    DMAMAP_DMA12_SPI4TX_1   /* DMA2:84 */


// BDMA
#define DMAMAP_SPI6_RX    DMAMAP_BDMA_SPI6_RX     /* BDMA:11 */
#define DMAMAP_SPI6_TX    DMAMAP_BDMA_SPI6_TX     /* BDMA:12 */
