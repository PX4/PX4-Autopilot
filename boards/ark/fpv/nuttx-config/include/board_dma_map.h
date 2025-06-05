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
#define DMAMAP_SPI1_RX    DMAMAP_DMA12_SPI1RX_0     // IIM-42653
#define DMAMAP_SPI1_TX    DMAMAP_DMA12_SPI1TX_0     // IIM-42653
#define DMAMAP_USART1_RX  DMAMAP_DMA12_USART1RX_0   // GPS1
#define DMAMAP_USART1_TX  DMAMAP_DMA12_USART1TX_0   // GPS1
#define DMAMAP_USART6_RX  DMAMAP_DMA12_USART6RX_0   // RC
#define DMAMAP_USART6_TX  DMAMAP_DMA12_USART6TX_0   // RC
#define DMAMAP_UART7_RX   DMAMAP_DMA12_UART7RX_0    // TELEM1
#define DMAMAP_UART7_TX   DMAMAP_DMA12_UART7TX_0    // TELEM1

// DMAMUX2 Using at most 8 Channels on DMA2 --------   Assigned
#define DMAMAP_USART2_RX  DMAMAP_DMA12_USART2RX_1   // VTX
#define DMAMAP_UART5_RX   DMAMAP_DMA12_UART5RX_1    // VTX
#define DMAMAP_UART5_TX   DMAMAP_DMA12_UART5TX_1    // VTX
// Timer 4 (DMAMAP_DMA12_TIM4UP_0)                  // TIM4UP/TIM4CH1-4
// Timer 5 (DMAMAP_DMA12_TIM5UP_0)                  // TIM5UP/TIM5CH1-4

// DMAMUX2 Using at most 8 Channels on BDMA --------   Assigned
#define DMAMAP_SPI6_RX    DMAMAP_BDMA_SPI6_RX       // External SPI
#define DMAMAP_SPI6_TX    DMAMAP_BDMA_SPI6_TX       // External SPI
