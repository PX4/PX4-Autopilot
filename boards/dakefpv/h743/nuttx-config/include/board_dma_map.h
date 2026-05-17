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

// DMAMUX1 — DMA1
#define DMAMAP_SPI1_RX    DMAMAP_DMA12_SPI1RX_0  /* DMA1:37 — ICM-42688P IMU1 */
#define DMAMAP_SPI1_TX    DMAMAP_DMA12_SPI1TX_0  /* DMA1:38 — ICM-42688P IMU1 */

#define DMAMAP_SPI4_RX    DMAMAP_DMA12_SPI4RX_0  /* DMA1:83 — ICM-42688P IMU2 */
#define DMAMAP_SPI4_TX    DMAMAP_DMA12_SPI4TX_0  /* DMA1:84 — ICM-42688P IMU2 */

// DMAMUX1 — DMA2 (6 streams remaining after SPI1/SPI4/TIM2/TIM4/TIM8 fill DMA1)
#define DMAMAP_USART1_RX  DMAMAP_DMA12_USART1RX_1 /* DMA2:41 — GPS1 (ttyS0) RX */
#define DMAMAP_USART1_TX  DMAMAP_DMA12_USART1TX_1 /* DMA2:42 — GPS1 (ttyS0) TX */

#define DMAMAP_USART6_RX  DMAMAP_DMA12_USART6RX_1 /* DMA2:71 — TELEM3 (ttyS3) RX */
#define DMAMAP_USART6_TX  DMAMAP_DMA12_USART6TX_1 /* DMA2:72 — TELEM3 (ttyS3) TX */

#define DMAMAP_UART5_RX   DMAMAP_DMA12_UART5RX_1  /* DMA2:65 — RC input (ttyS4) RX */
#define DMAMAP_UART5_TX   DMAMAP_DMA12_UART5TX_1  /* DMA2:66 — RC input (ttyS4) TX */

#define DMAMAP_UART7_RX   DMAMAP_DMA12_UART7RX_1  /* DMA2:79 — TELEM4 (ttyS5) RX */
#define DMAMAP_UART7_TX   DMAMAP_DMA12_UART7TX_1  /* DMA2:80 — TELEM4 (ttyS5) TX */

// Timer DMA assigned in timer_config.cpp
// TIM2_UP                                        /* DMA1 — M1-M4 DShot */
// TIM4_UP                                        /* DMA1 — M5-M8 DShot */
// TIM8_UP                                        /* DMA1 — S3-S4 DShot */
