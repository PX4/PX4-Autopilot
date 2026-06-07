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

/* DMA mapping for SPI1 (MAX7456 OSD) — DMA1 */
#define DMAMAP_SPI1_RX    DMAMAP_DMA12_SPI1RX_0
#define DMAMAP_SPI1_TX    DMAMAP_DMA12_SPI1TX_0

/* DMA mapping for SPI2 (ICM-42688 IMU) — DMA1 */
#define DMAMAP_SPI2_RX    DMAMAP_DMA12_SPI2RX_0
#define DMAMAP_SPI2_TX    DMAMAP_DMA12_SPI2TX_0

/* DMA mapping for SPI3 (W25N NAND flash) — DMA2 */
#define DMAMAP_SPI3_RX    DMAMAP_DMA12_SPI3RX_1
#define DMAMAP_SPI3_TX    DMAMAP_DMA12_SPI3TX_1

/* USART2 (GPS) RX DMA — DMA2 */
#define DMAMAP_USART2_RX  DMAMAP_DMA12_USART2RX_1

/* UART4 (TEL2) RX/TX DMA — DMA1 */
#define DMAMAP_UART4_RX   DMAMAP_DMA12_UART4RX_0
#define DMAMAP_UART4_TX   DMAMAP_DMA12_UART4TX_0
