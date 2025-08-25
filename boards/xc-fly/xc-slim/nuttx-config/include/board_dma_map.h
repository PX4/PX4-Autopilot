/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
// #define DMAMAP_SPI1_RX    DMAMAP_DMA12_SPI1RX_0 /* DMA1:37 */
// #define DMAMAP_SPI1_TX    DMAMAP_DMA12_SPI1TX_0 /* DMA1:38 */

#define DMAMAP_SPI2_RX    DMAMAP_DMA12_SPI2RX_0 /* DMA1:39 */
#define DMAMAP_SPI2_TX    DMAMAP_DMA12_SPI2TX_0 /* DMA1:40 */

// DMAMUX2
// #define DMAMAP_SPI3_RX    DMAMAP_DMA12_SPI3RX_0 /* DMA1:61 */
// #define DMAMAP_SPI3_TX    DMAMAP_DMA12_SPI3TX_0 /* DMA1:62 */

// #define DMAMAP_SPI6_RX    DMAMAP_BDMA_SPI6_RX /* BDMA:11 */
// #define DMAMAP_SPI6_TX    DMAMAP_BDMA_SPI6_TX /* BDMA:12 */

//TODO: UART DMA test
// #define DMAMAP_USART1_RX   DMAMAP_DMA12_USART1RX_1  /*DMA2:41*/
// #define DMAMAP_USART1_TX   DMAMAP_DMA12_USART1TX_1  /*DMA2:42*/

// #define DMAMAP_USART2_RX   DMAMAP_DMA12_USART2RX_1 /* DMA2:43 */
// #define DMAMAP_USART2_TX   DMAMAP_DMA12_USART2TX_1 /* DMA2:44 */

// #define DMAMAP_USART3_RX   DMAMAP_DMA12_USART3RX_1 /* DMA2:45 */
// #define DMAMAP_USART3_TX   DMAMAP_DMA12_USART3TX_1 /* DMA2:46 */

#define DMAMAP_UART4_RX    DMAMAP_DMA12_UART4RX_1 /* DMA1:63 */
#define DMAMAP_UART4_TX    DMAMAP_DMA12_UART4TX_1 /* DMA1:64 */

// #define DMAMAP_UART5_RX    DMAMAP_DMA12_UART5RX_0 /* DMA1:65 */
// #define DMAMAP_UART5_TX    DMAMAP_DMA12_UART5RX_0 /* DMA1:66 */
