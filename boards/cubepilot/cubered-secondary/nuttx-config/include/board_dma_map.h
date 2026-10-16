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

// DMA 1
#define DMAMAP_SPI1_RX    DMAMAP_DMA12_SPI1RX_0
#define DMAMAP_SPI1_TX    DMAMAP_DMA12_SPI1TX_0

#define DMAMAP_SPI2_RX    DMAMAP_DMA12_SPI2RX_0
#define DMAMAP_SPI2_TX    DMAMAP_DMA12_SPI2TX_0

#define DMAMAP_SPI4_RX    DMAMAP_DMA12_SPI4RX_0
#define DMAMAP_SPI4_TX    DMAMAP_DMA12_SPI4TX_0

// Note: the UART7_RX must not be on DMA channel 1. Channels 0, 2, 3, ... seem to be ok.
//       if it is on channel 1 we see weird cache coherency issues. Suddenly the last two chars are
//       "leftovers" from previous FIFO buffer rounds.
#define DMAMAP_UART7_RX   DMAMAP_DMA12_UART7RX_0
#define DMAMAP_UART7_TX   DMAMAP_DMA12_UART7TX_0

// DMA 2
// Timer 1
// Timer 2
