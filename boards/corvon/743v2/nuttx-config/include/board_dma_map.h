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

// DMAMUX1 has 16 streams: 8 on DMA1 + 8 on DMA2. DMA1 and DMA2 are
// independent controllers (separate AHB master ports), so high-bandwidth
// peripherals are split across both pools to avoid contention with the
// bdshot drivers, which claim 3 streams on DMA1 from timer_config.cpp.
//
// DMAMUX1 - DMA1 -------------------------------------------- Assigned
//                                                             V
//
// Timer  1 (M1-M4  bdshot)    - DMA1 Stream 0   (timer_config.cpp)   1
// Timer  3 (M5-M6  bdshot)    - DMA1 Stream 4   (timer_config.cpp)   2
// Timer  4 (M7-M10 bdshot)    - DMA1 Stream 5   (timer_config.cpp)   3
//
// DMAMUX1 - DMA2 -------------------------------------------- Assigned
//                                                             V
#define DMAMAP_SPI3_RX    DMAMAP_DMA12_SPI3RX_1  /*  DMA2:61 ICM-42688P + BMI088 */ //  1
#define DMAMAP_SPI3_TX    DMAMAP_DMA12_SPI3TX_1  /*  DMA2:62 ICM-42688P + BMI088 */ //  2
#define DMAMAP_UART4_RX   DMAMAP_DMA12_UART4RX_1 /*  DMA2:63 TELEM2 (companion)  */ //  3
#define DMAMAP_UART4_TX   DMAMAP_DMA12_UART4TX_1 /*  DMA2:64 TELEM2 (companion)  */ //  4
