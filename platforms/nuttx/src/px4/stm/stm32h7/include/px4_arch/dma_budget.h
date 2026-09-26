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

/**
 * @file dma_budget.h
 *
 * Build-time check that a board opting in to concurrent BDShot capture does not
 * oversubscribe DMA1 or DMA2.
 *
 * DMA1 and DMA2 have 8 streams each. NuttX stm32_dmachannel() hands out the first free
 * stream on the requested controller and returns NULL once it is exhausted. Nothing
 * reserves streams ahead of time, and some callers (e.g. the H7 serial driver) do not
 * check for NULL, so oversubscription fails silently at run time and which driver loses
 * depends on start order.
 *
 * Concurrent capture holds one stream per captured channel for the lifetime of the
 * driver, so a board that opts in adds after its io_timers[] and timer_io_channels[]:
 *
 *   PX4_VALIDATE_DMA_BUDGET(io_timers, timer_io_channels);
 *
 * The check only asserts when at least one timer uses concurrent capture. Boards that
 * keep round-robin capture are never checked, so their build result cannot change.
 *
 * Counted, per controller (worst case, as if every user were running):
 *  - NuttX serial:  CONFIG_<port>_RXDMA / _TXDMA together with DMAMAP_<port>_RX / _TX
 *  - NuttX SPI:     CONFIG_STM32H7_SPIn_DMA together with DMAMAP_SPIn_RX / _TX
 *  - NuttX ADC:     CONFIG_STM32H7_ADCn_DMA together with ADCn_DMA_CHAN
 *  - NuttX QSPI:    CONFIG_STM32H7_QSPI_DMA together with DMAMAP_QUADSPI (this also
 *                   covers boards/spracing/h7extreme/src/qspi.c, the only board-level
 *                   caller of stm32_dmachannel(), which uses the same pair)
 *  - PX4IO serial:  CONFIG_DRIVERS_PX4IO together with PX4IO_SERIAL_RX/TX_DMAMAP
 *  - Serial RGB LED: BOARD_HAS_N_S_RGB_LED together with S_RGB_LED_DMA
 *  - DShot burst:   one stream per io_timers[] entry that was given a DMA
 *  - DShot capture: one stream per registered channel of a concurrent-capture timer
 *
 * Not counted: the NuttX SPI slave driver, the only stm32_dmachannel() caller in the tree
 * that is left out. A new driver that calls stm32_dmachannel() with its own DMAMAP has to
 * be added here. Streams that map to MDMA or BDMA are ignored, as they are separate
 * controllers.
 */

#pragma once

#include <px4_arch/io_timer_hw_description.h>

namespace px4_dma_budget
{

static constexpr unsigned STREAMS_PER_CONTROLLER = 8;

// DMAMAP_CONTROLLER() decodes the controller from a DMAMAP value: DMA1 = 1, DMA2 = 2.
static constexpr unsigned DMA1_CONTROLLER = 1;
static constexpr unsigned DMA2_CONTROLLER = 2;

static inline constexpr unsigned on(uint32_t dmamap, unsigned controller)
{
	return (DMAMAP_CONTROLLER(dmamap) == controller) ? 1 : 0;
}

// Streams held by peripheral drivers, derived from the board's DMAMAP definitions and defconfig.
static inline constexpr unsigned peripheral_streams(unsigned c)
{
	unsigned n = 0;

#if defined(CONFIG_USART1_RXDMA) && defined(DMAMAP_USART1_RX)
	n += on(DMAMAP_USART1_RX, c);
#endif
#if defined(CONFIG_USART1_TXDMA) && defined(DMAMAP_USART1_TX)
	n += on(DMAMAP_USART1_TX, c);
#endif
#if defined(CONFIG_USART2_RXDMA) && defined(DMAMAP_USART2_RX)
	n += on(DMAMAP_USART2_RX, c);
#endif
#if defined(CONFIG_USART2_TXDMA) && defined(DMAMAP_USART2_TX)
	n += on(DMAMAP_USART2_TX, c);
#endif
#if defined(CONFIG_USART3_RXDMA) && defined(DMAMAP_USART3_RX)
	n += on(DMAMAP_USART3_RX, c);
#endif
#if defined(CONFIG_USART3_TXDMA) && defined(DMAMAP_USART3_TX)
	n += on(DMAMAP_USART3_TX, c);
#endif
#if defined(CONFIG_UART4_RXDMA) && defined(DMAMAP_UART4_RX)
	n += on(DMAMAP_UART4_RX, c);
#endif
#if defined(CONFIG_UART4_TXDMA) && defined(DMAMAP_UART4_TX)
	n += on(DMAMAP_UART4_TX, c);
#endif
#if defined(CONFIG_UART5_RXDMA) && defined(DMAMAP_UART5_RX)
	n += on(DMAMAP_UART5_RX, c);
#endif
#if defined(CONFIG_UART5_TXDMA) && defined(DMAMAP_UART5_TX)
	n += on(DMAMAP_UART5_TX, c);
#endif
#if defined(CONFIG_USART6_RXDMA) && defined(DMAMAP_USART6_RX)
	n += on(DMAMAP_USART6_RX, c);
#endif
#if defined(CONFIG_USART6_TXDMA) && defined(DMAMAP_USART6_TX)
	n += on(DMAMAP_USART6_TX, c);
#endif
#if defined(CONFIG_UART7_RXDMA) && defined(DMAMAP_UART7_RX)
	n += on(DMAMAP_UART7_RX, c);
#endif
#if defined(CONFIG_UART7_TXDMA) && defined(DMAMAP_UART7_TX)
	n += on(DMAMAP_UART7_TX, c);
#endif
#if defined(CONFIG_UART8_RXDMA) && defined(DMAMAP_UART8_RX)
	n += on(DMAMAP_UART8_RX, c);
#endif
#if defined(CONFIG_UART8_TXDMA) && defined(DMAMAP_UART8_TX)
	n += on(DMAMAP_UART8_TX, c);
#endif

#if defined(CONFIG_STM32H7_SPI1_DMA) && defined(DMAMAP_SPI1_RX)
	n += on(DMAMAP_SPI1_RX, c) + on(DMAMAP_SPI1_TX, c);
#endif
#if defined(CONFIG_STM32H7_SPI2_DMA) && defined(DMAMAP_SPI2_RX)
	n += on(DMAMAP_SPI2_RX, c) + on(DMAMAP_SPI2_TX, c);
#endif
#if defined(CONFIG_STM32H7_SPI3_DMA) && defined(DMAMAP_SPI3_RX)
	n += on(DMAMAP_SPI3_RX, c) + on(DMAMAP_SPI3_TX, c);
#endif
#if defined(CONFIG_STM32H7_SPI4_DMA) && defined(DMAMAP_SPI4_RX)
	n += on(DMAMAP_SPI4_RX, c) + on(DMAMAP_SPI4_TX, c);
#endif
#if defined(CONFIG_STM32H7_SPI5_DMA) && defined(DMAMAP_SPI5_RX)
	n += on(DMAMAP_SPI5_RX, c) + on(DMAMAP_SPI5_TX, c);
#endif
#if defined(CONFIG_STM32H7_SPI6_DMA) && defined(DMAMAP_SPI6_RX)
	n += on(DMAMAP_SPI6_RX, c) + on(DMAMAP_SPI6_TX, c);
#endif

#if defined(CONFIG_STM32H7_ADC1_DMA) && defined(ADC1_DMA_CHAN)
	n += on(ADC1_DMA_CHAN, c);
#endif
#if defined(CONFIG_STM32H7_ADC2_DMA) && defined(ADC2_DMA_CHAN)
	n += on(ADC2_DMA_CHAN, c);
#endif

#if defined(CONFIG_STM32H7_QSPI_DMA) && defined(DMAMAP_QUADSPI)
	n += on(DMAMAP_QUADSPI, c);
#endif

#if defined(CONFIG_DRIVERS_PX4IO) && defined(PX4IO_SERIAL_TX_DMAMAP)
	n += on(PX4IO_SERIAL_TX_DMAMAP, c) + on(PX4IO_SERIAL_RX_DMAMAP, c);
#endif

#if defined(BOARD_HAS_N_S_RGB_LED) && defined(S_RGB_LED_DMA)
	n += on(S_RGB_LED_DMA, c);
#endif

	return n;
}

// Streams claimed by the DShot driver.
//
// A timer's burst UP stream is freed and re-allocated every cycle in bidirectional mode,
// and round-robin capture reuses that same stream, so the peak for either is one per
// timer. Concurrent capture additionally holds one stream per channel it captures, for
// the lifetime of the driver.
static inline constexpr unsigned dshot_streams(const io_timers_t (&timers)[MAX_IO_TIMERS],
		const timer_io_channels_t (&channels)[MAX_TIMER_IO_CHANNELS], unsigned c)
{
	unsigned n = 0;

	for (unsigned i = 0; i < MAX_IO_TIMERS; i++) {
		if (timers[i].base == 0) {
			break;
		}

		if (timers[i].dshot.dma_map_up != 0) {
			n += on(timers[i].dshot.dma_map_up, c);
		}

		if (!timers[i].dshot.concurrent_capture) {
			continue;
		}

		for (unsigned ch = 0; ch < MAX_TIMER_IO_CHANNELS; ch++) {
			if (channels[ch].gpio_in == 0 && channels[ch].gpio_out == 0) {
				break;
			}

			if (channels[ch].timer_index == i) {
				n += on(timers[i].dshot.dma_map_ch[channels[ch].timer_channel], c);
			}
		}
	}

	return n;
}

static inline constexpr bool uses_concurrent_capture(const io_timers_t (&timers)[MAX_IO_TIMERS])
{
	for (unsigned i = 0; i < MAX_IO_TIMERS; i++) {
		if (timers[i].dshot.concurrent_capture) {
			return true;
		}
	}

	return false;
}

static inline constexpr bool fits(const io_timers_t (&timers)[MAX_IO_TIMERS],
				  const timer_io_channels_t (&channels)[MAX_TIMER_IO_CHANNELS], unsigned c)
{
	return !uses_concurrent_capture(timers)
	       || peripheral_streams(c) + dshot_streams(timers, channels, c) <= STREAMS_PER_CONTROLLER;
}

} // namespace px4_dma_budget

#define PX4_VALIDATE_DMA_BUDGET(timers_, channels_)									\
	static_assert(px4_dma_budget::fits(timers_, channels_, px4_dma_budget::DMA1_CONTROLLER),			\
		      "DMA1 is oversubscribed: concurrent BDShot capture plus the DMA users in "			\
		      "board_dma_map.h/board_config.h/defconfig/timer_config.cpp need more than 8 streams");	\
	static_assert(px4_dma_budget::fits(timers_, channels_, px4_dma_budget::DMA2_CONTROLLER),			\
		      "DMA2 is oversubscribed: concurrent BDShot capture plus the DMA users in "			\
		      "board_dma_map.h/board_config.h/defconfig/timer_config.cpp need more than 8 streams")
