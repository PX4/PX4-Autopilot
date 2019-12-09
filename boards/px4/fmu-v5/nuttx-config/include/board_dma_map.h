#pragma once

/*   DMA Channel/Stream Selections
 *
 *   DMAMAP_UART8_TX    = DMA1, Stream 0, Channel 5
 *   DMAMAP_USART3_RX   = DMA1, Stream 1, Channel 4
 *   DMAMAP_UART4_RX    = DMA1, Stream 2, Channel 4
 *   DMAMAP_USART3_TX_1 = DMA1, Stream 3, Channel 4
 *   DMAMAP_USART2_RX   = DMA1, Stream 5, Channel 4
 *   DMAMAP_UART8_RX    = DMA1, Stream 6, Channel 5
 *
 *   DMAMAP_USART6_RX_2 = DMA2, Stream 2, Channel 5
 *   DMAMAP_SDMMC1_1    = DMA2, Stream 3, Channel 4
 *   DMAMAP_TIM1_UP     = DMA2, Stream 5, Channel 6
 */

#define DMAMAP_USART3_TX DMAMAP_USART3_TX_1

#define DMAMAP_USART6_RX DMAMAP_USART6_RX_2
#define DMAMAP_SDMMC1    DMAMAP_SDMMC1_1
