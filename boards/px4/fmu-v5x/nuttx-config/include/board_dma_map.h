#pragma once

/*   DMA Channel/Stream Selections
 *
 *   DMAMAP_UART5_RX    = DMA1, Stream 0, Channel 4
 *   DMAMAP_USART3_RX   = DMA1, Stream 1, Channel 4
 *   DMAMAP_UART7_RX    = DMA1, Stream 3, Channel 5
 *   DMAMAP_TIM4_UP     = DMA1, Stream 6, Channel 2
 *
 *   DMAMAP_SDMMC2_1    = DMA2, Stream 0, Channel 11
 *   DMAMAP_USART6_RX_1 = DMA2, Stream 1, Channel 5
 *   DMAMAP_TIM1_UP     = DMA2, Stream 5, Channel 6
 *   DMAMAP_USART6_TX_2 = DMA2, Stream 7, Channel 5
 */

#define DMAMAP_SDMMC2    DMAMAP_SDMMC2_1
#define DMAMAP_USART6_RX DMAMAP_USART6_RX_1
#define DMAMAP_USART6_TX DMAMAP_USART6_TX_2
