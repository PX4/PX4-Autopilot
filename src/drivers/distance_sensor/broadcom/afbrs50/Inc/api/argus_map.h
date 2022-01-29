/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		Defines macros to work with pixel and ADC channel masks.
 *
 * @copyright
 *
 * Copyright (c) 2021, Broadcom Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/


#ifndef ARGUS_MAP_H
#define ARGUS_MAP_H

/*!***************************************************************************
 * @defgroup	argusmap ADC Channel Mapping
 * @ingroup		argusres
 *
 * @brief		Pixel ADC Channel Mapping
 *
 * @details		The ADC Channels of each pixel or auxiliary channel on the device
 * 				are numbered in a way that is convenient on the chip architecture.
 * 				The macros in this module are defined in order to map between the
 * 				chip internal channel number (ch) to the two-dimensional
 * 				x-y-indices or one-dimensional n-index representation.
 *
 * @addtogroup 	argusmap
 * @{
 *****************************************************************************/

#include "api/argus_def.h"
#include "utility/int_math.h"




/*!*****************************************************************************
 * @brief	Macro to determine the pixel ADC channel number from the x-z-indices.
 * @param	x The x-index of the pixel.
 * @param	y The y-index of the pixel.
 * @return	The ADC channel number of the pixel.
 ******************************************************************************/
#define PIXEL_XY2CH(x, y) ((((y) << 3U) & 0x10U) | (((x) ^ 0x07U) << 1U) | ((y) & 0x01U))

/*!*****************************************************************************
 * @brief	Macro to determine the pixel x-index from the ADC channel number.
 * @param	c The ADC channel number of the pixel.
 * @return	The x-index of the pixel.
 ******************************************************************************/
#define PIXEL_CH2X(c) ((((c) >> 1U) ^ 0x07U) & 0x07U)

/*!*****************************************************************************
 * @brief	Macro to determine the pixel y-index from the ADC channel number.
 * @param	c The ADC channel number of the pixel.
 * @return	The y-index of the pixel.
 ******************************************************************************/
#define PIXEL_CH2Y(c) ((((c) >> 3U) & 0x02U) | ((c) & 0x01U))


/*!*****************************************************************************
 * @brief	Macro to determine the n-index from the x-y-indices.
 * @param	x The x-index of the pixel.
 * @param	y The y-index of the pixel.
 * @return	The n-index of the pixel.
 ******************************************************************************/
#define PIXEL_XY2N(x, y) (((x) << 2U) | (y))

/*!*****************************************************************************
 * @brief	Macro to determine the pixel x-index from the n-index.
 * @param	n The n-index of the pixel.
 * @return	The x-index number of the pixel.
 ******************************************************************************/
#define PIXEL_N2X(n) ((n) >> 2U)

/*!*****************************************************************************
 * @brief	Macro to determine the pixel y-index from the n-index.
 * @param	n The n-index of the pixel.
 * @return	The y-index number of the pixel.
 ******************************************************************************/
#define PIXEL_N2Y(n) ((n) & 0x03U)


/*!*****************************************************************************
 * @brief	Macro to determine the pixel n-index from the ADC channel number.
 * @param	n The n-index of the pixel.
 * @return	The ADC channel number of the pixel.
 ******************************************************************************/
#define PIXEL_N2CH(n) ((((n) << 3U) & 0x10U) | ((((n) >> 1U) ^ 0x0EU) & 0x0EU) | ((n) & 0x01U))

/*!*****************************************************************************
 * @brief	Macro to determine the pixel
 * @param	c The ADC channel number of the pixel.
 * @return	The n-index of the pixel.
 ******************************************************************************/
#define PIXEL_CH2N(c) (((((c) << 1U) ^ 0x1CU) & 0x1CU) | (((c) >> 3U) & 0x02U) | ((c) & 0x01U))


/*!*****************************************************************************
 * @brief	Macro to determine if a pixel given by the n-index is enabled in a pixel mask.
 * @param	msk 32-bit pixel mask
 * @param	n n-index of the pixel.
 * @return 	True if the pixel (n) is enabled.
 ******************************************************************************/
#define PIXELN_ISENABLED(msk, n) (((msk) >> (n)) & 0x01U)

/*!*****************************************************************************
 * @brief	Macro to enable a pixel given by the n-index in a pixel mask.
 * @param	msk 32-bit pixel mask
 * @param	n n-index of the pixel to enable.
 ******************************************************************************/
#define PIXELN_ENABLE(msk, n) ((msk) |= (0x01U << (n)))

/*!*****************************************************************************
 * @brief	Macro disable a pixel given by the n-index in a pixel mask.
 * @param	msk 32-bit pixel mask
 * @param	n n-index of the pixel to disable.
 ******************************************************************************/
#define PIXELN_DISABLE(msk, n) ((msk) &= (~(0x01U << (n))))


/*!*****************************************************************************
 * @brief	Macro to determine if an ADC pixel channel is enabled from a pixel mask.
 * @param	msk The 32-bit pixel mask
 * @param	c The ADC channel number of the pixel.
 * @return 	True if the specified pixel ADC channel is enabled.
 ******************************************************************************/
#define PIXELCH_ISENABLED(msk, c) (PIXELN_ISENABLED(msk, PIXEL_CH2N(c)))

/*!*****************************************************************************
 * @brief	Macro to enable an ADC pixel channel in a pixel mask.
 * @param	msk The 32-bit pixel mask
 * @param	c The pixel ADC channel number to enable.
 ******************************************************************************/
#define PIXELCH_ENABLE(msk, c) (PIXELN_ENABLE(msk, PIXEL_CH2N(c)))

/*!*****************************************************************************
 * @brief	Macro to disable an ADC pixel channel in a pixel mask.
 * @param	msk The 32-bit pixel mask
 * @param	c The pixel ADC channel number to disable.
 ******************************************************************************/
#define PIXELCH_DISABLE(msk, c) (PIXELN_DISABLE(msk, PIXEL_CH2N(c)))


/*!*****************************************************************************
 * @brief	Macro to determine if a pixel given by the x-y-indices is enabled in a pixel mask.
 * @param	msk 32-bit pixel mask
 * @param	x x-index of the pixel.
 * @param	y y-index of the pixel.
 * @return 	True if the pixel (x,y) is enabled.
 ******************************************************************************/
#define PIXELXY_ISENABLED(msk, x, y) (PIXELN_ISENABLED(msk, PIXEL_XY2N(x, y)))

/*!*****************************************************************************
 * @brief	Macro to enable a pixel given by the x-y-indices in a pixel mask.
 * @param	msk 32-bit pixel mask
 * @param	x x-index of the pixel to enable.
 * @param	y y-index of the pixel to enable.
 ******************************************************************************/
#define PIXELXY_ENABLE(msk, x, y) (PIXELN_ENABLE(msk, PIXEL_XY2N(x, y)))

/*!*****************************************************************************
 * @brief	Macro disable a pixel given by the x-y-indices in a pixel mask.
 * @param	msk 32-bit pixel mask
 * @param	x x-index of the pixel to disable.
 * @param	y y-index of the pixel to disable.
 ******************************************************************************/
#define PIXELXY_DISABLE(msk, x, y) (PIXELN_DISABLE(msk, PIXEL_XY2N(x, y)))


/*!*****************************************************************************
 * @brief	Macro to determine if an ADC channel is enabled in a channel mask.
 * @param	msk 32-bit channel mask
 * @param	ch channel number of the ADC channel.
 * @return 	True if the ADC channel is enabled.
 ******************************************************************************/
#define CHANNELN_ISENABLED(msk, ch) (((msk) >> ((ch) - 32U)) & 0x01U)

/*!*****************************************************************************
 * @brief	Macro to determine if an ADC channel is enabled in a channel mask.
 * @param	msk 32-bit channel mask
 * @param	ch channel number of the ADC channel to enabled.
 ******************************************************************************/
#define CHANNELN_ENABLE(msk, ch) ((msk) |= (0x01U << ((ch) - 32U)))

/*!*****************************************************************************
 * @brief	Macro to determine if an ADC channel is disabled in a channel mask.
 * @param	msk 32-bit channel mask
 * @param	ch channel number of the ADC channel to disable.
 ******************************************************************************/
#define CHANNELN_DISABLE(msk, ch) ((msk) &= (~(0x01U << ((ch) - 32U))))


/*!*****************************************************************************
 * @brief	Macro to determine the number of enabled pixel/channels in a mask
 * 			via a popcount algorithm.
 * @param	pxmsk 32-bit pixel mask
 * @return 	The count of enabled pixel channels.
 ******************************************************************************/
#define PIXEL_COUNT(pxmsk) popcount(pxmsk)

/*!*****************************************************************************
 * @brief	Macro to determine the number of enabled channels via a popcount
 * 			algorithm.
 * @param	pxmsk 32-bit pixel mask
 * @param	chmsk 32-bit channel mask
 * @return 	The count of enabled ADC channels.
 ******************************************************************************/
#define CHANNEL_COUNT(pxmsk, chmsk) (popcount(pxmsk) + popcount(chmsk))

/*!*****************************************************************************
 * @brief	Converts a raw ADC channel mask to a x-y-sorted pixel mask.
 * @param	msk The raw ADC channel mask to be converted.
 * @return 	The converted x-y-sorted pixel mask.
 ******************************************************************************/
static inline uint32_t ChannelToPixelMask(uint32_t msk)
{
	uint32_t res = 0;

	for (uint_fast8_t n = 0; n < 32; n += 2) {
		res |= ((msk >> PIXEL_N2CH(n)) & 0x3U) << n;
	}

	return res;
}

/*! @} */
#endif /* ARGUS_MAP_H */
