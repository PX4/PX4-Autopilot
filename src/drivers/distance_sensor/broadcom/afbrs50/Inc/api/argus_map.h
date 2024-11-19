/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     Defines macros to work with pixel and ADC channel masks.
 *
 * @copyright
 *
 * Copyright (c) 2023, Broadcom Inc.
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
#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @defgroup    argus_map Pixel Channel Mapping
 * @ingroup     argus_api
 *
 * @brief       Pixel Channel Mapping
 *
 * @details     The ADC Channels of each pixel or auxiliary channel on the device
 *              are numbered in a way that is convenient on the chip architecture.
 *              The macros in this module are defined in order to map between the
 *              chip internal channel number (ch) to the two-dimensional
 *              x-y-indices or one-dimensional n-index representation.
 *
 * @addtogroup  argus_map
 * @{
 *****************************************************************************/

#include "utility/int_math.h"
#include <stdbool.h>
#include <assert.h>


/*!***************************************************************************
 * @brief   The device pixel field size in x direction (long edge).
 *****************************************************************************/
#define ARGUS_PIXELS_X  8

/*!***************************************************************************
 * @brief   The device pixel field size in y direction (short edge).
 *****************************************************************************/
#define ARGUS_PIXELS_Y  4

/*!***************************************************************************
 * @brief   The total device pixel count.
 *****************************************************************************/
#define ARGUS_PIXELS    ((ARGUS_PIXELS_X)*(ARGUS_PIXELS_Y))


/*!*****************************************************************************
 * @brief   Macro to determine the pixel ADC channel number from the x-z-indices.
 * @param   x The x-index of the pixel.
 * @param   y The y-index of the pixel.
 * @return  The ADC channel number of the pixel.
 ******************************************************************************/
#define PIXEL_XY2CH(x, y) ((((y) << 3U) & 0x10U) | (((x) ^ 0x07U) << 1U) | ((y) & 0x01U))

/*!*****************************************************************************
 * @brief   Macro to determine the pixel x-index from the ADC channel number.
 * @param   c The ADC channel number of the pixel.
 * @return  The x-index of the pixel.
 ******************************************************************************/
#define PIXEL_CH2X(c) ((((c) >> 1U) ^ 0x07U) & 0x07U)

/*!*****************************************************************************
 * @brief   Macro to determine the pixel y-index from the ADC channel number.
 * @param   c The ADC channel number of the pixel.
 * @return  The y-index of the pixel.
 ******************************************************************************/
#define PIXEL_CH2Y(c) ((((c) >> 3U) & 0x02U) | ((c) & 0x01U))


/*!*****************************************************************************
 * @brief   Macro to determine the n-index from the x-y-indices.
 * @param   x The x-index of the pixel.
 * @param   y The y-index of the pixel.
 * @return  The n-index of the pixel.
 ******************************************************************************/
#define PIXEL_XY2N(x, y) (((x) << 2U) | (y))

/*!*****************************************************************************
 * @brief   Macro to determine the pixel x-index from the n-index.
 * @param   n The n-index of the pixel.
 * @return  The x-index number of the pixel.
 ******************************************************************************/
#define PIXEL_N2X(n) ((n) >> 2U)

/*!*****************************************************************************
 * @brief   Macro to determine the pixel y-index from the n-index.
 * @param   n The n-index of the pixel.
 * @return  The y-index number of the pixel.
 ******************************************************************************/
#define PIXEL_N2Y(n) ((n) & 0x03U)


/*!*****************************************************************************
 * @brief   Macro to determine the pixel n-index from the ADC channel number.
 * @param   n The n-index of the pixel.
 * @return  The ADC channel number of the pixel.
 ******************************************************************************/
#define PIXEL_N2CH(n) ((((n) << 3U) & 0x10U) | ((((n) >> 1U) ^ 0x0EU) & 0x0EU) | ((n) & 0x01U))

/*!*****************************************************************************
 * @brief   Macro to determine the pixel
 * @param   c The ADC channel number of the pixel.
 * @return  The n-index of the pixel.
 ******************************************************************************/
#define PIXEL_CH2N(c) (((((c) << 1U) ^ 0x1CU) & 0x1CU) | (((c) >> 3U) & 0x02U) | ((c) & 0x01U))


/*!*****************************************************************************
 * @brief   Macro to create a pixel mask given by the pixels n-index.
 * @param   n n-index of the pixel.
 * @return  The pixel mask with only n-index pixel set.
 ******************************************************************************/
#define PIXELN_MASK(n) (0x01U << (n))

/*!*****************************************************************************
 * @brief   Macro to determine if a pixel given by the n-index is enabled in a pixel mask.
 * @param   msk 32-bit pixel mask
 * @param   n n-index of the pixel.
 * @return  True if the pixel (n) is enabled.
 ******************************************************************************/
#define PIXELN_ISENABLED(msk, n) (((msk) >> (n)) & 0x01U)

/*!*****************************************************************************
 * @brief   Macro to enable a pixel given by the n-index in a pixel mask.
 * @param   msk 32-bit pixel mask
 * @param   n n-index of the pixel to enable.
 ******************************************************************************/
#define PIXELN_ENABLE(msk, n) ((msk) |= (PIXELN_MASK(n)))

/*!*****************************************************************************
 * @brief   Macro disable a pixel given by the n-index in a pixel mask.
 * @param   msk 32-bit pixel mask
 * @param   n n-index of the pixel to disable.
 ******************************************************************************/
#define PIXELN_DISABLE(msk, n) ((msk) &= (~PIXELN_MASK(n)))


/*!*****************************************************************************
 * @brief   Macro to create a pixel mask given by the pixels ADC channel number.
 * @param   c The ADC channel number of the pixel.
 * @return  The 32-bit pixel mask with only pixel ADC channel set.
 ******************************************************************************/
#define PIXELCH_MASK(c) (0x01U << (PIXEL_CH2N(c)))

/*!*****************************************************************************
 * @brief   Macro to determine if an ADC pixel channel is enabled from a pixel mask.
 * @param   msk The 32-bit pixel mask
 * @param   c The ADC channel number of the pixel.
 * @return  True if the specified pixel ADC channel is enabled.
 ******************************************************************************/
#define PIXELCH_ISENABLED(msk, c) (PIXELN_ISENABLED(msk, PIXEL_CH2N(c)))

/*!*****************************************************************************
 * @brief   Macro to enable an ADC pixel channel in a pixel mask.
 * @param   msk The 32-bit pixel mask
 * @param   c The pixel ADC channel number to enable.
 ******************************************************************************/
#define PIXELCH_ENABLE(msk, c) (PIXELN_ENABLE(msk, PIXEL_CH2N(c)))

/*!*****************************************************************************
 * @brief   Macro to disable an ADC pixel channel in a pixel mask.
 * @param   msk The 32-bit pixel mask
 * @param   c The pixel ADC channel number to disable.
 ******************************************************************************/
#define PIXELCH_DISABLE(msk, c) (PIXELN_DISABLE(msk, PIXEL_CH2N(c)))


/*!*****************************************************************************
 * @brief   Macro to create a pixel mask given by the pixel x-y-indices.
 * @param   x x-index of the pixel.
 * @param   y y-index of the pixel.
 * @return  The 32-bit pixel mask with only pixel ADC channel set.
 ******************************************************************************/
#define PIXELXY_MASK(x, y) (0x01U << (PIXEL_XY2N(x, y)))

/*!*****************************************************************************
 * @brief   Macro to determine if a pixel given by the x-y-indices is enabled in a pixel mask.
 * @param   msk 32-bit pixel mask
 * @param   x x-index of the pixel.
 * @param   y y-index of the pixel.
 * @return  True if the pixel (x,y) is enabled.
 ******************************************************************************/
#define PIXELXY_ISENABLED(msk, x, y) (PIXELN_ISENABLED(msk, PIXEL_XY2N(x, y)))

/*!*****************************************************************************
 * @brief   Macro to enable a pixel given by the x-y-indices in a pixel mask.
 * @param   msk 32-bit pixel mask
 * @param   x x-index of the pixel to enable.
 * @param   y y-index of the pixel to enable.
 ******************************************************************************/
#define PIXELXY_ENABLE(msk, x, y) (PIXELN_ENABLE(msk, PIXEL_XY2N(x, y)))

/*!*****************************************************************************
 * @brief   Macro disable a pixel given by the x-y-indices in a pixel mask.
 * @param   msk 32-bit pixel mask
 * @param   x x-index of the pixel to disable.
 * @param   y y-index of the pixel to disable.
 ******************************************************************************/
#define PIXELXY_DISABLE(msk, x, y) (PIXELN_DISABLE(msk, PIXEL_XY2N(x, y)))


/*!*****************************************************************************
 * @brief   Macro to determine if an ADC channel is enabled in a channel mask.
 * @param   msk 32-bit channel mask
 * @param   ch channel number of the ADC channel.
 * @return  True if the ADC channel is enabled.
 ******************************************************************************/
#define CHANNELN_ISENABLED(msk, ch) (((msk) >> ((ch) - 32U)) & 0x01U)

/*!*****************************************************************************
 * @brief   Macro to determine if an ADC channel is enabled in a channel mask.
 * @param   msk 32-bit channel mask
 * @param   ch channel number of the ADC channel to enabled.
 ******************************************************************************/
#define CHANNELN_ENABLE(msk, ch) ((msk) |= (0x01U << ((ch) - 32U)))

/*!*****************************************************************************
 * @brief   Macro to determine if an ADC channel is disabled in a channel mask.
 * @param   msk 32-bit channel mask
 * @param   ch channel number of the ADC channel to disable.
 ******************************************************************************/
#define CHANNELN_DISABLE(msk, ch) ((msk) &= (~(0x01U << ((ch) - 32U))))


/*!*****************************************************************************
 * @brief   Macro to determine the number of enabled pixel/channels in a mask
 *          via a popcount algorithm.
 * @param   pxmsk 32-bit pixel mask
 * @return  The count of enabled pixel channels.
 ******************************************************************************/
#define PIXEL_COUNT(pxmsk) popcount(pxmsk)

/*!*****************************************************************************
 * @brief   Macro to determine the number of enabled channels via a popcount
 *          algorithm.
 * @param   pxmsk 32-bit pixel mask
 * @param   chmsk 32-bit channel mask
 * @return  The count of enabled ADC channels.
 ******************************************************************************/
#define CHANNEL_COUNT(pxmsk, chmsk) (popcount(pxmsk) + popcount(chmsk))

/*!*****************************************************************************
 * @brief   Converts a raw ADC channel mask to a x-y-sorted pixel mask.
 * @param   msk The raw ADC channel mask to be converted.
 * @return  The converted x-y-sorted pixel mask.
 ******************************************************************************/
static inline uint32_t ChannelToPixelMask(uint32_t msk)
{
	uint32_t res = 0;

	for (uint_fast8_t n = 0; n < 32; n += 2) {
		res |= ((msk >> PIXEL_N2CH(n)) & 0x3U) << n; // sets 2 bits at once
	}

	return res;
}

/*!*****************************************************************************
 * @brief   Converts a x-y-sorted pixel mask to a raw ADC channel mask.
 * @param   msk The x-y-sorted pixel channel mask to be converted.
 * @return  The converted raw ADC channel mask.
 ******************************************************************************/
static inline uint32_t PixelToChannelMask(uint32_t msk)
{
	uint32_t res = 0;

	for (uint_fast8_t ch = 0; ch < 32; ch += 2) {
		res |= ((msk >> PIXEL_CH2N(ch)) & 0x3U) << ch; // sets 2 bits at once
	}

	return res;
}


/*!*****************************************************************************
 * @brief   Shifts a pixel mask by a given offset.
 *
 * @details This moves the selected pixel pattern by a specified number of
 *          pixels in x and y direction.
 *          If the shift in y direction is odd (e.g +1), the pattern will be
 *          shifted by +0.5 or -0.5 in x direction due to the hexagonal shape
 *          of the pixel field. Thus, a center pixel (usually the Golden Pixel)
 *          is determined that is used to determine if the pattern is shifted
 *          by +0.5 or -0.5 pixels in x direction. The center pixel is then
 *          always shifted without changing the x index and the surrounding
 *          pixels are adopting its x index accordingly.
 *
 *          Example: Consider the flower pattern, i.e. the Golden Pixel (e.g.
 *          5/2) is selected and all is direct neighbors (i.e. 5/1, 6/1, 6/2,
 *          6/3, 5/3, 4/2). If the pattern is shifted by -1 in y direction, the
 *          new Golden Pixel would be 5/1. Now all surrounding pixels are
 *          selected, namely 4/0, 4/1, 4/2, 5/0, 5/2, 6/1). This yields again
 *          the flower around the Golden Pixel.
 *
 *          Thus, the pixels can not all be shifted by the same dx/dy values due
 *          to the hexagonal shape of the pixel field, e.g. the upper right
 *          neighbor of 5/2 is 5/1 but the upper right neighbor of 5/1 is NOT
 *          5/0 but 4/0!
 *          This happens only if the shift in y direction is an odd number.
 *          The algorithm to determine new indices is as follows:
 *          - If the shift in y direction is even (e.g. +2, -2), no compensation
 *            of the hexagonal shape is needed; skip compensation, simply
 *            add/subtract indices.
 *          - If the center pixel y index is even, pixels that will have even y
 *            index after the shift will be additionally shifted by -1 in x
 *            direction.
 *          - If the center pixel y index is odd, pixel that will have odd y
 *            index after the shift will be additionally shifted by +1 in x
 *            direction.
 *
 * @see     Please also refer to the function #Argus_GetCalibrationGoldenPixel
 *          to obtain the current Golden Pixel location.
 *
 * @param   pixel_mask The x-y-sorted pixel mask to be shifted.
 * @param   dx The number of pixel to shift in x direction.
 * @param   dy The number of pixel to shift in y direction.
 * @param   center_y The center y index of the pattern that is shifted.
 * @return  The shifted pixel mask.
 ******************************************************************************/
static inline uint32_t ShiftSelectedPixels(const uint32_t pixel_mask,
		const int8_t dx,
		const int8_t dy,
		const uint8_t center_y)
{
	if (dx == 0 && dy == 0) { return pixel_mask; }

	uint32_t shifted_mask = 0;

	for (int8_t x = 0; x < ARGUS_PIXELS_X; ++x) {
		for (int8_t y = 0; y < ARGUS_PIXELS_Y; ++y) {
			int8_t x_src = (int8_t)(x - dx);
			int8_t y_src = (int8_t)(y - dy);

			if (dy & 0x1) {
				/* Compensate for hexagonal pixel shape. */
				if ((center_y & 0x1) && (y & 0x1)) {
					x_src--;
				}

				if (!(center_y & 0x1) && !(y & 0x1)) {
					x_src++;
				}
			}

			if (x_src < 0 || x_src >= ARGUS_PIXELS_X) { continue; }

			if (y_src < 0 || y_src >= ARGUS_PIXELS_Y) { continue; }

			if (PIXELXY_ISENABLED(pixel_mask, x_src, y_src)) {
				PIXELXY_ENABLE(shifted_mask, x, y);
			}
		}
	}

	return shifted_mask;
}

/*!*****************************************************************************
 * @brief   Fills a pixel mask to a specified number of pixels around a center pixel.
 *
 * @details The pixel mask is iteratively filled with the nearest pixel to a
 *          specified center pixel until a specified number of pixels is achieved.
 *          The distance between two pixel is determined via a quadratic metric,
 *          i.e. dx^2 + dy^2. Pixels towards the lower x indices are preferred.
 *
 *          Note that the distance of only calculated approximately, e.g. the
 *          y distance of pixels is considered to be 2 instead of cos(60)*2.
 *
 *          Nothing is done if the number of pixels already exceeds the specified
 *          /p pixel_count parameter.
 *
 * @see     Please also refer to the function #Argus_GetCalibrationGoldenPixel
 *          to obtain the current Golden Pixel location.
 *
 * @param   pixel_mask The x-y-sorted pixel mask to be filled with pixels.
 * @param   pixel_count The final number of pixels in the pixel mask.
 * @param   center_x The center pixel x-index.
 * @param   center_y The center pixel y-index.
 * @return  The filled pixel mask with at least /p pixel_count pixels selected.
 ******************************************************************************/
static inline uint32_t FillPixelMask(uint32_t pixel_mask,
				     const uint8_t pixel_count,
				     const uint8_t center_x,
				     const uint8_t center_y)
{
	assert(pixel_count <= ARGUS_PIXELS);
	assert(center_x < ARGUS_PIXELS_X);
	assert(center_y < ARGUS_PIXELS_Y);

	if (pixel_count == ARGUS_PIXELS) { return 0xFFFFFFFFU; }

	/* If the pattern was shifted towards boundaries, the pixel count may have
	 * decreased. In this case, the pixels closest to the reference pixel are
	 * selected. Pixel towards lower x index are prioritized. */
	while (pixel_count > PIXEL_COUNT(pixel_mask)) {
		int32_t min_dist = INT32_MAX;
		int8_t min_x = -1;
		int8_t min_y = -1;

		/* Find nearest not selected pixel. */
		for (int8_t x = 0; x < ARGUS_PIXELS_X; ++x) {
			for (int8_t y = 0; y < ARGUS_PIXELS_Y; ++y) {
				if (!PIXELXY_ISENABLED(pixel_mask, x, y)) {
					int32_t distx = (x - center_x) << 1;

					if (!(y & 0x1)) { distx++; }

					if (!(center_y & 0x1)) { distx--; }

					const int32_t disty = (y - center_y) << 1;
					int32_t dist = distx * distx + disty * disty;

					if (dist < min_dist) {
						min_dist = dist;
						min_x = (int8_t)x;
						min_y = (int8_t)y;
					}
				}
			}
		}

		assert(min_x >= 0 && min_x < ARGUS_PIXELS_X);
		assert(min_y >= 0 && min_y < ARGUS_PIXELS_Y);
		assert(!PIXELXY_ISENABLED(pixel_mask, min_x, min_y));
		PIXELXY_ENABLE(pixel_mask, min_x, min_y);
	}

	return pixel_mask;
}

/*!*****************************************************************************
 * @brief   Fills a pixel mask with the direct neighboring pixels around a pixel.
 *
 * @details The pixel mask is iteratively filled with the direct neighbors of the
 *          specified center pixel.
 *
 *          Note that the function is able to handle corner and edge pixels and
 *          also to handle odd/even lines (which have different layouts)
 *
 * @param   x The selected pixel x-index.
 * @param   y The selected pixel y-index.
 * @return  The filled pixel mask with all direct neighbors of the selected pixel.
 ******************************************************************************/
static inline uint32_t GetAdjacentPixelsMask(const uint_fast8_t x,
		const uint_fast8_t y)
{
	assert(x < ARGUS_PIXELS_X);
	assert(y < ARGUS_PIXELS_Y);

	uint32_t mask = 0u;

	bool isXEdgeLow = (x == 0);
	bool isXEdgeHigh = (x == (ARGUS_PIXELS_X - 1));
	bool isYEdgeLow = (y == 0);
	bool isYEdgeHigh = (y == (ARGUS_PIXELS_Y - 1));

	if (y % 2 == 0) {
		if (!isYEdgeLow) { PIXELXY_ENABLE(mask, x,     y - 1); }

		if ((!isXEdgeHigh) && (!isYEdgeLow)) { PIXELXY_ENABLE(mask, x + 1, y - 1); }

		if (!isXEdgeHigh) { PIXELXY_ENABLE(mask, x + 1, y); }

		if ((!isXEdgeHigh) && (!isYEdgeHigh)) { PIXELXY_ENABLE(mask, x + 1, y + 1); }

		if (!isYEdgeHigh) { PIXELXY_ENABLE(mask, x,     y + 1); }

		if (!isXEdgeLow) { PIXELXY_ENABLE(mask, x - 1, y); }

	} else {
		if ((!isXEdgeLow) && (!isYEdgeLow)) { PIXELXY_ENABLE(mask, x - 1, y - 1); }

		if (!isYEdgeLow) { PIXELXY_ENABLE(mask, x,     y - 1); }

		if (!isXEdgeHigh) { PIXELXY_ENABLE(mask, x + 1, y); }

		if (!isYEdgeHigh) { PIXELXY_ENABLE(mask, x,     y + 1); }

		if ((!isXEdgeLow) && (!isYEdgeHigh)) { PIXELXY_ENABLE(mask, x - 1, y + 1); }

		if (!isXEdgeLow) { PIXELXY_ENABLE(mask, x - 1, y); }
	}

	return mask;
}


/*! @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif /* ARGUS_MAP_H */
