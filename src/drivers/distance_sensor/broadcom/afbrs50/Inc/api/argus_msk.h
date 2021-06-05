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

#ifndef ARGUS_MSK_H
#define ARGUS_MSK_H

/*!***************************************************************************
 * @defgroup	argusmap ADC Channel Mapping
 * @ingroup		argusres
 *
 * @brief		Pixel ADC Channel (n) to x-y-Index Mapping
 *
 * @details		The ADC Channels of each pixel or auxiliary channel on the device
 * 				is numbered in a way that is convenient on the chip. The macros
 * 				in this module are defined in order to obtain the x-y-indices of
 * 				each channel and vice versa.
 *
 * @addtogroup 	argusmap
 * @{
 *****************************************************************************/

#include "api/argus_def.h"
#include "utility/int_math.h"

/*!*****************************************************************************
 * @brief	Macro to determine the channel number of an specified Pixel.
 * @param	x The x index of the pixel.
 * @param	y The y index of the pixel.
 * @return	The channel number n of the pixel.
 ******************************************************************************/
#define PIXEL_XY2N(x, y) ((((x) ^ 7) << 1) | ((y) & 2) << 3 | ((y) & 1))

/*!*****************************************************************************
 * @brief	Macro to determine the x index of an specified Pixel channel.
 * @param	n The channel number of the pixel.
 * @return	The x index number of the pixel.
 ******************************************************************************/
#define PIXEL_N2X(n) ((((n) >> 1U) & 7) ^ 7)

/*!*****************************************************************************
 * @brief	Macro to determine the y index of an specified Pixel channel.
 * @param	n The channel number of the pixel.
 * @return	The y index number of the pixel.
 ******************************************************************************/
#define PIXEL_N2Y(n) (((n) & 1U) | (((n) >> 3) & 2U))

/*!*****************************************************************************
 * @brief	Macro to determine if a ADC Pixel channel was enabled from a pixel mask.
 * @param	msk The 32-bit pixel mask
 * @param	ch The channel number of the pixel.
 * @return 	True if the pixel channel n was enabled, false elsewise.
 ******************************************************************************/
#define PIXELN_ISENABLED(msk, ch) (((msk) >> (ch)) & 0x01U)

/*!*****************************************************************************
 * @brief	Macro enables an ADC Pixel channel in a pixel mask.
 * @param	msk The 32-bit pixel mask
 * @param	ch The channel number of the pixel.
 ******************************************************************************/
#define PIXELN_ENABLE(msk, ch) ((msk) |= (0x01U << (ch)))

/*!*****************************************************************************
 * @brief	Macro disables an ADC Pixel channel in a pixel mask.
 * @param	msk The 32-bit pixel mask
 * @param	ch The channel number of the pixel.
 ******************************************************************************/
#define PIXELN_DISABLE(msk, ch) ((msk) &= (~(0x01U << (ch))))

/*!*****************************************************************************
 * @brief	Macro to determine if an ADC Pixel channel was enabled from a pixel mask.
 * @param	msk 32-bit pixel mask
 * @param	x x index of the pixel.
 * @param	y y index of the pixel.
 * @return 	True if the pixel (x,y) was enabled, false elsewise.
 ******************************************************************************/
#define PIXELXY_ISENABLED(msk, x, y) (PIXELN_ISENABLED(msk, PIXEL_XY2N(x, y)))

/*!*****************************************************************************
 * @brief	Macro enables an ADC Pixel channel in a pixel mask.
 * @param	msk 32-bit pixel mask
 * @param	x x index of the pixel.
 * @param	y y index of the pixel.
 ******************************************************************************/
#define PIXELXY_ENABLE(msk, x, y) (PIXELN_ENABLE(msk, PIXEL_XY2N(x, y)))

/*!*****************************************************************************
 * @brief	Macro disables an ADC Pixel channel in a pixel mask.
 * @param	msk 32-bit pixel mask
 * @param	x x index of the pixel.
 * @param	y y index of the pixel.
 ******************************************************************************/
#define PIXELXY_DISABLE(msk, x, y) (PIXELN_DISABLE(msk, PIXEL_XY2N(x, y)))

/*!*****************************************************************************
 * @brief	Macro to determine if a ADC channel was enabled from a channel mask.
 * @param	msk 32-bit channel mask
 * @param	ch channel number of the ADC channel.
 * @return 	True if the ADC channel n was enabled, false elsewise.
 ******************************************************************************/
#define CHANNELN_ISENABLED(msk, ch) (((msk) >> ((ch) - 32U)) & 0x01U)

/*!*****************************************************************************
 * @brief	Macro to determine if a ADC channel was enabled from a channel mask.
 * @param	msk 32-bit channel mask
 * @param	ch channel number of the ADC channel.
 * @return 	True if the ADC channel n was enabled, false elsewise.
 ******************************************************************************/
#define CHANNELN_ENABLE(msk, ch) ((msk) |= (0x01U << ((ch) - 32U)))

/*!*****************************************************************************
 * @brief	Macro to determine if a ADC channel was enabled from a channel mask.
 * @param	msk 32-bit channel mask
 * @param	ch channel number of the ADC channel.
 * @return 	True if the ADC channel n was enabled, false elsewise.
 ******************************************************************************/
#define CHANNELN_DISABLE(msk, ch) ((msk) &= (~(0x01U << ((ch) - 32U))))


/*!*****************************************************************************
 * @brief	Macro to determine the number of enabled pixel channels via a popcount
 * 			algorithm.
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

/*! @} */
#endif /* ARGUS_MSK_H */
