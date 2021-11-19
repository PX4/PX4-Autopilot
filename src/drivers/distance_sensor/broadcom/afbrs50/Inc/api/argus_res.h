/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		Defines the generic measurement results data structure.
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

#ifndef ARGUS_RES_H
#define ARGUS_RES_H

/*!***************************************************************************
 * @defgroup	argusres Measurement Data
 * @ingroup		argusapi
 *
 * @brief		Measurement results data structures.
 *
 * @details		The interface defines all data structures that correspond to
 * 				the AFBR-S50 measurement results, e.g.
 * 					- 1D distance and amplitude values,
 * 					- 3D distance and amplitude values (i.e. per pixel),
 * 					- Auxiliary channel measurement results (VDD, IAPD, temperature, ...)
 * 					- Device and result status
 * 					- ...
 * 					.
 *
 * @addtogroup 	argusres
 * @{
 *****************************************************************************/

#include "argus_def.h"
#include "argus_px.h"
#include "argus_meas.h"

/*!***************************************************************************
 * @brief	The 1d measurement results data structure.
 * @details The 1d measurement results obtained by the Pixel Binning Algorithm.
 *****************************************************************************/
typedef struct {
	/*! Raw 1D range value in meter (Q9.22 format). The distance obtained by
	 *  the Pixel Binning Algorithm from the current measurement frame. */
	q9_22_t Range;

	/*! The 1D amplitude in LSB (Q12.4 format). The (maximum) amplitude obtained
	 *  by the Pixel Binning Algorithm from the current measurement frame.\n
	 *  Special value: 0 == No/Invalid Result. */
	uq12_4_t Amplitude;

	/*! The current signal quality metric of the 1D range value in percentage:\n
	 *  - 0: n/a,
	 *  - 1: bad signal,
	 *  - 100: good signal. */
	uint8_t SignalQuality;

} argus_results_bin_t;

/*!***************************************************************************
 * @brief	The auxiliary measurement results data structure.
 * @details	The auxiliary measurement results obtained by the auxiliary task.\n
 * 			Special values, i.e. 0xFFFFU, indicate no readout value available.
 *****************************************************************************/
typedef struct {
	/*! VDD ADC channel readout value.\n
	 *  Special Value if no value has been measured:\n
	 *  Invalid/NotAvailable = 0xFFFFU (UQ12_4_MAX) */
	uq12_4_t VDD;

	/*! Temperature sensor ADC channel readout value.\n
	 *  Special Value if no value has been measured:\n
	 *  Invalid/NotAvailable = 0x7FFFU (Q11_4_MAX) */
	q11_4_t TEMP;

	/*! Substrate Voltage ADC Channel readout value.\n
	 *  Special Value if no value has been measured:\n
	 *  Invalid/NotAvailable = 0xFFFFU (UQ12_4_MAX) */
	uq12_4_t VSUB;

	/*! VDD VCSEL ADC channel readout value.\n
	 *  Special Value if no value has been measured:\n
	 *  Invalid/NotAvailable = 0xFFFFU (UQ12_4_MAX) */
	uq12_4_t VDDL;

	/*! APD current ADC Channel readout value.\n
	 *  Special Value if no value has been measured:\n
	 *  Invalid/NotAvailable = 0xFFFFU (UQ12_4_MAX) */
	uq12_4_t IAPD;

	/*! Background Light Value in arbitrary. units,
	 *  estimated by the substrate voltage control task.\n
	 *  Special Value if no value is available:\n
	 *  Invalid/NotAvailable = 0xFFFFU (UQ12_4_MAX) */
	uq12_4_t BGL;

	/*! Shot Noise Amplitude in LSB units,
	 *  estimated by the shot noise monitor task from
	 *  the average amplitude of the passive pixels.\n
	 *  Special Value if no value is available:\n
	 *  Invalid/NotAvailable = 0xFFFFU (UQ12_4_MAX) */
	uq12_4_t SNA;

} argus_results_aux_t;

/*!***************************************************************************
 * @brief	The measurement results data structure.
 * @details This structure contains all information obtained by a single
 *			distance measurement on the device:
 *			 - The measurement status can be read from the #Status.
 *			 - A timing information is given via the #TimeStamp.
 *			 - Information about the frame state is in the #Frame structure.
 *			 - The 1D distance results are gathered under #Bin.
 *			 - The 3D distance results for each pixel is at #Pixels or #Pixel.
 *			 - Auxiliary values such as temperature can be found at #Auxiliary.
 *			 - Raw data from the device is stored in the #Data array.
 *			 .
 *
 *			The pixel x-y orientation is sketched in the following graph. Note that
 *			the laser source would be on the right side beyond the reference pixel.
 *			See also \link argusmap ADC Channel Mapping\endlink
 * @code
 * 			// Pixel Field: Pixel[x][y]
 * 			//
 * 			// 0  -----------> x
 * 			// |  O O O O O O O O
 * 			// |   O O O O O O O O
 * 			// |  O O O O O O O O          O (ref. Px)
 * 			// y   O O O O O O O O
 * @endcode
 *****************************************************************************/
typedef struct {
	/*! The \link #status_t status\endlink of the current measurement frame.
	 *   - 0 (i.e. #STATUS_OK) for a good measurement signal.
	 *   - > 0 for warnings and weak measurement signal.
	 *   - < 0 for errors and invalid measurement signal. */
	status_t Status;

	/*! Time in milliseconds (measured since the last MCU startup/reset)
	 *  when the measurement was triggered. */
	ltc_t TimeStamp;

	/*! The configuration for the current measurement frame. */
	argus_meas_frame_t Frame;

	/*! Raw x-y-sorted ADC results from the device.\n
	 *  Data is arranged as 32-bit values in following order:
	 *  index > phase; where index is pixel number n and auxiliary ADC channel.\n
	 *  Note that disabled pixels are skipped.\n
	 *  e.g. [n=0,p=0][n=0,p=1]..[n=0,p=3][n=1,p=0]...[n=1,p=3]...[n=31,p=3] */
	uint32_t Data[ARGUS_RAW_DATA_VALUES];

	union {
		/*! Pixel data indexed by channel number n.\n
		 *  Contains calibrated range, amplitude and pixel status among others.
		 *
		 *  Index n:
		 *  - 0..31: active pixels
		 *  - 32:    reference pixel
		 *
		 *  See also \link argusmap ADC Channel Mapping\endlink */
		argus_pixel_t Pixels[ARGUS_PIXELS + 1U];

		struct {
			/*! Pixel data indexed by x-y-indices.\n
			 *  The pixels are ordered in a two dimensional array that represent
			 *  the x and y indices of the pixel.\n
			*	See also \link argusmap ADC Channel Mapping\endlink
			 *
			 *  Contains calibrated range, amplitude and pixel status among others. */
			argus_pixel_t Pixel[ARGUS_PIXELS_X][ARGUS_PIXELS_Y];

			/*! Pixel data of the reference pixel.\n
			 *  The reference pixel is an additional pixel that is located at the TX
			 *  side in order to monitor the health state of the laser output source.
			 *  It is mainly used to verify normal operation of the laser source and
			 *  preventing the system from emitting continuous laser light that exceeds
			 *  the laser safety limits.
			 *
			 *  Contains calibrated range, amplitude and pixel status among others. */
			argus_pixel_t PixelRef;
		};
	};

	/*! The 1D measurement data, obtained by the the Pixel Binning Algorithm. */
	argus_results_bin_t Bin;

	/*! The auxiliary ADC channel data, e.g. sensor temperature. */
	argus_results_aux_t Auxiliary;

} argus_results_t;


/*! @} */
#endif /* ARGUS_RES_H */
