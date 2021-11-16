/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		Defines the device pixel measurement results data structure.
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

#ifndef ARGUS_PX_H
#define ARGUS_PX_H

/*!***************************************************************************
 * @addtogroup 	argusres
 * @{
 *****************************************************************************/

/*! Maximum amplitude value in UQ12.4 format. */
#define ARGUS_AMPLITUDE_MAX		(0xFFF0U)

/*! Maximum range value in Q9.22 format.
 * Also used as a special value to determine no object detected or infinity range. */
#define ARGUS_RANGE_MAX (Q9_22_MAX)

/*!***************************************************************************
 * @brief	Status flags for the evaluated pixel structure.
 *
 * @details Determines the pixel status. 0 means OK (#PIXEL_OK).
 * 			- [0]: #PIXEL_OFF: Pixel was disabled and not read from the device.
 * 			- [1]: #PIXEL_SAT: The pixel was saturated.
 * 			- [2]: #PIXEL_BIN_EXCL: The pixel was excluded from the 1D result.
 * 			- [3]: #PIXEL_AMPL_MIN: The pixel amplitude has evaluated to 0.
 * 			- [4]: #PIXEL_PREFILTERED: The was pre-filtered by static mask.
 * 			- [5]: #PIXEL_NO_SIGNAL: The pixel has no valid signal.
 * 			- [6]: #PIXEL_OUT_OF_SYNC: The pixel has lost signal trace.
 * 			- [7]: #PIXEL_STALLED: The pixel value is stalled due to errors.
 * 			.
 *****************************************************************************/
typedef enum {
	/*! 0x00: Pixel status OK. */
	PIXEL_OK = 0,

	/*! 0x01: Pixel is disabled (in hardware) and no data has been read from the device. */
	PIXEL_OFF = 1U << 0U,

	/*! 0x02: Pixel is saturated (i.e. at least one saturation bit for any
	 *        sample is set or the sample is in the invalidity area). */
	PIXEL_SAT = 1U << 1U,

	/*! 0x04: Pixel is excluded from the pixel binning (1d) result. */
	PIXEL_BIN_EXCL = 1U << 2U,

	/*! 0x08: Pixel amplitude minimum underrun
	 *        (i.e. the amplitude calculation yields 0). */
	PIXEL_AMPL_MIN = 1U << 3U,

	/*! 0x10: Pixel is pre-filtered by the static pixel binning pre-filter mask,
	 * 	      i.e. the pixel is disabled by software. */
	PIXEL_PREFILTERED = 1U << 4U,

	/*! 0x20: Pixel amplitude is below its threshold value. The received signal
	 *  	  strength is too low to evaluate a valid signal. The range value is
	 *  	  set to the maximum possible value (approx. 512 m). */
	PIXEL_NO_SIGNAL = 1U << 5U,

	/*! 0x40: Pixel is not in sync with respect to the dual frequency algorithm.
	 * 		  I.e. the pixel may have a correct value but is estimated into the
	 * 		  wrong unambiguous window. */
	PIXEL_OUT_OF_SYNC = 1U << 6U,

	/*! 0x80: Pixel is stalled due to one of the following reasons:
	 * 		  - #PIXEL_SAT
	 * 		  - #PIXEL_AMPL_MIN
	 * 		  - #PIXEL_OUT_OF_SYNC
	 * 		  - Global Measurement Error
	 * 		  .
	 * 		  A stalled pixel does not update its measurement data and keeps the
	 * 		  previous values. If the issue is resolved, the stall disappears and
	 * 		  the pixel is updating again. */
	PIXEL_STALLED = 1U << 7U

} argus_px_status_t;

/*!***************************************************************************
 * @brief	The evaluated measurement results per pixel.
 * @details	This structure contains the evaluated data for a single pixel.\n
 * 			If the amplitude is 0, the pixel is turned off or has invalid data.
 *****************************************************************************/
typedef struct {
	/*! Range Values from the device in meter. It is the actual distance before
	 *  software adjustments/calibrations. */
	q9_22_t Range;

	/*! Phase Values from the device in units of PI, i.e. 0 ... 2. */
	uq1_15_t Phase;

	/*! Amplitudes of measured signals in LSB.
	 *  Special values: 0 == Pixel Off, 0xFFFF == Overflow/Error */
	uq12_4_t Amplitude;

	/*! Pixel status; determines if the pixel is disabled, saturated, ..
	 *  See the \link #argus_px_status_t pixel status flags\endlink for more
	 *  information. */
	argus_px_status_t Status;

	/*! The unambiguous window determined by the dual frequency feature. */
	int8_t RangeWindow;

	/*! The raw amplitudes of measured signals in LSB. */
	uq12_4_t AmplitudeRaw;

} argus_pixel_t;

/*!***************************************************************************
 * @brief Representation of a correlation vector containing sine/cosine components.
 *****************************************************************************/
typedef struct {
	/*! The sine component. */
	q15_16_t S;

	/*! The cosine component. */
	q15_16_t C;

} argus_vector_t;

/*! @} */
#endif /* ARGUS_PX_H */
