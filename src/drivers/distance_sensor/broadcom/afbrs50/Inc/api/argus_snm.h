/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     Defines the Shot Noise Monitor (SNM) setup parameters.
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

#ifndef ARGUS_SNM_H
#define ARGUS_SNM_H
#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @defgroup    argus_snm Shot Noise Monitor
 * @ingroup     argus_api
 *
 * @brief       Shot Noise Monitor (SNM) parameter definitions and API functions.
 *
 * @details     The SNM is an algorithm to monitor and react on shot noise
 *              induced by harsh environment conditions like high ambient
 *              light.
 *
 *              The AFBR-S50 API provides three modes:
 *              - Dynamic: Automatic mode, automatically adopts to current
 *                         ambient conditions.
 *              - Static (Outdoor): Static mode, optimized for outdoor applications.
 *              - Static (Indoor): Static mode, optimized for indoor applications.
 *              .
 *
 * @addtogroup  argus_snm
 * @{
 *****************************************************************************/

/*! The Shot Noise Monitor modes enumeration. */
typedef enum argus_snm_mode_t {
	/*! Static Shot Noise Monitoring Mode, optimized for indoor applications.
	 *  Assumes the best case scenario, i.e. no bad influence from ambient conditions.
	 *  Thus it uses a fixed setting that will result in the best performance.
	 *  Equivalent to Shot Noise Monitoring disabled. */
	SNM_MODE_STATIC_INDOOR = 0U,

	/*! Static Shot Noise Monitoring Mode, optimized for outdoor applications.
	 *  Assumes the worst case scenario, i.e. it uses a fixed setting that will
	 *  work under all ambient conditions. */
	SNM_MODE_STATIC_OUTDOOR = 1U,

	/*! Dynamic Shot Noise Monitoring Mode.
	 *  Adopts the system performance dynamically to the current ambient conditions. */
	SNM_MODE_DYNAMIC = 2U,

} argus_snm_mode_t;


/*! @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif /* ARGUS_SNM_H */
