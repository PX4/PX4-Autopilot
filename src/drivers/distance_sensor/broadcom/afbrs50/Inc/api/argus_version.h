/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file contains the current API version number.
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

#ifndef ARGUS_VERSION_H
#define ARGUS_VERSION_H

/*!***************************************************************************
 * @defgroup	version API Version
 * @ingroup		argusapi
 *
 * @brief		API and library core version number
 *
 * @details		Contains the AFBR-S50 API and Library Core Version Number.
 *
 * @addtogroup 	version
 * @{
 *****************************************************************************/

/*! Major version number of the AFBR-S50 API. */
#define ARGUS_API_VERSION_MAJOR    1

/*! Minor version number of the AFBR-S50 API. */
#define ARGUS_API_VERSION_MINOR    3

/*! Bugfix version number of the AFBR-S50 API. */
#define ARGUS_API_VERSION_BUGFIX   5

/*! Build version nunber of the AFBR-S50 API. */
#define ARGUS_API_VERSION_BUILD    "20210812171515"

/*****************************************************************************/

/*! Construct the version number for drivers. */
#define MAKE_VERSION(major, minor, bugfix) \
	(((major) << 24) | ((minor) << 16) | (bugfix))

/*! Version number of the AFBR-S50 API. */
#define ARGUS_API_VERSION MAKE_VERSION((ARGUS_API_VERSION_MAJOR), \
				       (ARGUS_API_VERSION_MINOR), \
				       (ARGUS_API_VERSION_BUGFIX))

/*! @} */
#endif /* ARGUS_VERSION_H */
