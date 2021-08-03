/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides an interface for the optional debug module.
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

#ifndef ARGUS_PRINT_H
#define ARGUS_PRINT_H

/*!***************************************************************************
 * @defgroup	argus_log Debug: Logging Interface
 * @ingroup		argus_platform
 *
 * @brief		Logging interface for the AFBR-S50 API.
 *
 * @details		This interface provides logging utility functions.
 * 				Defines a printf-like function that is used to print error and
 * 				log messages.
 *
 * @addtogroup 	argus_log
 * @{
 *****************************************************************************/

#include "api/argus_def.h"

/*!***************************************************************************
 * @brief	A printf-like function to print formated data to an debugging interface.
 *
 * @details Writes the C string pointed by fmt_t to an output. If format
 * 			includes format specifiers (subsequences beginning with %), the
 * 			additional arguments following fmt_t are formatted and inserted in
 * 			the resulting string replacing their respective specifiers.
 *
 * 			To enable the print functionality, an implementation of the function
 * 			must be provided that maps the output to an interface like UART or
 * 			a debugging console, e.g. by forwarding to standard printf() method.
 *
 * @note	The implementation of this function is optional for the correct
 * 			execution of the API. If not implemented, a weak implementation
 * 			within the API will be used that does nothing. This will improve
 * 			the performance but no error messages are logged.
 *
 * @note	The naming is different from the standard printf() on purpose to
 * 			prevent builtin compiler optimizations.
 *
 * @param	fmt_s The usual print() format string.
 * @param	... The usual print() parameters. *
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t print(const char *fmt_s, ...);

/*! @} */
#endif /* ARGUS_PRINT_H */
