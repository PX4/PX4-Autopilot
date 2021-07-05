/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides an interface for the optional non-volatile memory.
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

#ifndef ARGUS_NVM_H
#define ARGUS_NVM_H

#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @defgroup	argus_nvm NVM: Non-Volatile Memory Layer
 * @ingroup		argus_platform
 *
 * @brief		Non-Volatile Memory Layer
 *
 * @details		This module provides functionality to access the non-volatile
 * 				memory (e.g. flash) on the underlying platform.
 *
 * 				This module is optional and only required if calibration data
 * 				needs to be stored within the API.
 *
 * @note		The implementation of this module is optional for the correct
 * 				execution of the API. If not implemented, a weak implementation
 * 				within the API will be used that disables the NVM feature.
 *
 * @addtogroup 	argus_nvm
 * @{
 *****************************************************************************/

#include "argus.h"

/*!***************************************************************************
 * @brief	Initializes the non-volatile memory unit and reserves a chunk of memory.
 *
 * @details The function is called upon API initialization sequence. If available,
 * 			the non-volatile memory module reserves a chunk of memory with the
 * 			provides number of bytes (size) and returns with #STATUS_OK.
 *
 * 			If not implemented, the function should return #ERROR_NOT_IMPLEMENTED
 * 			in oder to inform the API to not use the NVM module.
 *
 * 			After initialization, the API calls the #NVM_Write and #NVM_Read
 * 			methods to write within the reserved chunk of memory.
 *
 * @note	The implementation of this function is optional for the correct
 * 			execution of the API. If not implemented, a weak implementation
 * 			within the API will be used that disables the NVM feature.
 *
 * @param	size The required size of NVM to store all parameters.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t NVM_Init(uint32_t size);

/*!***************************************************************************
 * @brief	Write a block of data to the non-volatile memory.
 *
 * @details	The function is called whenever the API wants to write data into
 * 			the previously reserved (#NVM_Init) memory block. The data shall
 * 			be written at a given offset and with a given size.
 *
 *			If no NVM module is available, the function can return with error
 *			#ERROR_NOT_IMPLEMENTED.
 *
 * @note	The implementation of this function is optional for the correct
 * 			execution of the API. If not implemented, a weak implementation
 * 			within the API will be used that disables the NVM feature.
 *
 * @param	offset The index offset where the first byte needs to be written.
 * @param	size The number of bytes to be written.
 * @param	buf The pointer to the data buffer with the data to be written.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t NVM_Write(uint32_t offset, uint32_t size, uint8_t const *buf);

/*!***************************************************************************
 * @brief	Reads a block of data from the non-volatile memory.
 *
 * @details	The function is called whenever the API wants to read data from
 * 			the previously reserved (#NVM_Init) memory block. The data shall
 * 			be read at a given offset and with a given size.
 *
 *			If no NVM module is available, the function can return with error
 *			#ERROR_NOT_IMPLEMENTED.
 *
 * @note	The implementation of this function is optional for the correct
 * 			execution of the API. If not implemented, a weak implementation
 * 			within the API will be used that disables the NVM feature.
 *
 * @param	offset The index offset where the first byte needs to be read.
 * @param	size The number of bytes to be read.
 * @param	buf The pointer to the data buffer to copy the data to.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t NVM_Read(uint32_t offset, uint32_t size, uint8_t *buf);

#ifdef __cplusplus
}
#endif

/*! @} */
#endif // ARGUS_NVM_H
