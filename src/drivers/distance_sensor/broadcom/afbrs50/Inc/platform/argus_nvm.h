/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file provides an interface for the optional non-volatile memory.
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

#ifndef ARGUS_NVM_H
#define ARGUS_NVM_H
#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @defgroup    argus_nvm NVM: Non-Volatile Memory Layer
 * @ingroup     argus_hal
 *
 * @brief       Non-Volatile Memory Layer
 *
 * @details     This module provides functionality to access the non-volatile
 *              memory (e.g. flash) on the underlying platform.
 *
 *              This module is optional and only required if calibration data
 *              needs to be stored within the API.
 *
 * @note        The implementation of this module is optional for the correct
 *              execution of the API. If not implemented, a weak implementation
 *              within the API will be used that disables the NVM feature.
 *
 * @addtogroup  argus_nvm
 * @{
 *****************************************************************************/

#include "api/argus_def.h"

/*! The NVM block size in the non-volatile memory. */
#define ARGUS_NVM_BLOCK_SIZE 0x300 // 768 bytes

/*!***************************************************************************
 * @brief   Write a block of data to the non-volatile memory.
 *
 * @details The function is called whenever the API wants to write data into
 *          non-volatile memory, e.g. flash. Later, the API reads the written
 *          data via the #NVM_ReadBlock function.
 *
 *          The data shall be written to a specified memory block that is
 *          uniquely dedicated to each individual device. The /p id parameter
 *          is passed to the function that identifies the device. The /p id
 *          is composed of the device ID and module type, i.e. it is unique
 *          among all devices. If only a single device is used anyway, the
 *          /p id parameter can be ignored.
 *
 *          If no NVM module is available, the function can return with error
 *          #ERROR_NOT_IMPLEMENTED and the API ignores the NVM.
 *
 *          If write fails, e.g. due to lack of memory, a negative status
 *          must be returned, e.g. #ERROR_NVM_OUT_OF_RANGE.
 *
 *          The block size is fixed for a single device. The actual block size
 *          is defined with #ARGUS_NVM_BLOCK_SIZE.
 *
 * @note    The implementation of this function is optional for the correct
 *          execution of the API. If not implemented, a weak implementation
 *          within the API will be used that disables the NVM feature.
 *
 * @param   id The 32-bit ID number to identify the corresponding memory block.
 * @param   block_size The number of bytes to be written. Note that this value
 *                     is fixed, i.e. the API always writes the same data size.
 *                     The size is defined here: #ARGUS_NVM_BLOCK_SIZE.
 * @param   buf The pointer to the data buffer of size /p block_size that needs
 *              to be written to the NVM.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t NVM_WriteBlock(uint32_t id, uint32_t block_size, uint8_t const *buf);

/*!***************************************************************************
 * @brief   Reads a block of data from the non-volatile memory.
 *
 * @details The function is called whenever the API wants to read data from
 *          non-volatile memory, e.g. flash. The data will be previously
 *          stored using the #NVM_WriteBlock function. Otherwise, the function
 *          must return a corresponding error code, namely #ERROR_NVM_EMPTY.
 *
 *          The data shall be read from a specified memory block that is
 *          uniquely dedicated to each individual device. The /p id parameter
 *          is passed to the function that identifies the device. The /p id
 *          is composed of the device ID and module type, i.e. it is unique
 *          among all devices. If only a single device is used anyway, the
 *          /p id parameter can be ignored.
 *
 *          If no NVM module is available, the function can return with error
 *          #ERROR_NOT_IMPLEMENTED and the API ignores the NVM.
 *
 *          If read fails, e.g. if data has not been written previously,
 *          a negative status must be returned, e.g. #ERROR_NVM_EMPTY if no
 *          data has been written yet or any other negative error else-wise.
 *
 *          The block size is fixed for a single device. The actual block size
 *          is defined with #ARGUS_NVM_BLOCK_SIZE.
 *
 * @note    The implementation of this function is optional for the correct
 *          execution of the API. If not implemented, a weak implementation
 *          within the API will be used that disables the NVM feature.
 *
 * @param   id The 32-bit ID number to identify the corresponding memory block.
 * @param   block_size The number of bytes to be read. Note that this value
 *                     is fixed, i.e. the API always reads the same data size.
 *                     The size is defined here: #ARGUS_NVM_BLOCK_SIZE.
 * @param   buf The pointer to the data buffer of size /p block_size to copy
 *              the data to.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t NVM_ReadBlock(uint32_t id, uint32_t block_size, uint8_t *buf);

/*! @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // ARGUS_NVM_H
