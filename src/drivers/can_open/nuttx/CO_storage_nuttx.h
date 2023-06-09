/****************************************************************************
 *
 *   Copyright (c) 2014-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

// NuttX storage wrapper for CanOpenNode

#ifndef CO_STORAGE_NUTTX_H
#define CO_STORAGE_NUTTX_H

#include "storage/CO_storage.h"

#if ((CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE) || defined CO_DOXYGEN

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup CO_storageLinux Data storage with Linux
 * Data initialize, store and restore functions with Linux.
 *
 * @ingroup CO_socketCAN
 * @{
 *
 * See also @ref CO_storage.
 */


/**
 * Initialize data storage object (Nuttx specific)
 *
 * This function should be called by application after the program startup,
 * before @ref CO_CANopenInit(). This function initializes storage object,
 * OD extensions on objects 1010 and 1011, reads data from file, verifies them
 * and writes data to addresses specified inside entries. This function
 * internally calls @ref CO_storage_init().
 *
 * @param storage This object will be initialized. It must be defined by
 * application and must exist permanently.
 * @param CANmodule CAN device, used for @ref CO_LOCK_OD() macro.
 * @param OD_1010_StoreParameters OD entry for 0x1010 -"Store parameters".
 * Entry is optional, may be NULL.
 * @param OD_1011_RestoreDefaultParam OD entry for 0x1011 -"Restore default
 * parameters". Entry is optional, may be NULL.
 * @param entries Pointer to array of storage entries, see @ref CO_storage_init.
 * @param entriesCount Count of storage entries
 * @param [out] storageInitError If function returns CO_ERROR_DATA_CORRUPT,
 * then this variable contains a bit mask from subIndexOD values, where data
 * was not properly initialized. If other error, then this variable contains
 * index or erroneous entry.
 *
 * @return CO_ERROR_NO, CO_ERROR_DATA_CORRUPT if data can not be initialized,
 * CO_ERROR_ILLEGAL_ARGUMENT or CO_ERROR_OUT_OF_MEMORY.
 */
CO_ReturnError_t CO_storage_nuttx_init(CO_storage_t *storage,
									   CO_CANmodule_t *CANmodule,
									   OD_entry_t *OD_1010_StoreParameters,
									   OD_entry_t *OD_1011_RestoreDefaultParam,
									   CO_storage_entry_t *entries,
									   uint8_t entriesCount,
									   uint32_t *storageInitError);


/**
 * Automatically save data if differs from previous call.
 *
 * Should be called cyclically by program. Each interval it verifies, if crc
 * checksum of data differs from previous checksum. If it does, data are saved
 * into pre-opened file.
 *
 * @param storage This object
 * @param closeFiles If true, then all files will be closed. Use on end of the
 * program.
 *
 * @return 0 on success or bit mask from subIndexOD values, where data was not
 * able to be saved.
 */
uint32_t CO_storage_nuttx_auto_process(CO_storage_t *storage,
									   bool_t closeFiles);

/** @} */ /* CO_storage_nuttx */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE */

#endif /* CO_STORAGE_NUTTX_H */
