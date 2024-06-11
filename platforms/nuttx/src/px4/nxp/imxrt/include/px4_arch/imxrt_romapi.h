/****************************************************************************
 * platforms/nuttx/src/px4/nxp/imrt/include/px4_arch/imxrt_romapi.h
 *
 * Copyright 2017-2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
****************************************************************************/
#ifndef __PX4_ARCH_IMXRT_ROMAPI_H
#define __PX4_ARCH_IMXRT_ROMAPI_H

/****************************************************************************
 *
 * Included Files
 ****************************************************************************/

#include "board_config.h"

typedef int32_t status_t;
typedef struct flexspi_nor_config_s flexspi_nor_config_t;
typedef status_t (*clearCacheCommand_t)(uint32_t instance);

/*! @brief FLEXSPI Operation Context */
typedef enum _flexspi_operation {
	kFLEXSPIOperation_Command,  /*!< FLEXSPI operation: Only command, both TX and RX buffer are ignored. */
	kFLEXSPIOperation_Config,   /*!< FLEXSPI operation: Configure device mode, the TX FIFO size is fixed in LUT. */
	kFLEXSPIOperation_Write,    /*!< FLEXSPI operation: Write, only TX buffer is effective */
	kFLEXSPIOperation_Read,     /*!< FLEXSPI operation: Read, only Rx Buffer is effective. */
} flexspi_operation_t;

#define kFLEXSPIOperation_End   kFLEXSPIOperation_Read

/*! @brief FLEXSPI Transfer Context */
typedef struct _flexspi_xfer {
	flexspi_operation_t operation; /*!< FLEXSPI operation */
	uint32_t baseAddress;          /*!< FLEXSPI operation base address */
	uint32_t seqId;                /*!< Sequence Id */
	uint32_t seqNum;               /*!< Sequence Number */
	bool isParallelModeEnable;     /*!< Is a parallel transfer */
	uint32_t *txBuffer;            /*!< Tx buffer */
	uint32_t txSize;               /*!< Tx size in bytes */
	uint32_t *rxBuffer;            /*!< Rx buffer */
	uint32_t rxSize;               /*!< Rx size in bytes */
} flexspi_xfer_t;

/*! @brief convert the type for MISRA */
#define MISRA_CAST(to_type, to_var, from_type, from_var)      \
	do                                                        \
	{                                                         \
		union                                                 \
		{                                                     \
			to_type to_var_tmp;                               \
			from_type from_var_tmp;                           \
		} type_converter_var = {.from_var_tmp = (from_var)};  \
		(to_var)             = type_converter_var.to_var_tmp; \
	} while (false)

#ifdef __cplusplus
extern "C" {
#endif

extern struct flexspi_nor_config_s g_bootConfig;

/*!
 * @brief ROM API init
 *
 * Get the bootloader api entry address.
 */
void ROM_API_Init(void);

/*!
 * @name Enter Bootloader
 * @{
 */

/*!
 * @brief Enter Bootloader.
 *
 * @param arg A pointer to the storage for the bootloader param.
 *        refer to System Boot Chapter in device reference manual for details.
 */
void ROM_RunBootloader(void *arg);

/*@}*/

/*!
 * @name GetConfig
 * @{
 */
/*!
 * @brief Get FLEXSPI NOR Configuration Block based on specified option.
 *
 * @param instance storage the instance of FLEXSPI.
 * @param config A pointer to the storage for the driver runtime state.
 * @param option A pointer to the storage Serial NOR Configuration Option Context.
 *
 * @retval kStatus_Success Api was executed successfully.
 * @retval kStatus_InvalidArgument A invalid argument is provided.
 * @retval kStatus_ROM_FLEXSPI_InvalidSequence A invalid Sequence is provided.
 * @retval kStatus_ROM_FLEXSPI_SequenceExecutionTimeout Sequence Execution timeout.
 * @retval kStatus_ROM_FLEXSPI_DeviceTimeout the device timeout
 */
status_t ROM_FLEXSPI_NorFlash_GetConfig(uint32_t instance,
					flexspi_nor_config_t *config,
					serial_nor_config_option_t *option);

/*!
 * @name Initialization
 * @{
 */

/*!
 * @brief Initialize Serial NOR devices via FLEXSPI
 *
 * This function checks and initializes the FLEXSPI module for the other FLEXSPI APIs.
 *
 * @param instance storage the instance of FLEXSPI.
 * @param config A pointer to the storage for the driver runtime state.
 *
 * @retval kStatus_Success Api was executed successfully.
 * @retval kStatus_InvalidArgument A invalid argument is provided.
 * @retval kStatus_ROM_FLEXSPI_InvalidSequence A invalid Sequence is provided.
 * @retval kStatus_ROM_FLEXSPI_SequenceExecutionTimeout Sequence Execution timeout.
 * @retval kStatus_ROM_FLEXSPI_DeviceTimeout the device timeout
 */
status_t ROM_FLEXSPI_NorFlash_Init(uint32_t instance, flexspi_nor_config_t *config);

/*@}*/

/*!
 * @name Programming
 * @{
 */

/*!
 * @brief Program data to Serial NOR via FLEXSPI.
 *
 * This function programs the NOR flash memory with the dest address for a given
 * flash area as determined by the dst address and the length.
 *
 * @param instance storage the instance of FLEXSPI.
 * @param config  A pointer to the storage for the driver runtime state.
 * @param dst_addr A pointer to the desired flash memory to be programmed.
 *                @note It is recommended that use page aligned access;
 *                If the dst_addr is not aligned to page, the driver automatically
 *                aligns address down with the page address.
 * @param src A pointer to the source buffer of data that is to be programmed
 *            into the NOR flash.
 *
 * @retval kStatus_Success Api was executed successfully.
 * @retval kStatus_InvalidArgument A invalid argument is provided.
 * @retval kStatus_ROM_FLEXSPI_InvalidSequence A invalid Sequence is provided.
 * @retval kStatus_ROM_FLEXSPI_SequenceExecutionTimeout Sequence Execution timeout.
 * @retval kStatus_ROM_FLEXSPI_DeviceTimeout the device timeout
 */
status_t ROM_FLEXSPI_NorFlash_ProgramPage(uint32_t instance,
		flexspi_nor_config_t *config,
		uint32_t dst_addr,
		const uint32_t *src);

/*@}*/

/*!
 * @name Reading
 * @{
 */

/*!
 * @brief Read data from Serial NOR via FLEXSPI.
 *
 * This function read the NOR flash memory with the start address for a given
 * flash area as determined by the dst address and the length.
 *
 * @param instance storage the instance of FLEXSPI.
 * @param config  A pointer to the storage for the driver runtime state.
 * @param dst     A pointer to the dest buffer of data that is to be read from the NOR flash.
 *                @note It is recommended that use page aligned access;
 *                If the dstAddr is not aligned to page, the driver automatically
 *                aligns address down with the page address.
 * @param start   The start address of the desired NOR flash memory to be read.
 * @param lengthInBytes The length, given in bytes to be read.
 *
 * @retval kStatus_Success Api was executed successfully.
 * @retval kStatus_InvalidArgument A invalid argument is provided.
 * @retval kStatus_ROM_FLEXSPI_InvalidSequence A invalid Sequence is provided.
 * @retval kStatus_ROM_FLEXSPI_SequenceExecutionTimeout Sequence Execution timeout.
 * @retval kStatus_ROM_FLEXSPI_DeviceTimeout the device timeout
 */
status_t ROM_FLEXSPI_NorFlash_Read(
	uint32_t instance, flexspi_nor_config_t *config, uint32_t *dst, uint32_t start, uint32_t lengthInBytes);

/*@}*/

/*!
 * @name Erasing
 * @{
 */

/*!
 * @brief Erase Flash Region specified by address and length
 *
 * This function erases the appropriate number of flash sectors based on the
 * desired start address and length.
 *
 * @param instance storage the index of FLEXSPI.
 * @param config A pointer to the storage for the driver runtime state.
 * @param start The start address of the desired NOR flash memory to be erased.
 *              @note It is recommended that use sector-aligned access nor device;
 *              If dstAddr is not aligned with the sector,the driver automatically
 *              aligns address down with the sector address.
 * @param length The length, given in bytes to be erased.
 *              @note It is recommended that use sector-aligned access nor device;
 *              If length is not aligned with the sector,the driver automatically
 *              aligns up with the sector.
 * @retval kStatus_Success Api was executed successfully.
 * @retval kStatus_InvalidArgument A invalid argument is provided.
 * @retval kStatus_ROM_FLEXSPI_InvalidSequence A invalid Sequence is provided.
 * @retval kStatus_ROM_FLEXSPI_SequenceExecutionTimeout Sequence Execution timeout.
 * @retval kStatus_ROM_FLEXSPI_DeviceTimeout the device timeout
 */
status_t ROM_FLEXSPI_NorFlash_Erase(uint32_t instance, flexspi_nor_config_t *config, uint32_t start, uint32_t length);

/*!
 * @brief Erase one sector specified by address
 *
 * This function erases one of NOR flash sectors based on the desired address.
 *
 * @param instance storage the index of FLEXSPI.
 * @param config A pointer to the storage for the driver runtime state.
 * @param start The start address of the desired NOR flash memory to be erased.
 *                @note It is recommended that use sector-aligned access nor device;
 *                If dstAddr is not aligned with the sector, the driver automatically
 *                aligns address down with the sector address.
 *
 * @retval kStatus_Success Api was executed successfully.
 * @retval kStatus_InvalidArgument A invalid argument is provided.
 * @retval kStatus_ROM_FLEXSPI_InvalidSequence A invalid Sequence is provided.
 * @retval kStatus_ROM_FLEXSPI_SequenceExecutionTimeout Sequence Execution timeout.
 * @retval kStatus_ROM_FLEXSPI_DeviceTimeout the device timeout
 */
status_t ROM_FLEXSPI_NorFlash_EraseSector(uint32_t instance, flexspi_nor_config_t *config, uint32_t start);

/*!
 * @brief Erase one block specified by address
 *
 * This function erases one block of NOR flash based on the desired address.
 *
 * @param instance storage the index of FLEXSPI.
 * @param config A pointer to the storage for the driver runtime state.
 * @param start The start address of the desired NOR flash memory to be erased.
 *              @note It is recommended that use block-aligned access nor device;
 *              If dstAddr is not aligned with the block, the driver automatically
 *              aligns address down with the block address.
 *
 * @retval kStatus_Success Api was executed successfully.
 * @retval kStatus_InvalidArgument A invalid argument is provided.
 * @retval kStatus_ROM_FLEXSPI_InvalidSequence A invalid Sequence is provided.
 * @retval kStatus_ROM_FLEXSPI_SequenceExecutionTimeout Sequence Execution timeout.
 * @retval kStatus_ROM_FLEXSPI_DeviceTimeout the device timeout
 */
status_t ROM_FLEXSPI_NorFlash_EraseBlock(uint32_t instance, flexspi_nor_config_t *config, uint32_t start);

/*!
 * @brief Erase all the Serial NOR devices connected on FLEXSPI.
 *
 * @param instance storage the instance of FLEXSPI.
 * @param config A pointer to the storage for the driver runtime state.
 *
 * @retval kStatus_Success Api was executed successfully.
 * @retval kStatus_InvalidArgument A invalid argument is provided.
 * @retval kStatus_ROM_FLEXSPI_InvalidSequence A invalid Sequence is provided.
 * @retval kStatus_ROM_FLEXSPI_SequenceExecutionTimeout Sequence Execution timeout.
 * @retval kStatus_ROM_FLEXSPI_DeviceTimeout the device timeout
 */
status_t ROM_FLEXSPI_NorFlash_EraseAll(uint32_t instance, flexspi_nor_config_t *config);

/*@}*/

/*!
 * @name Command
 * @{
 */
/*!
 * @brief FLEXSPI command
 *
 * This function is used to perform the command write sequence to the NOR device.
 *
 * @param instance storage the index of FLEXSPI.
 * @param xfer A pointer to the storage FLEXSPI Transfer Context.
 *
 * @retval kStatus_Success Api was executed successfully.
 * @retval kStatus_InvalidArgument A invalid argument is provided.
 * @retval kStatus_ROM_FLEXSPI_InvalidSequence A invalid Sequence is provided.
 * @retval kStatus_ROM_FLEXSPI_SequenceExecutionTimeout Sequence Execution timeout.
 */
status_t ROM_FLEXSPI_NorFlash_CommandXfer(uint32_t instance, flexspi_xfer_t *xfer);

/*@}*/

/*!
 * @name UpdateLut
 * @{
 */

/*!
 * @brief Configure FLEXSPI Lookup table
 *
 * @param instance storage the index of FLEXSPI.
 * @param seqIndex storage the sequence Id.
 * @param lutBase A pointer to the look-up-table for command sequences.
 * @param seqNumber storage sequence number.
 *
 * @retval kStatus_Success Api was executed successfully.
 * @retval kStatus_InvalidArgument A invalid argument is provided.
 * @retval kStatus_ROM_FLEXSPI_InvalidSequence A invalid Sequence is provided.
 * @retval kStatus_ROM_FLEXSPI_SequenceExecutionTimeout Sequence Execution timeout.
 */
status_t ROM_FLEXSPI_NorFlash_UpdateLut(uint32_t instance,
					uint32_t seqIndex,
					const uint32_t *lutBase,
					uint32_t seqNumber);


/*@}*/

/*!
 * @name Device status
 * @{
 */
/*!
 * @brief Wait until device is idle.
 *
 * @param instance  Indicates the index of FLEXSPI.
 * @param config    A pointer to the storage for the driver runtime state
 * @param isParallelMode Indicates whether NOR flash is in parallel mode.
 * @param address  Indicates the operation(erase/program/read) address for serial NOR flash.
 *
 * @retval kStatus_Success Api was executed successfully.
 * @retval kStatus_InvalidArgument A invalid argument is provided.
 * @retval kStatus_ROM_FLEXSPI_SequenceExecutionTimeout Sequence Execution timeout.
 * @retval kStatus_ROM_FLEXSPI_InvalidSequence A invalid Sequence is provided.
 * @retval kStatus_ROM_FLEXSPI_DeviceTimeout Device timeout.
 */
status_t ROM_FLEXSPI_NorFlash_WaitBusy(uint32_t instance,
				       flexspi_nor_config_t *config,
				       bool isParallelMode,
				       uint32_t address);
/*@}*/

/*!
 * @name ClearCache
 * @{
 */

/*!
 * @name ClearCache
 * @{
 */

/*!
 * @brief Software reset for the FLEXSPI logic.
 *
 * This function sets the software reset flags for both AHB and buffer domain and
 * resets both AHB buffer and also IP FIFOs.
 *
 * @param instance storage the index of FLEXSPI.
 */
void ROM_FLEXSPI_NorFlash_ClearCache(uint32_t instance);

/*@}*/

#ifdef __cplusplus
}
#endif

#endif /* __PX4_ARCH_IMXRT_ROMAPI_H */
