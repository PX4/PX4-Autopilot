#include "load_CAN.h"
#include "load_CAN_app.h"
#include "can_hw.h"
#include <string.h>
#include <stdio.h>

static bool can_module_assign_address(can_module_s *module);
static bool can_module_validate_address(can_module_s *module);
static bool can_module_discover_if_addressed(can_module_s *module);
static bool can_module_erase_flash(can_module_s *module);
static bool can_module_flash_metadata(can_module_s *module);
static bool can_module_get_uid_and_config(can_module_s *module);
static bool can_module_initialize_crc(can_module_s *module);
static bool can_module_initialize_encryption(can_module_s *module);
static bool can_module_initialize_protection_mask(can_module_s *module);
static bool can_module_key_process(can_module_s *module);
static bool can_module_load_key(can_module_s *module);
static bool can_module_read_version(can_module_s *module);
static bool can_module_reset_address(can_module_s *module);
static bool can_module_send_file_crc(can_module_s *module, uint32_t crc);
static bool can_module_set_rdp_protections(can_module_s *module);
static bool can_module_set_wrp_protections(can_module_s *module);
static bool can_module_set_target_pointer(can_module_s *module, uint32_t target);
static bool can_module_unlock_ro(can_module_s *module);
static bool can_module_verify_flash_entry_crc(can_module_s *module);
static bool can_module_write_data(can_module_s *module, uint32_t data);
static bool initialize_address_can_modules(can_module_s *module);
static uint8_t log2p(uint32_t x);
static bool retreive_file_appended_metadata(struct ff_metadata_s *appendedInfo, FILE *fp);
//static uint32_t currentFlashAddress;

bool initialize_module_for_load(can_module_s *module)
{
	module->currentId = 0;
	memset(&module->status, 0, sizeof(module->status));
	module->endFlashAddr = 0;
	memset(&module->flashData, 0, sizeof(module->flashData));
	module->status.errorCode = CAN_MODULE_ERROR_NONE;
	return PROCESS_SUCCESS;
}

bool can_module_type_alternative_loaded(can_module_s *module)
{
	const size_t module_i = module - modules;

	for (size_t i = 0; i < module_i; i++) {
		if (module->module_type == loaded_types[i]) {
			return true;
		}
	}

	return false;
}

bool can_module_type_alternative_exists(can_module_s *module)
{
	const size_t module_i = module - modules;

	for (size_t i = module_i + 1; i < module_total; i++) {
		if (module->module_type == modules[i].module_type) {
			return true;
		}
	}

	return false;
}

bool can_load_all_modules(void)
{
	int usedModuleIndex = 0;

	if (init_modules_app()) {
		handle_can_module_error(&modules[0], CAN_MODULE_ERROR_GENERAL);
		return PROCESS_FAILURE;
	}

	for (size_t i = 0; i < module_total; i++) {
		modules[i].type_id_index = i;

		bool resendBroadcastMessages = true;
		int canIndex = 0;

		for (canIndex = 0; canIndex < usedModuleIndex; canIndex++) {
			if (modules[i].can_handle == used_handles[canIndex]) {
				resendBroadcastMessages = false;
				break;
			}
		}

		if (resendBroadcastMessages) {
			used_handles[usedModuleIndex] = modules[i].can_handle;
			usedModuleIndex++;
			DEBUG_OUTPUT("%s: Using CAN #%d", modules[i].module_type_name, canIndex);

		} else {
			DEBUG_OUTPUT("%s: No rebroadcast.  Using CAN #%d", modules[i].module_type_name, canIndex);
		}

		if (can_module_type_alternative_loaded(&modules[i])) {
			DEBUG_OUTPUT("Continuing to next CAN module since type already loaded");
			continue;
		}

		if (initialize_address_can_modules(&modules[i]) != PROCESS_SUCCESS || load_can_module(&modules[i]) != PROCESS_SUCCESS) {
			loaded_types[i] = 0;

			if (modules[i].optional_load) {
				DEBUG_OUTPUT("Continuing CAN load on failure to load optional module");

			} else {
				if (can_module_type_alternative_exists(&modules[i])) {
					DEBUG_OUTPUT("Continuing CAN load on failure to load module since type alternative exists");

				} else {
					DEBUG_OUTPUT("Aborting CAN load");
					handle_can_module_error(&modules[i], CAN_MODULE_ERROR_GENERAL);
					return PROCESS_FAILURE;
				}
			}

		} else {
			loaded_types[i] = modules[i].module_type;
		}
	}

	return PROCESS_SUCCESS;
}

void set_module_incremental(int i, can_module_s *module)
{
	module->type_id_index = i;

	if (module->module_ids.length == 1) {
		strncpy(module->module_name, module->module_type_name, CAN_MODULE_NAME_LENGTH);

	} else {
		snprintf(module->module_name, CAN_MODULE_NAME_LENGTH, "%s %ld", module->module_type_name, module->type_id_index + 1);
	}

	module->currentId = module->module_ids.ids[i];
}

bool initialize_address_can_modules(can_module_s *module)
{
	//jump all can modules from application.  Don't debug print because most modules don't listen to this message
	for (size_t i = 0; i < module->module_ids.length; i++) {
		set_module_incremental(i, module);

		if (can_module_jump_from_application_to_bl(module) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: unable to jump module to bootloader mode", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_GENERAL);
			return PROCESS_FAILURE;
		}

		DELAY_MS(1500);
	}

	if (module->status.rebroadcast) {
		//DEBUG_OUTPUT("%s: unlocking RO (broadcast)", module->module_type_name);
		//if(can_module_unlock_ro(module) != PROCESS_SUCCESS){
		//	DEBUG_OUTPUT("%s: unable to unlock module RO", module->module_type_name);
		//	handle_can_module_error(module, CAN_MODULE_ERROR_RO);
		//	return PROCESS_FAILURE;
		//}
		//
		//DELAY_MS(200);

		DEBUG_OUTPUT("%s: resetting address (broadcast)", module->module_type_name);

		if (can_module_reset_address(module) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: unable to reset", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_ADDRESSING);
			return PROCESS_FAILURE;
		}

		DELAY_MS(200);

		DEBUG_OUTPUT("%s: unlocking RO (broadcast)", module->module_type_name);

		if (can_module_unlock_ro(module) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: unable to unlock module RO", module->module_type_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_RO);
			return PROCESS_FAILURE;
		}

		DELAY_MS(5);

	} else {
		DEBUG_OUTPUT("%s: already reset", module->module_type_name);
	}

	for (size_t i = 0; i < module->module_ids.length; i++) {
		set_module_incremental(i, module);

		if (module->pre_load_callback != NULL) {
			module->pre_load_callback(module);
		}

		can_module_set_all_addressing_lines(1); //must be implemented by load_can_app

		DELAY_MS(5);

		if (module->dynamicallyAddressed) {
			DEBUG_OUTPUT("%s: activating addressing", module->module_name, module->currentId);
			can_module_set_addressing_line(module, 0); //must be implemented by load_can_app

			DELAY_MS(10);

			DEBUG_OUTPUT("%s: assigning address broadcast", module->module_name);

			if (can_module_assign_address(module) != PROCESS_SUCCESS) {
				DEBUG_OUTPUT("%s: unable assign address", module->module_name);
				handle_can_module_error(module, CAN_MODULE_ERROR_ADDRESSING);
				return PROCESS_FAILURE;
			}

		} else {
			DEBUG_OUTPUT("%s: fixed address", module->module_name);
		}

		DELAY_MS(100);

		DEBUG_OUTPUT("%s: validating address", module->module_name);

		if (can_module_validate_address(module) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: unable to validate address", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_ADDRESSING);
			return PROCESS_FAILURE;
		}
	}

	return PROCESS_SUCCESS;
}

bool load_can_module(can_module_s *module)
{
	if (initialize_module_for_load(module) != PROCESS_SUCCESS) {
		DEBUG_OUTPUT("Failed to initialize modules");
		handle_can_module_error(module, CAN_MODULE_ERROR_GENERAL);
		return PROCESS_FAILURE;
	}

	DELAY_MS(50);

	for (size_t i = 0; i < module->module_ids.length; i++) {

		set_module_incremental(i, module);

		if (i > 0) {
			DEBUG_OUTPUT("Repeating CAN load for %s", module->module_name);

		} else {
			DEBUG_OUTPUT("CAN loading %s", module->module_name);
		}

		DELAY_MS(5);

		DEBUG_OUTPUT("%s: recording device config", module->module_name);

		if (can_module_get_uid_and_config(module) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: failed to get device config", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_GENERAL);
			return PROCESS_FAILURE;
		}

		DELAY_MS(5);

		if (can_module_key_process(module) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: unable to verify keys", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_KEYS);
			return PROCESS_FAILURE;
		}

		DELAY_MS(50);

		if (can_module_discover_if_addressed(module) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: unable to validate addressing", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_ADDRESSING);
			return PROCESS_FAILURE;
		}

		DELAY_MS(5);

		if (module->status.addressed != true) {
			DEBUG_OUTPUT("%s: module failed addressing return verification", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_ADDRESSING);
			return PROCESS_FAILURE;
		}

		DELAY_MS(5);

		if (can_module_read_version(module) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: version message failed", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_GENERAL);
			return PROCESS_FAILURE;
		}

		DEBUG_OUTPUT("%s: UID: 0x%08X Bootloader version: %d", module->module_name, module->flashData.uid,
			     module->flashData.version);

		DELAY_MS(5);

		DEBUG_OUTPUT("%s: opening and checking firmware file", module->module_name);
		module->fat_file_can = fopen(module->filename, "rb");

		if (module->fat_file_can == NULL) { //open the file
			DEBUG_OUTPUT("%s: failed to open file", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_FILE_READ);
			return PROCESS_FAILURE;
		}

		fseek(module->fat_file_can, 0, SEEK_END);
		module->file_size = ftell(module->fat_file_can);

		if (retreive_file_appended_metadata(&module->file_metadata, module->fat_file_can)) {
			//if true, metadata was found, meaning the end of the binary is 16 bytes before end of file
			DEBUG_OUTPUT("%s: file version %d.%d.%d CRC:0x%X", module->module_name, module->file_metadata.major,
				     module->file_metadata.minor, module->file_metadata.patch, module->file_metadata.crc);
			module->firmware_size = module->file_size - 16;

			if (module->expected_metadata.major || module->expected_metadata.minor || module->expected_metadata.patch) {
				if (module->file_metadata.major != module->expected_metadata.major ||
				    module->file_metadata.minor != module->expected_metadata.minor ||
				    module->file_metadata.patch != module->expected_metadata.patch) {
					DEBUG_OUTPUT("%s: file version does not match expected version %d.%d.%d", module->module_name,
						     module->expected_metadata.major,
						     module->expected_metadata.minor, module->expected_metadata.patch);
					handle_can_module_error(module, CAN_MODULE_ERROR_FILE_CONTENTS);
					return PROCESS_FAILURE;

				} else {
					DEBUG_OUTPUT("%s: file version matches expected version", module->module_name);
				}
			}

			if (module->expected_metadata.crc) {
				if (module->file_metadata.crc != module->expected_metadata.crc) {
					DEBUG_OUTPUT("%s: file CRC does not match expected CRC:0x%X", module->module_name, module->expected_metadata.crc);
					handle_can_module_error(module, CAN_MODULE_ERROR_FILE_CONTENTS);
					return PROCESS_FAILURE;

				} else {
					DEBUG_OUTPUT("%s: file CRC matches expected CRC", module->module_name);
				}
			}

		} else {
			//if no appended version, binary ends 4 bytes before end of file
			DEBUG_OUTPUT("%s: no appended metadata", module->module_name);
			module->firmware_size = module->file_size - 4;

			if (module->expected_metadata.major || module->expected_metadata.minor || module->expected_metadata.patch
			    || module->expected_metadata.crc) {
				DEBUG_OUTPUT("%s: expected metadata not found in file", module->module_name);
				handle_can_module_error(module, CAN_MODULE_ERROR_FILE_CONTENTS);
				return PROCESS_FAILURE;
			}
		}

		fseek(module->fat_file_can, 0, SEEK_SET); //seek back to start

		const uint32_t vectors_size = 392; //hardcoded for now, because don't know how to get __Vectors_Size from PX4 yet

		if (module->firmware_size <= vectors_size) {
			DEBUG_OUTPUT("%s: filesize (1:%d) incorrect", module->module_name, module->firmware_size);
			handle_can_module_error(module, CAN_MODULE_ERROR_FILE_CONTENTS);
			return PROCESS_FAILURE;
		}

		if (module->firmware_size % 4 != 0) { // file is not composed of words
			DEBUG_OUTPUT("%s: filesize (2:%d) incorrect", module->module_name, module->firmware_size);
			handle_can_module_error(module, CAN_MODULE_ERROR_FILE_CONTENTS);
			return PROCESS_FAILURE;
		}

		if (module->firmware_size > module->flash_size) { // file will not fit in Flash
			DEBUG_OUTPUT("%s: filesize (3:%d) incorrect", module->module_name, module->firmware_size);
			handle_can_module_error(module, CAN_MODULE_ERROR_FILE_CONTENTS);
			return PROCESS_FAILURE;
		}

		if (can_module_initialize_protection_mask(module) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: failed to initialize protection mask.", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_PROTECTIONS);
			return PROCESS_FAILURE;
		}

		DELAY_MS(5);
		DEBUG_OUTPUT("%s: preparing firmware load", module->module_name);

		if (can_module_initialize_encryption(module) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: failed to initialize module encryption", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_ENCRYPTION);
			return PROCESS_FAILURE;
		}

		DELAY_MS(5);

		if (can_module_set_target_pointer(module, module->fw_address) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: failed to set flash pointer", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_GENERAL);
			return PROCESS_FAILURE;
		}

		DELAY_MS(5);

		if (can_module_erase_flash(module) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: failed to erase module flash", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_FLASH);
			return PROCESS_FAILURE;
		}

		DELAY_MS(5);

		if (can_module_initialize_crc(module) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: failed to initialize crc", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_INITIALIZE_LOAD);
			return PROCESS_FAILURE;
		}

		DEBUG_OUTPUT("%s: transmitting firmware file", module->module_name);
		uint32_t fileIdx = 0;
		uint32_t file_data;
		const uint32_t bytes_per_packet = sizeof(uint32_t);
		uint32_t bytes_read;

		while (fileIdx < module->firmware_size) {
			bytes_read = fread(&file_data, 1, bytes_per_packet, module->fat_file_can);

			if (bytes_read == 0) {
				DEBUG_OUTPUT("%s: failed to read firmware file", module->module_name);
				handle_can_module_error(module, CAN_MODULE_ERROR_FILE_READ);
				return PROCESS_FAILURE;
			}

			if (bytes_read != bytes_per_packet) {
				DEBUG_OUTPUT("word in file could not be read"); //replicating functionality of OG systemloader, but also throw warnings when things become misaligned
				handle_can_module_error(module, CAN_MODULE_ERROR_FILE_READ);
				return PROCESS_FAILURE;
			}

			fileIdx += bytes_read;

			if (can_module_write_data(module, file_data) != PROCESS_SUCCESS) {
				DEBUG_OUTPUT("%s: file transfer failed", module->module_name);
				handle_can_module_error(module, CAN_MODULE_ERROR_FLASH);
				return PROCESS_FAILURE;
			}
		}

		if ((bytes_read = fread(&module->file_crc, 1, sizeof(uint32_t), module->fat_file_can)) == 0) {
			DEBUG_OUTPUT("failed to read file CRC");
			handle_can_module_error(module, CAN_MODULE_ERROR_FILE_READ);
			return PROCESS_FAILURE;
		}

		if (bytes_read != sizeof(uint32_t)) {
			DEBUG_OUTPUT("failed to read entire CRC word");
			handle_can_module_error(module, CAN_MODULE_ERROR_FILE_READ);
			return PROCESS_FAILURE;
		}

		DELAY_MS(5);

		DEBUG_OUTPUT("%s: finalizing firmware load", module->module_name);

		if (can_module_send_file_crc(module, module->file_crc) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: unable to send file crc", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_CRC);
			return PROCESS_FAILURE;
		}

		DELAY_MS(5);

		if (can_module_set_target_pointer(module, module->fw_address) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: unable to reset flash pointer", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_GENERAL);
			return PROCESS_FAILURE;
		}

		DELAY_MS(5);

		if (can_module_verify_flash_entry_crc(module) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: CRC verification failed", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_CRC);
			return PROCESS_FAILURE;
		}

		DELAY_MS(5);

		if (can_module_set_rdp_protections(module) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: unable to set rdp", module->module_name);
			handle_can_module_error(module, CAN_MODULE_ERROR_PROTECTIONS);
			return PROCESS_FAILURE;
		}

		DELAY_MS(5);

		if (can_module_set_wrp_protections(module) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("%s: Warning: unable to set wrp", module->module_name);
			//handle_can_module_error(module, CAN_MODULE_ERROR_PROTECTIONS);
			//return PROCESS_FAILURE;
		}

		fclose(module->fat_file_can);
		module->fat_file_can = NULL;

		if (can_module_flash_metadata(module) != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("Warning: unable to write flash metadata");
		}

		DEBUG_OUTPUT("%s: finished firmware load\n", module->module_name, module->currentId);
	}

	if (module->post_load_callback != NULL) {
		module->post_load_callback(module);
	}

	return PROCESS_SUCCESS;
}

bool can_module_assign_address(can_module_s *module)
{
	uint8_t can_id_ofst = module->currentId;
	uint8_t type = module->module_type;
	// Assign the appropriate address
	module->can_handle->pTxMsg.StdId = can_id_assign_address;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 1;
	module->can_handle->pTxMsg.Data[0] = ((can_id_ofst << 3) & 0xF8) | ((type << 0) & 0x07);

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	DELAY_MS(200);

	return PROCESS_SUCCESS;
}

bool can_module_validate_address(can_module_s *module)
{
	uint8_t can_id_ofst = module->currentId;
	// Read module hardware ID
	module->can_handle->pTxMsg.StdId = can_id_hardware_identification | can_id_ofst;
	module->can_handle->pTxMsg.RTR = CAN_RTR_REMOTE;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 8;

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	if (hw_CAN_receive(module->can_handle, 100) == CAN_HW_FAIL) {
		DEBUG_OUTPUT("%s: device timed out when trying to assign address", module->module_name);
		return PROCESS_FAILURE;
	}

	// IF ID MATCHES

	module->status.hardwareId = (uint32_t)module->can_handle->pRxMsg.Data[0] | (uint32_t)(
					    module->can_handle->pRxMsg.Data[1] << 8) | (uint32_t)(module->can_handle->pRxMsg.Data[2] << 16) | (uint32_t)(
					    module->can_handle->pRxMsg.Data[3] << 24);
	module->status.deviceId = (uint16_t)(module->can_handle->pRxMsg.Data[4] & 0x0F) | ((
					  uint16_t)module->can_handle->pRxMsg.Data[5] << 4);
	module->status.revisionId = (uint16_t)module->can_handle->pRxMsg.Data[6] | (uint16_t)(
					    module->can_handle->pRxMsg.Data[7] << 8);
	uint8_t readType = (module->can_handle->pRxMsg.Data[4] & 0xE0) >> 5;
	uint8_t rdp = (module->can_handle->pRxMsg.Data[5] & 0x10) >> 5;

	if (module->status.hardwareId != module->hardware_id) {
		DEBUG_OUTPUT("%s: hardware id mismatch", module->module_name);
		return PROCESS_FAILURE;
	}

	DEBUG_OUTPUT("%s: hardware id verified", module->module_name);

	if (readType != module->module_type) {
		DEBUG_OUTPUT("%s: type mismatch", module->module_name);
		return PROCESS_FAILURE;
	}

	DEBUG_OUTPUT("%s: device type verified", module->module_name);

	if (rdp != 0) {
		DEBUG_OUTPUT("%s: RDP not set", module->module_name);
		return PROCESS_FAILURE;
	}

	return PROCESS_SUCCESS;
}

bool can_module_send_file_crc(can_module_s *module, uint32_t crc)
{
	module->can_handle->pTxMsg.StdId = can_id_command_ext_bootloader | module->currentId;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 5;
	module->can_handle->pTxMsg.Data[0] = ((module->module_type << 5) & 0xE0) | function_send_file_crc;
	module->can_handle->pTxMsg.Data[1] = (uint8_t)(crc >> 0);
	module->can_handle->pTxMsg.Data[2] = (uint8_t)(crc >> 8);
	module->can_handle->pTxMsg.Data[3] = (uint8_t)(crc >> 16);
	module->can_handle->pTxMsg.Data[4] = (uint8_t)(crc >> 24);

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	if (hw_CAN_receive(module->can_handle, 100) == CAN_HW_FAIL) {
		return PROCESS_FAILURE;
	}

	// Check response is 0 (function succeeded) and function is function_send_file_crc
	if (((module->can_handle->pRxMsg.Data[0] & 0x1F) != function_send_file_crc)
	    || (((module->can_handle->pRxMsg.Data[1] & 0xF8) >> 3) != 0)) {
		return PROCESS_FAILURE;
	}

	return PROCESS_SUCCESS;
}


bool can_module_verify_flash_entry_crc(can_module_s *module)
{
	module->can_handle->pTxMsg.StdId = can_id_command_bootloader | module->currentId;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 1;
	module->can_handle->pTxMsg.Data[0] = ((module->module_type << 5) & 0xE0) | function_verify_flash_entry;

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	if (hw_CAN_receive(module->can_handle, 1000) == CAN_HW_FAIL) {
		return PROCESS_FAILURE;
	}

	// Check response is 0 (function succeeded) and function is function_verify_flash_entry
	if (((module->can_handle->pRxMsg.Data[0] & 0x1F) != function_verify_flash_entry)
	    || (((module->can_handle->pRxMsg.Data[1] & 0xF8) >> 3) != 0)) {
		return PROCESS_FAILURE;
	}

	return PROCESS_SUCCESS;
}

bool can_module_get_uid_and_config(can_module_s *module)
{
	// Read MCU UID
	module->can_handle->pTxMsg.StdId = can_id_mcu_uid | module->currentId;
	module->can_handle->pTxMsg.RTR = CAN_RTR_REMOTE;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 4;

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	if (hw_CAN_receive(module->can_handle, 100) == CAN_HW_FAIL) {
		return PROCESS_FAILURE;
	}

	//IF ID MATCHES

	module->flashData.uid = (uint32_t)module->can_handle->pRxMsg.Data[0] | (uint32_t)(module->can_handle->pRxMsg.Data[1] <<
				8) | (uint32_t)(module->can_handle->pRxMsg.Data[2] << 16) | (uint32_t)(module->can_handle->pRxMsg.Data[3] << 24);

	return PROCESS_SUCCESS;
}

bool can_module_key_process(can_module_s *module)
{
	// Read module config
	module->can_handle->pTxMsg.StdId = can_id_device_configuration | module->currentId;
	module->can_handle->pTxMsg.RTR = CAN_RTR_REMOTE;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 6;

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	//IF ID MATCHES?

	if (hw_CAN_receive(module->can_handle, 100) == CAN_HW_FAIL) {
		return PROCESS_FAILURE;
	}

	module->status.keyErased = ((module->can_handle->pRxMsg.Data[4] & 0x08) >> 3) == 1;

	if (module->status.keyErased) {
		DEBUG_OUTPUT("%s: No key.  Loading keys.", module->module_name);

		if (DISABLE_REMOTE_KEYLOAD || can_module_load_key(module) != PROCESS_SUCCESS) {
			return PROCESS_FAILURE;
		}

	} else {
		DEBUG_OUTPUT("%s: Already keyed!", module->module_name);
	}

	return PROCESS_SUCCESS;
}

bool can_module_write_data(can_module_s *module, uint32_t data)
{
	module->can_handle->pTxMsg.StdId = can_id_command_bootloader | module->currentId;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 5;
	module->can_handle->pTxMsg.Data[0] = ((module->module_type << 5) & 0xE0) | function_write_encrypted_data;
	module->can_handle->pTxMsg.Data[1] = (uint8_t)(data >> 0);
	module->can_handle->pTxMsg.Data[2] = (uint8_t)(data >> 8);
	module->can_handle->pTxMsg.Data[3] = (uint8_t)(data >> 16);
	module->can_handle->pTxMsg.Data[4] = (uint8_t)(data >> 24);

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	if (hw_CAN_receive(module->can_handle, 10) == CAN_HW_FAIL) {
		//DEBUG_OUTPUT("%s: write_encrypted_data timeout\r\n", module->module_name, ((module->can_handle->pRxMsg.Data[1] & 0xF8)>>3));
		return PROCESS_FAILURE;
	}

	// Check response is 0 (function succeeded) and function is function_write_encrypted_data
	if (((module->can_handle->pRxMsg.Data[0] & 0x1F) == function_write_encrypted_data)) {
		if (((module->can_handle->pRxMsg.Data[1] & 0xF8) >> 3) == 10) {
			//DEBUG_OUTPUT("0");
			return PROCESS_SUCCESS;

		} else if (((module->can_handle->pRxMsg.Data[1] & 0xF8) >> 3) == 0) {
			//DEBUG_OUTPUT("1");
			return PROCESS_SUCCESS;

		} else {
			//DEBUG_OUTPUT("%s: write data failure: %d\r\n", module->module_name, ((module->can_handle->pRxMsg->Data[1] & 0xF8)>>3));
			return PROCESS_FAILURE;
		}

	} else {
		//DEBUG_OUTPUT("%s: data write response failure: %d\r\n", module->module_name, ((module->can_handle->pRxMsg->Data[1] & 0xF8)>>3));
		return PROCESS_FAILURE;
	}
}

bool can_module_set_rdp_protections(can_module_s *module)
{
	module->can_handle->pTxMsg.StdId = can_id_command_ext_bootloader | module->currentId;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 5;
	module->can_handle->pTxMsg.Data[0] = ((module->module_type << 5) & 0xE0) | function_set_rdp1;
	module->can_handle->pTxMsg.Data[1] = (uint8_t)(module->flashData.uid >> 0);
	module->can_handle->pTxMsg.Data[2] = (uint8_t)(module->flashData.uid >> 8);
	module->can_handle->pTxMsg.Data[3] = (uint8_t)(module->flashData.uid >> 16);
	module->can_handle->pTxMsg.Data[4] = (uint8_t)(module->flashData.uid >> 24);

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	DELAY_MS(5);

	if (hw_CAN_receive(module->can_handle, 100) == CAN_HW_FAIL) {
		return PROCESS_FAILURE;
	}

	// Check response is 0 (function succeeded) and function is function_set_rdp1
	if (((module->can_handle->pRxMsg.Data[0] & 0x1F) != function_set_rdp1)
	    || (((module->can_handle->pRxMsg.Data[1] & 0xF8) >> 3) != 0)) {
		return PROCESS_FAILURE;
	}

	return PROCESS_SUCCESS;
}

bool can_module_set_wrp_protections(can_module_s *module)
{
	module->can_handle->pTxMsg.StdId = can_id_command_ext_bootloader | module->currentId;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 5;
	module->can_handle->pTxMsg.Data[0] = ((module->module_type << 5) & 0xE0) | function_set_rdp1;
	module->can_handle->pTxMsg.Data[1] = (uint8_t)(module->flashData.uid >> 0);
	module->can_handle->pTxMsg.Data[2] = (uint8_t)(module->flashData.uid >> 8);
	module->can_handle->pTxMsg.Data[3] = (uint8_t)(module->flashData.uid >> 16);
	module->can_handle->pTxMsg.Data[4] = (uint8_t)(module->flashData.uid >> 24);

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	DELAY_MS(5);

	if (hw_CAN_receive(module->can_handle, 100) == CAN_HW_FAIL) {
		return PROCESS_FAILURE;
	}

	// Check response is 0 (function succeeded) and function is function_set_rdp1
	if (((module->can_handle->pRxMsg.Data[0] & 0x1F) != function_set_rdp1)
	    || (((module->can_handle->pRxMsg.Data[1] & 0xF8) >> 3) != 0)) {
		return PROCESS_FAILURE;
	}

	return PROCESS_SUCCESS;
}

bool can_module_initialize_protection_mask(can_module_s *module)
{
	module->can_handle->pTxMsg.StdId = can_id_command_bootloader | module->currentId;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 1;
	module->can_handle->pTxMsg.Data[0] = ((module->module_type << 5) & 0xE0) | function_initialize_protection_mask;

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	DELAY_MS(200);

	if (hw_CAN_receive(module->can_handle, 100) == CAN_HW_FAIL) {
		return PROCESS_FAILURE;
	}

	// Check response is 0 (function succeeded) and function is function_initialize_protection_mask
	if (((module->can_handle->pRxMsg.Data[0] & 0x1F) != function_initialize_protection_mask)
	    || (((module->can_handle->pRxMsg.Data[1] & 0xF8) >> 3) != 0)) {
		return PROCESS_FAILURE;
	}

	// Wait until MCU boots up
	DELAY_MS(200);
	return PROCESS_SUCCESS;
}

bool can_module_initialize_encryption(can_module_s *module)
{
	module->can_handle->pTxMsg.StdId = can_id_command_bootloader | module->currentId;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 1;
	module->can_handle->pTxMsg.Data[0] = ((module->module_type << 5) & 0xE0) | function_initialize_encryption_state;

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	DELAY_MS(5);

	if (hw_CAN_receive(module->can_handle, 1500) == CAN_HW_FAIL) {
		//DEBUG_OUTPUT("%s: Warning: initialize encryption timed out.", module->module_name);
		return PROCESS_FAILURE;
	}

	// Check response is 0 (function succeeded) and function is function_initialize_encryption_state
	if (((module->can_handle->pRxMsg.Data[0] & 0x1F) != function_initialize_encryption_state)
	    || (((module->can_handle->pRxMsg.Data[1] & 0xF8) >> 3) != 0)) {
		return PROCESS_FAILURE;
	}

	// Schedule encryption public key
	module->can_handle->pTxMsg.StdId = can_id_command_bootloader | module->currentId;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 5;
	module->can_handle->pTxMsg.Data[0] = ((module->module_type << 5) & 0xE0) | function_schedule_encryption_key;
	module->can_handle->pTxMsg.Data[1] = (uint8_t)((module->firmware_size) >> 0);
	module->can_handle->pTxMsg.Data[2] = (uint8_t)((module->firmware_size) >> 8);
	module->can_handle->pTxMsg.Data[3] = (uint8_t)((module->firmware_size) >> 16);
	module->can_handle->pTxMsg.Data[4] = (uint8_t)((module->firmware_size) >> 24);

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	if (hw_CAN_receive(module->can_handle, 100) == CAN_HW_FAIL) {
		return PROCESS_FAILURE;
	}

	// Check response is 0 (function succeeded) and function is function_schedule_encryption_key
	if (((module->can_handle->pRxMsg.Data[0] & 0x1F) != function_schedule_encryption_key)
	    || (((module->can_handle->pRxMsg.Data[1] & 0xF8) >> 3) != 0)) {
		return PROCESS_FAILURE;
	}

	return PROCESS_SUCCESS;
}

bool can_module_set_target_pointer(can_module_s *module, uint32_t target)
{
	module->can_handle->pTxMsg.StdId = can_id_command_bootloader | module->currentId;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 5;
	module->can_handle->pTxMsg.Data[0] = ((module->module_type << 5) & 0xE0) | function_set_target_pointer;
	module->can_handle->pTxMsg.Data[1] = (uint8_t)(target >> 0);
	module->can_handle->pTxMsg.Data[2] = (uint8_t)(target >> 8);
	module->can_handle->pTxMsg.Data[3] = (uint8_t)(target >> 16);
	module->can_handle->pTxMsg.Data[4] = (uint8_t)(target >> 24);

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	if (hw_CAN_receive(module->can_handle, 100) == CAN_HW_FAIL) {
		return PROCESS_FAILURE;
	}

	// Check response is 0 (function succeeded) and function is function_set_target_pointer
	if (((module->can_handle->pRxMsg.Data[0] & 0x1F) != function_set_target_pointer)
	    || (((module->can_handle->pRxMsg.Data[1] & 0xF8) >> 3) != 0)) {
		return PROCESS_FAILURE;
	}

	return PROCESS_SUCCESS;
}

bool can_module_erase_flash(can_module_s *module)
{
	module->can_handle->pTxMsg.StdId = can_id_command_bootloader | module->currentId;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 3;
	module->can_handle->pTxMsg.Data[0] = ((module->module_type << 5) & 0xE0) | function_erase_flash_region;
	module->can_handle->pTxMsg.Data[1] = (uint8_t)(((module->firmware_size) / 4) >> 0);
	module->can_handle->pTxMsg.Data[2] = (uint8_t)(((module->firmware_size) / 4) >> 8);

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	if (hw_CAN_receive(module->can_handle, 5000) == CAN_HW_FAIL) {
		return PROCESS_FAILURE;
	}

	// Check response is 0 (function succeeded) and function is function_erase_flash_region
	if (((module->can_handle->pRxMsg.Data[0] & 0x1F) != function_erase_flash_region)
	    || (((module->can_handle->pRxMsg.Data[1] & 0xF8) >> 3) != 0)) {
		return PROCESS_FAILURE;
	}

	return PROCESS_SUCCESS;
}

bool can_module_initialize_crc(can_module_s *module)
{
	module->can_handle->pTxMsg.StdId = can_id_command_bootloader | module->currentId;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 1;
	module->can_handle->pTxMsg.Data[0] = ((module->module_type << 5) & 0xE0) | function_reset_transfer_check;

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	if (hw_CAN_receive(module->can_handle, 100) == CAN_HW_FAIL) {
		return PROCESS_FAILURE;
	}

	return PROCESS_SUCCESS;
}

bool can_module_jump_from_application_to_bl(can_module_s *module)
{
	module->can_handle->pTxMsg.StdId = can_id_application_to_bl_jump | module->currentId;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 0;

	if (hw_CAN_transmit(module->can_handle, 100) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	return PROCESS_SUCCESS;
}

bool can_module_jump_from_bl_to_application(can_module_s *module, int id)
{
	module->can_handle->pTxMsg.StdId = can_id_execute_jump;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 5;
	module->can_handle->pTxMsg.Data[0] = ((id << 3) & 0xF8) | ((module->module_type << 0) & 0x07);
	module->can_handle->pTxMsg.Data[1] = (uint8_t)(module->fw_address >> 0);
	module->can_handle->pTxMsg.Data[2] = (uint8_t)(module->fw_address >> 8);
	module->can_handle->pTxMsg.Data[3] = (uint8_t)(module->fw_address >> 16);
	module->can_handle->pTxMsg.Data[4] = (uint8_t)(module->fw_address >> 24);

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	return PROCESS_SUCCESS;
}

bool jump_all_can_modules_to_application(void)
{
	for (size_t i = 0; i < module_total; i++) {
		if (can_module_jump_from_bl_to_application(&modules[i],
				0x1F) != PROCESS_SUCCESS) { //0x1F is broadcast.  To individually jump, use id.
			return PROCESS_FAILURE;
		}
	}

	return PROCESS_SUCCESS;
}

bool can_module_read_version(can_module_s *module)
{
	module->can_handle->pTxMsg.StdId = can_id_bootloader_version | module->currentId;
	module->can_handle->pTxMsg.RTR = CAN_RTR_REMOTE;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 8;

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	if (hw_CAN_receive(module->can_handle, 100) == CAN_HW_FAIL) {
		return PROCESS_FAILURE;
	}

	module->flashData.version = module->can_handle->pRxMsg.Data[0];

	return PROCESS_SUCCESS;;
}

bool can_module_discover_if_addressed(can_module_s *module)
{
	module->status.addressed = false;

	module->can_handle->pTxMsg.StdId = can_id_device_configuration | module->currentId;
	module->can_handle->pTxMsg.RTR = CAN_RTR_REMOTE;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 6;

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	if (hw_CAN_receive(module->can_handle, 100) != CAN_HW_FAIL) {
		module->status.addressed = true;
	}

	return PROCESS_SUCCESS;
}

bool can_module_unlock_ro(can_module_s *module)
{
	// Assign the appropriate address
	module->can_handle->pTxMsg.StdId = can_id_unlock_ro;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 1;
	module->can_handle->pTxMsg.Data[0] = ((0x1F << 3) & 0xF8) | ((0x07 << 0) & 0x07); // Address broadcast, type broadcast

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	return PROCESS_SUCCESS;
}

bool can_module_reset_address(can_module_s *module)
{
	module->can_handle->pTxMsg.StdId = can_id_assign_address;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 1;
	module->can_handle->pTxMsg.Data[0] = ((0x1F << 3) & 0xF8) | ((0x07 << 0) & 0x07); // Address broadcast, type broadcast

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	return PROCESS_SUCCESS;
}

bool can_module_flash_metadata(can_module_s *module)
{
	////note that this does not clear flash!  In order for UIDs and versions to be accurate, must be cleared before load.
	//module->startFlashAddr = currentFlashAddress;
	//HAL_FLASH_Unlock();
	//for(int i=0; i<(sizeof(flash_metadata_s)/sizeof(uint32_t)); i++){
	//	uint32_t data = ((uint32_t*)&module->flashData)[i];
	//	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, currentFlashAddress, data) != CAN_HW_OK){
	//		return PROCESS_FAILURE;
	//	}
	//	currentFlashAddress += sizeof(uint32_t);
	//}
	//HAL_FLASH_Lock();
	//module->endFlashAddr = currentFlashAddress;
	return PROCESS_SUCCESS;
}

bool can_module_load_key(can_module_s *module)
{
	uint8_t type = module->module_type;
	uint32_t can_id_ofst = module->currentId;

	DEBUG_OUTPUT("%s: \t Reset Key Buffer", module->module_name);
	module->can_handle->pTxMsg.StdId = can_id_command_bootloader | can_id_ofst;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 1;
	module->can_handle->pTxMsg.Data[0] = ((type << 5) & 0xE0) | function_reset_key_buffer;

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	if (hw_CAN_receive(module->can_handle, 100) == CAN_HW_FAIL) {
		DEBUG_OUTPUT("%s: Warning: device did not respond to resetting key buffer.", module->module_name);
	}

	DELAY_MS(200);

	DEBUG_OUTPUT("%s: \t Transmit Key Buffer", module->module_name);

	for (uint8_t i = 0 ; i < (module->private_key.size / 4) ; i++) {
		module->can_handle->pTxMsg.StdId = can_id_command_bootloader | can_id_ofst;
		module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
		module->can_handle->pTxMsg.IDE = CAN_ID_STD;
		module->can_handle->pTxMsg.DLC = 5;
		module->can_handle->pTxMsg.Data[0] = ((type << 5) & 0xE0) | function_append_key_buffer;
		module->can_handle->pTxMsg.Data[1] = module->private_key.key[4 * i];
		module->can_handle->pTxMsg.Data[2] = module->private_key.key[4 * i + 1];
		module->can_handle->pTxMsg.Data[3] = module->private_key.key[4 * i + 2];
		module->can_handle->pTxMsg.Data[4] = module->private_key.key[4 * i + 3];
		//DEBUG_OUTPUT(".");

		if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
			return PROCESS_FAILURE;
		}

		if (hw_CAN_receive(module->can_handle, 100) == CAN_HW_FAIL) {
			DEBUG_OUTPUT("%s: \t Key Timeout", module->module_name);
			return PROCESS_FAILURE;
		}

		if (((module->can_handle->pRxMsg.Data[0] & 0x1F) != function_append_key_buffer)
		    || (((module->can_handle->pRxMsg.Data[1] & 0xF8) >> 3) !=
			0)) { // Check response is 0 (function succeeded) and function is function_append_key_buffer
			DEBUG_OUTPUT("%s: \t Key update failed with response %d", module->module_name,
				     ((module->can_handle->pRxMsg.Data[1] & 0xF8) >> 3));
			return PROCESS_FAILURE;
		}
	}

	//DEBUG_OUTPUT("\r\n");

	DELAY_MS(5);

	module->can_handle->pTxMsg.StdId = can_id_command_bootloader | can_id_ofst;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 1;
	module->can_handle->pTxMsg.Data[0] = ((type << 5) & 0xE0) | function_initialize_encryption_state;

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	DELAY_MS(200);

	if (hw_CAN_receive(module->can_handle, 2000) == CAN_HW_FAIL) {
		DEBUG_OUTPUT("%s: \t Initialize encryption timeout", module->module_name);
		return PROCESS_FAILURE;
	}

	// Check response is 0 (function succeeded) and function is function_initialize_encryption_state and key size is correct
	if (!(((module->can_handle->pRxMsg.Data[0] & 0x1F) == function_initialize_encryption_state)
	      && (((module->can_handle->pRxMsg.Data[1] & 0xF8) >> 3) == 0)
	      && (module->can_handle->pRxMsg.Data[1] & 0x07) == (log2p(module->private_key.size) - 1))) {
		DEBUG_OUTPUT("%s: \t Key Verification failed.  Response=%d, Size=%d", module->module_name,
			     ((module->can_handle->pRxMsg.Data[1] & 0xF8) >> 3), (module->can_handle->pRxMsg.Data[1] & 0x07));
		return PROCESS_FAILURE;
	}

	return PROCESS_SUCCESS;
}

uint8_t log2p(uint32_t x)
{
	return (((x & 0xFFFF0000) != 0) ? 16 : 0) | (((x & 0xFF00FF00) != 0) ? 8 : 0) | (((x & 0xF0F0F0F0) != 0) ? 4 : 0) | (((
				x & 0xCCCCCCCC) != 0) ? 2 : 0) | (((x & 0xAAAAAAAA) != 0) ? 1 : 0);
}

bool retreive_file_appended_metadata(struct ff_metadata_s *appendedInfo,
				     FILE *fp)  //returns true if appended metadata could be retrieved
{
	appendedInfo->major = 0;
	appendedInfo->minor = 0;
	appendedInfo->patch = 0;
	appendedInfo->crc = 0xFFFFFFFF;

	uint32_t buffer;

	fseek(fp, 0, SEEK_END);
	uint32_t binaryEnd = ftell(fp);
	fseek(fp, binaryEnd - 16, SEEK_SET); // seek back and check last 4 words

	//Word 1: CRC32
	if (fread(&buffer, 1, sizeof(uint32_t), fp) < sizeof(uint32_t)) {
		return false;
	}

	appendedInfo->crc = buffer;

	//Words 2&3: "Version" tag
	if (fread(&buffer, 1, sizeof(uint32_t), fp) < sizeof(uint32_t)) {
		return false;
	}

	if (buffer != 0x53524556) {
		return false;
	}

	if (fread(&buffer, 1, sizeof(uint32_t), fp) < sizeof(uint32_t)) {
		return false;
	}

	if (buffer != 0x2d4e4f49) {
		return false;
	}

	//Word 4: version info
	if (fread(&buffer, 1, sizeof(uint32_t), fp) < sizeof(uint32_t)) {
		return false;
	}

	appendedInfo->major = (uint8_t)(buffer >> 24);
	appendedInfo->minor = (uint8_t)(buffer >> 16);
	appendedInfo->patch = (uint8_t)(buffer >> 0);
	return true;
}

// bool checkFileCRC(FILE* fp){
//ifappendedcrc == fff || does not equal
// 	uint32_t file_crc = 0xFFFFFFFF;
// 	uint32_t appended_crc = 0xFFFFFFFF;
// 	uint8_t data;
// 	uint32_t count = 0;

// 	f_lseek(fp, 0);
// 	for(int i=0; i<binaryEnd; i++){
// 		setLEDState(LED_FLASHING);
// 		if((f_read(fp, &data, 1, &count) != FR_OK) || (count != 1))
// 			return appendedInfo;
// 		file_crc = accumulate_crc32(crc32_polynomial, file_crc, &data, count);
// 	}

// 	appendedInfo.crc = file_crc;

// 	return appendedInfo;
// }
