#ifndef __FF_SL_LOAD_CAN_H
#define __FF_SL_LOAD_CAN_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "can_hw.h"

#define TOTAL_MAX_MODULES_PER_TYPE 8

#define PROCESS_SUCCESS false
#define PROCESS_FAILURE true

#define CAN_MODULE_NAME_LENGTH 16

typedef struct {
	uint8_t key[32];
	uint32_t size;
} encryption_key_s;

typedef enum {
	can_id_execute_jump = 0x010,
	can_id_assign_address = 0x011,
	can_id_unlock_ro = 0x012,
	can_id_tsu_status = 0x042A,
	can_id_tsu_version = 0x042B,
	can_id_esc_status = 0x0580,
	can_id_esc_version = 0x0590,
	can_id_command_bootloader = 0x060,
	can_id_command_ext_bootloader = 0x0E0,
	can_id_feedback_response = 0x160,
	can_id_hardware_identification = 0x180,
	can_id_mcu_uid = 0x1A0,
	can_id_device_configuration = 0x1C0,
	can_id_bootloader_version = 0x1E0,
	can_id_transfer_state = 0x200,
	can_id_mcu_reset = 0x380,
	can_id_clear_addresses = 0x381,
	can_id_allocate_addresses = 0x383,
	can_id_finalize_addresses = 0x382,
	can_id_application_to_bl_jump = 0x320,
} can_id_e;

typedef enum {
	function_reset_key_buffer = 0x00,
	function_append_key_buffer = 0x01,
	function_initialize_encryption_state = 0x02,
	function_initialize_protection_mask = 0x03,
	function_schedule_encryption_key = 0x04,
	function_set_target_pointer = 0x05,
	function_erase_flash_region = 0x06,
	function_reset_transfer_check = 0x07,
	function_write_encrypted_data = 0x08,
	function_verify_flash_entry = 0x09,
	function_reset_write_protection = 0x0A,
	function_set_write_protection = 0x0B,
	function_copy_data_block = 0x0C,
	function_verify_memory_entry = 0x0D,
	function_verify_erased_data = 0x0E,
	function_verify_nulled_data = 0x0F
} function_e;

typedef enum {
	function_set_rdp1 = 0x00,
	function_send_file_crc = 0x01,
	function_set_leds = 0x02
} function_ext_e;

typedef struct {
	const uint8_t ids[TOTAL_MAX_MODULES_PER_TYPE];
	const uint8_t length;
} module_ids_s;

typedef struct {
	bool addressed;
	uint32_t deviceId;
	uint32_t revisionId;
	uint32_t hardwareId;
	uint32_t errorCode;
	bool keyErased;
	bool rebroadcast;
} module_status_s;

typedef struct {
	uint32_t uid;
	uint32_t version;
} flash_metadata_s;

struct ff_metadata_s {
	uint8_t major;
	uint8_t minor;
	uint16_t patch;
	uint32_t crc;
};

enum can_module_error_code {
	CAN_MODULE_ERROR_NONE = 0,
	CAN_MODULE_ERROR_GENERAL,
	CAN_MODULE_ERROR_INITIALIZE_LOAD,
	CAN_MODULE_ERROR_ADDRESSING,
	CAN_MODULE_ERROR_RO,
	CAN_MODULE_ERROR_KEYS,
	CAN_MODULE_ERROR_FILE_READ,
	CAN_MODULE_ERROR_FILE_CONTENTS,
	CAN_MODULE_ERROR_PROTECTIONS,
	CAN_MODULE_ERROR_ENCRYPTION,
	CAN_MODULE_ERROR_FLASH,
	CAN_MODULE_ERROR_CRC,
};

typedef struct canModule {
	//configuration values
	const char *filename;
	const char *module_type_name;
	const encryption_key_s private_key;
	const uint32_t fw_address;
	const uint32_t flash_size;
	const uint8_t module_type;
	const module_ids_s module_ids;
	const uint32_t hardware_id;
	can_handle_s *can_handle;
	void (*pre_load_callback)(struct canModule *);
	void (*post_load_callback)(struct canModule *);
	//const uint32_t manifest_number;
	//generated values
	module_status_s status;
	FILE *fat_file_can;
	uint32_t file_size;
	struct ff_metadata_s expected_metadata;
	struct ff_metadata_s file_metadata;
	uint32_t firmware_size;
	uint32_t file_crc;
	uint32_t startFlashAddr;
	uint32_t endFlashAddr;
	flash_metadata_s flashData;
	uint32_t currentId;
	uint32_t type_id_index;
	char module_name[CAN_MODULE_NAME_LENGTH + 1];
	bool optional_load;
	bool dynamicallyAddressed;
} can_module_s;

bool initialize_module_for_load(can_module_s *module);
bool can_load_all_modules(void);
bool load_can_module(can_module_s *module);
bool jump_all_can_modules_to_application(void);
void set_module_incremental(int i, can_module_s *module);
bool can_module_jump_from_application_to_bl(can_module_s *module);
bool can_module_jump_from_bl_to_application(can_module_s *module, int id);
bool can_module_type_alternative_loaded(can_module_s *module);
bool can_module_type_alternative_exists(can_module_s *module);

#ifdef __cplusplus
}
#endif

#endif
