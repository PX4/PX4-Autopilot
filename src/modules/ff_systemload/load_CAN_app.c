#include "load_CAN_app.h"
#include "pwm_hw.h"
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>

#define LOG_LINE_LENGTH 50

const char system_log_filename[] = "/fs/microsd/ext_autostart/extras/freefly/astro/system.log";
const char loaded_versions_filename[] = "/fs/microsd/ext_autostart/extras/freefly/astro/loaded_versions.txt";

can_handle_s can1_handle = {.instance = "/dev/can1"}; // = {.instance = &can1_h};

static FILE *system_log;
static FILE *loaded_versions_log;

static void append_versions_log(can_module_s *module);
static bool assign_boom_id(can_module_s *module, int id);

can_load_status_e can_load_status = CAN_LOAD_IDLE;

can_module_s modules[] = {
	[0] = {
		.module_type_name = "Motor F45", // has to be attempted first because of bug with re-assigning address in bootloader
		.filename = "/fs/microsd/ext_autostart/extras/freefly/astro/esc_f45_fw.bin",
		.expected_metadata = {.major = 1, .minor = 0, .patch = 116},
		.private_key = {.size = 16, .key = {0x94, 0x8f, 0xf5, 0x1a, 0x64, 0xb8, 0xd7, 0x85, 0x3b, 0x2f, 0xf9, 0xab, 0xf2, 0x23, 0xb0, 0x5e}},
		.fw_address = 0x08008000,
		.flash_size = 94 * 1024,
		.dynamicallyAddressed = true,
		.hardware_id = 0x0b58a8ba, // F45 motor variant
		.module_type = 6,
		.module_ids = {.length = ESC_QTY, .ids = {1, 2, 3, 4}},
		.can_handle = &can1_handle,
		.post_load_callback = &append_versions_log,
	},
	[1] = {
		.module_type_name = "Motor F7010",
		.filename = "/fs/microsd/ext_autostart/extras/freefly/astro/esc_f7010_fw.bin",
		.expected_metadata = {.major = 1, .minor = 0, .patch = 215},
		.private_key = {.size = 16, .key = {0x94, 0x8f, 0xf5, 0x1a, 0x64, 0xb8, 0xd7, 0x85, 0x3b, 0x2f, 0xf9, 0xab, 0xf2, 0x23, 0xb0, 0x5e}},
		.fw_address = 0x08008000,
		.flash_size = 94 * 1024,
		.dynamicallyAddressed = true,
		.hardware_id = 0xc5a5359f, // F7010 motor variant
		.module_type = 6,
		.module_ids = {.length = ESC_QTY, .ids = {1, 2, 3, 4}},
		.can_handle = &can1_handle,
		.post_load_callback = &append_versions_log,
	},
	[2] = {
		.module_type_name = "Power",
		.filename = "/fs/microsd/ext_autostart/extras/freefly/astro/power_fw.bin",
		.private_key = {.size = 16, .key = {0x97, 0x0F, 0xD4, 0x2B, 0x19, 0xB2, 0x37, 0x7A, 0xCA, 0x43, 0x39, 0xA0, 0xE9, 0x0E, 0xDD, 0x10}},
		.fw_address = 0x08008000,
		.flash_size = 0x38000,
		.hardware_id = 0x4D600001,
		.module_type = 5,
		.module_ids = {.length = 1, .ids = {10}},
		.can_handle = &can1_handle,
		.post_load_callback = &append_versions_log,
	},
	[3] = {
		.module_type_name = "GPS",
		.filename = "/fs/microsd/ext_autostart/extras/freefly/astro/rtk_fw.bin",
		.private_key = {.size = 16, .key = {0x2c, 0x3a, 0x65, 0xee, 0xbe, 0xe3, 0x6e, 0x32, 0xe0, 0xca, 0x42, 0x0b, 0xf6, 0xd3, 0x43, 0x4d}},
		.fw_address = 0x08020000,
		.flash_size = 0x60000,
		.hardware_id = 0xf3cfa4d3,
		.module_type = 7,
		.module_ids = {.length = 1, .ids = {15}},
		.can_handle = &can1_handle,
		.post_load_callback = &append_versions_log,
		.optional_load = true,
	},

};

const size_t module_total = (sizeof(modules) / sizeof(can_module_s));
can_handle_s *used_handles[(sizeof(modules) / sizeof(can_module_s))] = {0};
uint8_t loaded_types[(sizeof(modules) / sizeof(can_module_s))] = {0};

//-----CAN Loader app specific functions----

bool init_modules_app(void)
{
	can_load_status = CAN_LOAD_IN_PROCESS;

	if (initAllCanPorts() != PROCESS_SUCCESS) {
		return PROCESS_FAILURE;
	}

	if (pwmPinsInit()) {
		DEBUG_OUTPUT("Unable to initialize addressing IO");
		return PROCESS_FAILURE;
	}

	system_log = fopen(system_log_filename, "w");

	if (system_log == NULL) {
		DEBUG_OUTPUT("Unable to open system log");
		return PROCESS_FAILURE;
	}

	loaded_versions_log = fopen(loaded_versions_filename, "w");

	if (loaded_versions_log == NULL) {
		DEBUG_OUTPUT("Unable to open versions logfile");
		return PROCESS_FAILURE;
	}

	return PROCESS_SUCCESS;
}

bool initAllCanPorts(void)
{
	if (hw_CAN_init(&can1_handle) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	return PROCESS_SUCCESS;
}

bool deInitAllCanPorts(void)
{
	if (hw_CAN_deinit(&can1_handle) != CAN_HW_OK) {
		DEBUG_OUTPUT("Failed to deinit CAN");
		return PROCESS_FAILURE;
	}

	return PROCESS_SUCCESS;
}

bool post_finish_load_callback(void)
{
	if (system_log != NULL) {
		fclose(system_log);
		system_log = NULL;
	}

	if (loaded_versions_log != NULL) {
		fclose(loaded_versions_log);
		loaded_versions_log = NULL;
	}

	deInitAllCanPorts();

	if (can_load_status == CAN_LOAD_IN_PROCESS) {
		can_load_status = CAN_LOAD_SUCCESS;
	}

	return PROCESS_SUCCESS;
}

void append_versions_log(can_module_s *module)
{
	if (loaded_versions_log == NULL) {
		DEBUG_OUTPUT("Loaded versions log not open");
		return;
	}

	char line[LOG_LINE_LENGTH] = {0};

	if (snprintf(line, sizeof(line), "%s VER: %" PRIu8 ".%" PRIu8 ".%" PRIu8 " CRC: %" PRIX32 "\n",
		     module->module_type_name, module->file_metadata.major, module->file_metadata.minor,
		     module->file_metadata.patch, module->file_metadata.crc) < 0) {
		DEBUG_OUTPUT("Unable to format version log line");
		return;
	}

	DEBUG_OUTPUT("%s: Load success: %s", module->module_type_name, line);

	if (fputs(line, loaded_versions_log) == EOF) {
		DEBUG_OUTPUT("Unable to write to loaded versions file");
	}
}

bool print_loaded_versions(void)
{
	FILE *file = fopen(loaded_versions_filename, "r");

	if (file == NULL) {
		DEBUG_OUTPUT("Unable to open loaded versions file");
		return true;
	}

	char line[LOG_LINE_LENGTH];

	while (fgets(line, sizeof(line), file)) {
		unsigned long len = strlen(line);

		if (len > 0 && line[len - 1] == '\n') {
			line[len - 1] = '\0';
		}

		DEBUG_OUTPUT("%s", line);
	}

	if (fclose(file) != 0) {
		DEBUG_OUTPUT("Unable to close loaded versions file");
		return true;
	}

	return false;
}

void can_module_set_all_addressing_lines(int state)
{
	// set all the PWM lines to high to start
	for (size_t i = 0; i < ESC_QTY; i++) {
		set_pwm_output(i, state);
	}
}

void can_module_set_addressing_line(can_module_s *module, int state)
{
	// Chip-select the appropriate module and set the module type and CAN ID
	// Select line is active low!
	switch (module->currentId) {
	case 1: //motor with boom ID 1 = ID 1
		set_pwm_output(0, state);
		break;

	case 2:
		set_pwm_output(1, state);
		break;

	case 3:
		set_pwm_output(2, state);
		break;

	case 4:
		set_pwm_output(3, state);
		break;

	default:
		DEBUG_OUTPUT("%s: no addressing line driven\r\n", module->module_name, module->currentId);
		break;
	}

	DELAY_MS(100); // wait a little bit for the PWM lines to get set via comms
}

void handle_can_module_error(can_module_s *module, enum can_module_error_code errorCode)
{
	can_load_status = CAN_LOAD_FAILURE;

	if (module->fat_file_can != NULL) {
		fclose(module->fat_file_can);
		module->fat_file_can = NULL;
	}

	if (module->optional_load) {
		return;
	}

	if (can_module_type_alternative_exists(module)) {
		return;
	}

	if (system_log != NULL) {
		fclose(system_log);
		system_log = NULL;
	}

	if (loaded_versions_log != NULL) {
		fclose(loaded_versions_log);
		loaded_versions_log = NULL;
	}
}

bool assign_all_esc_boom_ids(void)
{
	{
		//if(can_load_status != CAN_LOAD_FAILURE){
		can_module_s *esc = &modules[0];

		for (size_t i = 0; i < esc->module_ids.length; i++) {
			set_module_incremental(i, esc);
			can_module_jump_from_bl_to_application(esc, esc->currentId);
			DELAY_MS(200);

			if (assign_boom_id(esc, esc->currentId) != PROCESS_SUCCESS) {
				DEBUG_OUTPUT("%s: Failed to assign boom id", esc->module_name);
				return PROCESS_FAILURE;

			} else {
				DEBUG_OUTPUT("%s: Assigned boom id", esc->module_name);
			}
		}
	}
	return PROCESS_SUCCESS;
}

#define CAN_ADDR_BOOM_SET 0x21
#define CAN_ADDR_BOOM_ACK 0x20
#define CAN_VERIFICATION_BOOM_HR 0x50039871
#define CAN_VERIFICATION_BOOM_LR 0x05956200
#define CAN_VERIFICATION_BOOM_LR_MASK 0xFFFFFF00
#define CAN_VERIFICATION_BOOM_ACK_HR 0x1e62125e
#define CAN_VERIFICATION_BOOM_ACK_LR 0x2c7bdd00 //Transmit only, no mask

bool assign_boom_id(can_module_s *module, int id)
{
	can_module_set_all_addressing_lines(0);
	can_module_set_addressing_line(module, 1);

	module->can_handle->pTxMsg.StdId = CAN_ADDR_BOOM_SET;
	module->can_handle->pTxMsg.RTR = CAN_RTR_DATA;
	module->can_handle->pTxMsg.IDE = CAN_ID_STD;
	module->can_handle->pTxMsg.DLC = 8;
	module->can_handle->pTxMsg.Data[0] = id & 0xFF;
	module->can_handle->pTxMsg.Data[1] = (CAN_VERIFICATION_BOOM_LR >> 8) & 0xFF;
	module->can_handle->pTxMsg.Data[2] = (CAN_VERIFICATION_BOOM_LR >> 16) & 0xFF;
	module->can_handle->pTxMsg.Data[3] = (CAN_VERIFICATION_BOOM_LR >> 24) & 0xFF;
	module->can_handle->pTxMsg.Data[4] = (CAN_VERIFICATION_BOOM_HR >> 0) & 0xFF;
	module->can_handle->pTxMsg.Data[5] = (CAN_VERIFICATION_BOOM_HR >> 8) & 0xFF;
	module->can_handle->pTxMsg.Data[6] = (CAN_VERIFICATION_BOOM_HR >> 16) & 0xFF;
	module->can_handle->pTxMsg.Data[7] = (CAN_VERIFICATION_BOOM_HR >> 24) & 0xFF;

	if (hw_CAN_transmit(module->can_handle, 10) != CAN_HW_OK) {
		return PROCESS_FAILURE;
	}

	bool ackReceived = false;
	hrt_abstime reset_timestamp = hrt_absolute_time();

	while (hrt_elapsed_time(&reset_timestamp) < 1000000) {
		if (hw_CAN_receive(module->can_handle, 100) == CAN_HW_FAIL) {
			DEBUG_OUTPUT("%s: device timed out when trying to assign boom id", module->module_name);

			return PROCESS_FAILURE;
		}

		if (module->can_handle->pRxMsg.DLC < 8) {
			continue;
		}

		if (module->can_handle->pRxMsg.StdId != CAN_ADDR_BOOM_ACK) {
			continue;
		}

		uint32_t ack_code_lr = (module->can_handle->pRxMsg.Data[3] << 24) | (module->can_handle->pRxMsg.Data[2] << 16) |
				       (module->can_handle->pRxMsg.Data[1] << 8) | (module->can_handle->pRxMsg.Data[0] << 0);
		uint32_t ack_code_hr = (module->can_handle->pRxMsg.Data[7] << 24) | (module->can_handle->pRxMsg.Data[6] << 16) |
				       (module->can_handle->pRxMsg.Data[5] << 8) | (module->can_handle->pRxMsg.Data[4] << 0);

		// Check verification code and boom ID is correct
		if ((ack_code_lr != (CAN_VERIFICATION_BOOM_ACK_LR | module->currentId))
		    || (ack_code_hr != CAN_VERIFICATION_BOOM_ACK_HR)) {
			DEBUG_OUTPUT("%s: Received incorrect Boom ID ack...[%x %x : %x %x]", module->module_name, ack_code_lr, ack_code_hr,
				     module->currentId, CAN_VERIFICATION_BOOM_ACK_HR);
			continue;

		} else {
			ackReceived = true;
			break;
		}
	}

	if (ackReceived) {
		DEBUG_OUTPUT("%s: Boom ID ack received", module->module_name);
		return PROCESS_SUCCESS;

	} else {
		DEBUG_OUTPUT("%s: Boom ID process failed", module->module_name);
		return PROCESS_FAILURE;
	}
}

#define DEBUG_BUF_LENGTH 300

void debugPrint(char *fmt, ...)
{
	char formattedString[DEBUG_BUF_LENGTH] = {0};

	va_list arguments;
	va_start(arguments, fmt);

	vsnprintf(formattedString, DEBUG_BUF_LENGTH - 1, fmt, arguments);
	va_end(arguments);

	//int stringlength = strlen(formattedString);

	PX4_INFO("%s", formattedString);


	if (system_log == NULL) {
		return;
	}

	fputs(formattedString, system_log);
	fputs("\n", system_log);
}
