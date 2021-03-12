/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

/**
 * @file VescProtocol.h
 * @brief VESC protocol definitions
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <stdint.h>

static constexpr int MAXIMUM_PAYLOAD_LENGTH{512};
static constexpr int PACKET_OVERHEAD_LENGTH{6}; // Bytes: 1 start, 2 size, 2 CRC, 1 stop
static constexpr int MAXIMUM_PACKET_LENGTH{MAXIMUM_PAYLOAD_LENGTH + PACKET_OVERHEAD_LENGTH};

enum VescCommand : uint8_t {
	FW_VERSION = 0,
	JUMP_TO_BOOTLOADER,
	ERASE_NEW_APP,
	WRITE_NEW_APP_DATA,
	GET_VALUES,
	SET_DUTY,
	SET_CURRENT,
	SET_CURRENT_BRAKE,
	SET_RPM,
	SET_POS,
	SET_HANDBRAKE,
	SET_DETECT,
	SET_SERVO_POS,
	SET_MCCONF,
	GET_MCCONF,
	GET_MCCONF_DEFAULT,
	SET_APPCONF,
	GET_APPCONF,
	GET_APPCONF_DEFAULT,
	SAMPLE_PRINT,
	TERMINAL_CMD,
	PRINT,
	ROTOR_POSITION,
	EXPERIMENT_SAMPLE,
	DETECT_MOTOR_PARAM,
	DETECT_MOTOR_R_L,
	DETECT_MOTOR_FLUX_LINKAGE,
	DETECT_ENCODER,
	DETECT_HALL_FOC,
	REBOOT,
	ALIVE,
	GET_DECODED_PPM,
	GET_DECODED_ADC,
	GET_DECODED_CHUK,
	FORWARD_CAN,
	SET_CHUCK_DATA,
	CUSTOM_APP_DATA,
	NRF_START_PAIRING,
	GPD_SET_FSW,
	GPD_BUFFER_NOTIFY,
	GPD_BUFFER_SIZE_LEFT,
	GPD_FILL_BUFFER,
	GPD_OUTPUT_SAMPLE,
	GPD_SET_MODE,
	GPD_FILL_BUFFER_INT8,
	GPD_FILL_BUFFER_INT16,
	GPD_SET_BUFFER_INT_SCALE,
	GET_VALUES_SETUP,
	SET_MCCONF_TEMP,
	SET_MCCONF_TEMP_SETUP,
	GET_VALUES_SELECTIVE,
	GET_VALUES_SETUP_SELECTIVE,
	EXT_NRF_PRESENT,
	EXT_NRF_ESB_SET_CH_ADDR,
	EXT_NRF_ESB_SEND_DATA,
	EXT_NRF_ESB_RX_DATA,
	EXT_NRF_SET_ENABLED,
	DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,
	DETECT_APPLY_ALL_FOC,
	JUMP_TO_BOOTLOADER_ALL_CAN,
	ERASE_NEW_APP_ALL_CAN,
	WRITE_NEW_APP_DATA_ALL_CAN,
	PING_CAN,
	APP_DISABLE_OUTPUT,
	TERMINAL_CMD_SYNC,
	GET_IMU_DATA,
	BM_CONNECT,
	BM_ERASE_FLASH_ALL,
	BM_WRITE_FLASH,
	BM_REBOOT,
	BM_DISCONNECT,
	BM_MAP_PINS_DEFAULT,
	BM_MAP_PINS_NRF5X,
	ERASE_BOOTLOADER,
	ERASE_BOOTLOADER_ALL_CAN,
	PLOT_INIT,
	PLOT_DATA,
	PLOT_ADD_GRAPH,
	PLOT_SET_GRAPH,
	GET_DECODED_BALANCE,
	BM_MEM_READ,
	WRITE_NEW_APP_DATA_LZO,
	WRITE_NEW_APP_DATA_ALL_CAN_LZO,
	BM_WRITE_FLASH_LZO,
	SET_CURRENT_REL,
	CAN_FWD_FRAME,
	SET_BATTERY_CUT,
	SET_BLE_NAME,
	SET_BLE_PIN,
	SET_CAN_MODE,
	GET_IMU_CALIBRATION,
	GET_MCCONF_TEMP,

	// Custom configuration for hardware
	GET_CUSTOM_CONFIG_XML,
	GET_CUSTOM_CONFIG,
	GET_CUSTOM_CONFIG_DEFAULT,
	SET_CUSTOM_CONFIG,

	// BMS commands
	BMS_GET_VALUES,
	BMS_SET_CHARGE_ALLOWED,
	BMS_SET_BALANCE_OVERRIDE,
	BMS_RESET_COUNTERS,
	BMS_FORCE_BALANCE,
	BMS_ZERO_CURRENT_OFFSET,

	// Firmware update commands for different hardware types
	JUMP_TO_BOOTLOADER_HW,
	ERASE_NEW_APP_HW,
	WRITE_NEW_APP_DATA_HW,
	ERASE_BOOTLOADER_HW,
	JUMP_TO_BOOTLOADER_ALL_CAN_HW,
	ERASE_NEW_APP_ALL_CAN_HW,
	WRITE_NEW_APP_DATA_ALL_CAN_HW,
	ERASE_BOOTLOADER_ALL_CAN_HW,

	SET_ODOMETER,
};

static constexpr uint16_t CRC_TABLE[256] = {0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

struct VescVersion {
	uint8_t version_major;
	uint8_t version_minor;
	// char hardware_name[50];
	// uint8_t stm32_uuid_8[12];
	// bool pairing_done;
	// uint8_t test_version_number;
	// uint8_t hardware_type;
	// uint8_t custom_configuration;
};

struct VescValues {
	float fet_temperature;
	float motor_temperature;
	float motor_current;
	float input_current;
	float reset_average_id;
	float reset_average_iq;
	float duty_cycle;
	int32_t rpm;
	float input_voltage;
	float used_charge_Ah;
	float charged_charge_Ah;
	float used_energy_Wh;
	float charged_energy_wh;
	int32_t tachometer;
	int32_t tachometer_absolute;
	int8_t fault;
	float position_pid;
	int8_t controller_id;
	float ntc_temperature_mos1;
	float ntc_temperature_mos2;
	float ntc_temperature_mos3;
	float read_reset_average_vd;
	float read_reset_average_vq;
};
