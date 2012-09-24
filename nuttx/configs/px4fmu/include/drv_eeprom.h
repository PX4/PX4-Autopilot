/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file drv_eeprom.h
 *
 * Config for the non-MTD EEPROM driver.
 */

/* IMPORTANT: Adjust this number! */
#define MAX_EEPROMS					2

/* FMU onboard */
#define FMU_BASEBOARD_EEPROM_ADDRESS			0x57
#define FMU_BASEBOARD_EEPROM_TOTAL_SIZE_BYTES		128
#define FMU_BASEBOARD_EEPROM_PAGE_SIZE_BYTES		8
#define FMU_BASEBOARD_EEPROM_PAGE_WRITE_TIME_US		3300
#define FMU_BASEBOARD_EEPROM_BUS_CLOCK			400000    ///< 400 KHz max. clock

/**
 * @brief i2c I2C bus struct
 * @brief device_address The device address as stated in the datasheet, e.g. for a Microchip 24XX128 0x50 with all ID pins tied to GND
 * @brief total_size_bytes The total size in bytes, e.g. 16K = 16000 bytes for the Microchip 24XX128
 * @brief page_size_bytes The size of one page, e.g. 64 bytes for the Microchip 24XX128
 * @brief device_name The device name to register this device to, e.g. /dev/eeprom
 * @brief fail_if_missing Returns error if the EEPROM was not found. This is helpful if the EEPROM might be attached later when the board is running
 */
extern int
eeprom_attach(struct i2c_dev_s *i2c, uint8_t device_address, uint16_t total_size_bytes, uint16_t page_size_bytes, uint16_t page_write_time_us, const char* device_name, uint8_t fail_if_missing);

