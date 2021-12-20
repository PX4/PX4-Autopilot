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
#pragma once

#define HW_VERSION_EEPROM 		0x7		//!< Get hw_info from EEPROM
#define HW_EEPROM_VERSION_MIN	0x10	//!< Minimum supported version

#pragma pack(push, 1)

typedef struct {
	uint16_t format_version;
	uint16_t hw_extended_ver;
	uint16_t crc;
} mtd_mft_v0_t;

typedef struct {
	uint16_t format_version;
	uint16_t hw_extended_ver;
	//{device tree overlay}
	uint16_t crc;
} mtd_mft_v1_t;

typedef struct {
	uint16_t hw_extended_ver;
	//{device tree overlay}
} mtd_mft_t;

#pragma pack(pop)

#define MTD_MFT_v0 0U //<! EEPROM MTD MFT structure version 0
#define MTD_MFT_v1 1U //<! EEPROM MTD MFT structure version 1

#define MTD_MFT_OFFSET 0 //<! Offset in EEPROM where mtd_mft data starts

/************************************************************************************
  * Name: board_set_eeprom_hw_info
 *
 * Description:
 * Function for writing hardware info to EEPROM
 *
 * Input Parameters:
 *   *mtd_mft - pointer to mtd_mft to write hw_info
 *
 * Returned Value:
 *    0    - Successful storing to EEPROM
 *   -1    - Error while storing to EEPROM
 *
 ************************************************************************************/

#if !defined(BOARD_HAS_SIMPLE_HW_VERSIONING) && defined(BOARD_HAS_VERSIONING)
__EXPORT int board_set_eeprom_hw_info(mtd_mft_t *mtd_mft);
#else
static inline int board_set_eeprom_hw_info(mtd_mft_t *mtd_mft) { return -ENOSYS; }
#endif

/************************************************************************************
  * Name: board_get_eeprom_hw_info
 *
 * Description:
 * Function for reading hardware info from EEPROM
 *
 * Output Parameters:
 *   *mtd_mft - pointer to mtd_mft to read hw_info
 *
 * Returned Value:
 *    0    - Successful reading from EEPROM
 *   -1    - Error while reading from EEPROM
 *
 ************************************************************************************/

#if !defined(BOARD_HAS_SIMPLE_HW_VERSIONING) && defined(BOARD_HAS_VERSIONING)
__EXPORT int board_get_eeprom_hw_info(mtd_mft_t *mtd_mft);
#else
static inline int board_get_eeprom_hw_info(mtd_mft_t *mtd_mft) { return -ENOSYS; }
#endif
