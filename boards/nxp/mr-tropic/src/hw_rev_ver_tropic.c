/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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
 * @file hw_rev_ver_canhubk3.c
 * CANHUBK3 Hardware Revision and Version ID API
 */
#include <drivers/drv_adc.h>
#include <px4_arch/adc.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform/board_determine_hw_info.h>
#include <stdio.h>
#include <board_config.h>
#include <drivers/drv_sensor.h>

#include <systemlib/px4_macros.h>

#include <nuttx/spi/spi.h>

#if defined(BOARD_HAS_HW_VERSIONING)

#define DIR_READ     0x80
#define WHO_AMI_I    0x75
#define REG_BANK_SEL 0x76

#define HW_INFO_SIZE HW_INFO_VER_DIGITS + HW_INFO_REV_DIGITS

#define DEFAULT_SPI ((1 << 28) | DRV_IMU_DEVTYPE_ICM45686)


/****************************************************************************
 * Private Data
 ****************************************************************************/
static int hw_revision = 0;
static char hw_info[HW_INFO_SIZE] = {0};
#if defined(BOARD_HAS_HW_SPLIT_VERSIONING)
static char hw_base_info[HW_INFO_SIZE] = {0};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/************************************************************************************
 * Name: board_get_hw_type
 *
 * Description:
 *   Optional returns a 0 terminated string defining the HW type.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   a 0 terminated string defining the HW type. This my be a 0 length string ""
 *
 ************************************************************************************/

__EXPORT const char *board_get_hw_type_name()
{
	return (const char *) hw_info;
}

#if defined(BOARD_HAS_HW_SPLIT_VERSIONING)
/************************************************************************************
 * Name: board_get_hw_base_type_name
 *
 * Description:
 *   Optional returns a 0 terminated string defining the base type.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   a 0 terminated string defining the HW type. This my be a 0 length string ""
 *
 ************************************************************************************/

__EXPORT const char *board_get_hw_base_type_name()
{
	return (const char *) hw_base_info;
}
#endif

/************************************************************************************
 * Name: board_get_hw_version
 *
 * Description:
 *   Optional returns a integer HW version
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer value of this boards hardware version.
 *   A value of -1 is the default for boards not supporting the BOARD_HAS_VERSIONING API.
 *   A value of 0 is the default for boards supporting the API but not having version.
 *
 ************************************************************************************/

__EXPORT int board_get_hw_version()
{
	return 0;
}

/************************************************************************************
 * Name: board_get_hw_revision
 *
 * Description:
 *   Optional returns a integer HW revision
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer value of this boards hardware revision.
 *   A value of -1 is the default for boards not supporting the BOARD_HAS_VERSIONING API.
 *   A value of 0 is the default for boards supporting the API but not having revision.
 *
 ************************************************************************************/

__EXPORT int board_get_hw_revision()
{
	return hw_revision;
}

/************************************************************************************
  * Name: board_determine_hw_info
 *
 * Description:
 *	Uses GPIO to detect MR-CANHUBK3-ADAP
 *
 ************************************************************************************/

int board_determine_hw_info()
{
	struct spi_dev_s *spi = px4_spibus_initialize(3);

	SPI_SETFREQUENCY(spi, 8 * 1000 * 1000);
	SPI_SETMODE(spi, SPIDEV_MODE3);
	SPI_SETBITS(spi, 8);

	/* ICM Select bank 0 */
	SPI_SELECT(spi, DEFAULT_SPI, true);
	uint8_t cmd_bank_sel[2] = {0};
	cmd_bank_sel[0] = REG_BANK_SEL;
	cmd_bank_sel[1] = 0;
	SPI_EXCHANGE(spi, cmd_bank_sel, cmd_bank_sel, sizeof(cmd_bank_sel));
	SPI_SELECT(spi, DEFAULT_SPI, false);

	/* ICM Read WHOAM */
	SPI_SELECT(spi, DEFAULT_SPI, true);
	uint8_t cmd[2] = {0};
	cmd[0] = (WHO_AMI_I) | DIR_READ;
	SPI_EXCHANGE(spi, cmd, cmd, sizeof(cmd_bank_sel));
	SPI_SELECT(spi, DEFAULT_SPI, false);

	// Default is TROPIC_0 icm45686

	if (cmd[1] == 0x47) { // TROPIC_1 icm42688p
		hw_revision = 1;

	} else if (cmd[1] == 0x44) { // TROPIC_2 icm42686p
		hw_revision = 2;
	}

	/* Default bus 1 to 8MHz and de-assert the known chip selects.
	 */

	SPI_SETFREQUENCY(spi, 8 * 1000 * 1000);
	SPI_SETBITS(spi, 8);
	SPI_SETMODE(spi, SPIDEV_MODE3);

	sprintf(hw_info, "%03d", hw_revision);

	return 0;
}
#endif
