/****************************************************************************
 *
 *   Copyright (C) 2019, 2022 PX4 Development Team. All rights reserved.
 *   Author: @author David Sidrane <david_s5@nscdg.com>
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
 * @file board_hw_rev_ver.c
 * Implementation of IMXRT based Board Hardware Revision and Version ID API
 */
#include <drivers/drv_adc.h>
#include <px4_arch/adc.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_manifest.h>
#include <px4_platform/board_determine_hw_info.h>
#include <px4_platform/board_hw_eeprom_rev_ver.h>
#include <stdio.h>
#include <fcntl.h>
#include <board_config.h>

#include <lib/crc/crc.h>
#include <lib/systemlib/px4_macros.h>

#if defined(BOARD_HAS_HW_VERSIONING)

#  if defined(GPIO_HW_VER_REV_DRIVE)
#    define GPIO_HW_REV_DRIVE GPIO_HW_VER_REV_DRIVE
#    define GPIO_HW_VER_DRIVE GPIO_HW_VER_REV_DRIVE
#  endif

#define HW_INFO_SIZE (int) arraySize(HW_INFO_INIT_PREFIX) + HW_INFO_VER_DIGITS + HW_INFO_REV_DIGITS


/****************************************************************************
 * Private Data
 ****************************************************************************/
static int hw_version = 0;
static int hw_revision = 0;
static char hw_info[HW_INFO_SIZE] = {0};

/****************************************************************************
 * Protected Functions
 ****************************************************************************/

static int dn_to_ordinal(uint16_t dn)
{

	const struct {
		uint16_t low;  // High(n-1) + 1
		uint16_t high; // Average High(n)+Low(n+1) EX. 1356 = AVRG(1331,1382)
	} dn2o[] = {
		//   R1(up) R2(down)    V min       V Max       DN Min DN Max
		{0,   0   },   // 0                     No Resistors
		{1,   579 },   // 1  24.9K   442K   0.166255191  0.44102252    204    553
		{580, 967 },   // 2  32.4K   174K   0.492349322  0.770203609   605    966
		{968, 1356},   // 3  38.3K   115K   0.787901749  1.061597759   968    1331
		{1357, 1756},  // 4  46.4K   84.5K  1.124833577  1.386007306   1382   1738
		{1757, 2137},  // 5  51.1K   61.9K  1.443393279  1.685367869   1774   2113
		{2138, 2519},  // 6  61.9K   51.1K  1.758510242  1.974702534   2161   2476
		{2520, 2919},  // 7  84.5K   46.4K  2.084546498  2.267198261   2562   2842
		{2920, 3308},  // 8  115K    38.3K  2.437863827  2.57656294    2996   3230
		{3309, 3699},  // 9  174K    32.4K  2.755223792  2.847933804   3386   3571
		{3700, 4095},  // 10 442K    24.9K  3.113737849  3.147347506   3827   3946
	};

	for (unsigned int i = 0; i < arraySize(dn2o); i++) {
		if (dn >= dn2o[i].low && dn <= dn2o[i].high) {
			return i;
		}
	}

	return -1;
}

/************************************************************************************
 * Name: read_id_dn
 *
 * Description:
 *   Read the HW sense set to get a DN of the value formed by
 *                0 VDD
 *                |
 *                /
 *                \   R1
 *                /
 *                |
 *                +--------------- GPIO_HW_xxx_SENCE  | ADC channel N
 *                |
 *                /
 *                \ R2
 *                /
 *                |
 *                |
 *                +--------------- GPIO_HW_xxx_DRIVE or GPIO_HW_VER_REV_DRIVE
 *
 * Input Parameters:
 *   id          - pointer to receive the dn for the id set
 *   gpio_drive  - gpio that is the drive
 *   gpio_sense  - gpio that is the sence
 *   adc_channel - the Channel number associated with gpio_sense
 *
 * Returned Value:
 *    0    - Success and id is set
 *   -EIO  - FAiled to init or read the ADC
 *
 ************************************************************************************/

static int read_id_dn(int *id, uint32_t gpio_drive, uint32_t gpio_sense, int adc_channel)
{
	int rv = -EIO;

	const unsigned int samples  = 16;
	/*
	 * Step one is there resistors?
	 *
	 * If we set the mid-point of the ladder which is the ADC input to an
	 * output, then whatever state is driven out should be seen by the GPIO
	 * that is on the bottom of the ladder that is switched to an input.
	 * The SENCE line is effectively an output with a high value pullup
	 * resistor on it driving an input through a series resistor with a pull up.
	 * If present the series resistor will form a low pass filter due to stray
	 * capacitance, but this is fine as long as we give it time to settle.
	 */

	/*  Turn the drive lines to digital inputs with No pull up */

	imxrt_config_gpio(PX4_MAKE_GPIO_INPUT(gpio_drive) & ~IOMUX_PULL_MASK);

	/*  Turn the sense lines to digital outputs LOW */

	imxrt_config_gpio(PX4_MAKE_GPIO_OUTPUT_CLEAR(gpio_sense));



	up_udelay(100); /* About 10 TC assuming 485 K */

	/*  Read Drive lines while sense are driven low */

	int low = imxrt_gpio_read(PX4_MAKE_GPIO_INPUT(gpio_drive));


	/*  Write the sense lines HIGH */

	imxrt_gpio_write(PX4_MAKE_GPIO_OUTPUT_CLEAR(gpio_sense), 1);

	up_udelay(100); /* About 10 TC assuming 485 K */
	/*  Read Drive lines while sense are driven high */

	int high = imxrt_gpio_read(PX4_MAKE_GPIO_INPUT(gpio_drive));

	/* restore the pins to ANALOG */

	imxrt_config_gpio(gpio_sense);

	/*  Turn the drive lines to digital outputs LOW */

	imxrt_config_gpio(gpio_drive ^ GPIO_OUTPUT_SET);

	up_udelay(100); /* About 10 TC assuming 485 K */

	/* Are Resistors in place ?*/

	uint32_t dn_sum = 0;
	uint32_t dn = 0;

	if ((high ^ low) && low == 0) {
		/* Yes - Fire up the ADC (it has once control) */
		if (px4_arch_adc_init(HW_REV_VER_ADC_BASE) == OK) {

			/* Read the value */
			for (unsigned av = 0; av < samples; av++) {
				dn = px4_arch_adc_sample(HW_REV_VER_ADC_BASE, adc_channel);

				if (dn == UINT32_MAX) {
					break;
				}

				dn_sum  += dn;
			}

			if (dn != UINT32_MAX) {
				*id = dn_sum / samples;
				rv = OK;
			}
		}

	} else {
		/* No - No Resistors is ID 0 */
		*id = 0;
		rv = OK;
	}

	/*  Turn the drive lines to digital outputs High */

	imxrt_config_gpio(gpio_drive);

	return rv;
}


static int determine_hw_info(int *revision, int *version)
{

	int dn;
	int rv = read_id_dn(&dn, GPIO_HW_REV_DRIVE, GPIO_HW_REV_SENSE, ADC_HW_REV_SENSE_CHANNEL);

	if (rv == OK) {
		*revision =  dn_to_ordinal(dn);
		rv = read_id_dn(&dn, GPIO_HW_VER_DRIVE, GPIO_HW_VER_SENSE, ADC_HW_VER_SENSE_CHANNEL);

		if (rv == OK) {
			*version =  dn_to_ordinal(dn);
		}
	}

	return rv;
}

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
	return  hw_version;
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
	return  hw_revision;
}

/************************************************************************************
  * Name: board_determine_hw_info
 *
 * Description:
 *	Uses the HW revision and version detection added in FMUv5.
 *	See https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY
 *	HW REV and VER ID tab.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0  - on success or negated errono
 *   1) The values for integer value of this boards hardware revision is set
 *   2) The integer value of this boards hardware version is set.
 *   3) hw_info is populated
 *
 *   A value of 0 is the default for boards supporting the BOARD_HAS_HW_VERSIONING API.
 *   but not having R1 and R2.
 *
 ************************************************************************************/

int board_determine_hw_info()
{

	// Read ADC jumpering hw_info
	int rv = determine_hw_info(&hw_revision, &hw_version);

	if (rv == OK) {

		// MFT supported?
		const char *path;
		int rvmft = px4_mtd_query("MTD_MFT_VER", NULL, &path);

		if (rvmft == OK && path != NULL && hw_version == HW_ID_EEPROM) {

			mtd_mft_v0_t mtd_mft = {MTD_MFT_v0};
			rv = board_get_eeprom_hw_info(path, (mtd_mft_t *)&mtd_mft);

			if (rv == OK) {
				hw_version = mtd_mft.hw_extended_id;
			}
		}

		path = NULL;
		rvmft = px4_mtd_query("MTD_MFT_REV", NULL, &path);

		if (rvmft == OK && path != NULL && hw_revision == HW_ID_EEPROM) {

			mtd_mft_v0_t mtd_mft = {MTD_MFT_v0};
			rv = board_get_eeprom_hw_info(path, (mtd_mft_t *)&mtd_mft);

			if (rv == OK) {
				hw_revision = mtd_mft.hw_extended_id;
			}
		}
	}

	if (rv == OK) {
		snprintf(hw_info, sizeof(hw_info), HW_INFO_INIT_PREFIX HW_INFO_SUFFIX, hw_version, hw_revision);
	}

	return rv;
}

/************************************************************************************
  * Name: board_set_eeprom_hw_info
 *
 * Description:
 * Function for writing hardware info to EEPROM
 *
 * Input Parameters:
 *   *mtd_mft_unk - pointer to mtd_mft to write hw_info
 *
 * Returned Value:
 *    0    - Successful storing to EEPROM
 *   -1    - Error while storing to EEPROM
 *
 ************************************************************************************/

int board_set_eeprom_hw_info(const char *path, mtd_mft_t *mtd_mft_unk)
{
	if (mtd_mft_unk == NULL || path == NULL) {
		return -EINVAL;
	}

	// Later this will be a demux on type
	if (mtd_mft_unk->id != MTD_MFT_v0) {
		printf("Version is: %d, Only mft version %d is supported\n", mtd_mft_unk->id, MTD_MFT_v0);
		return -EINVAL;
	}

	mtd_mft_v0_t *mtd_mft = (mtd_mft_v0_t *)mtd_mft_unk;

	if (mtd_mft->hw_extended_id < HW_EEPROM_ID_MIN) {
		printf("hardware version for EEPROM must be greater than %x\n", HW_EEPROM_ID_MIN);
		return -EINVAL;
	}

	int fd = open(path, O_WRONLY);

	if (fd < 0) {
		return -errno;
	}

	int ret_val = OK;

	mtd_mft->crc = crc16_signature(CRC16_INITIAL, sizeof(*mtd_mft) - sizeof(mtd_mft->crc), (uint8_t *) mtd_mft);

	if (
		(MTD_MFT_OFFSET != lseek(fd, MTD_MFT_OFFSET, SEEK_SET)) ||
		(sizeof(*mtd_mft) != write(fd, mtd_mft, sizeof(*mtd_mft)))
	) {
		ret_val = -errno;
	}

	close(fd);

	return ret_val;
}

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
__EXPORT int board_get_eeprom_hw_info(const char *path, mtd_mft_t *mtd_mft)
{
	if (mtd_mft == NULL || path == NULL) {
		return -EINVAL;
	}

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		return -errno;
	}

	int ret_val = OK;
	mtd_mft_t format_version = {-1};

	if (
		(MTD_MFT_OFFSET != lseek(fd, MTD_MFT_OFFSET, SEEK_SET)) ||
		(sizeof(format_version) != read(fd, &format_version, sizeof(format_version)))
	) {
		ret_val = -errno;

	} else if (format_version.id != mtd_mft->id) {
		ret_val = -EPROTO;

	} else {

		uint16_t mft_size = 0;

		switch (format_version.id) {
		case MTD_MFT_v0: mft_size = sizeof(mtd_mft_v0_t); break;

		case MTD_MFT_v1: mft_size = sizeof(mtd_mft_v1_t); break;

		default:
			printf("[boot] Error, unknown version %d of mtd_mft in EEPROM\n", format_version.id);
			ret_val = -1;
			break;
		}

		if (ret_val == OK) {

			if (
				(MTD_MFT_OFFSET != lseek(fd, MTD_MFT_OFFSET, SEEK_SET)) ||
				(mft_size != read(fd, mtd_mft, mft_size))
			) {
				ret_val = -errno;

			} else {

				union {
					uint16_t w;
					uint8_t  b[2];
				} crc;

				uint8_t *bytes = (uint8_t *) mtd_mft;
				crc.w = crc16_signature(CRC16_INITIAL, mft_size - sizeof(crc), bytes);
				uint8_t *eeprom_crc = &bytes[mft_size - sizeof(crc)];

				if (!(crc.b[0] == eeprom_crc[0] && crc.b[1] == eeprom_crc[1])) {
					ret_val = -1;
				}
			}
		}
	}

	close(fd);
	return ret_val;
}

#endif
