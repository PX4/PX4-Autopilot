/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file pab_manifest.c
 *
 * This module supplies the interface to the manifest of hardware that is
 * optional and dependent on the HW_BASE_ID
 *
 * The manifest allows the system to know whether a hardware option
 * say for example the PX4IO is an no-pop option vs it is broken.
 *
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <board_config.h>

#if defined(BOARD_HAS_HW_SPLIT_VERSIONING)

#include <inttypes.h>
#include <stdbool.h>
#include <syslog.h>

#include "systemlib/px4_macros.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

typedef struct {
	hw_base_id_t             hw_base_id; /* The ID of the Base */
	const px4_hw_mft_item_t *mft;        /* The first entry */
	uint32_t                 entries;    /* the lenght of the list */
} px4_hw_mft_list_entry_t;

typedef px4_hw_mft_list_entry_t *px4_hw_mft_list_entry;
#define px4_hw_mft_list_uninitialized (px4_hw_mft_list_entry) -1

static const px4_hw_mft_item_t device_unsupported = {0, 0, 0};

// List of components on a specific base board configuration
// The ids of those components is given by the enum (px4_hw_mft_item_id_t)
// declared in board_common.h

// BASE ID 0 Auterion vXx base board
static const px4_hw_mft_item_t base_configuration_0[] = {
	{
		.id          =  PX4_MFT_PX4IO,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_USB,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_CAN2,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_CAN3,
		.present     = 0,
		.mandatory   = 0,
		.connection  = px4_hw_con_unknown,
	},
	{
		.id          =  PX4_MFT_PM2,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_ETHERNET,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_connector,
	},
	{
		.id          = PX4_MFT_T100_ETH,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_connector,
	},
};

// BASE ID 1 vXx base without px4io
static const px4_hw_mft_item_t base_configuration_1[] = {
	{
		.id          = PX4_MFT_PX4IO,
		.present     = 0,
		.mandatory   = 0,
		.connection  = px4_hw_con_unknown,
	},
	{
		.id          = PX4_MFT_USB,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_CAN2,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_CAN3,
		.present     = 0,
		.mandatory   = 0,
		.connection  = px4_hw_con_unknown,
	},
	{
		.id          =  PX4_MFT_PM2,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_ETHERNET,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_connector,
	},
	{
		.id          = PX4_MFT_T100_ETH,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_connector,
	},
};

// BASE ID 2 Modal AI Alaised to ID 0

// BASE ID 3 NXP T1 PHY
static const px4_hw_mft_item_t base_configuration_3[] = {
	{
		.id          = PX4_MFT_PX4IO,
		.present     = 0,
		.mandatory   = 0,
		.connection  = px4_hw_con_unknown,
	},
	{
		.id          = PX4_MFT_USB,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_CAN2,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_CAN3,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          =  PX4_MFT_PM2,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_ETHERNET,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_connector,
	},
	{
		.id          = PX4_MFT_T1_ETH,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_connector,
	},
};

// BASE ID 4 HB CM4 Alaised to ID 0

// BASE ID 5  HB min
static const px4_hw_mft_item_t base_configuration_5[] = {
	{
		.id          = PX4_MFT_PX4IO,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_USB,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_CAN2,
		.present     = 0,
		.mandatory   = 0,
		.connection  = px4_hw_con_unknown,
	},
	{
		.id          = PX4_MFT_CAN3,
		.present     = 0,
		.mandatory   = 0,
		.connection  = px4_hw_con_unknown,
	},
	{
		.id          = PX4_MFT_ETHERNET,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_connector,
	},
	{
		.id          = PX4_MFT_T100_ETH,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_connector,
	},
};

// BASE ID 6  Not allocated
// BASE ID 7  Read from EEPROM
// BASE ID 8  Skynode QS with USB - Alaised to ID 0

// BASE ID 9  Auterion Skynode base RC9 & older (no usb
static const px4_hw_mft_item_t base_configuration_9[] = {
	{
		.id          =  PX4_MFT_PX4IO,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_USB,
		.present     = 0,
		.mandatory   = 0,
		.connection  = px4_hw_con_unknown,
	},
	{
		.id          = PX4_MFT_CAN2,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_CAN3,
		.present     = 0,
		.mandatory   = 0,
		.connection  = px4_hw_con_unknown,
	},
	{
		.id          =  PX4_MFT_PM2,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_ETHERNET,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_connector,
	},
	{
		.id          = PX4_MFT_T100_ETH,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
};

// BASE ID 10  Skynode QS no USB Alaised to ID 9
// BASE ID 16  Auterion Skynode RC10, RC11, RC12, RC13 Alaised to ID 0

// BASE ID 17  Auterion Skynode RC13 with many parts removed
static const px4_hw_mft_item_t base_configuration_17[] = {
	{
		.id          = PX4_MFT_PX4IO,
		.present     = 0,
		.mandatory   = 0,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_USB,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_unknown,
	},
	{
		.id          = PX4_MFT_CAN2,
		.present     = 0,
		.mandatory   = 0,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_CAN3,
		.present     = 0,
		.mandatory   = 0,
		.connection  = px4_hw_con_unknown,
	},
	{
		.id          = PX4_MFT_PM2,
		.present     = 0,
		.mandatory   = 0,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_ETHERNET,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_connector,
	},
	{
		.id          = PX4_MFT_T100_ETH,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
};

// BASE ID 18  Auterion Skynode S
static const px4_hw_mft_item_t base_configuration_18[] = {
	{
		.id          = PX4_MFT_PX4IO,
		.present     = 0,
		.mandatory   = 0,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_USB,
		.present     = 0,
		.mandatory   = 0,
		.connection  = px4_hw_con_unknown,
	},
	{
		.id          = PX4_MFT_CAN2,
		.present     = 0,
		.mandatory   = 0,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_CAN3,
		.present     = 0,
		.mandatory   = 0,
		.connection  = px4_hw_con_unknown,
	},
	{
		.id          = PX4_MFT_PM2,
		.present     = 0,
		.mandatory   = 0,
		.connection  = px4_hw_con_onboard,
	},
	{
		.id          = PX4_MFT_ETHERNET,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_connector,
	},
	{
		.id          = PX4_MFT_T100_ETH,
		.present     = 1,
		.mandatory   = 1,
		.connection  = px4_hw_con_onboard,
	},
};

// BASE ID 0x100 Holybro Pixhawk Jetson Baseboard Alaised to ID 0
// BASE ID 0x150 ZeroOne Pixhawk Baseboard Alaised to ID 0

static px4_hw_mft_list_entry_t mft_lists[] = {
//  ver_rev
	{HW_BASE_ID(0),      base_configuration_0, arraySize(base_configuration_0)},   // std Base with PX4IO
	{HW_BASE_ID(1),      base_configuration_1, arraySize(base_configuration_1)},   // std Base No PX4IO
	{HW_BASE_ID(2),      base_configuration_0, arraySize(base_configuration_0)},   // CUAV Base
	{HW_BASE_ID(3),      base_configuration_3, arraySize(base_configuration_3)},   // NXP T1 PHY
	{HW_BASE_ID(4),      base_configuration_0, arraySize(base_configuration_0)},   // HB CM4 base
	{HW_BASE_ID(5),      base_configuration_5, arraySize(base_configuration_5)},   // HB Mini
	{HW_BASE_ID(8),      base_configuration_0, arraySize(base_configuration_0)},   // Auterion Skynode ver 8 Aliased to 0
	{HW_BASE_ID(9),      base_configuration_9, arraySize(base_configuration_9)},   // Auterion Skynode ver 9
	{HW_BASE_ID(10),     base_configuration_9, arraySize(base_configuration_9)},   // Auterion Skynode ver 10
	{HW_BASE_ID(16),     base_configuration_0, arraySize(base_configuration_0)},   // Auterion Skynode ver 16
	{HW_BASE_ID(17),     base_configuration_17, arraySize(base_configuration_17)}, // Auterion Skynode ver 17
	{HW_BASE_ID(18),     base_configuration_18, arraySize(base_configuration_18)}, // Auterion Skynode S ver 18
	{HW_BASE_ID(0x100),  base_configuration_0, arraySize(base_configuration_0)},   // Holybro Pixhawk Jetson Baseboard ver 0x100 Alaised to ID 0
	{HW_BASE_ID(0x150),  base_configuration_0, arraySize(base_configuration_0)},   // ZeroOne Pixhawk Baseboard ver 0x150
};

/************************************************************************************
 * Name: base_query_manifest
 *
 * Description:
 *   Optional returns manifest item.
 *
 * Input Parameters:
 *   manifest_id - the ID for the manifest item to retrieve
 *
 * Returned Value:
 *   0 - item is not in manifest => assume legacy operations
 *   pointer to a manifest item
 *
 ************************************************************************************/

__EXPORT px4_hw_mft_item board_query_manifest(px4_hw_mft_item_id_t id)
{
	static px4_hw_mft_list_entry boards_manifest = px4_hw_mft_list_uninitialized;

	if (boards_manifest == px4_hw_mft_list_uninitialized) {
		hw_base_id_t hw_base_id = GET_HW_BASE_ID();

		for (unsigned i = 0; i < arraySize(mft_lists); i++) {
			if (mft_lists[i].hw_base_id == hw_base_id) {
				boards_manifest = &mft_lists[i];
				break;
			}
		}

		if (boards_manifest == px4_hw_mft_list_uninitialized) {
			syslog(LOG_ERR, "[boot] Board %04" PRIx16 " is not supported!\n", hw_base_id);
		}
	}

	px4_hw_mft_item rv = &device_unsupported;

	if (boards_manifest != px4_hw_mft_list_uninitialized)
		for (unsigned int ndx = 0; ndx < boards_manifest->entries; ndx++) {
			if (boards_manifest->mft[ndx].id == id) {
				rv = &boards_manifest->mft[id];
				break;
			}
		}

	return rv;
}

__EXPORT int system_query_manifest(const char *sub,  const char *val, void *out)
{
	static const char *keys[] = PX4_MFT_MFT_STR_TYPES;
	static const px4_hw_mft_item_id_t item_ids[] = PX4_MFT_MFT_TYPES;
	px4_hw_mft_item rv = &device_unsupported;
	int id = -1;
	int intval = atoi(val);

	for (unsigned int k = 0; k < arraySize(keys); k++) {
		if (!strcmp(keys[k], sub)) {
			id = item_ids[k];
			break;
		}
	}

	if (id != -1) {
		// In case we have to filter when a FMUM is mounted to a BASE
		// For now just use the board
		rv = board_query_manifest(id);
		return rv->present == intval ? OK : -ENXIO;
	}

	return -ENOENT;
}
#endif
