/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file manifest.c
 *
 * This module supplies the interface to the manifest of hardware that is
 * optional and dependent on the HW REV and HW VER IDs
 *
 * The manifest allows the system to know whether a hardware option
 * say for example the PX4IO is an no-pop option vs it is broken.
 *
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <stdbool.h>
#include <syslog.h>

#include "systemlib/px4_macros.h"
#include "px4_log.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/


/************************************************************************************
 * Name: board_query_manifest
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
	return px4_hw_mft_unsupported;
}
