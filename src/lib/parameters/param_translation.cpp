/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#include "param_translation.h"


#include <px4_platform_common/log.h>
#include <lib/drivers/device/Device.hpp>
#include <drivers/drv_sensor.h>
#include <lib/parameters/param.h>
#include <lib/mathlib/mathlib.h>

param_modify_on_import_ret param_modify_on_import(bson_node_t node)
{
	// 2023-12-06: translate and invert FW_ARSP_MODE-> FW_USE_AIRSPD
	{
		if (strcmp("FW_ARSP_MODE", node->name) == 0) {
			if (node->i32 == 0) {
				node->i32 = 1;

			} else {
				node->i32 = 0;
			}

			strcpy(node->name, "FW_USE_AIRSPD");
			PX4_INFO("copying and inverting %s -> %s", "FW_ARSP_MODE", "FW_USE_AIRSPD");
			return param_modify_on_import_ret::PARAM_MODIFIED;
		}
	}

	// 2023-12-06: translate CBRK_AIRSPD_CHK-> SYS_HAS_NUM_ASPD
	{
		if (strcmp("CBRK_AIRSPD_CHK", node->name) == 0) {
			if (node->i32 == 162128) {
				node->i32 = 0;

				strcpy(node->name, "SYS_HAS_NUM_ASPD");
				PX4_INFO("copying %s -> %s", "CBRK_AIRSPD_CHK", "SYS_HAS_NUM_ASPD");

			}

			return param_modify_on_import_ret::PARAM_MODIFIED;
		}
	}

	// 2024-04-15 SYS_MC_EST_GROUP removed
	if ((node->type == bson_type_t::BSON_INT32) && (strcmp("SYS_MC_EST_GROUP", node->name) == 0)) {

		int32_t value = node->i32;

		// value 1 local_position_estimator, attitude_estimator_q (unsupported)
		if (value == 1) {
			// enable local_position_estimator
			int32_t lpe_en_val = 1;
			int lpe_en_ret = param_set(param_find("LPE_EN"), &lpe_en_val);

			// enable attitude_estimator_q
			int32_t att_en_val = 1;
			int att_en_ret = param_set(param_find("ATT_EN"), &att_en_val);

			// disable ekf2 (only if enabling lpe and att_w was successful)
			if (lpe_en_ret == PX4_OK && att_en_ret == PX4_OK) {
				int32_t ekf2_en_val = 0;
				param_set(param_find("EKF2_EN"), &ekf2_en_val);

			} else {
				int32_t ekf2_en_val = 1;
				param_set(param_find("EKF2_EN"), &ekf2_en_val);
			}

			return param_modify_on_import_ret::PARAM_MODIFIED;
		}

		// value 2 ekf2 (recommended)
		if (value == 2) {
			// disable local_position_estimator
			int32_t lpe_en_val = 0;
			param_set(param_find("LPE_EN"), &lpe_en_val);

			// disable attitude_estimator_q
			int32_t att_en_val = 0;
			param_set(param_find("ATT_EN"), &att_en_val);

			// enable ekf2
			int32_t ekf2_en_val = 1;
			param_set(param_find("EKF2_EN"), &ekf2_en_val);

			return param_modify_on_import_ret::PARAM_MODIFIED;
		}

		// value 3 Q attitude estimator (no position)
		if (value == 3) {
			// disable local_position_estimator
			int32_t lpe_en_val = 0;
			param_set(param_find("LPE_EN"), &lpe_en_val);

			// enable attitude_estimator_q
			int32_t att_en_val = 1;
			int att_en_ret = param_set(param_find("ATT_EN"), &att_en_val);

			// disable ekf2 (only if enabling att_w was successful)
			if (att_en_ret == PX4_OK) {
				int32_t ekf2_en_val = 0;
				param_set(param_find("EKF2_EN"), &ekf2_en_val);

			} else {
				int32_t ekf2_en_val = 1;
				param_set(param_find("EKF2_EN"), &ekf2_en_val);
			}

			return param_modify_on_import_ret::PARAM_MODIFIED;
		}
	}

	return param_modify_on_import_ret::PARAM_NOT_MODIFIED;
}
