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

bool param_modify_on_import(bson_node_t node)
{
	// 2022-04-11: translate VT_PTCH_MIN to VT_PITCH_MIN
	{
		if (strcmp("VT_PTCH_MIN", node->name) == 0) {
			strcpy(node->name, "VT_PITCH_MIN");
			PX4_INFO("copying %s -> %s", "VT_PTCH_MIN", "VT_PITCH_MIN");
			return true;
		}
	}

	// 2022-04-11: translate VT_LND_PTCH_MIN to VT_LND_PITCH_MIN
	{
		if (strcmp("VT_LND_PTCH_MIN", node->name) == 0) {
			strcpy(node->name, "VT_LND_PITCH_MIN");
			PX4_INFO("copying %s -> %s", "VT_LND_PTCH_MIN", "VT_LND_PITCH_MIN");
			return true;
		}
	}


	// 2021-10-21: translate NAV_GPSF_LT to FW_GPSF_LT and NAV_GPSF_R to FW_GPSF_R
	{
		if (strcmp("NAV_GPSF_LT", node->name) == 0) {
			strcpy(node->name, "FW_GPSF_LT");
			node->i32 = static_cast<int32_t>(node->d);
			node->type = BSON_INT32;
			PX4_INFO("copying %s -> %s", "NAV_GPSF_LT", "FW_GPSF_LT");
			return true;
		}

		if (strcmp("NAV_GPSF_R", node->name) == 0) {
			strcpy(node->name, "FW_GPSF_R");
			PX4_INFO("copying %s -> %s", "NAV_GPSF_R", "FW_GPSF_R");
			return true;
		}
	}

	// 2022-03-15: translate notch filter IMU_GYRO_NF_FREQ to IMU_GYRO_NF0_FRQ and IMU_GYRO_NF_BW -> IMU_GYRO_NF0_BW
	{
		if (strcmp("IMU_GYRO_NF_FREQ", node->name) == 0) {
			strcpy(node->name, "IMU_GYRO_NF0_FRQ");
			PX4_INFO("copying %s -> %s", "IMU_GYRO_NF_FREQ", "IMU_GYRO_NF0_FRQ");
			return true;
		}

		if (strcmp("IMU_GYRO_NF_BW", node->name) == 0) {
			strcpy(node->name, "IMU_GYRO_NF0_BW");
			PX4_INFO("copying %s -> %s", "IMU_GYRO_NF_BW", "IMU_GYRO_NF0_BW");
			return true;
		}
	}

	// 2022-04-25 (v1.13 alpha): translate MS4525->MS4525DO and MS5525->MS5525DSO
	{
		if (strcmp("SENS_EN_MS4525", node->name) == 0) {
			strcpy(node->name, "SENS_EN_MS4525DO");
			PX4_INFO("copying %s -> %s", "SENS_EN_MS4525", "SENS_EN_MS4525DO");
			return true;
		}

		if (strcmp("SENS_EN_MS5525", node->name) == 0) {
			strcpy(node->name, "SENS_EN_MS5525DS");
			PX4_INFO("copying %s -> %s", "SENS_EN_MS5525", "SENS_EN_MS5525DS");
			return true;
		}
	}

	// 2022-06-09: migrate EKF2_WIND_NOISE->EKF2_WIND_NSD
	{
		if (strcmp("EKF2_WIND_NOISE", node->name) == 0) {
			node->d /= 10.0; // at 100Hz (EKF2 rate), NSD is sqrt(100) times smaller than std_dev
			strcpy(node->name, "EKF2_WIND_NSD");
			PX4_INFO("param migrating EKF2_WIND_NOISE (removed) -> EKF2_WIND_NSD: value=%.3f", node->d);
			return true;
		}
	}

	// 2022-06-09: translate ASPD_SC_P_NOISE->ASPD_SCALE_NSD and ASPD_W_P_NOISE->ASPD_WIND_NSD
	{
		if (strcmp("ASPD_SC_P_NOISE", node->name) == 0) {
			strcpy(node->name, "ASPD_SCALE_NSD");
			PX4_INFO("copying %s -> %s", "ASPD_SC_P_NOISE", "ASPD_SCALE_NSD");
			return true;
		}

		if (strcmp("ASPD_W_P_NOISE", node->name) == 0) {
			strcpy(node->name, "ASPD_WIND_NSD");
			PX4_INFO("copying %s -> %s", "ASPD_W_P_NOISE", "ASPD_WIND_NSD");
			return true;
		}
	}

	// 2022-07-07: translate FW_THR_CRUISE->FW_THR_TRIM
	{
		if (strcmp("FW_THR_CRUISE", node->name) == 0) {
			strcpy(node->name, "FW_THR_TRIM");
			PX4_INFO("copying %s -> %s", "FW_THR_CRUISE", "FW_THR_TRIM");
			return true;
		}
	}

	// 2022-08-04: migrate EKF2_RNG_AID->EKF2_RNG_CTRL and EKF2_HGT_MODE->EKF2_HGT_REF
	{
		if (strcmp("EKF2_RNG_AID", node->name) == 0) {
			strcpy(node->name, "EKF2_RNG_CTRL");
			PX4_INFO("param migrating EKF2_RNG_AID (removed) -> EKF2_RNG_CTRL: value=%" PRId32, node->i32);
			return true;
		}

		if (strcmp("EKF2_HGT_MODE", node->name) == 0) {
			strcpy(node->name, "EKF2_HGT_REF");

			// If was in range height mode, set range aiding to "always"
			if (node->i32 == 2) {
				int32_t rng_mode = 2;
				param_set_no_notification(param_find("EKF2_RNG_CTRL"), &rng_mode);
			}

			PX4_INFO("param migrating EKF2_HGT_MODE (removed) -> EKF2_HGT_REF: value=%" PRId32, node->i32);
			return true;
		}
	}

	// 2022-07-18: translate VT_FW_DIFTHR_SC->VT_FW_DIFTHR_S_Y
	{
		if (strcmp("VT_FW_DIFTHR_SC", node->name) == 0) {
			strcpy(node->name, "VT_FW_DIFTHR_S_Y");
			PX4_INFO("copying %s -> %s", "VT_FW_DIFTHR_SC", "VT_FW_DIFTHR_S_Y");
			return true;
		}
	}

	// 2022-11-11: translate VT_F_TRANS_THR/VT_PSHER_RMP_DT -> VT_PSHER_SLEW
	{
		if (strcmp("VT_PSHER_RMP_DT", node->name) == 0) {
			strcpy(node->name, "VT_PSHER_SLEW");
			double _param_vt_f_trans_thr = param_find("VT_F_TRANS_THR");
			node->d = _param_vt_f_trans_thr / node->d;
			PX4_INFO("copying %s -> %s", "VT_PSHER_RMP_DT", "VT_PSHER_SLEW");
		}
	}

	// 2022-11-09: translate several fixed-wing launch parameters
	{
		if (strcmp("LAUN_ALL_ON", node->name) == 0) {
			strcpy(node->name, "FW_LAUN_DETCN_ON");
			PX4_INFO("copying %s -> %s", "LAUN_ALL_ON", "FW_LAUN_DETCN_ON");
			return true;
		}

		if (strcmp("LAUN_CAT_A", node->name) == 0) {
			strcpy(node->name, "FW_LAUN_AC_THLD");
			PX4_INFO("copying %s -> %s", "LAUN_CAT_A", "FW_LAUN_AC_THLD");
			return true;
		}

		if (strcmp("LAUN_CAT_T", node->name) == 0) {
			strcpy(node->name, "FW_LAUN_AC_T");
			PX4_INFO("copying %s -> %s", "LAUN_CAT_T", "FW_LAUN_AC_T");
			return true;
		}

		if (strcmp("LAUN_CAT_MDEL", node->name) == 0) {
			strcpy(node->name, "FW_LAUN_MOT_DEL");
			PX4_INFO("copying %s -> %s", "LAUN_CAT_MDEL", "FW_LAUN_MOT_DEL");
			return true;
		}
	}

	return false;

	//2023-02-08: translate L1 parameters after removing l1 control
	{
		if (strcmp("RWTO_L1_PERIOD", node->name) == 0) {
			strcpy(node->name, "RWTO_NPFG_PERIOD");
			PX4_INFO("copying %s -> %s", "RWTO_L1_PERIOD", "RWTO_NPFG_PERIOD");
			return true;
		}

		if (strcmp("FW_L1_R_SLEW_MAX", node->name) == 0) {
			strcpy(node->name, "FW_PN_R_SLEW_MAX");
			PX4_INFO("copying %s -> %s", "FW_L1_R_SLEW_MAX", "FW_PN_R_SLEW_MAX");
			return true;
		}

		if (strcmp("FW_L1_PERIOD", node->name) == 0) {
			strcpy(node->name, "NPFG_PERIOD");
			PX4_INFO("copying %s -> %s", "FW_L1_PERIOD", "NPFG_PERIOD");
			return true;
		}
	}
}
