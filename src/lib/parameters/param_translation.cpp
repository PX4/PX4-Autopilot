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


#include <inttypes.h>
#include <px4_platform_common/log.h>
#include <lib/drivers/device/Device.hpp>
#include <drivers/drv_sensor.h>
#include <lib/parameters/param.h>
#include <lib/mathlib/mathlib.h>
#include <uORB/topics/vtx.h>

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

	// 2025-03-19: translate ASPD_FALLBACK_GW to ASPD_FALLBACK
	{
		if (strcmp("ASPD_FALLBACK_GW", node->name) == 0) {
			strcpy(node->name, "ASPD_FALLBACK");
			PX4_INFO("copying %s -> %s", "ASPD_FALLBACK_GW", "ASPD_FALLBACK");
		}
	}

	// 2025-08-22: translate SDLOG_MODE (disabled) to SDLOG_BACKEND (no logging backend)
	{
		if (strcmp("SDLOG_MODE", node->name) == 0) {
			if (node->i32 == -1) {
				node->i32 = 0;

				int32_t sdlog_backend_val = 0;
				param_set(param_find("SDLOG_BACKEND"), &sdlog_backend_val);
				PX4_INFO("migrating %s -> %s", "SDLOG_MODE", "SDLOG_BACKEND");
			}
		}
	}

	// 2025-11-17: translate MNT_RANGE_PITCH to MNT_MAX_PITCH, MNT_MIN_PITCH
	{
		if (strcmp("MNT_RANGE_PITCH", node->name) == 0) {
			if (node->d > DBL_EPSILON) {
				float mnt_max_pitch = static_cast<float>(node->d) * 0.5f;
				float mnt_min_pitch = static_cast<float>(-node->d) * 0.5f;
				param_set(param_find("MNT_MAX_PITCH"), &mnt_max_pitch);
				param_set(param_find("MNT_MIN_PITCH"), &mnt_min_pitch);
				PX4_INFO("migrating %s -> %s, %s", "MNT_RANGE_PITCH", "MNT_MAX_PITCH", "MNT_MIN_PITCH");
			}

		}
	}

	// 2026-03-11: translate EKF2_GPS_POS_X/Y/Z -> SENS_GPS0_OFFX/OFFY/OFFZ
	{
		if (strcmp("EKF2_GPS_POS_X", node->name) == 0) {
			strcpy(node->name, "SENS_GPS0_OFFX");
			PX4_INFO("migrating %s -> %s", "EKF2_GPS_POS_X", "SENS_GPS0_OFFX");
			return param_modify_on_import_ret::PARAM_MODIFIED;
		}

		if (strcmp("EKF2_GPS_POS_Y", node->name) == 0) {
			strcpy(node->name, "SENS_GPS0_OFFY");
			PX4_INFO("migrating %s -> %s", "EKF2_GPS_POS_Y", "SENS_GPS0_OFFY");
			return param_modify_on_import_ret::PARAM_MODIFIED;
		}

		if (strcmp("EKF2_GPS_POS_Z", node->name) == 0) {
			strcpy(node->name, "SENS_GPS0_OFFZ");
			PX4_INFO("migrating %s -> %s", "EKF2_GPS_POS_Z", "SENS_GPS0_OFFZ");
			return param_modify_on_import_ret::PARAM_MODIFIED;
		}
	}

	// 2026-03-11: translate EKF2_GPS_DELAY to SENS_GPS0_DELAY and SENS_GPS1_DELAY
	{
		if (strcmp("EKF2_GPS_DELAY", node->name) == 0) {
			int32_t delay_ms = static_cast<int32_t>(node->d);
			param_set(param_find("SENS_GPS0_DELAY"), &delay_ms);
			param_set(param_find("SENS_GPS1_DELAY"), &delay_ms);
			PX4_INFO("migrating %s -> %s, %s", "EKF2_GPS_DELAY", "SENS_GPS0_DELAY", "SENS_GPS1_DELAY");
		}
	}

	// 2026-03-11: translate MOT_POLE_COUNT to per-motor DSHOT_MOT_POL1-12
	{
		if ((node->type == bson_type_t::BSON_INT32) && (strcmp("MOT_POLE_COUNT", node->name) == 0)) {
			for (int i = 1; i <= 12; i++) {
				char name[20];
				snprintf(name, sizeof(name), "DSHOT_MOT_POL%d", i);
				param_set(param_find(name), &node->i32);
			}

			PX4_INFO("migrating %s -> DSHOT_MOT_POL1-12 (value=%" PRId32 ")", "MOT_POLE_COUNT", node->i32);
			return param_modify_on_import_ret::PARAM_SKIP_IMPORT;
		}
	}

	// 2026-03-19: translate EKF2_ENGINE_WRM to EKF2_POS_LOCK
	{
		if (strcmp("EKF2_ENGINE_WRM", node->name) == 0) {
			int32_t delay_ms = static_cast<int32_t>(node->d);
			param_set(param_find("EKF2_POS_LOCK"), &delay_ms);
			PX4_INFO("migrating %s -> %s", "EKF2_ENGINE_WRM", "EKF2_POS_LOCK");
			return param_modify_on_import_ret::PARAM_SKIP_IMPORT;
		}
	}

	// 2026-03-19: translate RC*_REV from float to int32
	{
		if ((node->type == bson_type_t::BSON_DOUBLE) && (strncmp("RC", node->name, 2) == 0)
		    && strstr(node->name, "_REV") != nullptr) {
			node->i32 = (node->d < 0.0) ? -1 : 1;
			node->type = bson_type_t::BSON_INT32;
			PX4_INFO("migrating %s from float to int32", node->name);
			return param_modify_on_import_ret::PARAM_MODIFIED;
		}
	}

	// 2026-06-12: merge COM_RC_OVERRIDE + COM_RC_STICK_OV into MAN_OVERRIDE_SPD
	{
		if ((node->type == bson_type_t::BSON_INT32) && (strcmp("COM_RC_OVERRIDE", node->name) == 0) && (node->i32 == 0)) {
			node->d = -1.0;
			node->type = bson_type_t::BSON_DOUBLE;
			strcpy(node->name, "MAN_OVERRIDE_SPD");
			PX4_INFO("migrating %s -> %s (disabled)", "COM_RC_OVERRIDE", "MAN_OVERRIDE_SPD");
			return param_modify_on_import_ret::PARAM_MODIFIED;
		}
	}

	// 2026-06-22: translate HEATER*_IMU_ID to HEATER*_SENS_ID
	//	HEATER_MAX_INSTANCES == 3 (in heater.h)
	{
		if ((node->type == bson_type_t::BSON_INT32) && (strncmp("HEATER", node->name, 6) == 0)
		    && strstr(node->name, "_IMU_ID") != nullptr) {
			char old_name[BSON_MAXNAME];
			strncpy(old_name, node->name, sizeof(old_name));
			char new_name[16];
			snprintf(new_name, sizeof(new_name), "HEATER%c_SENS_ID", node->name[6]);
			strcpy(node->name, new_name);
			PX4_INFO("migrating %s -> %s", old_name, new_name);
			return param_modify_on_import_ret::PARAM_MODIFIED;
		}
	}

	// 2026-06-29: MPC_LAND_RC_HELP replaced by the MPC_AUTO_NUDGING bitmask (landing nudging = bit 1).
	// The old in-Hold yaw nudge is not migrated; it now lives in bit 0 (yaw nudging in all auto modes).
	{
		if ((node->type == bson_type_t::BSON_INT32) && (strcmp("MPC_LAND_RC_HELP", node->name) == 0)) {
			if (node->i32 != 0) {
				int32_t nudging = 0;
				param_get(param_find("MPC_AUTO_NUDGING"), &nudging);
				nudging |= (1 << 1);
				param_set(param_find("MPC_AUTO_NUDGING"), &nudging);
			}

			PX4_INFO("migrating MPC_LAND_RC_HELP -> MPC_AUTO_NUDGING bit 1 (value=%" PRId32 ")", node->i32);
			return param_modify_on_import_ret::PARAM_SKIP_IMPORT;
		}
	}

	// 2026-07-13: translate COM_ARM_TRAFF (arming check only) to COM_TRAFF_AVOID (arming check + failsafe action)
	{
		if ((node->type == bson_type_t::BSON_INT32) && (strcmp("COM_ARM_TRAFF", node->name) == 0)) {
			// old: 0 Disabled, 1 Warning only (arming allowed), 2 Enforce all modes, 3 Enforce mission only
			// new: 0 Disabled, 1 Warning (arming allowed), 2 Error (arming blocked)
			// Old value 3 (mission-only enforcement) is intentionally mapped to 2 (all modes): mode-scoped
			// arming enforcement is no longer supported, so we err on the restrictive side.
			// COM_ARM_TRAFF never triggered an in-flight failsafe action, so the new failsafe
			// action defaults to Warning either way; only the arming behavior is preserved.
			if (node->i32 == 1) {
				node->i32 = 1;

			} else if (node->i32 >= 2) {
				node->i32 = 2;
			}

			strcpy(node->name, "COM_TRAFF_AVOID");
			PX4_INFO("migrating %s -> %s", "COM_ARM_TRAFF", "COM_TRAFF_AVOID");
			return param_modify_on_import_ret::PARAM_MODIFIED;
		}
	}

	// 2026-08-05: the protocol selection moved out of VTX_DEVICE into VTX_PROTOCOL. VTX_DEVICE keeps
	// the old layout that holds the device in its high byte, so only the protocol has to be derived:
	// the Peak THOR T67 speaks SmartAudio, the Rush MAX SOLO speaks Tramp. A value that is not listed
	// stays untouched and acts as a generic device on SmartAudio, which is what both parameters
	// default to, so the old value 0 needs nothing.
	{
		static constexpr int32_t PROTOCOL_SMART_AUDIO = 0; // VTX_PROTOCOL value, not in msg/Vtx.msg

		if ((node->type == bson_type_t::BSON_INT32) && (strcmp("VTX_DEVICE", node->name) == 0)) {
			int32_t device = -1;
			int32_t protocol = PROTOCOL_SMART_AUDIO;

			switch (node->i32) {
			case 100: // generic device, Tramp
				device = vtx_s::DEVICE_UNKNOWN << 8;
				protocol = vtx_s::PROTOCOL_TRAMP;
				break;

			case 5120: // Peak THOR T67, SmartAudio only
				device = vtx_s::DEVICE_PEAK_THOR_T67 << 8;
				protocol = PROTOCOL_SMART_AUDIO;
				break;

			case 10240: // Rush MAX SOLO, Tramp only
				device = vtx_s::DEVICE_RUSH_MAX_SOLO << 8;
				protocol = vtx_s::PROTOCOL_TRAMP;
				break;
			}

			if (device >= 0) {
				node->i32 = device;
				param_set(param_find("VTX_PROTOCOL"), &protocol);
				PX4_INFO("migrating VTX_DEVICE -> VTX_DEVICE %" PRId32 " + VTX_PROTOCOL %" PRId32,
					 device, protocol);
				return param_modify_on_import_ret::PARAM_MODIFIED;
			}
		}
	}

	// 2026-08-18: UAVCAN_ECU_MAXF replaced by per-tank (idx 1 based)
	{
		if ((node->type == bson_type_t::BSON_DOUBLE) && (strcmp("UAVCAN_ECU_MAXF", node->name) == 0)) {
			strcpy(node->name, "UAVCAN_ECU_MAXF1");
			PX4_INFO("copying %s -> %s", "UAVCAN_ECU_MAXF", "UAVCAN_ECU_MAXF1");
			return param_modify_on_import_ret::PARAM_MODIFIED;
		}
	}

	// 2026-08-31: invert serial mapping (*_CONFIG port index) to SER_<tag>_PROT
	if (node->type == bson_type_t::BSON_INT32) {
		if (strcmp("RC_INPUT_PROTO", node->name) == 0) {
			return param_modify_on_import_ret::PARAM_SKIP_IMPORT;
		}

		struct PortTag {
			int32_t index;
			const char *prot_name;
		};

		static constexpr PortTag kPorts[] = {
			{6, "SER_URT6_PROT"},
			{101, "SER_TEL1_PROT"},
			{102, "SER_TEL2_PROT"},
			{103, "SER_TEL3_PROT"},
			{104, "SER_TEL4_PROT"},
			{201, "SER_GPS1_PROT"},
			{202, "SER_GPS2_PROT"},
			{203, "SER_GPS3_PROT"},
			{300, "SER_RC_PROT"},
			{301, "SER_WIFI_PROT"},
			{401, "SER_EXT2_PROT"},
		};

		struct OldConfig {
			const char *name;
			int32_t protocol_id;
			int32_t default_port;
			const char *ethernet_param;
		};

		static constexpr OldConfig kOld[] = {
			{"MAV_0_CONFIG", 1, 101, "MAV_ETH_EN"},
			{"MAV_1_CONFIG", 1, 0, "MAV_ETH_EN"},
			{"MAV_2_CONFIG", 1, 0, "MAV_ETH_EN"},
			{"GPS_1_CONFIG", 5, 201, nullptr},
			{"GPS_2_CONFIG", 5, 0, nullptr},
			{"SEP_PORT1_CFG", 6, 0, nullptr},
			{"SEP_PORT2_CFG", 6, 0, nullptr},
			{"RC_SBUS_PRT_CFG", 10, 0, nullptr},
			{"RC_DSM_PRT_CFG", 11, 0, nullptr},
			{"RC_CRSF_PRT_CFG", 12, 0, nullptr},
			{"RC_GHST_PRT_CFG", 13, 0, nullptr},
			{"RC_PORT_CONFIG", 10, 300, nullptr},
			{"UXRCE_DDS_CFG", 20, 0, "UXRCE_DDS_ETH"},
			{"TEL_FRSKY_CONFIG", 21, 0, nullptr},
			{"TEL_HOTT_CONFIG", 22, 0, nullptr},
			{"ISBD_CONFIG", 23, 0, nullptr},
			{"MSP_OSD_CONFIG", 24, 0, nullptr},
			{"VTX_SER_CFG", 25, 0, nullptr},
			{"DSHOT_TEL_CFG", 26, 0, nullptr},
			{"SENS_TFMINI_CFG", 30, 0, nullptr},
			{"SENS_SF0X_CFG", 31, 0, nullptr},
			{"SENS_EN_SF45_CFG", 32, 0, nullptr},
			{"SENS_LEDDAR1_CFG", 33, 0, nullptr},
			{"SENS_CM8JL65_CFG", 34, 0, nullptr},
			{"SENS_ULAND_CFG", 35, 0, nullptr},
			{"SENS_EN_GRF_CFG", 36, 0, nullptr},
			{"SENS_ASDT1_CFG", 37, 0, nullptr},
			{"SENS_VN_CFG", 40, 0, nullptr},
			{"SENS_MS_CFG", 41, 0, nullptr},
			{"SENS_ILABS_CFG", 42, 0, nullptr},
			{"SENS_SBG_CFG", 43, 0, nullptr},
			{"SENS_BAHRS_CFG", 44, 0, nullptr},
			{"UWB_PORT_CFG", 50, 0, nullptr},
			{"SENS_FTX_CFG", 51, 0, nullptr},
			{"RBCLW_SER_CFG", 52, 0, nullptr},
			{"VERTIQ_IO_CFG", 53, 0, nullptr},
			{"SENS_TFLOW_CFG", 54, 0, nullptr},
			{"MXS_SER_CFG", 55, 0, nullptr},
		};

		static uint32_t claimed_ports;

		auto claim = [&](int32_t index) {
			for (unsigned i = 0; i < sizeof(kPorts) / sizeof(kPorts[0]); i++) {
				if (kPorts[i].index == index) {
					if (claimed_ports & (1u << i)) {
						return false;
					}

					claimed_ports |= (1u << i);
					return true;
				}
			}

			return false;
		};

		auto prot_name = [&](int32_t index) -> const char* {
			for (const auto &p : kPorts)
			{
				if (p.index == index) {
					return p.prot_name;
				}
			}

			return nullptr;
		};

		for (const auto &old : kOld) {
			if (strcmp(old.name, node->name) != 0) {
				continue;
			}

			const int32_t value = node->i32;

			if (value == 1000 && old.ethernet_param) {
				if (old.default_port != 0) {
					int32_t z = 0;
					param_set(param_find(prot_name(old.default_port)), &z);
					claim(old.default_port);
				}

				strcpy(node->name, old.ethernet_param);
				node->i32 = 1;
				PX4_INFO("migrating %s -> %s", old.name, old.ethernet_param);
				return param_modify_on_import_ret::PARAM_MODIFIED;
			}

			if (value == 0) {
				if (old.default_port != 0 && claim(old.default_port)) {
					strcpy(node->name, prot_name(old.default_port));
					PX4_INFO("migrating %s -> %s (disabled)", old.name, node->name);
					return param_modify_on_import_ret::PARAM_MODIFIED;
				}

				return param_modify_on_import_ret::PARAM_SKIP_IMPORT;
			}

			const char *dest = prot_name(value);

			if (dest == nullptr) {
				return param_modify_on_import_ret::PARAM_SKIP_IMPORT;
			}

			if (old.default_port != 0 && value != old.default_port) {
				int32_t z = 0;
				param_set(param_find(prot_name(old.default_port)), &z);
				claim(old.default_port);
			}

			if (!claim(value)) {
				PX4_WARN("dropping %s, %s already assigned", old.name, dest);
				return param_modify_on_import_ret::PARAM_SKIP_IMPORT;
			}

			strcpy(node->name, dest);
			node->i32 = old.protocol_id;
			PX4_INFO("migrating %s -> %s=%" PRId32, old.name, dest, old.protocol_id);
			return param_modify_on_import_ret::PARAM_MODIFIED;
		}
	}

	return param_modify_on_import_ret::PARAM_NOT_MODIFIED;
}
