/****************************************************************************
 *
 *   Copyright (c) 2020, 2021 PX4 Development Team. All rights reserved.
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
	// migrate COM_ARM_AUTH -> COM_ARM_AUTH_ID, COM_ARM_AUTH_MET and COM_ARM_AUTH_TO (2020-11-06). This can be removed after the next release (current release=1.11)
	if (node->type == BSON_INT32) {
		if (strcmp("COM_ARM_AUTH", node->name) == 0) {
			union {
				struct {
					uint8_t authorizer_system_id;
					uint16_t auth_method_arm_timeout_msec;
					uint8_t authentication_method;
				} __attribute__((packed)) struct_value;
				int32_t param_value;
			} old_param;
			old_param.param_value = node->i32;

			int32_t method = old_param.struct_value.authentication_method;
			param_set_no_notification(param_find("COM_ARM_AUTH_MET"), &method);

			float timeout = old_param.struct_value.auth_method_arm_timeout_msec / 1000.f;
			param_set_no_notification(param_find("COM_ARM_AUTH_TO"), &timeout);

			strcpy(node->name, "COM_ARM_AUTH_ID");
			node->i32 = old_param.struct_value.authorizer_system_id;

			PX4_INFO("migrating COM_ARM_AUTH: %" PRId32 " -> COM_ARM_AUTH_ID:%" PRId8 ", COM_ARM_AUTH_MET: %" PRId32
				 " and COM_ARM_AUTH_TO: %f",
				 old_param.param_value,
				 old_param.struct_value.authorizer_system_id,
				 method,
				 (double)timeout);
		}
	}

	// 2021-08-27: translate LED_RGB_MAXBRT (0-15) to SYS_RGB_MAXBRT(0.f-1.f)
	if (node->type == BSON_INT32) {
		if (strcmp("LED_RGB_MAXBRT", node->name) == 0) {
			// convert integer (0-15) to float percentage
			node->d = math::constrain(static_cast<double>(node->i32) / 15., 0., 1.);
			node->type = BSON_DOUBLE;
			strcpy(node->name, "SYS_RGB_MAXBRT");
			PX4_INFO("param migrating LED_RGB_MAXBRT (removed) -> SYS_RGB_MAXBRT: value=%.3f", node->d);
			return true;
		}
	}

	// 2020-08-23 (v1.12 alpha): translate GPS blending parameters from EKF2 -> SENS
	{
		if (strcmp("EKF2_GPS_MASK", node->name) == 0) {
			strcpy(node->name, "SENS_GPS_MASK");
			PX4_INFO("copying %s -> %s", "EKF2_GPS_MASK", "SENS_GPS_MASK");
			return true;
		}

		if (strcmp("EKF2_GPS_TAU", node->name) == 0) {
			strcpy(node->name, "SENS_GPS_TAU");
			PX4_INFO("copying %s -> %s", "EKF2_GPS_TAU", "SENS_GPS_TAU");
			return true;
		}
	}

	// 2021-01-31 (v1.12 alpha): translate PWM_MIN/PWM_MAX/PWM_DISARMED to PWM_MAIN
	{
		if (strcmp("PWM_MIN", node->name) == 0) {
			strcpy(node->name, "PWM_MAIN_MIN");
			PX4_INFO("copying %s -> %s", "PWM_MIN", "PWM_MAIN_MIN");
			return true;
		}

		if (strcmp("PWM_MAX", node->name) == 0) {
			strcpy(node->name, "PWM_MAIN_MAX");
			PX4_INFO("copying %s -> %s", "PWM_MAX", "PWM_MAIN_MAX");
			return true;
		}

		if (strcmp("PWM_RATE", node->name) == 0) {
			strcpy(node->name, "PWM_MAIN_RATE");
			PX4_INFO("copying %s -> %s", "PWM_RATE", "PWM_MAIN_RATE");
			return true;
		}

		if (strcmp("PWM_DISARMED", node->name) == 0) {
			strcpy(node->name, "PWM_MAIN_DISARM");
			PX4_INFO("copying %s -> %s", "PWM_DISARMED", "PWM_MAIN_DISARM");
			return true;
		}
	}

	// 2021-04-30: translate ASPD_STALL to FW_AIRSPD_STALL
	{
		if (strcmp("ASPD_STALL", node->name) == 0) {
			strcpy(node->name, "FW_AIRSPD_STALL");
			PX4_INFO("copying %s -> %s", "ASPD_STALL", "FW_AIRSPD_STALL");
			return true;
		}
	}

	// 2021-07-12: translate VT_DWN_PITCH_MAX to VT_PITCH_MIN
	{
		if (strcmp("VT_DWN_PITCH_MAX", node->name) == 0) {
			strcpy(node->name, "VT_PITCH_MIN");
			node->d *= -1;
			PX4_INFO("copying and inverting sign %s -> %s", "VT_DWN_PITCH_MAX", "VT_PITCH_MIN");
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

	return false;
}
