/****************************************************************************
 *
 *   Copyright (c) 2015-2021 PX4 Development Team. All rights reserved.
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

#include "camera_interface.h"
#include <px4_platform_common/log.h>
#include <board_config.h>

void CameraInterface::get_pins()
{
	// Set all pins as invalid
	for (unsigned i = 0; i < arraySize(_pins); i++) {
		_pins[i] = -1;
	}

	param_t p_ctrl_alloc = param_find("SYS_CTRL_ALLOC");
	bool ctrl_alloc = false;

	if (p_ctrl_alloc != PARAM_INVALID) {
		param_get(p_ctrl_alloc, &ctrl_alloc);
	}

	if (ctrl_alloc) {
		unsigned pin_index = 0;

		for (unsigned i = 0; i < 16 && pin_index < arraySize(_pins); ++i) {
			char param_name[17];
			snprintf(param_name, sizeof(param_name), "%s_%s%d", PARAM_PREFIX, "FUNC", i + 1);
			param_t function_handle = param_find(param_name);
			int32_t function;

			if (function_handle != PARAM_INVALID && param_get(function_handle, &function) == 0) {
				if (function == 2000) { // Camera_Trigger
					_pins[pin_index++] = i;
				}
			}
		}

	} else {
		// Get parameter handle
		param_t p_pin = param_find("TRIG_PINS");
		param_t p_pin_ex = param_find("TRIG_PINS_EX");

		if (p_pin == PARAM_INVALID && p_pin_ex == PARAM_INVALID) {
			PX4_ERR("param TRIG_PINS not found");
			return;
		}

		int32_t pin_list = 0;
		int32_t pin_list_ex = 0;

		if (p_pin_ex != PARAM_INVALID) {
			param_get(p_pin_ex, &pin_list_ex);
		}

		if (p_pin != PARAM_INVALID) {
			param_get(p_pin, &pin_list);
		}

		if (pin_list_ex == 0) {

			// Convert number to individual channels

			unsigned i = 0;
			int single_pin;

			while ((single_pin = pin_list % 10)) {

				_pins[i] = single_pin - 1;

				if (_pins[i] < 0) {
					_pins[i] = -1;
				}

				pin_list /= 10;
				i++;
			}

		} else {
			unsigned int p = 0;

			for (unsigned int i = 0; i < arraySize(_pins); i++) {
				int32_t v = (pin_list_ex & (1 << i)) ? i  : -1;

				if (v > 0) {
					_pins[p++] = v;
				}
			}
		}
	}
}
