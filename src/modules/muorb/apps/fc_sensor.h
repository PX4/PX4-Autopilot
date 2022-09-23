/****************************************************************************
 *
 * Copyright (C) 2022 ModalAI, Inc. All rights reserved.
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

#ifndef FC_SENSOR_H
#define FC_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef void (*fc_receive_cb)(const char *topic,
			      const uint8_t *data,
			      uint32_t length_in_bytes);
typedef void (*fc_advertise_cb)(const char *topic);
typedef void (*fc_add_subscription_cb)(const char *topic);
typedef void (*fc_remove_subscription_cb)(const char *topic);

typedef struct {
	fc_receive_cb               rx_callback;
	fc_advertise_cb             ad_callback;
	fc_add_subscription_cb      sub_callback;
	fc_remove_subscription_cb   remove_sub_callback;
} fc_callbacks;

int fc_sensor_initialize(bool enable_debug_messages, fc_callbacks *callbacks);
int fc_sensor_advertise(const char *topic);
int fc_sensor_subscribe(const char *topic);
int fc_sensor_unsubscribe(const char *topic);
int fc_sensor_send_data(const char *topic,
			const uint8_t *data,
			uint32_t length_in_bytes);
#ifdef __cplusplus
}
#endif

#endif // FC_SENSOR_H
