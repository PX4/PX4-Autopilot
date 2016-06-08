/****************************************************************************
 *
 * Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
#pragma once

#include <systemlib/visibility.h>
#include <stdint.h>

extern "C" {

	int px4muorb_orb_initialize() __EXPORT;

	int px4muorb_set_absolute_time_offset(int32_t time_diff_us) __EXPORT;

	int px4muorb_get_absolute_time(uint64_t *time_us) __EXPORT;

	int px4muorb_add_subscriber(const char *name) __EXPORT;

	int px4muorb_remove_subscriber(const char *name) __EXPORT;

	int px4muorb_send_topic_data(const char *name, const uint8_t *data, int data_len_in_bytes) __EXPORT;

	int px4muorb_is_subscriber_present(const char *topic_name, int *status) __EXPORT;

	int px4muorb_receive_msg(int *msg_type, char *topic_name, int topic_name_len, uint8_t *data, int data_len_in_bytes,
				 int *bytes_returned) __EXPORT;

	int px4muorb_receive_bulk_data(uint8_t *_BulkTransferBuffer, int max_size_in_bytes,
				       int *length_in_bytes, int *topic_count) __EXPORT;

	int px4muorb_unblock_recieve_msg(void) __EXPORT;

}
