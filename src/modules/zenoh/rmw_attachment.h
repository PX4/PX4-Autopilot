/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file rmw_attachment.h
 *
 * ROS2 RMW Attachment helper
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#include <zenoh-pico.h>

#pragma once

/* Derived from ROS2 rmw https://github.com/ros2/rmw/blob/e6addf2411b8ee8a2ac43d691533b8c05ae8f1b6/rmw/include/rmw/types.h#L44 */
#define RMW_GID_STORAGE_SIZE 16u

/* See rmw_zenoh design.md for more information https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md#publishers */
#define RMW_ATTACHEMENT_SIZE (8u + 8u + 1u + RMW_GID_STORAGE_SIZE)

typedef struct __attribute__((__packed__)) RmwAttachment {
	int64_t sequence_number;
	int64_t time;
	uint8_t rmw_gid_size;
	uint8_t rmw_gid[RMW_GID_STORAGE_SIZE];
} RmwAttachment;
