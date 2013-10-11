/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file drv_dataman.h
 *
 * DATAMANAGER driver.
 */
#ifndef _DATAMANAGER_H
#define _DATAMANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#define DATAMANAGER_DEVICE_PATH "/fs/microsd/data"
#define DATAMANAGER_REVISION 0x0001

enum {
	DM_KEY_RTL_POINT = 0,
	DM_KEY_RETURN_POINT,
	DM_KEY_SAFE_POINTS,
	DM_KEY_WAY_POINTS,
	DM_KEY_FENCE_POINTS,
	DM_KEY_NUM_KEYS
};

enum {
	DM_KEY_RTL_POINT_MAX = 1,
	DM_KEY_RETURN_POINT_MAX = 1,
	DM_KEY_SAFE_POINTS_MAX = 5,
	DM_KEY_FENCE_POINTS_MAX = 10,
	DM_KEY_WAY_POINTS_MAX = 128,
};

enum {
	DM_PERSIST_POWER_ON_RESET = 0,  /* Data survives resets */
	DM_PERSIST_IN_FLIGHT_RESET,     /* Data survives in-flight resets only */
	DM_PERSIST_VOLATILE             /* Data does not survive resets */
};

enum {
	DM_INIT_REASON_POWER_ON = 0,  /* Data survives resets */
	DM_INIT_REASON_IN_FLIGHT     /* Data survives in-flight resets only */
};

#define DM_MAX_DATA_SIZE 126

__EXPORT int		dm_open(void);
__EXPORT void		dm_close(int fd);
__EXPORT ssize_t	dm_read(int fd, unsigned char item, unsigned char index, char *buffer, size_t buflen);
__EXPORT ssize_t	dm_write(int fd, unsigned char item, unsigned char index, unsigned char persistence, const char *buffer, size_t buflen);
__EXPORT int		dm_restart(unsigned char restart_type);

#ifdef __cplusplus
}
#endif

#endif
