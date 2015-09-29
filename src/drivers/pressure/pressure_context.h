/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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

#ifndef PRESSURE_CONTEXT_H_
#define PRESSURE_CONTEXT_H_

#include <semaphore.h>
#include <pthread.h>
#include <dev_fs_lib.h>
#include <dev_fs_lib_i2c.h>

#include "pressure_api.h"

/**
 * The maximum length of the device path used when naming the port or bus resources to be
 * opened.
 *
 * TODO: Must be moved back into DspAL header files.
 */
#define MAX_LEN_DEVICE_PATH_IN_BYTES 32

struct pressure_context
{
   char device_path[MAX_LEN_DEVICE_PATH_IN_BYTES];
   int fildes;
   int last_error;
   pthread_mutex_t mutex;
   sem_t new_data_sem;
   float altimeter_setting_in_mbars;
   pthread_t read_thread_handle;
   char is_thread_running;
   struct pressure_sensor_data sensor_data;
};

#endif /* PRESSURE_CONTEXT_H_ */
