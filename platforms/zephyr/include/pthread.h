/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

// <zephyr/kernel.h> is included here, and its struct device conflicts with namespace device in C++.
// As visibility.h includes this header first, PX4 C++ files always see struct zephyr_device, and can't
// use the Zephyr device drivers: C files can.
#ifdef __cplusplus
#define device zephyr_device
#include_next <pthread.h>
#undef device
#else
#include_next <pthread.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

// PX4 sets the thread priorities of NuttX, which px4_sched.c maps to those of Zephyr, and destroys the attributes of
// threads that still run, which px4_pthread.c allows. Only PX4 code, built with this header, calls these functions:
// the rest of the image keeps the functions of Zephyr.
int px4_pthread_attr_init(pthread_attr_t *attr);
int px4_pthread_attr_destroy(pthread_attr_t *attr);
int px4_pthread_attr_setschedpolicy(pthread_attr_t *attr, int policy);
int px4_pthread_attr_setschedparam(pthread_attr_t *attr, const struct sched_param *param);
int px4_pthread_setschedparam(pthread_t thread, int policy, const struct sched_param *param);

#ifdef __cplusplus
}
#endif

#define pthread_attr_init		px4_pthread_attr_init
#define pthread_attr_destroy		px4_pthread_attr_destroy
#define pthread_attr_setschedpolicy	px4_pthread_attr_setschedpolicy
#define pthread_attr_setschedparam	px4_pthread_attr_setschedparam
#define pthread_setschedparam		px4_pthread_setschedparam
