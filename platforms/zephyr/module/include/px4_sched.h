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

/**
 * @file px4_sched.h
 *
 * PX4 uses the thread priorities of NuttX on Zephyr, from 1 to 255, which
 * the POSIX layer maps to the preemptible priorities of Zephyr in the same
 * order.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get the POSIX priority of Zephyr for a PX4 priority
 *
 * The top PX4_ZEPHYR_PRIORITY_BAND PX4 priorities, those of the work queues,
 * map one to one. The others are compressed below them.
 *
 * @param priority PX4 priority, from SCHED_PRIORITY_MIN to SCHED_PRIORITY_MAX
 * @return POSIX priority of SCHED_RR, from 0 to CONFIG_NUM_PREEMPT_PRIORITIES - 1
 */
int px4_zephyr_posix_priority(int priority);

/** Number of the highest PX4 priorities that map one to one */
#define PX4_ZEPHYR_PRIORITY_BAND 64

#ifdef __cplusplus
}
#endif
