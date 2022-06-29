/****************************************************************************
 *
 *   Copyright (c) 2022 Technology Innovation Institute. All rights reserved.
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

#include <stdbool.h>

#include <px4_platform_common/px4_config.h>

#include <px4_platform/board_ctrl.h>
#include <px4_platform/micro_hal.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/sem.h>

#include <drivers/drv_hrt.h>

#ifndef MODULE_NAME
#  define MODULE_NAME "hrt_ioctl"
#endif

#define HRT_ENTRY_QUEUE_MAX_SIZE 3
static px4_sem_t g_wait_sem;
static struct hrt_call *next_hrt_entry[HRT_ENTRY_QUEUE_MAX_SIZE];
static int hrt_entry_queued = 0;
static bool suppress_entry_queue_error = false;
static bool hrt_entry_queue_error = false;

void hrt_usr_call(void *arg)
{
	// This is called from hrt interrupt
	if (hrt_entry_queued < HRT_ENTRY_QUEUE_MAX_SIZE) {
		next_hrt_entry[hrt_entry_queued++] = (struct hrt_call *)arg;

	} else {
		hrt_entry_queue_error = true;
	}

	px4_sem_post(&g_wait_sem);
}

int hrt_ioctl(unsigned int cmd, unsigned long arg);

void hrt_ioctl_init(void)
{
	/* Create a semaphore for handling hrt driver callbacks */
	px4_sem_init(&g_wait_sem, 0, 0);

	/* this is a signalling semaphore */
	px4_sem_setprotocol(&g_wait_sem, SEM_PRIO_NONE);

	/* register ioctl callbacks */
	px4_register_boardct_ioctl(_HRTIOCBASE, hrt_ioctl);
}

/* These functions are inlined in all but NuttX protected/kernel builds */

latency_info_t get_latency(uint16_t bucket_idx, uint16_t counter_idx)
{
	latency_info_t ret = {latency_buckets[bucket_idx], latency_counters[counter_idx]};
	return ret;
}

void reset_latency_counters(void)
{
	for (int i = 0; i <= get_latency_bucket_count(); i++) {
		latency_counters[i] = 0;
	}
}

/* board_ioctl interface for user-space hrt driver */
int
hrt_ioctl(unsigned int cmd, unsigned long arg)
{
	hrt_boardctl_t *h = (hrt_boardctl_t *)arg;

	switch (cmd) {
	case HRT_WAITEVENT: {
			irqstate_t flags;
			px4_sem_wait(&g_wait_sem);
			/* Atomically update the pointer to user side hrt entry */
			flags = px4_enter_critical_section();

			/* This should be always true, but check it anyway */
			if (hrt_entry_queued > 0) {
				*(struct hrt_call **)arg = next_hrt_entry[--hrt_entry_queued];
				next_hrt_entry[hrt_entry_queued] = NULL;

			} else {
				hrt_entry_queue_error = true;
			}

			px4_leave_critical_section(flags);

			/* Warn once for entry queue being full */
			if (hrt_entry_queue_error && !suppress_entry_queue_error) {
				PX4_ERR("HRT entry error, queue size now %d", hrt_entry_queued);
				suppress_entry_queue_error = true;
			}
		}
		break;

	case HRT_ABSOLUTE_TIME:
		*(hrt_abstime *)arg = hrt_absolute_time();
		break;

	case HRT_CALL_AFTER:
		hrt_call_after(h->entry, h->time, (hrt_callout)hrt_usr_call, h->entry);
		break;

	case HRT_CALL_AT:
		hrt_call_at(h->entry, h->time, (hrt_callout)hrt_usr_call, h->entry);
		break;

	case HRT_CALL_EVERY:
		hrt_call_every(h->entry, h->time, h->interval, (hrt_callout)hrt_usr_call, h->entry);
		break;

	case HRT_CANCEL:
		if (h && h->entry) {
			hrt_cancel(h->entry);

		} else {
			PX4_ERR("HRT_CANCEL called with NULL entry");
		}

		break;

	case HRT_GET_LATENCY: {
			latency_boardctl_t *latency = (latency_boardctl_t *)arg;
			latency->latency = get_latency(latency->bucket_idx, latency->counter_idx);
		}
		break;

	case HRT_RESET_LATENCY:
		reset_latency_counters();
		break;

	default:
		return -EINVAL;
	}

	return OK;
}
