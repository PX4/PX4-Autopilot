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
#include <nuttx/kmalloc.h>
#include <queue.h>

#ifndef MODULE_NAME
#  define MODULE_NAME "hrt_ioctl"
#endif

struct usr_hrt_call {
	struct sq_entry_s list_item; /* List head for local sl_list */
	struct hrt_call entry;       /* Kernel side entry for HRT driver */
	struct hrt_call *usr_entry;  /* Reference to user side entry */
};

static sq_queue_t callout_queue;
static sq_queue_t callout_freelist;
static sq_queue_t callout_inflight;

/* Find (pop) first entry for user from queue, the queue must be locked prior */

struct usr_hrt_call *pop_user(sq_queue_t *queue, const px4_hrt_handle_t handle)
{
	sq_entry_t *queued;

	sq_for_every(queue, queued) {
		struct usr_hrt_call *e = (void *)queued;

		if (e->entry.callout_sem == handle) {
			sq_rem(queued, queue);
			return e;
		}
	}

	return NULL;
}

/* Find (pop) entry from queue, the queue must be locked prior */

struct usr_hrt_call *pop_entry(sq_queue_t *queue, const px4_hrt_handle_t handle, struct hrt_call *entry)
{
	sq_entry_t *queued;

	sq_for_every(queue, queued) {
		struct usr_hrt_call *e = (void *)queued;

		if (e->usr_entry == entry && e->entry.callout_sem == handle) {
			sq_rem(queued, queue);
			return e;
		}
	}

	return NULL;
}

/**
 * Copy user entry to kernel space. Either re-uses existing one or if none can
 * be found, creates a new.
 *
 * handle  : user space handle to identify who is behind the HRT request
 * entry   : user space HRT entry
 * callout : user callback
 * arg     : user argument passed in callback
 */
static struct usr_hrt_call *dup_entry(const px4_hrt_handle_t handle, struct hrt_call *entry, hrt_callout callout,
				      void *arg)
{
	struct usr_hrt_call *e = NULL;

	irqstate_t flags = px4_enter_critical_section();

	/* check if this is already queued */
	e = pop_entry(&callout_queue, handle, entry);

	/* it was not already queued, get from freelist */
	if (!e) {
		e = (void *)sq_remfirst(&callout_freelist);
	}

	px4_leave_critical_section(flags);

	if (!e) {
		/* Allocate a new kernel side item for the user call */

		e = kmm_malloc(sizeof(struct usr_hrt_call));
	}

	if (e) {

		/* Store the user side callout function and argument to the user's handle */
		entry->callout = callout;
		entry->arg = arg;

		/* Store reference to the kernel side entry to the user side struct and
		 * references to the semaphore and user side entry to the kernel side item
		 */

		e->entry.callout_sem = handle;
		e->usr_entry = entry;

		/* Add this to the callout_queue list */
		flags = px4_enter_critical_section();
		sq_addfirst(&e->list_item, &callout_queue);
		px4_leave_critical_section(flags);

	} else {
		PX4_ERR("out of memory");

	}

	return e;
}

void hrt_usr_call(void *arg)
{
	// This is called from hrt interrupt
	struct usr_hrt_call *e = (struct usr_hrt_call *)arg;
	sq_addfirst(&e->list_item, &callout_inflight);
	px4_sem_post(e->entry.callout_sem);
}

int hrt_ioctl(unsigned int cmd, unsigned long arg);

void hrt_ioctl_init(void)
{
	sq_init(&callout_queue);
	sq_init(&callout_freelist);
	sq_init(&callout_inflight);

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
			struct hrt_boardctl *ioc_parm = (struct hrt_boardctl *)arg;
			px4_sem_t *callout_sem = (px4_sem_t *)ioc_parm->handle;
			struct usr_hrt_call *e;
			px4_sem_wait(callout_sem);

			/* Atomically update the pointer to user side hrt entry */
			flags = px4_enter_critical_section();
			e = pop_user(&callout_inflight, callout_sem);

			if (e) {
				ioc_parm->callout = e->usr_entry->callout;
				ioc_parm->arg = e->usr_entry->arg;

				// If the period is 0, the callout is no longer queued by hrt driver
				// move it back to freelist
				if (e->entry.period == 0) {
					sq_rem((sq_entry_t *)e, &callout_queue);
					sq_addfirst((sq_entry_t *)e, &callout_freelist);

				}

			} else {
				PX4_ERR("HRT_WAITEVENT error no entry");
			}

			px4_leave_critical_section(flags);
		}
		break;

	case HRT_ABSOLUTE_TIME:
		*(hrt_abstime *)arg = hrt_absolute_time();
		break;

	case HRT_CALL_AFTER: {
			struct usr_hrt_call *e = dup_entry(h->handle, h->entry, h->callout, h->arg);

			if (e) {
				hrt_call_after(&e->entry, h->time, (hrt_callout)hrt_usr_call, e);
			}
		}
		break;

	case HRT_CALL_AT: {
			struct usr_hrt_call *e = dup_entry(h->handle, h->entry, h->callout, h->arg);

			if (e) {
				hrt_call_at(&e->entry, h->time, (hrt_callout)hrt_usr_call, e);
			}
		}
		break;

	case HRT_CALL_EVERY: {
			struct usr_hrt_call *e = dup_entry(h->handle, h->entry, h->callout, h->arg);

			if (e) {
				hrt_call_every(&e->entry, h->time, h->interval, (hrt_callout)hrt_usr_call, e);
			}
		}
		break;

	case HRT_CANCEL:
		if (h && h->entry) {
			/* Find the user entry */
			irqstate_t flags = px4_enter_critical_section();
			struct usr_hrt_call *e = pop_entry(&callout_queue, h->handle, h->entry);

			if (e) {
				hrt_cancel(&e->entry);
				sq_addfirst((sq_entry_t *)e, &callout_freelist);

			}

			px4_leave_critical_section(flags);

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

	case HRT_REGISTER: {
			px4_sem_t *callback_sem = kmm_malloc(sizeof(px4_sem_t));

			/* Create a semaphore for handling hrt driver callbacks */
			if (px4_sem_init(callback_sem, 0, 0) == 0) {

				/* this is a signalling semaphore */
				px4_sem_setprotocol(callback_sem, SEM_PRIO_NONE);
				*(px4_sem_t **)arg = callback_sem;

			} else {
				*(px4_sem_t **)arg = NULL;
				return -ENOMEM;
			}

		}

		break;

	case HRT_UNREGISTER: {
			px4_sem_t *callback_sem = *(px4_sem_t **)arg;
			struct usr_hrt_call *e;
			irqstate_t flags;

			flags = px4_enter_critical_section();

			while ((e = (void *)sq_remfirst(&callout_queue))) {
				if (callback_sem == e->entry.callout_sem) {
					hrt_cancel(&e->entry);
					/* Remove potential inflight entry as well */
					sq_rem(&e->list_item, &callout_inflight);
					kmm_free(e);
				}
			}

			px4_sem_destroy(callback_sem);

			px4_leave_critical_section(flags);

			*(px4_sem_t **)arg = NULL;
			kmm_free(callback_sem);
		}
		break;

	case HRT_ABSTIME_BASE: {
#ifdef PX4_USERSPACE_HRT
		*(uintptr_t *)arg = hrt_absolute_time_usr_base();
#else
		*(uintptr_t *)arg = NULL;
#endif
		}
		break;
	default:
		return -EINVAL;
	}

	return OK;
}
