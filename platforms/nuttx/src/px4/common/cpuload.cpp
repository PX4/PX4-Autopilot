/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file cpuload.c
 *
 * Measurement of CPU load of each individual task.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Petri Tanskanen <petri.tanskanen@inf.ethz.ch>
 */
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform/cpuload.h>

#include <drivers/drv_hrt.h>

#if defined(__PX4_NUTTX) && defined(CONFIG_SCHED_INSTRUMENTATION)
__BEGIN_DECLS
# include <nuttx/sched_note.h>

__EXPORT struct system_load_s system_load;

/* Simple hashing via PID; shamelessly ripped from NuttX scheduler. All rights
 * and credit belong to whomever authored this logic.
 */

#define HASH(i) ((i) & (hashtab_size - 1))

struct system_load_taskinfo_s **hashtab;
volatile int hashtab_size;

void init_task_hash(void)
{
	hashtab_size = 4;
	hashtab = (struct system_load_taskinfo_s **)kmm_zalloc(sizeof(*hashtab) * hashtab_size);
}

static struct system_load_taskinfo_s *get_task_info(pid_t pid)
{
	struct system_load_taskinfo_s *ret = NULL;
	irqstate_t flags = px4_enter_critical_section();

	if (hashtab) {
		ret = hashtab[HASH(pid)];
	}

	px4_leave_critical_section(flags);
	return ret;
}

static void drop_task_info(pid_t pid)
{
	irqstate_t flags = px4_enter_critical_section();
	hashtab[HASH(pid)] = NULL;
	px4_leave_critical_section(flags);
}

static int hash_task_info(struct system_load_taskinfo_s *task_info, pid_t pid)
{
	struct system_load_taskinfo_s **newtab;
	void *temp;
	int hash;
	int i;

	/* Use critical section to protect the hash table */

	irqstate_t flags = px4_enter_critical_section();

	/* Keep trying until we get it or run out of memory */

retry:

	/* Calculate hash */

	hash = HASH(pid);

	/* Check if the entry is available */

	if (hashtab[hash] == NULL) {
		hashtab[hash] = task_info;
		px4_leave_critical_section(flags);
		return OK;
	}

	/* No can do, double the size of the hash table */

	newtab = (struct system_load_taskinfo_s **)kmm_zalloc(hashtab_size * 2 * sizeof(*newtab));

	if (newtab == NULL) {
		px4_leave_critical_section(flags);
		return -ENOMEM;
	}

	hashtab_size *= 2;

	/* Start using the new hash table */

	for (i = 0; i < hashtab_size / 2; i++) {
		struct system_load_taskinfo_s *info = hashtab[i];

		if (info && info->tcb) {
			hash = HASH(info->tcb->pid);
			newtab[hash] = hashtab[i];

		} else {
			newtab[i] = NULL;
		}
	}

	temp = hashtab;
	hashtab = newtab;
	kmm_free(temp);

	/* Try again */

	goto retry;
}

#if defined(CONFIG_SEGGER_SYSVIEW)
#  include <nuttx/note/note_sysview.h>
#  ifndef CONFIG_SEGGER_SYSVIEW_PREFIX
#   error Systemview enabled but prefix is not
#  endif
#endif

static px4::atomic_int cpuload_monitor_all_count{0};

void cpuload_monitor_start()
{
	if (cpuload_monitor_all_count.fetch_add(1) == 0) {
		// if the count was previously 0 (idle thread only) then clear any existing runtime data
		irqstate_t flags = px4_enter_critical_section();

		system_load.start_time = hrt_absolute_time();

		for (int i = CONFIG_SMP_NCPUS; i < CONFIG_FS_PROCFS_MAX_TASKS; i++) {
			system_load.tasks[i].total_runtime = 0;
			system_load.tasks[i].curr_start_time = 0;
		}

		px4_leave_critical_section(flags);
	}
}

void cpuload_monitor_stop()
{
	if (cpuload_monitor_all_count.fetch_sub(1) <= 1) {
		// don't allow the count to go negative
		cpuload_monitor_all_count.store(0);
	}
}

void cpuload_initialize_once()
{
	/* Initialize hashing */

	init_task_hash();

	for (auto &task : system_load.tasks) {
		task.valid = false;
	}

	// perform static initialization of "system" threads
	for (system_load.total_count = 0; system_load.total_count < CONFIG_SMP_NCPUS; system_load.total_count++) {
		system_load.tasks[system_load.total_count].total_runtime = 0;
		system_load.tasks[system_load.total_count].curr_start_time = 0;
		system_load.tasks[system_load.total_count].tcb = nxsched_get_tcb(
					system_load.total_count);	// it is assumed that these static threads have consecutive PIDs
		system_load.tasks[system_load.total_count].valid = true;
		hash_task_info(&system_load.tasks[system_load.total_count], system_load.total_count);
	}

	system_load.initialized = true;
}

void sched_note_start(FAR struct tcb_s *tcb)
{
	// find first free slot
	if (system_load.initialized) {
		for (auto &task : system_load.tasks) {
			if (!task.valid) {
				// slot is available
				task.total_runtime = 0;
				task.curr_start_time = 0;
				task.tcb = tcb;
				task.valid = true;
				system_load.total_count++;
				// add to the hashlist
				hash_task_info(&task, tcb->pid);
				break;
			}
		}
	}

#ifdef CONFIG_SEGGER_SYSVIEW
	sysview_sched_note_start(tcb);
#endif
}

void sched_note_stop(FAR struct tcb_s *tcb)
{
	if (system_load.initialized) {
		for (auto &task : system_load.tasks) {
			if (task.tcb && task.tcb->pid == tcb->pid) {
				// mark slot as free
				task.valid = false;
				task.total_runtime = 0;
				task.curr_start_time = 0;
				task.tcb = nullptr;
				system_load.total_count--;
				// drop from the tasklist
				drop_task_info(tcb->pid);
				break;
			}
		}
	}

#ifdef CONFIG_SEGGER_SYSVIEW
	sysview_sched_note_stop(tcb);
#endif
}

void sched_note_suspend(FAR struct tcb_s *tcb)
{
	if (system_load.initialized) {
		if (tcb->pid < CONFIG_SMP_NCPUS) {
			system_load.tasks[tcb->pid].total_runtime += hrt_elapsed_time(&system_load.tasks[tcb->pid].curr_start_time);
			return;

		} else {
			if (cpuload_monitor_all_count.load() == 0) {
				return;
			}
		}

		struct system_load_taskinfo_s *task = get_task_info(tcb->pid);

		if (task) {
			// Task ending its current scheduling run
			if (task->valid && (task->curr_start_time > 0)
			    && task->tcb && task->tcb->pid == tcb->pid) {
				task->total_runtime += hrt_elapsed_time(&task->curr_start_time);
			}
		}
	}

#ifdef CONFIG_SEGGER_SYSVIEW
	sysview_sched_note_suspend(tcb);
#endif
}

void sched_note_resume(FAR struct tcb_s *tcb)
{
	if (system_load.initialized) {
		if (tcb->pid < CONFIG_SMP_NCPUS) {
			hrt_store_absolute_time(&system_load.tasks[tcb->pid].curr_start_time);
			return;

		} else {
			if (cpuload_monitor_all_count.load() == 0) {
				return;
			}
		}

		struct system_load_taskinfo_s *task = get_task_info(tcb->pid);

		if (task) {
			if (task->valid && task->tcb && task->tcb->pid == tcb->pid) {
				// curr_start_time is accessed from an IRQ handler (in logger), so we need
				// to make the update atomic
				hrt_store_absolute_time(&task->curr_start_time);
			}
		}
	}

#ifdef CONFIG_SEGGER_SYSVIEW
	sysview_sched_note_resume(tcb);
#endif
}

#ifdef CONFIG_SEGGER_SYSVIEW

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
void sched_note_irqhandler(int irq, FAR void *handler, bool enter)
{
	sysview_sched_note_irqhandler(irq, handler, enter);
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
void sched_note_syscall_enter(int nr);
{
	sysview_sched_note_syscall_enter(nr);
}

void sched_note_syscall_enter(int nr);
{
	sysview_sched_note_syscall_enter(nr);
}
#endif

#endif

#if defined(CONFIG_SMP) && defined(CONFIG_SCHED_INSTRUMENTATION)
void sched_note_cpu_start(FAR struct tcb_s *tcb, int cpu)
{
	/* Not interesting for us */
}

void sched_note_cpu_started(FAR struct tcb_s *tcb)
{
	/* Not interesting for us */
}
#endif

#if defined(CONFIG_SMP) && defined(CONFIG_SCHED_INSTRUMENTATION_SWITCH)
void sched_note_cpu_pause(FAR struct tcb_s *tcb, int cpu)
{
	/* Not interesting for us */
}

void sched_note_cpu_paused(FAR struct tcb_s *tcb)
{
	/* Handled via sched_note_suspend */
}

void sched_note_cpu_resume(FAR struct tcb_s *tcb, int cpu)
{
	/* Not interesting for us */
}

void sched_note_cpu_resumed(FAR struct tcb_s *tcb)
{
	/* Handled via sched_note_resume */
}
#endif

__END_DECLS
#endif // PX4_NUTTX && CONFIG_SCHED_INSTRUMENTATION
