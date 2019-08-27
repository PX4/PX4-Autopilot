/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
 *   Author: David Sidrane<david_s5@nscdg.com>
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

#include <px4_platform_common/config.h>
#include <nuttx/progmem.h>
#include <nuttx/compiler.h>
#include <stdlib.h>
#include <syslog.h>

#include <nuttx/arch.h>

#include <systemlib/cpuload.h>
#include "resources.hpp"

#if !defined(CONFIG_NSH_LIBRARY)

static const char *
tstate_name(const tstate_t s)
{
	switch (s) {
	case TSTATE_TASK_INVALID:    return "init";

	case TSTATE_TASK_PENDING:    return "PEND";

	case TSTATE_TASK_READYTORUN: return "READY";

	case TSTATE_TASK_RUNNING:    return "RUN";

	case TSTATE_TASK_INACTIVE:   return "inact";

	case TSTATE_WAIT_SEM:        return "w:sem";
#ifndef CONFIG_DISABLE_SIGNALS

	case TSTATE_WAIT_SIG:        return "w:sig";
#endif
#ifndef CONFIG_DISABLE_MQUEUE

	case TSTATE_WAIT_MQNOTEMPTY: return "w:mqe";

	case TSTATE_WAIT_MQNOTFULL:  return "w:mqf";
#endif
#ifdef CONFIG_PAGING

	case TSTATE_WAIT_PAGEFILL:   return "w:pgf";
#endif

	default:
		return "ERROR";
	}
}

void stack_check(void)
{

	for (int i = 0; i < CONFIG_MAX_TASKS; i++) {

		if (system_load.tasks[i].tcb) {
			size_t stack_size = system_load.tasks[i].tcb->adj_stack_size;
			ssize_t stack_free = 0;

#if CONFIG_ARCH_INTERRUPTSTACK > 3

			if (system_load.tasks[i].tcb->pid == 0) {
				stack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~3);
				stack_free = up_check_intstack_remain();

			} else {
#endif
				stack_free = up_check_tcbstack_remain(system_load.tasks[i].tcb);
#if CONFIG_ARCH_INTERRUPTSTACK > 3
			}

#endif
			::syslog(LOG_INFO, "%4d %*-s %8lld %5u/%5u %3u (%3u) ",
				 system_load.tasks[i].tcb->pid,
				 CONFIG_TASK_NAME_SIZE, system_load.tasks[i].tcb->name,
				 (system_load.tasks[i].total_runtime / 1000),
				 stack_size - stack_free,
				 stack_size,
				 system_load.tasks[i].tcb->sched_priority,
				 system_load.tasks[i].tcb->base_priority);

#if CONFIG_RR_INTERVAL > 0
			/* print scheduling info with RR time slice */
			::syslog(LOG_INFO, " %6d\n", system_load.tasks[i].tcb->timeslice);
#else
			/* print task state instead */
			::syslog(LOG_INFO, " %-6s\n", tstate_name((tstate_t)system_load.tasks[i].tcb->task_state));
#endif
		}
	}
}

static void free_getprogmeminfo(struct mallinfo *mem)
{
	size_t page = 0, stpage = 0xFFFF;
	size_t pagesize = 0;
	ssize_t status;

	mem->arena    = 0;
	mem->fordblks = 0;
	mem->uordblks = 0;
	mem->mxordblk = 0;

	for (status = 0, page = 0; status >= 0; page++) {
		status = up_progmem_ispageerased(page);
		pagesize = up_progmem_pagesize(page);

		mem->arena += pagesize;

		/* Is this beginning of new free space section */

		if (status == 0) {
			if (stpage == 0xFFFF) { stpage = page; }

			mem->fordblks += pagesize;

		} else if (status != 0) {
			mem->uordblks += pagesize;

			if (stpage != 0xFFFF && up_progmem_isuniform()) {
				stpage = page - stpage;

				if (stpage > (size_t) mem->mxordblk) {
					mem->mxordblk = stpage;
				}

				stpage = 0xFFFF;
			}
		}
	}

	mem->mxordblk *= pagesize;
}

void free_check(void)
{
	struct mallinfo data;
	struct mallinfo prog;

#ifdef CONFIG_CAN_PASS_STRUCTS
	data = mallinfo();
#else
	(void)mallinfo(&data);
#endif

	free_getprogmeminfo(&prog);


	::syslog(LOG_INFO, "              total       used       free    largest\n");

	::syslog(LOG_INFO, "Data:   %11d%11d%11d%11d\n",
		 data.arena, data.uordblks, data.fordblks, data.mxordblk);

	::syslog(LOG_INFO, "Prog:   %11d%11d%11d%11d\n",
		 prog.arena, prog.uordblks, prog.fordblks, prog.mxordblk);
}
#endif /* CONFIG_NSH_LIBRARY */
