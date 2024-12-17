/****************************************************************************
 *
 *   Copyright (C) 2021 Technology Innovation Institute. All rights reserved.
 *   Author: @author Jukka Laitinen <jukkax@ssrc.tii.ae>
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
 * @file board_reset.cpp
 * Implementation of Microchip PolarFire based Board RESET API
 */

#include <px4_platform_common/px4_config.h>
#include <errno.h>
#include <nuttx/atomic.h>
#include <nuttx/board.h>
#include <nuttx/sched.h>
#include <hardware/mpfs_wdog.h>

#include "riscv_internal.h"

#ifdef CONFIG_SMP
/* The CPU mask for all (valid) CPUs */

#define ALL_CPUS ((1 << CONFIG_SMP_NCPUS) - 1)

/* PX4 has -nostdinc++ set, which confuses the logic in <nuttx/atomic.h>
 * making it pull the C-versions of atomic_fetch_add/atomic_load which use
 * __auto_type, which should match the counters here.
 */

#define __auto_type int *

/* With SMP, first we need to wait for all CPUs to pause (g_cpus_paused).
 * Then the CPUs can be reset, but CPU0 must be reset last (g_cpus_ready).
 * Otherwise a race can occur on g_cpus_paused as CPU0 clears .bss.
 *
 * Note: We cannot use locks that allow synchronization (like semaphores) here
 * because we want the system reset to happen as fast as possible.
 */

static int g_cpus_paused;
static int g_cpus_ready;

/* Handle for nxsched_smp_call_async */

static struct smp_call_data_s g_reboot_data;
#endif

extern "C" void __start(void);

static void board_reset_enter_bootloader()
{
	/* Reset the whole SoC */

	up_systemreset();
}

static void board_reset_enter_bootloader_and_continue_boot()
{
	/* Reset by triggering WDOG */

	putreg32(WDOG_FORCE_IMMEDIATE_RESET, MPFS_WDOG0_LO_BASE + MPFS_WDOG_FORCE_OFFSET);

	/* Wait for the reset */

	for (; ;);
}

static int board_reset_enter_app(FAR void *arg)
{
	uintptr_t hartid;

	/* Mask local interrupts */

	up_irq_save();

	/* It is now safe to read and hold hartid locally (CPU cannot change) */

	hartid = riscv_mhartid();

#ifdef CONFIG_SMP
	/* Notify that this CPU is paused */

	atomic_fetch_add(&g_cpus_paused, 1);

	/* Wait for ALL CPUs to be ready */

	while (atomic_load(&g_cpus_paused) < CONFIG_SMP_NCPUS);

	/* Notify that this CPU has now ready */

	atomic_fetch_add(&g_cpus_ready, 1);

	/* CPU0 must then wait for other CPUs to start */

	if (this_cpu() == 0) {
		while (atomic_load(&g_cpus_ready) < CONFIG_SMP_NCPUS);
	}

#endif

	/* Make sure the CPU cache is flushed so data is not lost */

	up_flush_dcache_all();

	asm volatile
	(
		"fence\n"
		"mv   a0, %0\n"
		"tail __start\n" :: "r"(hartid) : "memory"
	);

	/* Never reached */

	return 0;
}

int board_reset(int status)
{
#if defined(BOARD_HAS_ON_RESET)
	board_on_reset(status);
#endif

	if (status == 1) {
		board_reset_enter_bootloader();

	} else if (status == 2) {
		board_reset_enter_bootloader_and_continue_boot();
	}

#ifdef CONFIG_SMP
	g_reboot_data.func = board_reset_enter_app;
	g_reboot_data.arg  = NULL;
	g_cpus_paused      = 0;
	g_cpus_ready       = 0;

	/* Reset the other CPUs via SMP call.
	 *
	 * The SMP call in fact does run the callback on this CPU as well if
	 * requested to do so, but it does it BEFORE dispatching the request to the
	 * other CPUs. This is why we need to remove ourself from the CPU set.
	 */

	cpu_set_t cpuset = ALL_CPUS;
	CPU_CLR(this_cpu(), &cpuset);

	/* We cannot wait in smp_call because the target CPUs are reset, thus they
	 * will never acknowledge that they have run the handler!
	 */

	nxsched_smp_call_async(cpuset, &g_reboot_data);
#endif

	/* Just reboot via reset vector */

	board_reset_enter_app(NULL);

	return 0;
}
