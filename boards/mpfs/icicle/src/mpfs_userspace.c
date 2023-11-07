/****************************************************************************
 * boards/risc-v/mpfs/common/kernel/mpfs_userspace.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>

#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>
#include <nuttx/wqueue.h>
#include <nuttx/userspace.h>

#if defined(CONFIG_BUILD_PROTECTED) && !defined(__KERNEL__)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_NUTTX_USERSPACE
#  error "CONFIG_NUTTX_USERSPACE not defined"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* These 'addresses' of these values are setup by the linker script.
 * They are not actual uint32_t storage locations!
 * They are only used meaningfully in the following way:
 *
 *  - The linker script defines, for example, the symbol_sdata.
 *  - The declaration extern uint32_t _sdata; makes C happy.  C will believe
 *    that the value _sdata is the address of a uint32_t variable _data
 *    (it is not!).
 *  - We can recover the linker value then by simply taking the address of
 *    of _data.  like:  uint32_t *pdata = &_sdata;
 */

extern uint32_t _stext;           /* Start of .text */
extern uint32_t _etext;           /* End_1 of .text + .rodata */
extern const uint32_t _eronly;    /* End+1 of read only section */
extern uint32_t _sdata;           /* Start of .data */
extern uint32_t _edata;           /* End+1 of .data */
extern uint32_t _sbss;            /* Start of .bss */
extern uint32_t _ebss;            /* End+1 of .bss */

extern uintptr_t *__ld_usram_end; /* End+1 of user ram section */

/* This is the user space entry point */

int CONFIG_INIT_ENTRYPOINT(int argc, char *argv[]);

const struct userspace_s userspace locate_data(".userspace") = {
	/* General memory map */

	.us_entrypoint    = (main_t)CONFIG_INIT_ENTRYPOINT,
	.us_textstart     = (uintptr_t) &_stext,
	.us_textend       = (uintptr_t) &_etext,
	.us_datasource    = (uintptr_t) &_eronly,
	.us_datastart     = (uintptr_t) &_sdata,
	.us_dataend       = (uintptr_t) &_edata,
	.us_bssstart      = (uintptr_t) &_sbss,
	.us_bssend        = (uintptr_t) &_ebss,

	.us_heapend       = (uintptr_t) &__ld_usram_end,

	/* Memory manager heap structure */

	.us_heap          = &g_mmheap,

	/* Task/thread startup routines */

	.task_startup     = nxtask_startup,

	/* Signal handler trampoline */

	.signal_handler   = up_signal_handler,

	/* User-space work queue support (declared in include/nuttx/wqueue.h) */

#ifdef CONFIG_LIBC_USRWORK
	.work_usrstart    = work_usrstart,
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_BUILD_PROTECTED && !__KERNEL__ */
