/****************************************************************************
 * configs/same70-xplained/scripts/memory.ld
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/* The SAME70Q21 has 2048Kb of FLASH beginning at address 0x0040:0000 and
 * 384Kb of SRAM beginining at 0x2040:0000
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x0000:0000
 * where the code expects to begin execution by jumping to the entry point in
 * the 0x0400:0000 address range.
 *
 * NOTE: that the DTCM address of 0x2000:0000 is used for SRAM.  If DTCM is
 * disabled, then the accesses will actually occur on the AHB bus.
 *
 * The user space partition will be spanned with a single region of size
 * 2**n bytes.  The alignment of the user-space region must be the same.
 * As a consequence, as the user-space increases in size, the alignment
 * requirement also increases.  The sizes below give the largest possible
 * user address spaces (but leave far too much for the OS).
 *
 * The solution to this wasted memory is to (1) use more than one region to
 * span the user spaces, or (2) poke holes in a larger region to trim it
 * to fit better.
 *
 * A detailed memory map for the 384KB SRAM region is as follows:
 *
 *   0x20000 0000:  Kernel .data region.  Typical size: 0.1KB
 *   ------- ----   Kernel .bss  region.  Typical size: 1.8KB
 *   0x20000 0800:  Kernel IDLE thread stack (approximate).  Size is
 *                  determined by CONFIG_IDLETHREAD_STACKSIZE and
 *                  adjustments for alignment.  Typical is 1KB.
 *   ------- ----   Padded to 4KB
 *   0x20002 0000:  User .data region.  Size is variable.
 *   ------- ----   User .bss region  Size is variable.
 *   0x20000 2000:  Beginning of kernel heap.  Size determined by
 *                  CONFIG_MM_KERNEL_HEAPSIZE.
 *   ------- ----   Beginning of user heap.  Can vary with other settings.
 *   0x20004 0000:  End+1 of mappable internal SRAM
 */

MEMORY
{
    /* 2048KiB of internal FLASH */

    kflash (rx)      : ORIGIN = 0x00400000, LENGTH = 1M
    uflash (rx)      : ORIGIN = 0x00500000, LENGTH = 1M

    /* 384Kb of internal SRAM */

    ksram (rwx)      : ORIGIN = 0x20000000, LENGTH = 128K
    usram (rwx)      : ORIGIN = 0x20020000, LENGTH = 128K
    xsram (rwx)      : ORIGIN = 0x20040000, LENGTH = 128K
}
